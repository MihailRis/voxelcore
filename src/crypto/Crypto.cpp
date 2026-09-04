#include "Crypto.hpp"

#include <openssl/bn.h>
#include <openssl/core_names.h>
#include <openssl/crypto.h>
#include <openssl/ec.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/kdf.h>
#include <openssl/macros.h>
#include <openssl/obj_mac.h>
#include <openssl/param_build.h>
#include <openssl/params.h>
#include <openssl/rand.h>
#include <openssl/rsa.h>

#include <climits>
#include <memory>
#include <stdexcept>
#include <string>

namespace crypto {
    Error::Error(ErrorCode code, const std::string& message)
        : std::runtime_error(message), errorCode(code) {
    }

    ErrorCode Error::code() const noexcept {
        return errorCode;
    }

    const char* Error::codeName() const noexcept {
        switch (errorCode) {
            case ErrorCode::InvalidArgument:
                return "invalid_argument";
            case ErrorCode::InvalidKey:
                return "invalid_key";
            case ErrorCode::InvalidSignature:
                return "invalid_signature";
            case ErrorCode::AuthenticationFailed:
                return "authentication_failed";
            case ErrorCode::UnsupportedAlgorithm:
                return "unsupported_algorithm";
            case ErrorCode::OutputTooLarge:
                return "output_too_large";
            case ErrorCode::InvalidState:
                return "invalid_state";
            case ErrorCode::BackendError:
                return "backend_error";
        }
        return "backend_error";
    }

    namespace {
        constexpr std::size_t AEAD_TAG_SIZE = 16;

        template <typename T, void (*Free)(T*)>
        using Handle = std::unique_ptr<T, decltype(Free)>;

        using BnHandle = Handle<BIGNUM, BN_free>;
        using EcGroupHandle = Handle<EC_GROUP, EC_GROUP_free>;
        using EcPointHandle = Handle<EC_POINT, EC_POINT_free>;
        using EvpCipherCtxHandle = Handle<EVP_CIPHER_CTX, EVP_CIPHER_CTX_free>;
        using EvpMdCtxHandle = Handle<EVP_MD_CTX, EVP_MD_CTX_free>;
        using EvpPkeyHandle = Handle<EVP_PKEY, EVP_PKEY_free>;
        using EvpPkeyCtxHandle = Handle<EVP_PKEY_CTX, EVP_PKEY_CTX_free>;
        using ParamBldHandle = Handle<OSSL_PARAM_BLD, OSSL_PARAM_BLD_free>;
        using ParamHandle = Handle<OSSL_PARAM, OSSL_PARAM_free>;

        const unsigned char* bytes(std::string_view value) {
            return reinterpret_cast<const unsigned char*>(value.data());
        }

        std::string opensslError(const std::string& operation) {
            const unsigned long code = ERR_get_error();
            if (code == 0) {
                return operation;
            }
            char message[256];
            ERR_error_string_n(code, message, sizeof(message));
            return operation + ": " + message;
        }

        [[noreturn]] void fail(const std::string& operation) {
            throw Error(ErrorCode::BackendError, opensslError(operation));
        }

        int checkedSize(std::size_t size, const char* name) {
            if (size > static_cast<std::size_t>(INT_MAX)) {
                throw Error(
                    ErrorCode::OutputTooLarge,
                    std::string(name) + " is too large"
                );
            }
            return static_cast<int>(size);
        }

        const EVP_MD* getShaDigest(std::string_view hash) {
            if (hash == "SHA256") return EVP_sha256();
            if (hash == "SHA384") return EVP_sha384();
            if (hash == "SHA512") return EVP_sha512();
            throw Error(
                ErrorCode::UnsupportedAlgorithm,
                "unsupported hash: " + std::string(hash)
            );
        }

        const EVP_MD* getDigest(std::string_view hash) {
            if (hash == "MD5") return EVP_md5();
            return getShaDigest(hash);
        }

        const char* curveName(std::string_view curve) {
            if (curve == "P-256") return "prime256v1";
            if (curve == "P-384") return "secp384r1";
            if (curve == "P-521") return "secp521r1";
            throw Error(
                ErrorCode::UnsupportedAlgorithm,
                "unsupported curve: " + std::string(curve)
            );
        }

        int curveNid(std::string_view curve) {
            if (curve == "P-256") return NID_X9_62_prime256v1;
            if (curve == "P-384") return NID_secp384r1;
            if (curve == "P-521") return NID_secp521r1;
            curveName(curve);
            return NID_undef;
        }

        std::size_t curvePrivateSize(std::string_view curve) {
            if (curve == "P-256") return 32;
            if (curve == "P-384") return 48;
            if (curve == "P-521") return 66;
            curveName(curve);
            return 0;
        }

        EvpPkeyHandle fromData(
            const char* algorithm,
            int selection,
            OSSL_PARAM_BLD* builder,
            ErrorCode failureCode = ErrorCode::BackendError
        ) {
            ParamHandle params(
                OSSL_PARAM_BLD_to_param(builder), OSSL_PARAM_free
            );
            EvpPkeyCtxHandle ctx(
                EVP_PKEY_CTX_new_from_name(nullptr, algorithm, nullptr),
                EVP_PKEY_CTX_free
            );
            EVP_PKEY* rawKey = nullptr;
            if (!params || !ctx || EVP_PKEY_fromdata_init(ctx.get()) <= 0 ||
                EVP_PKEY_fromdata(
                    ctx.get(), &rawKey, selection, params.get()
                ) <= 0) {
                throw Error(
                    failureCode,
                    opensslError(
                        std::string("failed to import ") + algorithm + " key"
                    )
                );
            }
            return EvpPkeyHandle(rawKey, EVP_PKEY_free);
        }

        void checkPublicKey(EVP_PKEY* key, const char* name) {
            EvpPkeyCtxHandle ctx(
                EVP_PKEY_CTX_new(key, nullptr), EVP_PKEY_CTX_free
            );
            if (!ctx || EVP_PKEY_public_check(ctx.get()) != 1) {
                throw Error(
                    ErrorCode::InvalidKey,
                    opensslError(std::string("invalid ") + name + " public key")
                );
            }
        }

        EvpPkeyHandle makeEcdsaPublicKey(
            std::string_view curve, std::string_view encoded
        ) {
            if (encoded.empty() ||
                static_cast<unsigned char>(encoded[0]) != 0x04) {
                throw Error(
                    ErrorCode::InvalidKey,
                    "ECDSA public key must be an uncompressed SEC1 point"
                );
            }
            ParamBldHandle builder(OSSL_PARAM_BLD_new(), OSSL_PARAM_BLD_free);
            const char* group = curveName(curve);
            if (!builder ||
                OSSL_PARAM_BLD_push_utf8_string(
                    builder.get(), OSSL_PKEY_PARAM_GROUP_NAME, group, 0
                ) != 1 ||
                OSSL_PARAM_BLD_push_octet_string(
                    builder.get(),
                    OSSL_PKEY_PARAM_PUB_KEY,
                    encoded.data(),
                    encoded.size()
                ) != 1) {
                fail("failed to prepare ECDSA public key");
            }
            auto result = fromData(
                "EC", EVP_PKEY_PUBLIC_KEY, builder.get(), ErrorCode::InvalidKey
            );
            checkPublicKey(result.get(), "ECDSA");
            return result;
        }

        EvpPkeyHandle makeRsaPublicKey(
            std::string_view modulus, std::string_view exponent
        ) {
            if (modulus.empty() || exponent.empty()) {
                throw Error(
                    ErrorCode::InvalidKey,
                    "RSA modulus and exponent must not be empty"
                );
            }
            BnHandle n(
                BN_bin2bn(
                    bytes(modulus),
                    checkedSize(modulus.size(), "modulus"),
                    nullptr
                ),
                BN_free
            );
            BnHandle e(
                BN_bin2bn(
                    bytes(exponent),
                    checkedSize(exponent.size(), "exponent"),
                    nullptr
                ),
                BN_free
            );
            ParamBldHandle builder(OSSL_PARAM_BLD_new(), OSSL_PARAM_BLD_free);
            if (!n || !e || !builder) {
                fail("failed to prepare RSA public key");
            }
            if (BN_is_zero(n.get()) || BN_is_zero(e.get())) {
                throw Error(ErrorCode::InvalidKey, "invalid RSA public key");
            }
            if (OSSL_PARAM_BLD_push_BN(
                    builder.get(), OSSL_PKEY_PARAM_RSA_N, n.get()
                ) != 1 ||
                OSSL_PARAM_BLD_push_BN(
                    builder.get(), OSSL_PKEY_PARAM_RSA_E, e.get()
                ) != 1) {
                fail("invalid RSA public key");
            }
            auto result = fromData(
                "RSA", EVP_PKEY_PUBLIC_KEY, builder.get(), ErrorCode::InvalidKey
            );
            checkPublicKey(result.get(), "RSA");
            return result;
        }

        bool verifyDigestSignature(
            EVP_PKEY* key,
            const EVP_MD* md,
            std::string_view message,
            std::string_view signature,
            int rsaPadding,
            int saltLength
        ) {
            EvpMdCtxHandle ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
            EVP_PKEY_CTX* pkeyCtx = nullptr;
            if (!ctx ||
                EVP_DigestVerifyInit(ctx.get(), &pkeyCtx, md, nullptr, key) !=
                    1) {
                fail("signature verification initialization failed");
            }
            if (rsaPadding != 0) {
                if (EVP_PKEY_CTX_set_rsa_padding(pkeyCtx, rsaPadding) <= 0) {
                    fail("failed to configure RSA padding");
                }
                if (rsaPadding == RSA_PKCS1_PSS_PADDING &&
                    EVP_PKEY_CTX_set_rsa_pss_saltlen(pkeyCtx, saltLength) <=
                        0) {
                    fail("failed to configure RSA-PSS salt length");
                }
            }
            if (EVP_DigestVerifyUpdate(
                    ctx.get(), message.data(), message.size()
                ) != 1) {
                fail("signature verification update failed");
            }
            const int result = EVP_DigestVerifyFinal(
                ctx.get(), bytes(signature), signature.size()
            );
            if (result < 0) {
                throw Error(
                    ErrorCode::InvalidSignature,
                    opensslError("invalid signature encoding")
                );
            }
            if (result == 0) ERR_clear_error();
            return result == 1;
        }

        EvpPkeyHandle rawPrivateKey(
            int type, std::string_view key, const char* name
        ) {
            if (key.size() != 32) {
                throw Error(
                    ErrorCode::InvalidKey,
                    std::string(name) + " private key must be 32 bytes"
                );
            }
            EvpPkeyHandle result(
                EVP_PKEY_new_raw_private_key(
                    type, nullptr, bytes(key), key.size()
                ),
                EVP_PKEY_free
            );
            if (!result) fail(std::string("invalid ") + name + " private key");
            return result;
        }

        EvpPkeyHandle rawPublicKey(
            int type, std::string_view key, const char* name
        ) {
            if (key.size() != 32) {
                throw Error(
                    ErrorCode::InvalidKey,
                    std::string(name) + " public key must be 32 bytes"
                );
            }
            EvpPkeyHandle result(
                EVP_PKEY_new_raw_public_key(
                    type, nullptr, bytes(key), key.size()
                ),
                EVP_PKEY_free
            );
            if (!result) fail(std::string("invalid ") + name + " public key");
            return result;
        }

        const EVP_CIPHER* aesGcmCipher(std::size_t keySize) {
            if (keySize == 16) return EVP_aes_128_gcm();
            if (keySize == 32) return EVP_aes_256_gcm();
            throw Error(
                ErrorCode::InvalidKey, "AES-GCM key must be 16 or 32 bytes"
            );
        }

        Bytes aeadEncrypt(
            const EVP_CIPHER* cipher,
            std::string_view key,
            std::string_view nonce,
            std::string_view aad,
            std::string_view plaintext
        ) {
            if (nonce.empty()) {
                throw Error(
                    ErrorCode::InvalidArgument, "nonce must not be empty"
                );
            }
            const int nonceSize = checkedSize(nonce.size(), "nonce");
            const int aadSize = checkedSize(aad.size(), "aad");
            const int plaintextSize =
                checkedSize(plaintext.size(), "plaintext");
            EvpCipherCtxHandle ctx(EVP_CIPHER_CTX_new(), EVP_CIPHER_CTX_free);
            if (!ctx ||
                EVP_EncryptInit_ex(
                    ctx.get(), cipher, nullptr, nullptr, nullptr
                ) != 1 ||
                EVP_CIPHER_CTX_ctrl(
                    ctx.get(), EVP_CTRL_AEAD_SET_IVLEN, nonceSize, nullptr
                ) != 1 ||
                EVP_EncryptInit_ex(
                    ctx.get(), nullptr, nullptr, bytes(key), bytes(nonce)
                ) != 1) {
                fail("AEAD encryption initialization failed");
            }
            int written = 0;
            if (!aad.empty() &&
                EVP_EncryptUpdate(
                    ctx.get(), nullptr, &written, bytes(aad), aadSize
                ) != 1) {
                fail("AEAD AAD processing failed");
            }
            Bytes output(plaintext.size() + AEAD_TAG_SIZE);
            int outputSize = 0;
            if (!plaintext.empty() && EVP_EncryptUpdate(
                                          ctx.get(),
                                          output.data(),
                                          &outputSize,
                                          bytes(plaintext),
                                          plaintextSize
                                      ) != 1) {
                fail("AEAD encryption failed");
            }
            if (EVP_EncryptFinal_ex(
                    ctx.get(), output.data() + outputSize, &written
                ) != 1) {
                fail("AEAD encryption finalization failed");
            }
            outputSize += written;
            if (EVP_CIPHER_CTX_ctrl(
                    ctx.get(),
                    EVP_CTRL_AEAD_GET_TAG,
                    AEAD_TAG_SIZE,
                    output.data() + outputSize
                ) != 1) {
                fail("AEAD tag retrieval failed");
            }
            output.resize(static_cast<std::size_t>(outputSize) + AEAD_TAG_SIZE);
            return output;
        }

        Bytes aeadDecrypt(
            const EVP_CIPHER* cipher,
            std::string_view key,
            std::string_view nonce,
            std::string_view aad,
            std::string_view ciphertextWithTag
        ) {
            if (nonce.empty()) {
                throw Error(
                    ErrorCode::InvalidArgument, "nonce must not be empty"
                );
            }
            if (ciphertextWithTag.size() < AEAD_TAG_SIZE) {
                throw Error(
                    ErrorCode::AuthenticationFailed,
                    "ciphertext is shorter than the authentication tag"
                );
            }
            const std::size_t ciphertextSize =
                ciphertextWithTag.size() - AEAD_TAG_SIZE;
            const int nonceSize = checkedSize(nonce.size(), "nonce");
            const int aadSize = checkedSize(aad.size(), "aad");
            const int inputSize = checkedSize(ciphertextSize, "ciphertext");
            EvpCipherCtxHandle ctx(EVP_CIPHER_CTX_new(), EVP_CIPHER_CTX_free);
            if (!ctx ||
                EVP_DecryptInit_ex(
                    ctx.get(), cipher, nullptr, nullptr, nullptr
                ) != 1 ||
                EVP_CIPHER_CTX_ctrl(
                    ctx.get(), EVP_CTRL_AEAD_SET_IVLEN, nonceSize, nullptr
                ) != 1 ||
                EVP_DecryptInit_ex(
                    ctx.get(), nullptr, nullptr, bytes(key), bytes(nonce)
                ) != 1) {
                fail("AEAD decryption initialization failed");
            }
            int written = 0;
            if (!aad.empty() &&
                EVP_DecryptUpdate(
                    ctx.get(), nullptr, &written, bytes(aad), aadSize
                ) != 1) {
                fail("AEAD AAD processing failed");
            }
            Bytes output(ciphertextSize + AEAD_TAG_SIZE);
            int outputSize = 0;
            if (ciphertextSize != 0 && EVP_DecryptUpdate(
                                           ctx.get(),
                                           output.data(),
                                           &outputSize,
                                           bytes(ciphertextWithTag),
                                           inputSize
                                       ) != 1) {
                fail("AEAD decryption failed");
            }
            if (EVP_CIPHER_CTX_ctrl(
                    ctx.get(),
                    EVP_CTRL_AEAD_SET_TAG,
                    AEAD_TAG_SIZE,
                    const_cast<unsigned char*>(
                        bytes(ciphertextWithTag) + ciphertextSize
                    )
                ) != 1) {
                fail("AEAD tag setup failed");
            }
            const int result = EVP_DecryptFinal_ex(
                ctx.get(), output.data() + outputSize, &written
            );
            if (result != 1) {
                ERR_clear_error();
                throw Error(
                    ErrorCode::AuthenticationFailed, "authentication failed"
                );
            }
            outputSize += written;
            output.resize(static_cast<std::size_t>(outputSize));
            return output;
        }

        struct EcKey {
            EvpPkeyHandle key;
            Bytes publicKey;
        };

        EcKey ecPrivateKey(
            std::string_view curve, std::string_view privateKey
        ) {
            const std::size_t expectedSize = curvePrivateSize(curve);
            if (privateKey.size() != expectedSize) {
                throw Error(
                    ErrorCode::InvalidKey,
                    std::string(curve) + " private key must be " +
                        std::to_string(expectedSize) + " bytes"
                );
            }
            EcGroupHandle group(
                EC_GROUP_new_by_curve_name(curveNid(curve)), EC_GROUP_free
            );
            BnHandle scalar(
                BN_bin2bn(
                    bytes(privateKey),
                    checkedSize(privateKey.size(), "private key"),
                    nullptr
                ),
                BN_free
            );
            BnHandle order(BN_new(), BN_free);
            if (!group || !scalar || !order || BN_is_zero(scalar.get()) ||
                EC_GROUP_get_order(group.get(), order.get(), nullptr) != 1 ||
                BN_cmp(scalar.get(), order.get()) >= 0) {
                throw Error(ErrorCode::InvalidKey, "invalid EC private key");
            }
            EcPointHandle publicPoint(EC_POINT_new(group.get()), EC_POINT_free);
            if (!publicPoint || EC_POINT_mul(
                                    group.get(),
                                    publicPoint.get(),
                                    scalar.get(),
                                    nullptr,
                                    nullptr,
                                    nullptr
                                ) != 1) {
                fail("EC public key derivation failed");
            }
            const std::size_t publicSize = EC_POINT_point2oct(
                group.get(),
                publicPoint.get(),
                POINT_CONVERSION_UNCOMPRESSED,
                nullptr,
                0,
                nullptr
            );
            Bytes publicKey(publicSize);
            if (publicSize == 0 || EC_POINT_point2oct(
                                       group.get(),
                                       publicPoint.get(),
                                       POINT_CONVERSION_UNCOMPRESSED,
                                       publicKey.data(),
                                       publicKey.size(),
                                       nullptr
                                   ) != publicSize) {
                fail("EC public key encoding failed");
            }
            ParamBldHandle builder(OSSL_PARAM_BLD_new(), OSSL_PARAM_BLD_free);
            if (!builder ||
                OSSL_PARAM_BLD_push_utf8_string(
                    builder.get(),
                    OSSL_PKEY_PARAM_GROUP_NAME,
                    curveName(curve),
                    0
                ) != 1 ||
                OSSL_PARAM_BLD_push_BN(
                    builder.get(), OSSL_PKEY_PARAM_PRIV_KEY, scalar.get()
                ) != 1 ||
                OSSL_PARAM_BLD_push_octet_string(
                    builder.get(),
                    OSSL_PKEY_PARAM_PUB_KEY,
                    publicKey.data(),
                    publicKey.size()
                ) != 1) {
                fail("failed to prepare EC private key");
            }
            return {
                fromData("EC", EVP_PKEY_KEYPAIR, builder.get()),
                std::move(publicKey)
            };
        }

        KeyPair rawKeyPair(const char* algorithm) {
            EvpPkeyHandle key(
                EVP_PKEY_Q_keygen(nullptr, nullptr, algorithm), EVP_PKEY_free
            );
            if (!key) fail(std::string(algorithm) + " key generation failed");
            KeyPair result {Bytes(32), Bytes(32)};
            std::size_t privateSize = result.privateKey.size();
            std::size_t publicSize = result.publicKey.size();
            if (EVP_PKEY_get_raw_private_key(
                    key.get(), result.privateKey.data(), &privateSize
                ) != 1 ||
                EVP_PKEY_get_raw_public_key(
                    key.get(), result.publicKey.data(), &publicSize
                ) != 1) {
                fail(std::string(algorithm) + " key export failed");
            }
            result.privateKey.resize(privateSize);
            result.publicKey.resize(publicSize);
            return result;
        }

        Bytes rawPublicFromPrivate(
            int type, std::string_view privateKey, const char* name
        ) {
            auto key = rawPrivateKey(type, privateKey, name);
            Bytes result(32);
            std::size_t size = result.size();
            if (EVP_PKEY_get_raw_public_key(key.get(), result.data(), &size) !=
                1) {
                fail(std::string(name) + " public key derivation failed");
            }
            result.resize(size);
            return result;
        }

        Bytes signDigest(
            EVP_PKEY* key, const EVP_MD* md, std::string_view message
        ) {
            EvpMdCtxHandle ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
            if (!ctx ||
                EVP_DigestSignInit(ctx.get(), nullptr, md, nullptr, key) != 1 ||
                EVP_DigestSignUpdate(
                    ctx.get(), message.data(), message.size()
                ) != 1) {
                fail("signature initialization failed");
            }
            std::size_t size = 0;
            if (EVP_DigestSignFinal(ctx.get(), nullptr, &size) != 1) {
                fail("signature size query failed");
            }
            Bytes signature(size);
            if (EVP_DigestSignFinal(ctx.get(), signature.data(), &size) != 1) {
                fail("signing failed");
            }
            signature.resize(size);
            return signature;
        }
    }

    struct HashContext::Impl {
        explicit Impl(std::string_view name)
            : digest(getDigest(name)), ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free) {
            if (!ctx || EVP_DigestInit_ex(ctx.get(), digest, nullptr) != 1) {
                fail("hash context initialization failed");
            }
        }

        const EVP_MD* digest;
        EvpMdCtxHandle ctx;
        bool finalized = false;
    };

    HashContext::HashContext(std::string_view hash)
        : impl(std::make_unique<Impl>(hash)) {
    }

    HashContext::~HashContext() = default;
    HashContext::HashContext(HashContext&&) noexcept = default;
    HashContext& HashContext::operator=(HashContext&&) noexcept = default;

    void HashContext::update(std::string_view data) {
        if (impl->finalized) {
            throw Error(ErrorCode::InvalidState, "hash context is finalized");
        }
        if (EVP_DigestUpdate(impl->ctx.get(), data.data(), data.size()) != 1) {
            fail("hash update failed");
        }
    }

    Bytes HashContext::final() {
        if (impl->finalized) {
            throw Error(ErrorCode::InvalidState, "hash context is finalized");
        }
        Bytes output(static_cast<std::size_t>(EVP_MD_size(impl->digest)));
        unsigned int size = 0;
        if (EVP_DigestFinal_ex(impl->ctx.get(), output.data(), &size) != 1) {
            fail("hash finalization failed");
        }
        impl->finalized = true;
        output.resize(size);
        return output;
    }

    void HashContext::reset() {
        if (EVP_DigestInit_ex(impl->ctx.get(), impl->digest, nullptr) != 1) {
            fail("hash reset failed");
        }
        impl->finalized = false;
    }

    Bytes digest(std::string_view hash, std::string_view data) {
        const EVP_MD* md = getDigest(hash);
        Bytes output(static_cast<std::size_t>(EVP_MD_size(md)));
        unsigned int size = 0;
        if (EVP_Digest(
                data.data(), data.size(), output.data(), &size, md, nullptr
            ) != 1) {
            fail("digest calculation failed");
        }
        output.resize(size);
        return output;
    }

    Bytes hmac(
        std::string_view hash, std::string_view key, std::string_view data
    ) {
        const EVP_MD* md = getShaDigest(hash);
        Bytes output(static_cast<std::size_t>(EVP_MD_size(md)));
        unsigned int size = 0;
        if (!HMAC(
                md,
                key.data(),
                checkedSize(key.size(), "HMAC key"),
                bytes(data),
                data.size(),
                output.data(),
                &size
            )) {
            fail("HMAC calculation failed");
        }
        output.resize(size);
        return output;
    }

    KeyPair ed25519KeyPair() {
        return rawKeyPair("ED25519");
    }

    Bytes ed25519Public(std::string_view privateKey) {
        return rawPublicFromPrivate(EVP_PKEY_ED25519, privateKey, "Ed25519");
    }

    Bytes ed25519Sign(std::string_view privateKey, std::string_view message) {
        auto key = rawPrivateKey(EVP_PKEY_ED25519, privateKey, "Ed25519");
        EvpMdCtxHandle ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
        if (!ctx || EVP_DigestSignInit(
                        ctx.get(), nullptr, nullptr, nullptr, key.get()
                    ) != 1) {
            fail("Ed25519 signing initialization failed");
        }
        std::size_t size = 0;
        if (EVP_DigestSign(
                ctx.get(), nullptr, &size, bytes(message), message.size()
            ) != 1) {
            fail("Ed25519 signature size query failed");
        }
        Bytes signature(size);
        if (EVP_DigestSign(
                ctx.get(),
                signature.data(),
                &size,
                bytes(message),
                message.size()
            ) != 1) {
            fail("Ed25519 signing failed");
        }
        signature.resize(size);
        return signature;
    }

    bool ed25519Verify(
        std::string_view publicKey,
        std::string_view message,
        std::string_view signature
    ) {
        if (signature.size() != 64) {
            throw Error(
                ErrorCode::InvalidSignature,
                "Ed25519 signature must be 64 bytes"
            );
        }
        auto key = rawPublicKey(EVP_PKEY_ED25519, publicKey, "Ed25519");
        EvpMdCtxHandle ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
        if (!ctx || EVP_DigestVerifyInit(
                        ctx.get(), nullptr, nullptr, nullptr, key.get()
                    ) != 1) {
            fail("Ed25519 verification initialization failed");
        }
        const int result = EVP_DigestVerify(
            ctx.get(),
            bytes(signature),
            signature.size(),
            bytes(message),
            message.size()
        );
        if (result < 0) {
            throw Error(
                ErrorCode::InvalidSignature,
                opensslError("invalid Ed25519 signature")
            );
        }
        if (result == 0) ERR_clear_error();
        return result == 1;
    }

    KeyPair ecdsaKeyPair(std::string_view curve) {
        const std::size_t privateSize = curvePrivateSize(curve);
        EvpPkeyHandle key(
            EVP_PKEY_Q_keygen(nullptr, nullptr, "EC", curveName(curve)),
            EVP_PKEY_free
        );
        if (!key) fail("ECDSA key generation failed");
        BIGNUM* rawPrivate = nullptr;
        if (EVP_PKEY_get_bn_param(
                key.get(), OSSL_PKEY_PARAM_PRIV_KEY, &rawPrivate
            ) != 1) {
            fail("ECDSA private key export failed");
        }
        BnHandle privateKey(rawPrivate, BN_free);
        KeyPair result {Bytes(privateSize), {}};
        if (BN_bn2binpad(
                privateKey.get(),
                result.privateKey.data(),
                checkedSize(privateSize, "private key")
            ) != static_cast<int>(privateSize)) {
            fail("ECDSA private key encoding failed");
        }
        result.publicKey = ecPrivateKey(
            curve,
            std::string_view(
                reinterpret_cast<const char*>(result.privateKey.data()),
                result.privateKey.size()
            )
        ).publicKey;
        return result;
    }

    Bytes ecdsaPublic(std::string_view curve, std::string_view privateKey) {
        return ecPrivateKey(curve, privateKey).publicKey;
    }

    Bytes ecdsaSign(
        std::string_view curve,
        std::string_view privateKey,
        std::string_view message,
        std::string_view hash
    ) {
        auto key = ecPrivateKey(curve, privateKey);
        return signDigest(key.key.get(), getShaDigest(hash), message);
    }

    bool ecdsaVerify(
        std::string_view curve,
        std::string_view publicKey,
        std::string_view message,
        std::string_view signature,
        std::string_view hash
    ) {
        auto key = makeEcdsaPublicKey(curve, publicKey);
        return verifyDigestSignature(
            key.get(), getShaDigest(hash), message, signature, 0, 0
        );
    }

    bool rsaPkcs1Verify(
        std::string_view hash,
        std::string_view modulus,
        std::string_view exponent,
        std::string_view message,
        std::string_view signature
    ) {
        auto key = makeRsaPublicKey(modulus, exponent);
        return verifyDigestSignature(
            key.get(),
            getShaDigest(hash),
            message,
            signature,
            RSA_PKCS1_PADDING,
            0
        );
    }

    bool rsaPssVerify(
        std::string_view hash,
        std::string_view modulus,
        std::string_view exponent,
        std::string_view message,
        std::string_view signature,
        int saltLength
    ) {
        if (saltLength < -1) {
            throw Error(
                ErrorCode::InvalidArgument,
                "RSA-PSS salt length must be -1 or non-negative"
            );
        }
        auto key = makeRsaPublicKey(modulus, exponent);
        return verifyDigestSignature(
            key.get(),
            getShaDigest(hash),
            message,
            signature,
            RSA_PKCS1_PSS_PADDING,
            saltLength == -1 ? RSA_PSS_SALTLEN_DIGEST : saltLength
        );
    }

    KeyPair x25519KeyPair() {
        return rawKeyPair("X25519");
    }

    Bytes x25519Public(std::string_view privateKey) {
        return rawPublicFromPrivate(EVP_PKEY_X25519, privateKey, "X25519");
    }

    Bytes x25519(std::string_view privateKey, std::string_view peerPublicKey) {
        auto key = rawPrivateKey(EVP_PKEY_X25519, privateKey, "X25519");
        auto peer = rawPublicKey(EVP_PKEY_X25519, peerPublicKey, "X25519");
        EvpPkeyCtxHandle ctx(
            EVP_PKEY_CTX_new(key.get(), nullptr), EVP_PKEY_CTX_free
        );
        if (!ctx || EVP_PKEY_derive_init(ctx.get()) != 1 ||
            EVP_PKEY_derive_set_peer(ctx.get(), peer.get()) != 1) {
            fail("X25519 key exchange initialization failed");
        }
        Bytes output(32);
        std::size_t size = output.size();
        if (EVP_PKEY_derive(ctx.get(), output.data(), &size) != 1) {
            fail("X25519 key exchange failed");
        }
        output.resize(size);
        const Bytes zero(output.size(), 0);
        if (constantTimeEqual(
                std::string_view(
                    reinterpret_cast<const char*>(output.data()), output.size()
                ),
                std::string_view(
                    reinterpret_cast<const char*>(zero.data()), zero.size()
                )
            )) {
            throw Error(
                ErrorCode::InvalidKey,
                "X25519 produced an all-zero shared secret"
            );
        }
        return output;
    }

    Bytes p256Public(std::string_view privateKey) {
        return ecPrivateKey("P-256", privateKey).publicKey;
    }

    Bytes p256Shared(
        std::string_view privateKey, std::string_view peerPublicKey
    ) {
        if (peerPublicKey.size() != 65 ||
            static_cast<unsigned char>(peerPublicKey[0]) != 0x04) {
            throw Error(
                ErrorCode::InvalidKey,
                "P-256 public key must be a 65-byte uncompressed SEC1 point"
            );
        }
        auto key = ecPrivateKey("P-256", privateKey);
        auto peer = makeEcdsaPublicKey("P-256", peerPublicKey);
        EvpPkeyCtxHandle ctx(
            EVP_PKEY_CTX_new(key.key.get(), nullptr), EVP_PKEY_CTX_free
        );
        if (!ctx || EVP_PKEY_derive_init(ctx.get()) <= 0 ||
            EVP_PKEY_derive_set_peer(ctx.get(), peer.get()) <= 0) {
            fail("P-256 key exchange initialization failed");
        }
        std::size_t size = 0;
        if (EVP_PKEY_derive(ctx.get(), nullptr, &size) <= 0) {
            fail("P-256 shared secret size query failed");
        }
        Bytes output(size);
        if (EVP_PKEY_derive(ctx.get(), output.data(), &size) <= 0) {
            fail("P-256 key exchange failed");
        }
        output.resize(size);
        return output;
    }

    Bytes aesGcmEncrypt(
        std::string_view key,
        std::string_view nonce,
        std::string_view aad,
        std::string_view plaintext
    ) {
        return aeadEncrypt(
            aesGcmCipher(key.size()), key, nonce, aad, plaintext
        );
    }

    Bytes aesGcmDecrypt(
        std::string_view key,
        std::string_view nonce,
        std::string_view aad,
        std::string_view ciphertextWithTag
    ) {
        return aeadDecrypt(
            aesGcmCipher(key.size()), key, nonce, aad, ciphertextWithTag
        );
    }

    Bytes chacha20Poly1305Encrypt(
        std::string_view key,
        std::string_view nonce,
        std::string_view aad,
        std::string_view plaintext
    ) {
        if (key.size() != 32) {
            throw Error(
                ErrorCode::InvalidKey, "ChaCha20-Poly1305 key must be 32 bytes"
            );
        }
        if (nonce.size() != 12) {
            throw Error(
                ErrorCode::InvalidArgument,
                "ChaCha20-Poly1305 nonce must be 12 bytes"
            );
        }
        return aeadEncrypt(EVP_chacha20_poly1305(), key, nonce, aad, plaintext);
    }

    Bytes chacha20Poly1305Decrypt(
        std::string_view key,
        std::string_view nonce,
        std::string_view aad,
        std::string_view ciphertextWithTag
    ) {
        if (key.size() != 32) {
            throw Error(
                ErrorCode::InvalidKey, "ChaCha20-Poly1305 key must be 32 bytes"
            );
        }
        if (nonce.size() != 12) {
            throw Error(
                ErrorCode::InvalidArgument,
                "ChaCha20-Poly1305 nonce must be 12 bytes"
            );
        }
        return aeadDecrypt(
            EVP_chacha20_poly1305(), key, nonce, aad, ciphertextWithTag
        );
    }

    Bytes randomBytes(std::size_t length) {
        checkedSize(length, "random byte count");
        Bytes output(length);
        if (length != 0 &&
            RAND_bytes(output.data(), static_cast<int>(length)) != 1) {
            fail("secure random generation failed");
        }
        return output;
    }

    bool constantTimeEqual(std::string_view left, std::string_view right) {
        return left.size() == right.size() &&
               (left.empty() ||
                CRYPTO_memcmp(left.data(), right.data(), left.size()) == 0);
    }

    Bytes hkdfExtract(
        std::string_view hash, std::string_view salt, std::string_view ikm
    ) {
        const EVP_MD* md = getShaDigest(hash);
        EvpPkeyCtxHandle ctx(
            EVP_PKEY_CTX_new_id(EVP_PKEY_HKDF, nullptr), EVP_PKEY_CTX_free
        );
        Bytes output(static_cast<std::size_t>(EVP_MD_size(md)));
        std::size_t size = output.size();
        if (!ctx || EVP_PKEY_derive_init(ctx.get()) <= 0 ||
            EVP_PKEY_CTX_set_hkdf_mode(
                ctx.get(), EVP_PKEY_HKDEF_MODE_EXTRACT_ONLY
            ) <= 0 ||
            EVP_PKEY_CTX_set_hkdf_md(ctx.get(), md) <= 0 ||
            EVP_PKEY_CTX_set1_hkdf_salt(
                ctx.get(), bytes(salt), checkedSize(salt.size(), "HKDF salt")
            ) <= 0 ||
            EVP_PKEY_CTX_set1_hkdf_key(
                ctx.get(),
                bytes(ikm),
                checkedSize(ikm.size(), "HKDF input key material")
            ) <= 0 ||
            EVP_PKEY_derive(ctx.get(), output.data(), &size) <= 0) {
            fail("HKDF extract failed");
        }
        output.resize(size);
        return output;
    }

    Bytes hkdfExpand(
        std::string_view hash,
        std::string_view prk,
        std::string_view info,
        std::size_t length
    ) {
        const EVP_MD* md = getShaDigest(hash);
        const std::size_t maxLength =
            static_cast<std::size_t>(EVP_MD_size(md)) * 255;
        if (length > maxLength) {
            throw Error(
                ErrorCode::OutputTooLarge,
                "HKDF output exceeds 255 digest blocks"
            );
        }
        EvpPkeyCtxHandle ctx(
            EVP_PKEY_CTX_new_id(EVP_PKEY_HKDF, nullptr), EVP_PKEY_CTX_free
        );
        Bytes output(length);
        std::size_t size = output.size();
        if (!ctx || EVP_PKEY_derive_init(ctx.get()) <= 0 ||
            EVP_PKEY_CTX_set_hkdf_mode(
                ctx.get(), EVP_PKEY_HKDEF_MODE_EXPAND_ONLY
            ) <= 0 ||
            EVP_PKEY_CTX_set_hkdf_md(ctx.get(), md) <= 0 ||
            EVP_PKEY_CTX_set1_hkdf_key(
                ctx.get(),
                bytes(prk),
                checkedSize(prk.size(), "HKDF pseudorandom key")
            ) <= 0 ||
            (!info.empty() &&
             EVP_PKEY_CTX_add1_hkdf_info(
                 ctx.get(), bytes(info), checkedSize(info.size(), "HKDF info")
             ) <= 0) ||
            (length != 0 &&
             EVP_PKEY_derive(ctx.get(), output.data(), &size) <= 0)) {
            fail("HKDF expand failed");
        }
        output.resize(size);
        return output;
    }

    Bytes pbkdf2(
        std::string_view hash,
        std::string_view password,
        std::string_view salt,
        std::uint32_t iterations,
        std::size_t length
    ) {
        if (iterations == 0 ||
            iterations > static_cast<std::uint32_t>(INT_MAX)) {
            throw Error(
                ErrorCode::InvalidArgument,
                "PBKDF2 iterations must be between 1 and INT_MAX"
            );
        }
        const EVP_MD* md = getShaDigest(hash);
        const int outputSize = checkedSize(length, "PBKDF2 output");
        Bytes output(length);
        if (length != 0 && PKCS5_PBKDF2_HMAC(
                               password.data(),
                               checkedSize(password.size(), "password"),
                               bytes(salt),
                               checkedSize(salt.size(), "salt"),
                               static_cast<int>(iterations),
                               md,
                               outputSize,
                               output.data()
                           ) != 1) {
            fail("PBKDF2 failed");
        }
        return output;
    }

    Bytes scrypt(
        std::string_view password,
        std::string_view salt,
        std::uint64_t n,
        std::uint64_t r,
        std::uint64_t p,
        std::size_t length,
        std::uint64_t maxMemory
    ) {
        if (n <= 1 || (n & (n - 1)) != 0 || r == 0 || p == 0) {
            throw Error(
                ErrorCode::InvalidArgument,
                "scrypt requires power-of-two N > 1 and positive r and p"
            );
        }
        checkedSize(length, "scrypt output");
        Bytes output(length);
        if (length != 0 && EVP_PBE_scrypt(
                               password.data(),
                               password.size(),
                               bytes(salt),
                               salt.size(),
                               n,
                               r,
                               p,
                               maxMemory,
                               output.data(),
                               output.size()
                           ) != 1) {
            fail("scrypt failed");
        }
        return output;
    }

    const char* backendVersion() {
        return OpenSSL_version(OPENSSL_VERSION);
    }

    Features features() {
        auto hasDigest = [](const char* name) {
            EVP_MD* algorithm = EVP_MD_fetch(nullptr, name, nullptr);
            const bool available = algorithm != nullptr;
            EVP_MD_free(algorithm);
            ERR_clear_error();
            return available;
        };
        auto hasCipher = [](const char* name) {
            EVP_CIPHER* algorithm = EVP_CIPHER_fetch(nullptr, name, nullptr);
            const bool available = algorithm != nullptr;
            EVP_CIPHER_free(algorithm);
            ERR_clear_error();
            return available;
        };
        auto hasKeyAlgorithm = [](const char* name) {
            EvpPkeyCtxHandle ctx(
                EVP_PKEY_CTX_new_from_name(nullptr, name, nullptr),
                EVP_PKEY_CTX_free
            );
            const bool available = ctx != nullptr;
            ERR_clear_error();
            return available;
        };
        auto hasKdf = [](const char* name) {
            EVP_KDF* algorithm = EVP_KDF_fetch(nullptr, name, nullptr);
            const bool available = algorithm != nullptr;
            EVP_KDF_free(algorithm);
            ERR_clear_error();
            return available;
        };
        EVP_MAC* hmacAlgorithm = EVP_MAC_fetch(nullptr, "HMAC", nullptr);
        const bool hasHmac = hmacAlgorithm != nullptr;
        EVP_MAC_free(hmacAlgorithm);
        ERR_clear_error();
        const bool hasEc = hasKeyAlgorithm("EC");
        return {
            hasDigest("SHA256"),
            hasDigest("SHA384"),
            hasDigest("SHA512"),
            hasDigest("MD5"),
            hasHmac,
            hasKeyAlgorithm("ED25519"),
            hasEc,
            hasKeyAlgorithm("RSA"),
            hasKeyAlgorithm("X25519"),
            hasEc,
            hasCipher("AES-128-GCM"),
            hasCipher("AES-256-GCM"),
            hasCipher("CHACHA20-POLY1305"),
            RAND_status() == 1,
            hasKdf("HKDF"),
            hasKdf("PBKDF2"),
            hasKdf("SCRYPT")
        };
    }
}
