#include "crypto/Crypto.hpp"

#include <gtest/gtest.h>
#include <openssl/bn.h>
#include <openssl/core_names.h>
#include <openssl/evp.h>
#include <openssl/rsa.h>

#include <memory>
#include <string>

namespace {
    using PkeyHandle = std::unique_ptr<EVP_PKEY, decltype(&EVP_PKEY_free)>;
    using MdCtxHandle = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;

    crypto::Bytes hex(const std::string& value) {
        crypto::Bytes result;
        result.reserve(value.size() / 2);
        for (std::size_t i = 0; i < value.size(); i += 2) {
            result.push_back(
                static_cast<std::uint8_t>(
                    std::stoul(value.substr(i, 2), nullptr, 16)
                )
            );
        }
        return result;
    }

    std::string binary(const crypto::Bytes& value) {
        return {reinterpret_cast<const char*>(value.data()), value.size()};
    }

    crypto::Bytes sign(
        EVP_PKEY* key,
        const EVP_MD* digest,
        std::string_view message,
        int rsaPadding = 0,
        int saltLength = 0
    ) {
        MdCtxHandle ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
        EVP_PKEY_CTX* pkeyCtx = nullptr;
        EXPECT_NE(ctx, nullptr);
        EXPECT_EQ(
            EVP_DigestSignInit(ctx.get(), &pkeyCtx, digest, nullptr, key), 1
        );
        if (rsaPadding != 0) {
            EXPECT_GT(EVP_PKEY_CTX_set_rsa_padding(pkeyCtx, rsaPadding), 0);
            if (rsaPadding == RSA_PKCS1_PSS_PADDING) {
                EXPECT_GT(
                    EVP_PKEY_CTX_set_rsa_pss_saltlen(pkeyCtx, saltLength), 0
                );
            }
        }
        EXPECT_EQ(
            EVP_DigestSignUpdate(ctx.get(), message.data(), message.size()), 1
        );
        std::size_t size = 0;
        EXPECT_EQ(EVP_DigestSignFinal(ctx.get(), nullptr, &size), 1);
        crypto::Bytes signature(size);
        EXPECT_EQ(EVP_DigestSignFinal(ctx.get(), signature.data(), &size), 1);
        signature.resize(size);
        return signature;
    }

    crypto::Bytes keyInteger(EVP_PKEY* key, const char* name) {
        BIGNUM* raw = nullptr;
        EXPECT_EQ(EVP_PKEY_get_bn_param(key, name, &raw), 1);
        std::unique_ptr<BIGNUM, decltype(&BN_free)> value(raw, BN_free);
        crypto::Bytes result(
            static_cast<std::size_t>(BN_num_bytes(value.get()))
        );
        BN_bn2bin(value.get(), result.data());
        return result;
    }
}

TEST(Crypto, HashesAndHmacMatchKnownVectors) {
    EXPECT_EQ(
        crypto::digest("SHA256", "abc"),
        hex("ba7816bf8f01cfea414140de5dae2223"
            "b00361a396177a9cb410ff61f20015ad")
    );
    EXPECT_EQ(
        crypto::digest("SHA384", "abc"),
        hex("cb00753f45a35e8bb5a03d699ac65007"
            "272c32ab0eded1631a8b605a43ff5bed"
            "8086072ba1e7cc2358baeca134c825a7")
    );
    EXPECT_EQ(
        crypto::digest("SHA512", "abc"),
        hex("ddaf35a193617abacc417349ae204131"
            "12e6fa4e89a97ea20a9eeee64b55d39"
            "a2192992a274fc1a836ba3c23a3feebbd"
            "454d4423643ce80e2a9ac94fa54ca49f")
    );
    EXPECT_EQ(
        crypto::digest("MD5", "abc"), hex("900150983cd24fb0d6963f7d28e17f72")
    );
    const crypto::Bytes key(20, 0x0b);
    EXPECT_EQ(
        crypto::hmac("SHA256", binary(key), "Hi There"),
        hex("b0344c61d8db38535ca8afceaf0bf12b"
            "881dc200c9833da726e9376c2e32cff7")
    );
}

TEST(Crypto, StreamingHashCanBeResetAndRejectsUpdateAfterFinal) {
    crypto::HashContext context("SHA256");
    context.update("a");
    context.update(std::string("b\0c", 3));
    EXPECT_EQ(
        context.final(), crypto::digest("SHA256", std::string("ab\0c", 4))
    );
    try {
        context.update("more");
        FAIL() << "update after final must fail";
    } catch (const crypto::Error& error) {
        EXPECT_EQ(error.code(), crypto::ErrorCode::InvalidState);
    }
    context.reset();
    context.update("abc");
    EXPECT_EQ(context.final(), crypto::digest("SHA256", "abc"));
}

TEST(Crypto, Ed25519MatchesRfc8032Vector) {
    auto publicKey =
        hex("d75a980182b10ab7d54bfed3c964073a"
            "0ee172f3daa62325af021a68f707511a");
    auto signature =
        hex("e5564300c360ac729086e2cc806e828a"
            "84877f1eb8e5d974d873e06522490155"
            "5fb8821590a33bacc61e39701cf9b46b"
            "d25bf5f0595bbe24655141438e7a100b");
    EXPECT_TRUE(
        crypto::ed25519Verify(binary(publicKey), "", binary(signature))
    );
    signature[0] ^= 1;
    EXPECT_FALSE(
        crypto::ed25519Verify(binary(publicKey), "", binary(signature))
    );
}

TEST(Crypto, Ed25519GeneratesKeysAndSigns) {
    const auto keyPair = crypto::ed25519KeyPair();
    ASSERT_EQ(keyPair.privateKey.size(), std::size_t {32});
    ASSERT_EQ(keyPair.publicKey.size(), std::size_t {32});
    EXPECT_EQ(
        crypto::ed25519Public(binary(keyPair.privateKey)), keyPair.publicKey
    );
    const auto signature =
        crypto::ed25519Sign(binary(keyPair.privateKey), std::string("a\0b", 3));
    EXPECT_TRUE(
        crypto::ed25519Verify(
            binary(keyPair.publicKey), std::string("a\0b", 3), binary(signature)
        )
    );
}

TEST(Crypto, EcdsaVerifiesAllSupportedCurves) {
    struct Case {
        const char* curve;
        const char* opensslCurve;
        const EVP_MD* digest;
        const char* hash;
        std::size_t coordinateSize;
    };
    const Case cases[] {
        {"P-256", "prime256v1", EVP_sha256(), "SHA256", 32},
        {"P-384", "secp384r1", EVP_sha384(), "SHA384", 48},
        {"P-521", "secp521r1", EVP_sha512(), "SHA512", 66}
    };
    for (const auto& item : cases) {
        SCOPED_TRACE(item.curve);
        PkeyHandle key(
            EVP_PKEY_Q_keygen(nullptr, nullptr, "EC", item.opensslCurve),
            EVP_PKEY_free
        );
        ASSERT_NE(key, nullptr);
        BIGNUM* rawX = nullptr;
        BIGNUM* rawY = nullptr;
        ASSERT_EQ(
            EVP_PKEY_get_bn_param(key.get(), OSSL_PKEY_PARAM_EC_PUB_X, &rawX), 1
        );
        ASSERT_EQ(
            EVP_PKEY_get_bn_param(key.get(), OSSL_PKEY_PARAM_EC_PUB_Y, &rawY), 1
        );
        std::unique_ptr<BIGNUM, decltype(&BN_free)> x(rawX, BN_free);
        std::unique_ptr<BIGNUM, decltype(&BN_free)> y(rawY, BN_free);
        crypto::Bytes publicKey(item.coordinateSize * 2 + 1);
        publicKey[0] = 0x04;
        ASSERT_EQ(
            BN_bn2binpad(
                x.get(), publicKey.data() + 1, item.coordinateSize
            ),
            static_cast<int>(item.coordinateSize)
        );
        ASSERT_EQ(
            BN_bn2binpad(
                y.get(),
                publicKey.data() + 1 + item.coordinateSize,
                item.coordinateSize
            ),
            static_cast<int>(item.coordinateSize)
        );
        auto signature = sign(key.get(), item.digest, "message");
        EXPECT_TRUE(
            crypto::ecdsaVerify(
                item.curve,
                binary(publicKey),
                "message",
                binary(signature),
                item.hash
            )
        );
        EXPECT_FALSE(
            crypto::ecdsaVerify(
                item.curve,
                binary(publicKey),
                "tampered",
                binary(signature),
                item.hash
            )
        );
    }
}

TEST(Crypto, EcdsaGeneratesKeysAndSignsOnAllSupportedCurves) {
    struct Case {
        const char* curve;
        const char* hash;
        std::size_t privateSize;
        std::size_t publicSize;
    };
    const Case cases[] {
        {"P-256", "SHA256", 32, 65},
        {"P-384", "SHA384", 48, 97},
        {"P-521", "SHA512", 66, 133}
    };
    for (const auto& item : cases) {
        SCOPED_TRACE(item.curve);
        const auto keyPair = crypto::ecdsaKeyPair(item.curve);
        EXPECT_EQ(keyPair.privateKey.size(), item.privateSize);
        EXPECT_EQ(keyPair.publicKey.size(), item.publicSize);
        EXPECT_EQ(
            crypto::ecdsaPublic(item.curve, binary(keyPair.privateKey)),
            keyPair.publicKey
        );
        const auto signature = crypto::ecdsaSign(
            item.curve, binary(keyPair.privateKey), "message", item.hash
        );
        EXPECT_TRUE(
            crypto::ecdsaVerify(
                item.curve,
                binary(keyPair.publicKey),
                "message",
                binary(signature),
                item.hash
            )
        );
    }
}

TEST(Crypto, RsaVerifiesPkcs1AndPssSignatures) {
    PkeyHandle key(
        EVP_PKEY_Q_keygen(nullptr, nullptr, "RSA", 2048), EVP_PKEY_free
    );
    ASSERT_NE(key, nullptr);
    const auto modulus = keyInteger(key.get(), OSSL_PKEY_PARAM_RSA_N);
    const auto exponent = keyInteger(key.get(), OSSL_PKEY_PARAM_RSA_E);

    auto pkcs1 = sign(key.get(), EVP_sha256(), "message", RSA_PKCS1_PADDING);
    EXPECT_TRUE(
        crypto::rsaPkcs1Verify(
            "SHA256",
            binary(modulus),
            binary(exponent),
            "message",
            binary(pkcs1)
        )
    );
    EXPECT_FALSE(
        crypto::rsaPkcs1Verify(
            "SHA256",
            binary(modulus),
            binary(exponent),
            "tampered",
            binary(pkcs1)
        )
    );

    auto pss = sign(
        key.get(),
        EVP_sha256(),
        "message",
        RSA_PKCS1_PSS_PADDING,
        RSA_PSS_SALTLEN_DIGEST
    );
    EXPECT_TRUE(
        crypto::rsaPssVerify(
            "SHA256",
            binary(modulus),
            binary(exponent),
            "message",
            binary(pss),
            -1
        )
    );
    EXPECT_FALSE(
        crypto::rsaPssVerify(
            "SHA256",
            binary(modulus),
            binary(exponent),
            "tampered",
            binary(pss),
            -1
        )
    );
}

TEST(Crypto, X25519MatchesRfc7748PublicKeyVector) {
    const auto privateKey =
        hex("77076d0a7318a57d3c16c17251b26645"
            "df4c2f87ebc0992ab177fba51db92c2a");
    EXPECT_EQ(
        crypto::x25519Public(binary(privateKey)),
        hex("8520f0098930a754748b7ddcb43ef75a0"
            "dbf3a0d26381af4eba4a98eaa9b4e6a")
    );
}

TEST(Crypto, X25519GeneratedKeyPairsDeriveTheSameSecret) {
    const auto alice = crypto::x25519KeyPair();
    const auto bob = crypto::x25519KeyPair();
    EXPECT_EQ(alice.privateKey.size(), std::size_t {32});
    EXPECT_EQ(alice.publicKey.size(), std::size_t {32});
    EXPECT_EQ(crypto::x25519Public(binary(alice.privateKey)), alice.publicKey);
    EXPECT_EQ(
        crypto::x25519(binary(alice.privateKey), binary(bob.publicKey)),
        crypto::x25519(binary(bob.privateKey), binary(alice.publicKey))
    );
}

TEST(Crypto, P256DerivesPublicKeyAndSharedSecret) {
    crypto::Bytes privateA(32, 0);
    crypto::Bytes privateB(32, 0);
    privateA.back() = 1;
    privateB.back() = 2;
    const auto publicA = crypto::p256Public(binary(privateA));
    const auto publicB = crypto::p256Public(binary(privateB));
    EXPECT_EQ(
        publicA,
        hex("046b17d1f2e12c4247f8bce6e563a440"
            "f277037d812deb33a0f4a13945d898c296"
            "4fe342e2fe1a7f9b8ee7eb4a7c0f9e16"
            "2bce33576b315ececbb6406837bf51f5")
    );
    EXPECT_EQ(
        crypto::p256Shared(binary(privateA), binary(publicB)),
        crypto::p256Shared(binary(privateB), binary(publicA))
    );
}

TEST(Crypto, AesGcmMatchesKnownVectorAndRejectsTampering) {
    const crypto::Bytes key(16, 0);
    const crypto::Bytes nonce(12, 0);
    const crypto::Bytes plaintext(16, 0);
    auto ciphertext = crypto::aesGcmEncrypt(
        binary(key), binary(nonce), "", binary(plaintext)
    );
    EXPECT_EQ(
        ciphertext,
        hex("0388dace60b6a392f328c2b971b2fe78"
            "ab6e47d42cec13bdf53a67b21257bddf")
    );
    EXPECT_EQ(
        crypto::aesGcmDecrypt(
            binary(key), binary(nonce), "", binary(ciphertext)
        ),
        plaintext
    );
    ciphertext.back() ^= 1;
    EXPECT_THROW(
        crypto::aesGcmDecrypt(
            binary(key), binary(nonce), "", binary(ciphertext)
        ),
        std::runtime_error
    );
}

TEST(Crypto, Chacha20Poly1305RoundTripsAndRejectsTampering) {
    const crypto::Bytes key(32, 0);
    const crypto::Bytes nonce(12, 0);
    auto ciphertext = crypto::chacha20Poly1305Encrypt(
        binary(key), binary(nonce), "aad", "plaintext"
    );
    EXPECT_EQ(
        binary(
            crypto::chacha20Poly1305Decrypt(
                binary(key), binary(nonce), "aad", binary(ciphertext)
            )
        ),
        "plaintext"
    );
    ciphertext.back() ^= 1;
    EXPECT_THROW(
        crypto::chacha20Poly1305Decrypt(
            binary(key), binary(nonce), "aad", binary(ciphertext)
        ),
        std::runtime_error
    );
}

TEST(Crypto, HkdfMatchesRfc5869Vector) {
    const crypto::Bytes ikm(22, 0x0b);
    const auto salt = hex("000102030405060708090a0b0c");
    const auto info = hex("f0f1f2f3f4f5f6f7f8f9");
    const auto prk = crypto::hkdfExtract("SHA256", binary(salt), binary(ikm));
    EXPECT_EQ(
        prk,
        hex("077709362c2e32df0ddc3f0dc47bba63"
            "90b6c73bb50f9c3122ec844ad7c2b3e5")
    );
    EXPECT_EQ(
        crypto::hkdfExpand("SHA256", binary(prk), binary(info), 42),
        hex("3cb25f25faacd57a90434f64d0362f2a"
            "2d2d0a90cf1a5a4c5db02d56ecc4c5bf"
            "34007208d5b887185865")
    );
}

TEST(Crypto, PasswordKdfsMatchKnownVectors) {
    EXPECT_EQ(
        crypto::pbkdf2("SHA256", "password", "salt", 1, 32),
        hex("120fb6cffcf8b32c43e7225256c4f837"
            "a86548c92ccc35480805987cb70be17b")
    );
    EXPECT_EQ(
        crypto::scrypt("", "", 16, 1, 1, 64),
        hex("77d6576238657b203b19ca42c18a0497"
            "f16b4844e3074ae8dfdffa3fede21442f"
            "cd0069ded0948f8326a753a0fc81f17e"
            "8d3e0fb2e0d3628cf35e20c38d18906")
    );
}

TEST(Crypto, ReportsRuntimeCapabilitiesAndStableErrors) {
    const auto available = crypto::features();
    EXPECT_TRUE(available.sha256);
    EXPECT_TRUE(available.ed25519);
    EXPECT_TRUE(available.ecdsa);
    EXPECT_TRUE(available.x25519);
    EXPECT_TRUE(available.aes128Gcm);
    EXPECT_TRUE(available.pbkdf2);
    EXPECT_TRUE(available.scrypt);
    EXPECT_EQ(crypto::API_VERSION, 1);

    const crypto::Bytes key(16, 0);
    const crypto::Bytes nonce(12, 0);
    auto ciphertext =
        crypto::aesGcmEncrypt(binary(key), binary(nonce), "", "secret");
    ciphertext.back() ^= 1;
    try {
        crypto::aesGcmDecrypt(
            binary(key), binary(nonce), "", binary(ciphertext)
        );
        FAIL() << "tampered ciphertext must fail";
    } catch (const crypto::Error& error) {
        EXPECT_EQ(error.code(), crypto::ErrorCode::AuthenticationFailed);
        EXPECT_STREQ(error.codeName(), "authentication_failed");
    }
}

TEST(Crypto, UtilitiesHandleBinaryData) {
    const std::string binaryValue("a\0b", 3);
    EXPECT_TRUE(crypto::constantTimeEqual(binaryValue, binaryValue));
    EXPECT_FALSE(crypto::constantTimeEqual(binaryValue, "a"));
    EXPECT_FALSE(crypto::constantTimeEqual("left", "lest"));
    EXPECT_EQ(crypto::randomBytes(32).size(), std::size_t {32});
}
