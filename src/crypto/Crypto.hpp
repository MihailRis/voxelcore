#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace crypto {
    constexpr int API_VERSION = 1;
    using Bytes = std::vector<std::uint8_t>;

    enum class ErrorCode {
        InvalidArgument,
        InvalidKey,
        InvalidSignature,
        AuthenticationFailed,
        UnsupportedAlgorithm,
        OutputTooLarge,
        InvalidState,
        BackendError
    };

    class Error : public std::runtime_error {
    public:
        Error(ErrorCode code, const std::string& message);
        ErrorCode code() const noexcept;
        const char* codeName() const noexcept;
    private:
        ErrorCode errorCode;
    };

    struct KeyPair {
        Bytes privateKey;
        Bytes publicKey;
    };

    struct Features {
        bool sha256;
        bool sha384;
        bool sha512;
        bool md5;
        bool hmac;
        bool ed25519;
        bool ecdsa;
        bool rsa;
        bool x25519;
        bool p256;
        bool aes128Gcm;
        bool aes256Gcm;
        bool chacha20Poly1305;
        bool random;
        bool hkdf;
        bool pbkdf2;
        bool scrypt;
    };

    class HashContext {
    public:
        explicit HashContext(std::string_view hash);
        ~HashContext();
        HashContext(HashContext&&) noexcept;
        HashContext& operator=(HashContext&&) noexcept;
        HashContext(const HashContext&) = delete;
        HashContext& operator=(const HashContext&) = delete;

        void update(std::string_view data);
        Bytes final();
        void reset();
    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };

    Bytes digest(std::string_view hash, std::string_view data);
    Bytes hmac(
        std::string_view hash, std::string_view key, std::string_view data
    );

    KeyPair ed25519KeyPair();
    Bytes ed25519Public(std::string_view privateKey);
    Bytes ed25519Sign(std::string_view privateKey, std::string_view message);

    bool ed25519Verify(
        std::string_view publicKey,
        std::string_view message,
        std::string_view signature
    );
    bool ecdsaVerify(
        std::string_view curve,
        std::string_view publicKey,
        std::string_view message,
        std::string_view signature,
        std::string_view hash
    );
    KeyPair ecdsaKeyPair(std::string_view curve);
    Bytes ecdsaPublic(std::string_view curve, std::string_view privateKey);
    Bytes ecdsaSign(
        std::string_view curve,
        std::string_view privateKey,
        std::string_view message,
        std::string_view hash
    );
    bool rsaPkcs1Verify(
        std::string_view hash,
        std::string_view modulus,
        std::string_view exponent,
        std::string_view message,
        std::string_view signature
    );
    bool rsaPssVerify(
        std::string_view hash,
        std::string_view modulus,
        std::string_view exponent,
        std::string_view message,
        std::string_view signature,
        int saltLength
    );

    KeyPair x25519KeyPair();
    Bytes x25519Public(std::string_view privateKey);
    Bytes x25519(std::string_view privateKey, std::string_view peerPublicKey);
    Bytes p256Public(std::string_view privateKey);
    Bytes p256Shared(
        std::string_view privateKey, std::string_view peerPublicKey
    );

    Bytes aesGcmEncrypt(
        std::string_view key,
        std::string_view nonce,
        std::string_view aad,
        std::string_view plaintext
    );
    Bytes aesGcmDecrypt(
        std::string_view key,
        std::string_view nonce,
        std::string_view aad,
        std::string_view ciphertextWithTag
    );
    Bytes chacha20Poly1305Encrypt(
        std::string_view key,
        std::string_view nonce,
        std::string_view aad,
        std::string_view plaintext
    );
    Bytes chacha20Poly1305Decrypt(
        std::string_view key,
        std::string_view nonce,
        std::string_view aad,
        std::string_view ciphertextWithTag
    );

    Bytes randomBytes(std::size_t length);
    bool constantTimeEqual(std::string_view left, std::string_view right);
    Bytes hkdfExtract(
        std::string_view hash, std::string_view salt, std::string_view ikm
    );
    Bytes hkdfExpand(
        std::string_view hash,
        std::string_view prk,
        std::string_view info,
        std::size_t length
    );
    Bytes pbkdf2(
        std::string_view hash,
        std::string_view password,
        std::string_view salt,
        std::uint32_t iterations,
        std::size_t length
    );
    Bytes scrypt(
        std::string_view password,
        std::string_view salt,
        std::uint64_t n,
        std::uint64_t r,
        std::uint64_t p,
        std::size_t length,
        std::uint64_t maxMemory = 0
    );

    const char* backendVersion();
    Features features();
}
