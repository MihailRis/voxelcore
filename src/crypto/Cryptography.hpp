#pragma once

#include <vector>

#include "typedefs.hpp"

namespace crypto {
    class Cipher {
    public:
        virtual ~Cipher() = default;

        virtual std::vector<uint8_t> encrypt(const std::vector<uint8_t>& plaintext) = 0;
        virtual std::vector<uint8_t> decrypt(const std::vector<uint8_t>& ciphertext) = 0;
    };

    class HashAlgorithm {
    public:
        virtual ~HashAlgorithm() = default;

        virtual void init(int keySize) = 0;

        virtual std::vector<uint8_t> digest(const std::vector<uint8_t>& data) = 0;
    };

    class SignatureAlgorithm {
    public:
        virtual ~SignatureAlgorithm() = default;

        virtual void init(int keySize, const std::vector<uint8_t>& privateKey, const std::vector<uint8_t>& publicKey) = 0;

        virtual std::vector<uint8_t> sign(const std::vector<uint8_t>& message) = 0;
        virtual bool verify(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature) = 0;
    };

    class SymmetricCipher : public Cipher {
    public:
        virtual void init(int keySize, const std::vector<uint8_t>& key, const std::vector<uint8_t>& iv) = 0;
    };

    class AsymmetricCipher : public Cipher {
    public:
        virtual void init(int keySize, const std::vector<uint8_t>& privateKey, const std::vector<uint8_t>& publicKey) = 0;
    };

    class AES : public SymmetricCipher {
    public:
        void init(int keySize, const std::vector<uint8_t>& key, const std::vector<uint8_t>& iv) override;
        std::vector<uint8_t> encrypt(const std::vector<uint8_t>& plaintext) override;
        std::vector<uint8_t> decrypt(const std::vector<uint8_t>& ciphertext) override;

    private:
        struct Impl;
        std::unique_ptr<Impl> pImpl;
    };

    class RSA : public AsymmetricCipher {
    public:
        void init(int keySize, const std::vector<uint8_t>& privateKey, const std::vector<uint8_t>& publicKey) override;
        std::vector<uint8_t> encrypt(const std::vector<uint8_t>& plaintext) override;
        std::vector<uint8_t> decrypt(const std::vector<uint8_t>& ciphertext) override;

    private:
        struct Impl;
        std::unique_ptr<Impl> pImpl;
    };

    class ECDH {
    public:
        virtual ~ECDH() = default;

        void init(
            int keySize,
            const std::vector<uint8_t>& privateKey,
            const std::vector<uint8_t>& publicKey
            );

        std::vector<uint8_t> deriveSharedSecret(const std::vector<uint8_t>& peerPublicKey);
    private:
        struct Impl;
        std::unique_ptr<Impl> pImpl;
    };

    class SHA : public HashAlgorithm {
    public:
        void init(int keySize) override;

        std::vector<uint8_t> digest(const std::vector<uint8_t>& data) override;
    private:
        struct Impl;
        std::unique_ptr<Impl> pImpl;
    };

    class ECDSA : public SignatureAlgorithm {
    public:
        void init(int keySize, const std::vector<uint8_t>& privateKey, const std::vector<uint8_t>& publicKey) override;

        std::vector<uint8_t> sign(const std::vector<uint8_t>& message) override;
        bool verify(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature) override;
    private:
        struct Impl;
        std::unique_ptr<Impl> pImpl;
    };
}