#include <stdexcept>
#include <mbedtls/aes.h>
#include <mbedtls/gcm.h>

#include "../Cryptography.hpp"

struct crypto::AES::Impl {
    mbedtls_aes_context cbc_ctx{};  // для CBC
    mbedtls_gcm_context gcm_ctx{};  // для GCM

    Impl() {
        mbedtls_aes_init(&cbc_ctx);
        mbedtls_gcm_init(&gcm_ctx);
    }
    ~Impl() {
        mbedtls_aes_free(&cbc_ctx);
        mbedtls_gcm_free(&gcm_ctx);
    }
};

static std::vector<uint8_t> pkcs7_pad(const std::vector<uint8_t>& data) {
    size_t blockSize = 16;
    size_t padLen = blockSize - data.size() % blockSize;
    std::vector<uint8_t> padded = data;
    padded.insert(padded.end(), padLen, static_cast<uint8_t>(padLen));
    return padded;
}

void crypto::AES::checkIV() const {
    if(this->mode.value() == CBC || this->mode.value() == GCM) {
        if (!this->iv.has_value())
            throw std::runtime_error("IV must be defined when using CBC mode");

        if (this->mode.value() == CBC) {
            if (this->iv.value().size() != 16)
                throw std::runtime_error("IV size must be 16 bytes when using the CBC mode");
        } else {
            if (this->iv.value().size() != 12)
                throw std::runtime_error("IV size must be 12 bytes when using the GCM mode");
        }
    }
}

static std::vector<uint8_t> pkcs7_unpad(const std::vector<uint8_t>& data) {
    if (data.empty())
        throw std::runtime_error("data is empty for unpadding");

    uint8_t padLen = data.back();
    if (padLen == 0 || padLen > 16) throw std::runtime_error("invalid padding");
    return { data.begin(), data.end() - padLen };
}

static void check_init(bool inited) {
    if (!inited)
        throw std::runtime_error("cipher is not initialized");
}

const std::vector<uint8_t>& crypto::AES::getKey() const {
    check_init(this->inited);
    return this->key;
}

const crypto::AESPadding& crypto::AES::getPadding() const {
    check_init(this->inited);
    return this->padding.value();
}

const crypto::AESMode& crypto::AES::getMode() const {
    check_init(this->inited);
    return *mode;
}

void crypto::AES::update(
            int keySize,
            const std::vector<uint8_t>& key,
            const AESPadding& padding,
            const AESMode& mode
            ) {
    if (keySize != 128 && keySize != 192 && keySize != 256)
        throw std::runtime_error("unsupported AES key size");

    if (keySize / 8 != key.size())
        throw std::runtime_error("invalid count of bytes in key");

    this->keySize = keySize;
    this->key = key;
    this->mode = mode;
    this->padding = padding;

    if (this->mode.value() == GCM) {
        if (mbedtls_gcm_setkey(&pImpl->gcm_ctx, MBEDTLS_CIPHER_ID_AES, key.data(), keySize) != 0)
            throw std::runtime_error("AES-GCM setkey failed");
    }
    else if (this->mode.value() == CBC) {
        if (mbedtls_aes_setkey_enc(&pImpl->cbc_ctx, key.data(), keySize) != 0)
            throw std::runtime_error("AES-CBC setkey_enc failed");
    } else throw std::runtime_error("unsupported AES mode");

    this->inited = true;
}

std::vector<uint8_t> crypto::AES::encrypt(const std::vector<uint8_t>& plaintext) {
    check_init(this->inited);

    this->checkIV();

    std::vector<uint8_t> ciphertext;
    std::vector<uint8_t> iv_copy;

    int res;

    switch (this->mode.value()) {
        case CBC:
            iv_copy = this->iv.value();

            std::vector<uint8_t> padded;

            switch (this->padding.value()) {
                case PKCS7:
                    padded = pkcs7_pad(plaintext);
                    break;

                default:
                    throw std::runtime_error("unsupported AES padding");
            }

            ciphertext = std::vector<uint8_t>(padded.size());

            res = mbedtls_aes_crypt_cbc(
                &pImpl->cbc_ctx,
                MBEDTLS_AES_ENCRYPT,
                padded.size(),
                iv_copy.data(),
                padded.data(),
                ciphertext.data()
            );

            break;

        case GCM:
            iv_copy = this->iv.value();

            res = 0;
            break;

        default:
            throw std::runtime_error("unsupported AES mode");
    }

    if (res != 0) throw std::runtime_error("AES encryption failed");

    return ciphertext;
}

std::vector<uint8_t> crypto::AES::decrypt(const std::vector<uint8_t>& ciphertext) {
    check_init(this->inited);

    if (ciphertext.size() % 16 != 0)
        throw std::runtime_error("ciphertext size must be multiple of 16");

    std::vector<uint8_t> plaintext(ciphertext.size());

    this->checkIV();

    std::vector<uint8_t> iv_copy;

    int res;

    switch (this->mode.value()) {
        case CBC:
            iv_copy = this->iv.value();

            res = mbedtls_aes_crypt_cbc(
                &pImpl->cbc_ctx,
                MBEDTLS_AES_DECRYPT,
                ciphertext.size(),
                iv_copy.data(),
                ciphertext.data(),
                plaintext.data()
            );

            break;

        case GCM:
            iv_copy = this->iv.value();

            res = 0;
            break;

        default:
            throw std::runtime_error("unsupported AES mode");
    }

    if (res != 0) throw std::runtime_error("AES decryption failed");

    if (this->mode.value() == CBC) {
        switch (this->padding.value()) {
            case PKCS7:
                return pkcs7_unpad(plaintext);

            default:
                throw std::runtime_error("unsupported AES padding");
        }
    }

    return plaintext;
}