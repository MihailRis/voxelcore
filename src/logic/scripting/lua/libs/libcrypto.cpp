#include <climits>
#include <cstdint>
#include <new>
#include <stdexcept>
#include <string>
#include <string_view>

#include "api_lua.hpp"
#include "crypto/Crypto.hpp"

namespace {
    using HashContext = crypto::HashContext;

    constexpr const char* HASH_CONTEXT_METATABLE =
        "voxelcore.crypto.HashContext";

    int pushBytes(lua::State* L, const crypto::Bytes& value) {
        return lua::pushlstring(L, value.data(), value.size());
    }

    int pushFailure(lua::State* L, bool nil, const char* error) {
        nil ? lua::pushnil(L) : lua::pushboolean(L, false);
        lua::pushstring(L, error);
        return 2;
    }

    int pushKeyPair(lua::State* L, const crypto::KeyPair& keyPair) {
        pushBytes(L, keyPair.privateKey);
        pushBytes(L, keyPair.publicKey);
        return 2;
    }

    template <typename Operation>
    int verify(lua::State* L, Operation operation) {
        try {
            if (operation()) return lua::pushboolean(L, true);
            return pushFailure(L, false, "invalid_signature");
        } catch (const crypto::Error& error) {
            return pushFailure(L, false, error.codeName());
        } catch (const std::exception& error) {
            return pushFailure(L, false, "backend_error");
        }
    }

    template <typename Operation>
    int decrypt(lua::State* L, Operation operation) {
        try {
            return pushBytes(L, operation());
        } catch (const crypto::Error& error) {
            return pushFailure(L, true, error.codeName());
        } catch (const std::exception& error) {
            return pushFailure(L, true, "backend_error");
        }
    }

    std::size_t requireSize(lua::State* L, int index, const char* name) {
        if (lua::type(L, index) != LUA_TNUMBER) {
            throw std::runtime_error(std::string(name) + " must be a number");
        }
        const lua::Number number = lua::tonumber(L, index);
        const lua::Integer value = lua::tointeger(L, index);
        if (number != static_cast<lua::Number>(value)) {
            throw std::runtime_error(std::string(name) + " must be an integer");
        }
        if (value < 0) {
            throw std::runtime_error(
                std::string(name) + " must be non-negative"
            );
        }
        return static_cast<std::size_t>(value);
    }

    int requireInt(lua::State* L, int index, const char* name) {
        if (lua::type(L, index) != LUA_TNUMBER) {
            throw std::runtime_error(std::string(name) + " must be a number");
        }
        const lua::Number number = lua::tonumber(L, index);
        const lua::Integer value = lua::tointeger(L, index);
        if (number != static_cast<lua::Number>(value)) {
            throw std::runtime_error(std::string(name) + " must be an integer");
        }
        if (value < INT_MIN || value > INT_MAX) {
            throw std::runtime_error(std::string(name) + " is out of range");
        }
        return static_cast<int>(value);
    }

    int hash(lua::State* L, std::string_view name) {
        lua::check_argc(L, 1);
        return pushBytes(L, crypto::digest(name, lua::require_lstring(L, 1)));
    }

    int l_sha256(lua::State* L) {
        return hash(L, "SHA256");
    }
    int l_sha384(lua::State* L) {
        return hash(L, "SHA384");
    }
    int l_sha512(lua::State* L) {
        return hash(L, "SHA512");
    }
    int l_md5(lua::State* L) {
        return hash(L, "MD5");
    }

    int l_hash(lua::State* L) {
        lua::check_argc(L, 2);
        return pushBytes(
            L,
            crypto::digest(
                lua::require_lstring(L, 1), lua::require_lstring(L, 2)
            )
        );
    }

    crypto::HashContext* requireHashContext(lua::State* L) {
        return static_cast<crypto::HashContext*>(
            luaL_checkudata(L, 1, HASH_CONTEXT_METATABLE)
        );
    }

    int l_hash_context_gc(lua::State* L) {
        requireHashContext(L)->~HashContext();
        return 0;
    }

    int l_hash_context_update(lua::State* L) {
        lua::check_argc(L, 2);
        requireHashContext(L)->update(lua::require_lstring(L, 2));
        lua::pushvalue(L, 1);
        return 1;
    }

    int l_hash_context_final(lua::State* L) {
        lua::check_argc(L, 1);
        return pushBytes(L, requireHashContext(L)->final());
    }

    int l_hash_context_reset(lua::State* L) {
        lua::check_argc(L, 1);
        requireHashContext(L)->reset();
        lua::pushvalue(L, 1);
        return 1;
    }

    void ensureHashContextMetatable(lua::State* L) {
        if (luaL_newmetatable(L, HASH_CONTEXT_METATABLE)) {
            lua::pushcfunction(L, lua::wrap<l_hash_context_gc>);
            lua::setfield(L, "__gc");
            lua::pushcfunction(L, lua::wrap<l_hash_context_update>);
            lua::setfield(L, "update");
            lua::pushcfunction(L, lua::wrap<l_hash_context_final>);
            lua::setfield(L, "final");
            lua::pushcfunction(L, lua::wrap<l_hash_context_reset>);
            lua::setfield(L, "reset");
            lua::pushvalue(L, -1);
            lua::setfield(L, "__index");
        }
        lua::pop(L);
    }

    int l_hash_new(lua::State* L) {
        lua::check_argc(L, 1);
        const auto name = lua::require_lstring(L, 1);
        ensureHashContextMetatable(L);
        void* memory = lua_newuserdata(L, sizeof(crypto::HashContext));
        new (memory) crypto::HashContext(name);
        luaL_getmetatable(L, HASH_CONTEXT_METATABLE);
        lua_setmetatable(L, -2);
        return 1;
    }

    int l_hmac(lua::State* L) {
        lua::check_argc(L, 3);
        return pushBytes(
            L,
            crypto::hmac(
                lua::require_lstring(L, 1),
                lua::require_lstring(L, 2),
                lua::require_lstring(L, 3)
            )
        );
    }

    int l_ed25519_verify(lua::State* L) {
        lua::check_argc(L, 3);
        const auto key = lua::require_lstring(L, 1);
        const auto message = lua::require_lstring(L, 2);
        const auto signature = lua::require_lstring(L, 3);
        return verify(L, [=] {
            return crypto::ed25519Verify(key, message, signature);
        });
    }

    int l_ed25519_keypair(lua::State* L) {
        lua::check_argc(L, 0);
        return pushKeyPair(L, crypto::ed25519KeyPair());
    }

    int l_ed25519_public(lua::State* L) {
        lua::check_argc(L, 1);
        return pushBytes(L, crypto::ed25519Public(lua::require_lstring(L, 1)));
    }

    int l_ed25519_sign(lua::State* L) {
        lua::check_argc(L, 2);
        return pushBytes(
            L,
            crypto::ed25519Sign(
                lua::require_lstring(L, 1), lua::require_lstring(L, 2)
            )
        );
    }

    int l_ecdsa_verify(lua::State* L) {
        lua::check_argc(L, 5);
        const auto curve = lua::require_lstring(L, 1);
        const auto key = lua::require_lstring(L, 2);
        const auto message = lua::require_lstring(L, 3);
        const auto signature = lua::require_lstring(L, 4);
        const auto hashName = lua::require_lstring(L, 5);
        return verify(L, [=] {
            return crypto::ecdsaVerify(
                curve, key, message, signature, hashName
            );
        });
    }

    int l_ecdsa_keypair(lua::State* L) {
        lua::check_argc(L, 1);
        return pushKeyPair(L, crypto::ecdsaKeyPair(lua::require_lstring(L, 1)));
    }

    int l_ecdsa_public(lua::State* L) {
        lua::check_argc(L, 2);
        return pushBytes(
            L,
            crypto::ecdsaPublic(
                lua::require_lstring(L, 1), lua::require_lstring(L, 2)
            )
        );
    }

    int l_ecdsa_sign(lua::State* L) {
        lua::check_argc(L, 4);
        return pushBytes(
            L,
            crypto::ecdsaSign(
                lua::require_lstring(L, 1),
                lua::require_lstring(L, 2),
                lua::require_lstring(L, 3),
                lua::require_lstring(L, 4)
            )
        );
    }

    int l_rsa_pkcs1_verify(lua::State* L) {
        lua::check_argc(L, 5);
        const auto hashName = lua::require_lstring(L, 1);
        const auto modulus = lua::require_lstring(L, 2);
        const auto exponent = lua::require_lstring(L, 3);
        const auto message = lua::require_lstring(L, 4);
        const auto signature = lua::require_lstring(L, 5);
        return verify(L, [=] {
            return crypto::rsaPkcs1Verify(
                hashName, modulus, exponent, message, signature
            );
        });
    }

    int l_rsa_pss_verify(lua::State* L) {
        lua::check_argc(L, 6);
        const auto hashName = lua::require_lstring(L, 1);
        const auto modulus = lua::require_lstring(L, 2);
        const auto exponent = lua::require_lstring(L, 3);
        const auto message = lua::require_lstring(L, 4);
        const auto signature = lua::require_lstring(L, 5);
        const int saltLength = requireInt(L, 6, "salt_length");
        return verify(L, [=] {
            return crypto::rsaPssVerify(
                hashName, modulus, exponent, message, signature, saltLength
            );
        });
    }

    int l_x25519_public(lua::State* L) {
        lua::check_argc(L, 1);
        return pushBytes(L, crypto::x25519Public(lua::require_lstring(L, 1)));
    }

    int l_x25519_keypair(lua::State* L) {
        lua::check_argc(L, 0);
        return pushKeyPair(L, crypto::x25519KeyPair());
    }

    int l_x25519(lua::State* L) {
        lua::check_argc(L, 2);
        return pushBytes(
            L,
            crypto::x25519(
                lua::require_lstring(L, 1), lua::require_lstring(L, 2)
            )
        );
    }

    int l_p256_public(lua::State* L) {
        lua::check_argc(L, 1);
        return pushBytes(L, crypto::p256Public(lua::require_lstring(L, 1)));
    }

    int l_p256_keypair(lua::State* L) {
        lua::check_argc(L, 0);
        return pushKeyPair(L, crypto::ecdsaKeyPair("P-256"));
    }

    int l_p256_shared(lua::State* L) {
        lua::check_argc(L, 2);
        return pushBytes(
            L,
            crypto::p256Shared(
                lua::require_lstring(L, 1), lua::require_lstring(L, 2)
            )
        );
    }

    int l_aes_gcm_encrypt(lua::State* L) {
        lua::check_argc(L, 4);
        return pushBytes(
            L,
            crypto::aesGcmEncrypt(
                lua::require_lstring(L, 1),
                lua::require_lstring(L, 2),
                lua::require_lstring(L, 3),
                lua::require_lstring(L, 4)
            )
        );
    }

    int l_aes_gcm_decrypt(lua::State* L) {
        lua::check_argc(L, 4);
        const auto key = lua::require_lstring(L, 1);
        const auto nonce = lua::require_lstring(L, 2);
        const auto aad = lua::require_lstring(L, 3);
        const auto ciphertext = lua::require_lstring(L, 4);
        return decrypt(L, [=] {
            return crypto::aesGcmDecrypt(key, nonce, aad, ciphertext);
        });
    }

    int l_chacha20_poly1305_encrypt(lua::State* L) {
        lua::check_argc(L, 4);
        return pushBytes(
            L,
            crypto::chacha20Poly1305Encrypt(
                lua::require_lstring(L, 1),
                lua::require_lstring(L, 2),
                lua::require_lstring(L, 3),
                lua::require_lstring(L, 4)
            )
        );
    }

    int l_chacha20_poly1305_decrypt(lua::State* L) {
        lua::check_argc(L, 4);
        const auto key = lua::require_lstring(L, 1);
        const auto nonce = lua::require_lstring(L, 2);
        const auto aad = lua::require_lstring(L, 3);
        const auto ciphertext = lua::require_lstring(L, 4);
        return decrypt(L, [=] {
            return crypto::chacha20Poly1305Decrypt(key, nonce, aad, ciphertext);
        });
    }

    int l_random_bytes(lua::State* L) {
        lua::check_argc(L, 1);
        return pushBytes(L, crypto::randomBytes(requireSize(L, 1, "length")));
    }

    int l_constant_time_equal(lua::State* L) {
        lua::check_argc(L, 2);
        return lua::pushboolean(
            L,
            crypto::constantTimeEqual(
                lua::require_lstring(L, 1), lua::require_lstring(L, 2)
            )
        );
    }

    int l_hkdf_extract(lua::State* L) {
        lua::check_argc(L, 3);
        return pushBytes(
            L,
            crypto::hkdfExtract(
                lua::require_lstring(L, 1),
                lua::require_lstring(L, 2),
                lua::require_lstring(L, 3)
            )
        );
    }

    int l_hkdf_expand(lua::State* L) {
        lua::check_argc(L, 4);
        return pushBytes(
            L,
            crypto::hkdfExpand(
                lua::require_lstring(L, 1),
                lua::require_lstring(L, 2),
                lua::require_lstring(L, 3),
                requireSize(L, 4, "length")
            )
        );
    }

    int l_pbkdf2(lua::State* L) {
        lua::check_argc(L, 5);
        const auto iterations = requireSize(L, 4, "iterations");
        if (iterations == 0 || iterations > UINT32_MAX) {
            throw std::runtime_error(
                "iterations must be between 1 and 4294967295"
            );
        }
        return pushBytes(
            L,
            crypto::pbkdf2(
                lua::require_lstring(L, 1),
                lua::require_lstring(L, 2),
                lua::require_lstring(L, 3),
                static_cast<std::uint32_t>(iterations),
                requireSize(L, 5, "length")
            )
        );
    }

    int l_scrypt(lua::State* L) {
        const auto argc = lua::check_argc(L, 6, 7);
        const auto maxMemory =
            argc == 7 ? requireSize(L, 7, "max_memory") : std::size_t {0};
        return pushBytes(
            L,
            crypto::scrypt(
                lua::require_lstring(L, 1),
                lua::require_lstring(L, 2),
                requireSize(L, 3, "n"),
                requireSize(L, 4, "r"),
                requireSize(L, 5, "p"),
                requireSize(L, 6, "length"),
                maxMemory
            )
        );
    }

    void setFeature(lua::State* L, const char* name, bool available) {
        lua::pushboolean(L, available);
        lua::setfield(L, name);
    }

    int l_features(lua::State* L) {
        lua::check_argc(L, 0);
        const auto features = crypto::features();
        lua::createtable(L, 0, 26);
        lua::pushstring(L, "OpenSSL");
        lua::setfield(L, "backend");
        lua::pushstring(L, crypto::backendVersion());
        lua::setfield(L, "backend_version");
        lua::pushinteger(L, crypto::API_VERSION);
        lua::setfield(L, "api_version");
        setFeature(L, "sha256", features.sha256);
        setFeature(L, "sha384", features.sha384);
        setFeature(L, "sha512", features.sha512);
        setFeature(L, "md5", features.md5);
        setFeature(L, "hmac", features.hmac);
        setFeature(L, "streaming_hash", features.sha256);
        setFeature(L, "ed25519", features.ed25519);
        setFeature(L, "ed25519_verify", features.ed25519);
        setFeature(L, "ecdsa", features.ecdsa);
        setFeature(L, "ecdsa_verify", features.ecdsa);
        setFeature(L, "rsa_pkcs1_verify", features.rsa);
        setFeature(L, "rsa_pss_verify", features.rsa);
        setFeature(L, "x25519", features.x25519);
        setFeature(L, "p256", features.p256);
        setFeature(L, "aes_128_gcm", features.aes128Gcm);
        setFeature(L, "aes_256_gcm", features.aes256Gcm);
        setFeature(L, "aes_gcm", features.aes128Gcm && features.aes256Gcm);
        setFeature(L, "chacha20_poly1305", features.chacha20Poly1305);
        setFeature(L, "random_bytes", features.random);
        setFeature(L, "constant_time_equal", true);
        setFeature(L, "hkdf", features.hkdf);
        setFeature(L, "pbkdf2", features.pbkdf2);
        setFeature(L, "scrypt", features.scrypt);
        return 1;
    }
}

const luaL_Reg cryptolib[] = {
    {"sha256", lua::wrap<l_sha256>},
    {"sha384", lua::wrap<l_sha384>},
    {"sha512", lua::wrap<l_sha512>},
    {"md5", lua::wrap<l_md5>},
    {"hash", lua::wrap<l_hash>},
    {"hash_new", lua::wrap<l_hash_new>},
    {"hmac", lua::wrap<l_hmac>},
    {"ed25519_keypair", lua::wrap<l_ed25519_keypair>},
    {"ed25519_public", lua::wrap<l_ed25519_public>},
    {"ed25519_sign", lua::wrap<l_ed25519_sign>},
    {"ed25519_verify", lua::wrap<l_ed25519_verify>},
    {"ecdsa_keypair", lua::wrap<l_ecdsa_keypair>},
    {"ecdsa_public", lua::wrap<l_ecdsa_public>},
    {"ecdsa_sign", lua::wrap<l_ecdsa_sign>},
    {"ecdsa_verify", lua::wrap<l_ecdsa_verify>},
    {"rsa_pkcs1_verify", lua::wrap<l_rsa_pkcs1_verify>},
    {"rsa_pss_verify", lua::wrap<l_rsa_pss_verify>},
    {"x25519_keypair", lua::wrap<l_x25519_keypair>},
    {"x25519_public", lua::wrap<l_x25519_public>},
    {"x25519", lua::wrap<l_x25519>},
    {"x25519_shared", lua::wrap<l_x25519>},
    {"p256_keypair", lua::wrap<l_p256_keypair>},
    {"p256_public", lua::wrap<l_p256_public>},
    {"p256_shared", lua::wrap<l_p256_shared>},
    {"aes_gcm_encrypt", lua::wrap<l_aes_gcm_encrypt>},
    {"aes_gcm_decrypt", lua::wrap<l_aes_gcm_decrypt>},
    {"chacha20_poly1305_encrypt", lua::wrap<l_chacha20_poly1305_encrypt>},
    {"chacha20_poly1305_decrypt", lua::wrap<l_chacha20_poly1305_decrypt>},
    {"random_bytes", lua::wrap<l_random_bytes>},
    {"constant_time_equal", lua::wrap<l_constant_time_equal>},
    {"hkdf_extract", lua::wrap<l_hkdf_extract>},
    {"hkdf_expand", lua::wrap<l_hkdf_expand>},
    {"pbkdf2", lua::wrap<l_pbkdf2>},
    {"scrypt", lua::wrap<l_scrypt>},
    {"features", lua::wrap<l_features>},
    {NULL, NULL}
};

void initialize_cryptolib(lua::State* L) {
    ensureHashContextMetatable(L);
    if (lua::getglobal(L, "crypto")) {
        lua::pushinteger(L, crypto::API_VERSION);
        lua::setfield(L, "API_VERSION");
        lua::pop(L);
    }
}
