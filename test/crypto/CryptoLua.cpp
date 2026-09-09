#include <gtest/gtest.h>

#include <memory>

#include "logic/scripting/lua/libs/api_lua.hpp"

namespace {
    struct LuaCloser {
        void operator()(lua::State* state) const {
            lua_close(state);
        }
    };
}

TEST(CryptoLua, PublicV1ApiWorksFromLua) {
    std::unique_ptr<lua::State, LuaCloser> state(luaL_newstate());
    ASSERT_NE(state, nullptr);
    luaL_openlibs(state.get());
    lua::openlib(state.get(), "crypto", cryptolib);
    initialize_cryptolib(state.get());

    const char* script = R"lua(
        assert(crypto.API_VERSION == 1)
        local features = crypto.features()
        assert(features.api_version == 1)
        assert(features.backend == "OpenSSL")

        local binary = "a\0b"
        assert(crypto.hash("SHA256", binary) == crypto.sha256(binary))
        local stream = crypto.hash_new("SHA256")
        assert(stream:update("a"):update("\0b"):final() == crypto.sha256(binary))
        stream:reset():update("abc")
        assert(stream:final() == crypto.sha256("abc"))

        local private, public = crypto.ed25519_keypair()
        assert(crypto.ed25519_public(private) == public)
        local signature = crypto.ed25519_sign(private, binary)
        assert(crypto.ed25519_verify(public, binary, signature) == true)
        local ok, error_code = crypto.ed25519_verify(public, "changed", signature)
        assert(ok == false and error_code == "invalid_signature")

        local x_private_a, x_public_a = crypto.x25519_keypair()
        local x_private_b, x_public_b = crypto.x25519_keypair()
        assert(crypto.x25519(x_private_a, x_public_b) ==
               crypto.x25519_shared(x_private_b, x_public_a))

        local p_private_a, p_public_a = crypto.p256_keypair()
        local p_private_b, p_public_b = crypto.p256_keypair()
        assert(crypto.p256_public(p_private_a) == p_public_a)
        assert(crypto.p256_shared(p_private_a, p_public_b) ==
               crypto.p256_shared(p_private_b, p_public_a))

        local key = crypto.random_bytes(16)
        local nonce = crypto.random_bytes(12)
        local ciphertext = crypto.aes_gcm_encrypt(key, nonce, "aad", binary)
        assert(crypto.aes_gcm_decrypt(key, nonce, "aad", ciphertext) == binary)
        local plaintext, decrypt_error = crypto.aes_gcm_decrypt(
            key, nonce, "changed", ciphertext
        )
        assert(plaintext == nil and decrypt_error == "authentication_failed")

        assert(#crypto.pbkdf2("SHA256", "password", "salt", 1, 32) == 32)
        assert(#crypto.scrypt("password", "salt", 16, 1, 1, 32) == 32)
    )lua";

    const int result = luaL_dostring(state.get(), script);
    if (result != 0) {
        FAIL() << lua_tostring(state.get(), -1);
    }
}
