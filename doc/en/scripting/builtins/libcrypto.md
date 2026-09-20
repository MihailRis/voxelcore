# *crypto* library

The library provides common cryptographic functions

## Hashing

```lua
crypto.sha256(data: str) -> str
crypto.sha384(data: str) -> str
crypto.sha512(data: str) -> str
crypto.md5(data: str) -> str

crypto.hash(hash: str, data: str) -> str
crypto.hmac(hash: str, key: str, data: str) -> str
```

Supported names are `SHA256`, `SHA384`, `SHA512` and `MD5`

MD5 is needed for compatibility with old formats. SHA256 is better for new data

Large files can be passed in parts

```lua
local hash = crypto.hash_new("SHA256")

hash:update(part1)
hash:update(part2)

local result = hash:final()
```

The context can be cleared with `reset` after `final`

## Signatures

```lua
crypto.ed25519_keypair() -> private_key, public_key
crypto.ed25519_public(private_key: str) -> str
crypto.ed25519_sign(private_key: str, message: str) -> str
crypto.ed25519_verify(public_key: str, message: str, signature: str)
    -> true
    -> false, error
```

Ed25519 keys are 32 bytes long. A signature is 64 bytes long

```lua
crypto.ecdsa_keypair(curve: str) -> private_key, public_key
crypto.ecdsa_public(curve: str, private_key: str) -> str
crypto.ecdsa_sign(curve: str, private_key: str, message: str, hash: str) -> str
crypto.ecdsa_verify(curve: str, public_key: str, message: str,
                    signature: str, hash: str)
    -> true
    -> false, error
```

Available curves are `P-256`, `P-384` and `P-521`

An ECDSA public key is stored as an uncompressed SEC1 point. A signature is
stored in DER format

```lua
crypto.rsa_pkcs1_verify(hash: str, modulus: str, exponent: str,
                        message: str, signature: str)
    -> true
    -> false, error

crypto.rsa_pss_verify(hash: str, modulus: str, exponent: str,
                      message: str, signature: str, salt_length: int)
    -> true
    -> false, error
```

RSA modulus and exponent are passed in big endian. A `salt_length` value of `-1`
uses the hash size

## Key exchange

```lua
crypto.x25519_keypair() -> private_key, public_key
crypto.x25519_public(private_key: str) -> str
crypto.x25519(private_key: str, peer_public_key: str) -> str
crypto.x25519_shared(private_key: str, peer_public_key: str) -> str

crypto.p256_keypair() -> private_key, public_key
crypto.p256_public(private_key: str) -> str
crypto.p256_shared(private_key: str, peer_public_key: str) -> str
```

`x25519` and `x25519_shared` perform the same operation

The shared secret should not be used as a ready key. Pass it through HKDF first

## Encryption

```lua
crypto.aes_gcm_encrypt(key: str, nonce: str, aad: str, plaintext: str) -> str
crypto.aes_gcm_decrypt(key: str, nonce: str, aad: str, ciphertext: str)
    -> plaintext
    -> nil, error

crypto.chacha20_poly1305_encrypt(key: str, nonce: str, aad: str,
                                 plaintext: str) -> str
crypto.chacha20_poly1305_decrypt(key: str, nonce: str, aad: str,
                                 ciphertext: str)
    -> plaintext
    -> nil, error
```

AES GCM accepts 16 and 32 byte keys

ChaCha20 Poly1305 uses a 32 byte key and a 12 byte nonce

A 16 byte tag is appended to encrypted data

Never reuse a nonce with the same key

## Other functions

```lua
crypto.random_bytes(length: int) -> str
crypto.constant_time_equal(left: str, right: str) -> bool

crypto.hkdf_extract(hash: str, salt: str, ikm: str) -> str
crypto.hkdf_expand(hash: str, prk: str, info: str, length: int) -> str

crypto.pbkdf2(hash: str, password: str, salt: str,
              iterations: int, length: int) -> str
crypto.scrypt(password: str, salt: str, n: int, r: int, p: int,
              length: int, [optional]max_memory: int=0) -> str

crypto.features() -> table
```

Use scrypt or PBKDF2 with a random salt for passwords. A regular SHA256 hash is
not suitable for passwords

`features` returns the OpenSSL version and a list of available features

An invalid signature returns `false, "invalid_signature"`

An invalid encryption tag returns `nil, "authentication_failed"`

Invalid arguments raise a regular Lua error
