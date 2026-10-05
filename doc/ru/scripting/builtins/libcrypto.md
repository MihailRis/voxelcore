# Библиотека *crypto*

Библиотека предоставляет основные криптографические функции

## Хеширование

```lua
crypto.sha256(data: str) -> str
crypto.sha384(data: str) -> str
crypto.sha512(data: str) -> str
crypto.md5(data: str) -> str

crypto.hash(hash: str, data: str) -> str
crypto.hmac(hash: str, key: str, data: str) -> str
```

Поддерживаются `SHA256`, `SHA384`, `SHA512` и `MD5`

MD5 нужен для совместимости со старыми форматами. Для новых данных лучше
использовать SHA256

Большой файл можно передавать частями

```lua
local hash = crypto.hash_new("SHA256")

hash:update(part1)
hash:update(part2)

local result = hash:final()
```

После `final` контекст можно очистить через `reset`

## Подписи

```lua
crypto.ed25519_keypair() -> private_key, public_key
crypto.ed25519_public(private_key: str) -> str
crypto.ed25519_sign(private_key: str, message: str) -> str
crypto.ed25519_verify(public_key: str, message: str, signature: str)
    -> true
    -> false, error
```

Ключи Ed25519 имеют длину 32 байта. Подпись имеет длину 64 байта

```lua
crypto.ecdsa_keypair(curve: str) -> private_key, public_key
crypto.ecdsa_public(curve: str, private_key: str) -> str
crypto.ecdsa_sign(curve: str, private_key: str, message: str, hash: str) -> str
crypto.ecdsa_verify(curve: str, public_key: str, message: str,
                    signature: str, hash: str)
    -> true
    -> false, error
```

Доступны кривые `P-256`, `P-384` и `P-521`

Публичный ключ ECDSA записывается как несжатая SEC1 точка. Подпись хранится в
DER формате

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

Модуль и экспонента RSA передаются в big endian. Значение `salt_length = -1`
использует размер хеша

## Обмен ключами

```lua
crypto.x25519_keypair() -> private_key, public_key
crypto.x25519_public(private_key: str) -> str
crypto.x25519(private_key: str, peer_public_key: str) -> str
crypto.x25519_shared(private_key: str, peer_public_key: str) -> str

crypto.p256_keypair() -> private_key, public_key
crypto.p256_public(private_key: str) -> str
crypto.p256_shared(private_key: str, peer_public_key: str) -> str
```

`x25519` и `x25519_shared` выполняют одну и ту же операцию

Общий секрет не стоит использовать как готовый ключ. Сначала его нужно
обработать через HKDF

## Шифрование

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

AES GCM принимает ключи размером 16 или 32 байта

ChaCha20 Poly1305 использует ключ размером 32 байта и nonce размером 12 байт

В конец зашифрованных данных добавляется тег размером 16 байт

Нельзя повторно использовать один nonce с тем же ключом

## Остальные функции

```lua
crypto.random_bytes(length: int) -> str
crypto.constant_time_equal(left: str, right: str) -> bool

crypto.hkdf_extract(hash: str, salt: str, ikm: str) -> str
crypto.hkdf_expand(hash: str, prk: str, info: str, length: int) -> str

crypto.pbkdf2(hash: str, password: str, salt: str,
              iterations: int, length: int) -> str
crypto.scrypt(password: str, salt: str, n: int, r: int, p: int,
              length: int, [опционально]max_memory: int=0) -> str

crypto.features() -> table
```

Для хранения паролей следует использовать scrypt или PBKDF2 со случайной
солью. Обычный SHA256 для паролей не подходит

`features` возвращает версию OpenSSL и список доступных возможностей

Неверная подпись возвращает `false, "invalid_signature"`

Неверный тег при расшифровке возвращает `nil, "authentication_failed"`

Ошибки в аргументах вызывают обычную Lua ошибку
