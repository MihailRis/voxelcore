# Библиотека crypto

```py
crypto.encrypt(data: Bytearray|table, algorithm: str, params: table, [опционально] useTable: bool) -> Bytearray|table
```

Шифрует байты по указанному алгоритму с параметрами **params**

```py
crypto.decrypt(data: Bytearray|table, algorithm: str, params: table, [опционально] useTable: bool) -> Bytearray|table
```

Дешифрует байты по указанному алгоритму с параметрами **params**

```py
crypto.digest(data: Bytearray|table, algorithm: str, params: table) -> Bytearray|table
```

Вычисляет хэш из байт по указанному алгоритму

```py
crypto.generate_keys_pair(algorithm: str, [опционально] useTable: bool) -> Bytearray|table, Bytearray|table
```

Генерирует пару асимметричных ключей для нужного алгоритма (приватный и публичный)  

В **params** могут быть различные поля, в зависимости от переданного алгоритма

## Параметры

## Базовые

### iv
Тип: `Bytearray|table`  
Описание: Вектор инициализации  
Размер: 16 байт для **AES/CBC**, 12 байт для **AES/GCM**

## RSA & AES

### key
Тип: `Bytearray|table`  
Описание: Публичный/приватный или симметричный ключ шифрования  
Размер: 128/192/256 бит (16/24/32 байта) для **AES**, 1024/2048/4096 бит (128/256/512 байт) для **RSA**


## AES

### iv
Тип: `Bytearray|table`  
Описание: Вектор инициализации  
Размер: 16 байт для **AES/CBC**, 12 байт для **AES/GCM**

## AES/GCM

### tag
Тип: `Bytearray|table`  
Описание: Тег аутентификации
Размер: 16 байт

## Псевдонимы для алгоритмов

**AES/CBC** = `aes/cbc`  
**AES/GCM** = `aes/gcm`  
**RSA** = `rsa`
