# Binary JSON Format Specification

Format version: 1.1

This binary data format is developed for use as binary version of JSON in [VoxelEngine-Cpp](https://github.com/MihailRis/VoxelEngine-Cpp) and not compatible with [BSON](https://bsonspec.org/spec.html) due to elements/entries syntax and type codes differences

## Basic types

byteorder: little-endian
| Name    | Size    | Definition              |
| ------- | ------- | ----------------------- |
| byte    | 1 byte  | 8 bit unsigned integer  |
| int16   | 2 bytes | 16 bit signed integer   |
| int32   | 4 bytes | 32 bit signed integer   |
| int64   | 8 bytes | 64 bit unsigned integer |
| uint32  | 4 bytes | 32 bit unsigned integer |
| float64 | 8 bytes | 64 bit floating point   |

## Syntax (RFC 5234)

```bnf
file      = %x01 document / cdocument   file content
document  = uint32 (*entry) %x00        uint32 stores bytes number 
                                        of the encoded document 
                                        including the uint32 size
entry     = cstring value
value     = %x01 document
          / %x02 (*value) %x00          list of values
          / %x03 byte                   8 bit unsigned integer
          / %x04 int16                  16 bit signed integer
          / %x05 int32                  32 bit signed integer
          / %x06 int64                  64 bit signed integer
          / %x07 float64                number
          / %x08 string                 utf-8 encoded string
          / %x09 uint32 (*byte)         bytes array
          / %x0A                        boolean 'false'
          / %x0B                        boolean 'true'
          / %x0C                        null value
cdocument = %x1F %x8B (16*byte)         gzip-compressed data:
                                            %x01 document
cstring   = (*%x01-FF) %x00
string    = uint32 (*byte)              uint32 stores number of the 
                                        encoded string bytes

float64   = 8byte
int64     = 8byte
uint32    = 4byte
int32     = 4byte
int16     = 2byte
byte      = %x00-FF
```
