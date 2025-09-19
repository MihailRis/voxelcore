# *random* library

A library of functions for generating random numbers.

## Non-deterministic numbers

```lua
-- Generates a random number in the range [0..1)
random.random() --> number

-- Generates a random integer in the range [0..n]
random.random(n) --> number

-- Generates a random integer in the range [a..b]
random.random(a, b) --> number

-- Generates a random byte array of length n
random.bytes(n: number) -> Bytearray

-- Generates a UUID version 4
random.uuid() -> str
```

## Pseudorandom numbers

The library provides the Random class - a generator with its own isolated state.

```lua
local rng = random.Random()

-- Used similarly to math.random
local a = rng:random() --> [0..1)
local b = rng:random(10) --> [0..10]
local c = rng:random(5, 20) --> [5..20]

-- Sets the generator state to generate a reproducible sequence of random numbers
rng:seed(42)
```
