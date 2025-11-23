# Библиотека Vec*n*

*vecn* содержит набор функций для работы с векторами размерностью 2, 3 или 4.
Большинство функций имеют несколько вариантов списка аргументов (перегрузок).

> [!WARNING]
>
> vecn, где n == размерность вектора (2, 3, 4), т.е vec2, vec3, vec4
> 

## Типы данных

На данной странице будут использоваться условные обозначения типов.
- vector - массив из двух, трех или четырех чисел
- vec2 - массив из двух чисел
- vec3 - массив из трех чисел
- vec4 - массив из четырех чисел

> [!WARNING]
>
> Аннотации типов являются частью документации и не указываются при вызове использовании.


## Операции с векторами

#### Сложение - *vecn.add(...)*

```lua
-- возвращает результат сложения векторов
vecn.add(a: vector, b: vector)

-- возвращает результат сложения вектора и скаляра
vecn.add(a: vector, b: number)

-- записывает результат сложения двух векторов в dst
vecn.add(a: vector, b: vector, dst: vector)
```

#### Вычитание - *vecn.sub(...)*

```lua
-- возвращает результат вычитания векторов
vecn.sub(a: vector, b: vector)

-- возвращает результат вычитания скаляра из вектора
vecn.sub(a: vector, b: number)

-- записывает результат вычитания двух векторов в dst
vecn.sub(a: vector, b: vector, dst: vector)
```

#### Умножение - *vecn.mul(...)*

```lua
-- возвращает результат умножения векторов
vecn.mul(a: vector, b: vector)

-- возвращает результат умножения вектора на скаляр
vecn.mul(a: vector, b: number)
```

#### Инверсия - *vecn.inv(...)*

```lua
-- возвращает результат инверсии (противоположный) вектора
vecn.inverse(a: vector)

-- записывает инвертированный вектор в dst
vecn.inverse(v: vector, dst: vector)
```

####  Деление - *vecn.div(...)*

```lua
-- возвращает результат деления векторов
vecn.div(a: vector, b: vector)

-- возвращает результат деления вектора на скаляр
vecn.div(a: vector, b: number)

-- записывает результат деления двух векторов в dst
vecn.div(a: vector, b: vector, dst: vector)
```

#### Нормализация - *vecn.norm(...)*

```lua
-- возвращает нормализованный вектор
vecn.normalize(a: vector)

-- записывает нормализованный вектор в dst
vecn.normalize(v: vector, dst: vector)
```

#### Длина вектора - *vecn.len(...)*

```lua
-- возвращает длину вектора
vecn.length(a: vector)

```

#### Дистанция - *vecn.distance(...)*

```lua
-- возвращает расстояние между двумя векторами
vecn.distance(a: vector, b: vector)
```

#### Абсолютное значение - *vecn.abs(...)*

```lua
-- возвращает вектор с абсолютными значениями
vecn.abs(a: vector)

-- записывает абсолютное значение вектора в dst
vecn.abs(v: vector, dst: vector)
```

#### Округление - *vecn.round(...)*

```lua
-- возвращает вектор с округленными значениями
vecn.round(a: vector)

-- записывает округленный вектор в dst
vecn.round(v: vector, dst: vector)
```

#### Степень - *vecn.pow(...)*

```lua
-- возвращает вектор с элементами, возведенными в степень
vecn.pow(a: vector, b: number)

-- записывает вектор, возведенный в степень, в dst
vecn.pow(v: vector, exponent: number, dst: vector)
```

#### Скалярное произведение - *vecn.dot(...)*
```lua
-- возвращает скалярное произведение векторов
vecn.dot(a: vector, b: vector)
```

#### Смешивание - *vecn.mix(...)*

```lua
-- возвращает вектор a * (1.0 - t) + b * t
vecn.mix(a: vector, b: vector, t: number)

-- записывает в dst вектор a * (1.0 - t) + b * t
vecn.mix(a: vector, b: vector, t: number, dst: vector)
```

#### Перевод в строку - *vecn.tostring(...)*
> [!WARNING]
> Возвращает только тогда, когда содержимым является вектор
```lua
-- возвращает строку представляющую содержимое вектора
vecn.tostring(a: vector)
```

## Специфические функции

Функции относящиеся к конкретным размерностям векторов.

```lua
-- возвращает случайный вектор, координаты которого равномерно распределены на сфере заданного радиуса
vec3.spherical_rand(radius: number)

-- записывает случайный вектор, координаты которого равномерно распределены на сфере заданного радиуса в dst
vec3.spherical_rand(radius: number, dst: vec3)

-- возвращает угол направления вектора v в градусах [0, 360]
vec2.angle(v: vec2)

-- возвращает угол направления вектора {x, y} в градусах [0, 360]
vec2.angle(x: number, y: number) 

-- возвращает повернутый вектор на угол в градусах против часовой стрелки
vec2.rotate(v: vec2, angle: number) -> vec2

-- записывает повернутый вектор на угол в градусах против часовой стрелки в dst
vec2.rotate(v: vec2, angle: number, dst: vec2) -> vec2
```


## Пример
```lua
-- создание векторов разной размерности
local v1_3d = {1, 2, 2}
local v2_3d = {10, 20, 40}
local v3_4d = {1, 2, 4, 1}
local v4_2d = {1, 0}
local scal = 6 -- обычный скаляр

-- сложение векторов
local result_add = vec3.add(v1_3d, v2_3d)
print("add: " .. vec3.tostring(result_add)) -- {11, 22, 42}

-- вычитание векторов
local result_sub = vec3.sub(v2_3d, v1_3d)
print("sub: " .. vec3.tostring(result_sub)) -- {9, 18, 38}

-- умножение векторов
local result_mul = vec3.mul(v1_3d, v2_3d)
print("mul: " .. vec3.tostring(result_mul)) -- {10, 40, 80}

-- умножение вектора на скаляр
local result_mul_scal = vec3.mul(v1_3d, scal)
print("mul_scal: " .. vec3.tostring(result_mul_scal)) -- {6, 12, 12}

-- нормализация вектора
local result_norm = vec3.normalize(v1_3d)
print("norm: " .. vec3.tostring(result_norm)) -- {0.333, 0.667, 0.667}

-- дистанция между векторами
local result_distance = vec3.distance(v1_3d, v2_3d)
print("distance: " .. result_distance) -- 43

-- длина вектора
local result_len = vec3.length(v1_3d)
print("len: " .. result_len) -- 3

-- абсолютное значение вектора
local result_abs = vec3.abs(v1_3d)
print("abs: " .. vec3.tostring(result_abs)) -- {1, 2, 2}

-- округление вектора
local result_round = vec3.round(v1_3d)
print("round: " .. vec3.tostring(result_round)) -- {1, 2, 2}

-- степень вектора
local result_pow = vec3.pow(v1_3d, 2)
print("pow: " .. vec3.tostring(result_pow)) -- {1, 4, 4}

-- скалярное произведение векторов
local result_dot = vec3.dot(v1_3d, v2_3d)
print("dot: " .. result_dot) -- 250

-- смешивание векторов
local result_mix = vec3.mix(v1_3d, v2_3d, 0.25)
print("mix: " .. vec3.tostring(result_mix)) -- {3.25, 6.5, 11.5}

```
