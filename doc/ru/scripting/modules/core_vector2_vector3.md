# Модули core:vector2, core:vector3

## Vector2
### Операции над векторами


```lua
function vector2:round(decimals: number) -> round[vec2]
```
Округление компонентов вектора

```lua
function vector2:len() -> number
```
Длина вектора

```lua
function vector2:norm() -> number
```
Нормализация вектора

```lua
function vector2:abtw(vector: vec2) -> number
```
Угол между двумя векторами в радианах


```lua
function vector2:proj(vector: vec2) -> vec2
```
Проекция вектора

```lua
function vector2:dot(vector: vec2) -> number
```
Векторное (внутреннее) произведение

```lua
function vector2:lerp(
                     --Целевой вектор
                     b: vec2, 
                     --Интерполяционный коэффициент (от 0 до 1)
                     t: number
                     ) -> vec2
```
Линейная интерполяция вектора

```lua
function vector2:dist(vector: vec2) -> number
```
Дистанция между двумя векторами

```lua
function vector2:cross(vector: vec2) -> number
```
Векторное (внешнее) произведение

```lua
function vector2:rot(
                    --Угол поворота вектора на заданный угол (в радианах)
                    angle: number->rad, 
                    --Вращение вектора относительно осей ("x", "y", "z")
                    axis: str, 
                    --Если истина, то угол поворота переводится автоматически из градусов в радианы
                    convert2deg: bool
                    ) -> vec2
```
Поворот вектора

### Операции с векторами

```lua
    local vec2 = require("core:vector2")

    local v1 = vec2(5, 10)
    local v2 = vec2(10, 15)
    
    -- vec2 .. vec2
    sum_vectors = v1 + v2 -- (15, 25)
    sub_vectors = v1 - v2 -- (-5, -5)
    mul_vectors = v1 * v2 -- (50, 150)
    div_vectors = v1 / v2 -- (0.5, 0.66667.)
    pow_vectors = v1 ^ v2 -- (9765625, 1e+15)

    --vec2 .. scalar
    sum_vec2_scalar = v1 + 10 -- (15, 25)
    sub_vec2_scalar = v1 - 12 -- (-7, -2)
    mul_vec2_scalar = v1 * 20 -- (100, 200)
    div_vec2_scalar = v1 / 1 -- (5, 10)
    pow_vec2_scalar= v1 ^ 2 -- (25, 100)
```



## Vector3
### Операции над векторами

```lua
function vector3:round(decimals: number) -> round[vec3]
```
Округление компонентов вектора
```lua
function vector3:len() -> number
```
Длина вектора

```lua
function vector3:norm() -> number
```
Нормализация вектора

```lua
function vector3:abtw(vector: vec3) -> number
```
Угол между двумя векторами в радианах

```lua
function vector3:isParallel(vector: vec3) -> bool
```
Параллельность вектора к другому вектору


```lua
function vector3:proj(vector: vec3) -> vec3
```
Проекция вектора
...

```lua
function vector3:dot(vector: vec3) -> number
```
Векторное (внутреннее) произведение

```lua
function vector3:lerp(
                     --Целевой вектор
                     b: vec3, 
                     --Интерполяционный коэффициент (от 0 до 1)
                     t: number
                     ) -> vec3
```
Линейная интерполяция вектора

```lua
function vector3:dist(vector: vec3) -> number
```
Дистанция между двумя векторами

```lua
function vector3:dist2line(point1: vec3, point2: vec3) -> number
```
Дистанция до линии

```lua
function vector3:cross(vector: vec3) -> number
```
Векторное (внешнее) произведение

```lua
function vector3:rot(
                    --Угол поворота вектора на заданный угол (в радианах)
                    angle: number->rad, 
                    --Вращение вектора относительно осей ("x", "y", "z")
                    axis: str, 
                    --Если истина, то угол поворота переводится автоматически из градусов в радианы
                    convert2deg: bool
                    ) -> vec2
```
Поворот вектора

### Операции с векторами

```lua
    
local vec3 = require("core:vector3")

local v1 = vec3(1, 2, 3)
local v2 = vec3(4, 5, 6)

-- vec3.. vec3
local sum_vectors = v1 + v2 -- (5, 7, 9)
local sub_vectors = v1 - v2 -- (-3, -3, -3)
local mul_vectors = v1 * v2 -- (4, 10, 18)
local div_vectors = v1 / v2 -- (0.25, 0.4, 0.5)
local pow_vectors = v1 ^ v2 -- (1, 32, 216)

-- vec3.. scalar
local scalar_vector = v1 * 2 -- (2, 4, 6)
local scalar_number = 2 * v1 -- (2, 4, 6)
-- ..etc

```

