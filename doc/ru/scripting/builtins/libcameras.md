# Библиотека *cameras*

Библиотека предназначена для работы с камерами.

## Основные функции

```lua
cameras.get(name: string) -> table
-- или
cameras.get(index: int) -> table
```

Возвращает камеру по имени или индексу.

## Методы камеры

```lua
-- получаем камеру
local cam = cameras.get("пак:имя-камеры")

-- возвращает индекс камеры
cam:get_index() -> int

-- возвращает имя камеры
cam:get_name() -> string

-- возвращает позицию камеры
cam:get_pos() -> vec3
-- устанавливает позицию камеры
cam:set_pos(pos: vec3)

-- возращает вращение камеры
cam:get_rot() -> mat4
-- устанавливает вращение камеры
cam:set_rot(rot: mat4)

-- возвращает значение приближения камеры
cam:get_zoom() -> number
-- устанавливает значение приближения камеры
cam:set_zoom(zoom: number)

-- возвращает угол поля зрения камеры по Y (в градусах)
cam:get_fov() -> number
-- устанавливает угол поля зрения камеры по Y (в градусах)
cam:set_fov(fov: number)

-- возвращает true если ось Y отражена
cam:is_flipped() -> boolean
-- отражает ось Y при значении true
cam:set_flipped(flipped: boolean)

-- проверяет, включен ли режим перспективы
cam:is_perspective() -> boolean
-- включает/выключает режим перспективы
cam:set_perspective(perspective: boolean)

-- возвращает вектор направления камеры
cam:get_front() -> vec3
-- возвращает вектор направления направо
cam:get_right() -> vec3
-- возвращает вектор направления вверх
cam:get_up() -> vec3

-- направляет камеру на заданную точку
cam:look_at(point: vec3)

-- направляет камеру на заданную точку с фактором интерполяции
cam:look_at(point: vec3, t: number)
```

Переключение камеры возможно через функцию [player.set_camera](libplayer.md#камеры).