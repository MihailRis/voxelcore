# Библиотека *mat4*

*mat4* содержит набор функций для работы с матрицами трансформации размерностью 4x4.

Большинство функций имеют несколько вариантов списка агрументов (перегрузок).

## Единичная матрица - *mat4.idt(...)*

```lua
-- создает единичную матрицу
mat4.idt()

-- записывает единичную матрицу в dst
mat4.idt(dst: matrix)
```

## Определитель матрицы - *mat4.determinant(...)*

```lua
-- вычисляет определитель матрицы
mat4.determinant(m: matrix)
```

## Матрица из кватерниона - *mat4.from_quat(...)*

```lua
-- создает матрицу вращения по кватерниону
mat4.from_quat(quaternion: quat)

-- записывает матрицу вращения по кватерниону в dst
mat4.from_quat(quaternion: quat, dst: matrix)
```

## Умножение матриц - *mat4.mul(...)*

```lua
-- возвращает результат умножения матриц
mat4.mul(a: matrix, b: matrix)
-- записывает результат умножения матриц в dst
mat4.mul(a: matrix, b: matrix, dst: matrix)

-- возвращает результат умножения матрицы и вектора
mat4.mul(a: matrix, v: vector)
-- записывает результат умножения матрицы и вектора в dst
mat4.mul(a: matrix, v: vector, dst: vector)
```

## Инверсия матрицы - *mat4.inverse(...)*

```lua
-- возвращает результат инверсии матрицы
mat4.inverse(m: matrix)
-- записывает результат инверсии матрицы в dst
mat4.inverse(m: matrix, dst: matrix)
```

##  Транспонирование матрицы - *mat4.transpose(...)*

```lua
-- возвращает результат транспонирования матрицы
mat4.transpose(m: matrix)
-- записывает результат транспонирования матрицы в dst
mat4.transpose(m: matrix, dst: matrix)
```

## Смещение - *mat4.translate(...)*

```lua
-- создает матрицу смещения
mat4.translate(translation: vec3)
-- возвращает результат применения смещения к матрице m
mat4.translate(m: matrix, translation: vec3)
-- записывает результат применения смещения к матрице m в dst
mat4.translate(m: matrix, translation: vec3, dst: matrix)
```
## Масштабирование - *mat4.scale(...)*

```lua
-- создает матрицу масштабирования
mat4.scale(scale: vec3)
-- возвращает результат применения масштабирования к матрице m
mat4.scale(m: matrix, scale: vec3)
-- записывает результат применения масштабирования к матрице m в dst
mat4.scale(m: matrix, scale: vec3, dst: matrix)
```

## Вращение - *mat4.rotate(...)*

Угол поворота (angle) указывается в градусах.

```lua
-- создает матрицу поворота (angle - угол поворота) по заданной оси (axis - единичный вектор)
mat4.rotate(axis: vec3, angle: number)
-- возвращает результат применения вращения к матрице m
mat4.rotate(m: matrix, axis: vec3, angle: number)
-- записывает результат применения вращения к матрице m в dst
mat4.rotate(m: matrix, axis: vec3, angle: number, dst: matrix)
```

## Декомпозиция - *mat4.decompose(...)*

Раскладывает матрицу трансформации на составляющие.

```lua
mat4.decompose(m: matrix)
-- возвращает таблицу:
{
    scale=vec3,
    rotation=matrix,
    quaternion=quat,
    translation=vec3,
    skew=vec3,
    perspective=vec4 
} или nil
```

## Отслеживание точки - *mat4.look_at(...)*

```lua
-- cоздаёт матрицу вида с точки 'eye' на точку 'center', где вектор 'up' определяет верх.
mat4.look_at(eye: vec3, center: vec3, up: vec3)
-- записывает матрицу вида в dst
mat4.look_at(eye: vec3, center: vec3, up: vec3, dst: matrix)
```

## Расчёт матрицы перспективы - *mat4.perspective*

```lua
-- возвращает расчитанную матрицу перспективы.
-- параметр fov принимает значение вертикального поля зрения в градусах.
-- параметр ratio задаёт отношение сторон (сторона X к стороне Y, пример: для монитора 800:600 отношение сторон равно 4 к 3, или 4 / 3).
-- параметр near задаёт расстояние от камеры до плоскости Near.
-- параметр far задаёт расстояние от камеры до плоскости Far.
mat4.perspective(fov: number, ratio: number, near: number, far: number) -> mat4

-- расчитывает матрицу перспективы на основе переданных значений и записывает её в переменную dst.
mat4.perspective(fov: number, ratio: number, near: number, far: number, dst: mat4) -> nil
```

## Перевод в строку - *mat4.tostring(...)*

```lua
-- возвращает строку представляющую содержимое матрицы
mat4.tostring(m: matrix)
-- возвращает строку представляющую содержимое матрицы, многострочную, если multiline = true
mat4.tostring(m: matrix, multiline: boolean)
```
