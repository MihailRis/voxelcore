# Встроенные компоненты

## *core:pathfinding*

Компонент для построение путей движения мобов.

```lua
local pathfinding = entity:require_component("core:pathfinding")

-- Устанавливает цель движения, не сбрасывая текущий маршрут
pathfinding.set_target(target: vec3)

-- Возвращает текущую цель движения
pathfinding.get_target() --> vec3

-- Устанавливает высоту преодолимого прыжком препятствия
pathfinding.set_jump_height(height: number)

-- Возвращает текущий построенный маршрут или nil
pathfinding.get_route()

-- Сбрасывает текущий построенный маршрут
pathfinding.reset_route()

-- Возвращает следующую точку маршрута, по текущим координатам.
-- (следует использовать компонент core:mob - функция mob.follow_waypoints)
pathfinding.next_waypoint() --> vec3 или nil

-- Устанавливает интервал перестройки маршрута в тактах обновления.
pathfinding.set_refresh_interval(interval: number)
```

## *core:mob*

Компонент для управления движением (включая полёт) и вращением мобов.

```lua
local mob = entity:require_component("core:mob")

-- Выполняет прыжок с силой jump_force * multiplier
mob.jump([опционально] multiplier: number = 1.0)


-- Вертикальное движение (работает в полёте или в плавании (в будущем))
mob.move_vertical(
    -- Скорость вертикального движения
    speed: number,
    -- Текущая скорость сущности (для минимизации вызовов rigidbody:get_vel())
    [опционально] current_velocity
)

-- Горизонтальное движение
mob.go(
    -- 2D вектор направления движения
    dir: vec2,
    -- Множитель скорости
    speed_multiplier: number,
    -- Бег
    sprint: bool,
    -- Присядь
    crouch: bool,
    -- Текущая скорость сущности (для минимизации вызовов rigidbody:get_vel())
    [опционально] current_velocity
)

-- Меняет направление взгляда сущности, направляя на указанную точку
mob.look_at(
    -- Целевая точка
    point: vec3,
    -- Менять ли направление всей сущности
    change_dir: bool = false
)

-- Движение по построенному маршруту.
-- Если не указан pathfinding, требуется наличие у сущности компонента core:pathfinding
mob.follow_waypoints(
    -- Возможная замена компонента pathfinding
    [опционально] pathfinding
)

-- Устанавливает направление всей сущности
mob.set_dir(dir: vec3)

-- Проверяет, включён ли режим полёта
mob.is_flight() --> bool

-- Включает/выключает режим полёта
mob.set_flight(flag: bool)
```
