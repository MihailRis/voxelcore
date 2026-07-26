# Библиотека *world*

```lua
-- Проверяет, открыт ли мир
world.is_open() -> boolean

-- Возвращает информацию о мирах.
world.get_list() -> table<{
    -- название мира
    name: string,
    -- предпросмотр (автоматически загружаемая текстура)
    icon: string,
    -- версия движка, на которой был сохранен мир
    version: {int, int}
}>

-- Возвращает текущее игровое время от 0.0 до 1.0, где 0.0 и 1.0 - полночь, 0.5 - полдень.
world.get_day_time() -> number

-- Устанавливает указанное игровое время.
world.set_day_time(time: number)

-- Устанавливает указанную скорость смены времени суток.
world.set_day_time_speed(value: number)

-- Возвращает скорость скорость смены времени суток.
world.get_day_time_speed() -> number

-- Возвращает суммарное время, прошедшее в мире.
world.get_total_time() -> number

-- Возвращает зерно мира.
world.get_seed() -> int

-- Возвращает имя генератора.
world.get_generator() -> string

-- Проверяет существование мира по имени.
world.exists(name: string) -> boolean

-- Проверяет является ли текущее время днём. От 0.333(8 утра) до 0.833(8 вечера).
world.is_day() -> boolean

-- Проверяет является ли текущее время ночью. От 0.833(8 вечера) до 0.333(8 утра).
world.is_night() -> boolean

-- Возвращает общее количество загруженных в память чанков
world.count_chunks() -> int

-- Возвращает сжатые данные чанка для отправки.
-- Если чанк не загружен, возвращает сохранённые данные.
-- На данный момент включает:
-- 1. Данные вокселей (id и состояние)
-- 2. Метаданные (поля) вокселей
world.get_chunk_data(x: int, z: int) -> Bytearray или nil

-- Изменяет чанк на основе сжатых данных.
-- Возвращает true если чанк существует.
world.set_chunk_data(
    x: int, z: int,
    -- сжатые данные чанка
    data: Bytearray
) -> boolean

-- Сохраняет данные чанка в регион.
-- Изменения будет записаны в файл только после сохранения мира.
world.save_chunk_data(
    x: int, z: int,
    -- сжатые данные чанка
    data: Bytearray
)

-- Бросает луч из точки start в направлении dir.
world.raycast(
    params: table {
        [==[ обязательные параметры ]==]
        -- стартовая точка
        start: vec3,
        -- вектор направления луча
        dir: vec3,
        -- максимальное расстояние
        distance: number,

        [==[ дополнительные параметры (сущности) ]==]
        -- учитывать сущности
        entities: bool = true,
        -- uid игнорируемой сущности
        ignore_uid: number = 0 (ENTITY_NONE),
        -- id типов сущностей используемых для фильтрации
        filter_entities: table<string|number> = nil,
        -- исключающий режим фильтрации:
        --   при значении (true) filter_entities определяет список игнорируемых
        entities_exclusion: bool = false,
        
        [==[ дополнительные параметры (блоки) ]==]
        -- id типов блоков используемых для фильтрации
        filter_blocks: table<string|number> = nil,
        -- исключающий режим фильтрации:
        --   при значении (true) filter_blocks определяет список игнорируемых
        blocks_exclusion: bool = true,
        -- учитывать блоки с selectable=false
        nonselect_blocks: bool = false;
    }
) -> {
    block: int или nil, -- id блока
    entity: int или nil, -- uid сущности
    endpoint: vec3, -- точка касания луча
    iendpoint: vec3, -- позиция блока, которого касается луч
    length: number, -- длина луча
    normal: vec3, -- вектор нормали поверхности, которой касается луч
} или nil если луч не коснулся блока или сущности
```
