# Предзагрузка ассетов (файл *preload.json*)

Для загрузки ассетов, не загружаемых автоматически, такие как звуки, дополнительные текстуры, используется файл `preload.json`, создающийся в папке контент-пака.

Ассеты в файле разделяются на категории:
- animation - анимация
- fonts - шрифты
- models - модели
- shaders - шейдеры
- skeletons - скелеты
- sounds - звуки
- textures - текстуры

> [!NOTE]
> При загрузке звука подгружаются все его вариации, по шаблону:
> (звук: sound_name) -> *sound_name.ogg, sound_name_1.ogg, sound_name_2.ogg, ...*
> или *sound_name_0.ogg, sound_name_1.ogg, sound_name_2.ogg, ...*

Добавление звука `пак:sounds/events/explosion.ogg` со всеми его вариантами:
```json
{
    "sounds": [
        "events/explosion"
    ]
}
```
Будет доступен под именем: "events/explosion"

В случае, если нужно будет работать с PCM данными звука (сейчас не доступно из скриптинга), требуется указать параметр `keep-pcm`:
```json
{
    "sounds": [
        {
            "name": "events/explosion",
            "keep-pcm": true
        }
    ]
}
```


Пример файла из пакета `core:` (`res/preload.json`):
```json
{
    "shaders": [
        "ui3d",
        "screen",
        "background",
        "skybox_gen"
    ],
    "textures": [
        "misc/moon",
        "misc/sun",
        "gui/crosshair"
    ]
}
```
