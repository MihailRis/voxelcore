# *gfx.weather* library

A library for managing audio/visual weather effects.

Weather settings:

| Property      | Description                      | Default           |
| ------------- | -------------------------------- | ----------------- |
| fall          | Precipitation (see table 2)      | {}                |
| clouds        | Cloudiness [0.0, 1.0]            | 0.0               |
| fog_opacity   | Maximum fog density [0.0, 1.0]   | 0.0               |
| fog_dencity   | Fog density                      | 1.0               |
| fog_curve     | Fog curve                        | 1.0               |
| thunder_rate  | Thunder rate [0.0, 1.0]          | 0.0               |
| sky_tint      | Sky color multiplier             | {1, 1, 1}         |
| clouds_tint   | Cloud color multiplier           | {1, 1, 1}         |
| min_sky_light | Minimum sky-emitted light (moon) | {0.2, 0.25, 0.33} |

Precipitation:

| Property      | Description                                  | Default |
| ------------- | -------------------------------------------- | ------- |
| texture       | Precipitation texture                        | ""      |
| noise         | Precipitation noise                          | ""      |
| vspeed        | Precipitation vertical speed                 | 1.0     |
| hspeed        | Maximum horizontal speed of precipitation    | 0.1     |
| scale         | Precipitation UV scale                       | 0.1     |
| min_opacity   | Minimum alpha multiplier for precipitation   | 0.0     |
| max_opacity   | Maximum alpha multiplier for precipitation   | 1.0     |
| max_intensity | Maximum precipitation intensity              | 1.0     |
| opaque        | Disable precipitation translucency           | false   |
| splash        | Precipitation splash particle settings table | {}      |

```lua
-- Smoothly switches weather
gfx.weather.change(
    -- weather settings table
    weather: table,
    -- weather change duration in seconds
    time: number,
    -- weather preset name
    [optional] name: str
)

-- Returns weather preset name
gfx.weather.get_current() -> str

-- Returns a copy of the weather settings table
gfx.weather.get_current_data() -> table

-- Returns the current precipitation intensity
gfx.weather.get_fall_intensity() -> number

-- Checks if weather is currently switching
gfx.weather.is_transition() -> bool
```
