# *time* Library

```lua
-- Returns the time since engine startup in seconds and milliseconds.
time.uptime() -> number

-- Returns the time delta in seconds and milliseconds (time elapsed since the previous frame)
time.delta() -> number

-- Returns UTC time in seconds
time.utc_time() -> int

-- Returns local (system) time in seconds
time.local_time() -> int

-- Returns the local time offset from UTC in seconds
time.utc_offset() -> int

-- Returns high-precision time in seconds (elapsed since the engine startup).
time.precise_time() -> number
```
