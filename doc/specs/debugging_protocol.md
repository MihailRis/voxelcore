# VC-DBG protocol v1

## Notes

- '?' in name means that the attribute is optional.

## Connecting

### Step 1

Connection initiating with binary header exchange:

```
'v' 'c' '-' 'd' 'b' 'g' NUL XX
76  63  2D  64  62  67  00  XX
```

XX - protocol version number

Client sends header to the server. Then server responds.
Server closes connection after sending header if it mismatches.

### Step 2

Client sends 'connect' command.

## Messages

Message is:
- 32 bit little-endian unsigned integer - number of encoded message bytes.
- message itself (UTF-8 encoded json).

## Client-to-server

### Establishing connection

```json
{
    "type": "connect",
    "?disconnect-action": "resume|detach|terminate"
}
```

Configuring connection. Disconnect-action is action that debugged instance must perform on debugging client connection closed/refused.

- `resume` - Resume from pause mode and start listening for client.
- `detach` - Resume from pause mode and stop server.
- `terminate` - Stop the debugged application.


### Specific action signals.

```json
{
    "type": "pause|resume|terminate|detach"
}
```

### Breakpoints management

```json
{
    "type": "set-breakpoint|remove-breakpoint",
    "source": "entry_point:path",
    "line": 1
}
```

### Local value details request

```json
{
    "type": "get-value",
    "frame": 0,
    "local": 1,
    "path": ["path", "to", 1, "value"]
}
```

- `frame` - Call stack frame index (indexing from most recent call)
- `local` - Local variable index (based on `paused` event stack trace)
- `path` - Requsted value path segments. Example: `['a', 'b', 5]` is `local_variable.a.b[5]`

Responds with:

```json
{
    "type": "value",
    "frame": 0,
    "local": 1,
    "path": ["path", "to", 1, "value"],
    "value": "value itself"
}
```

Example: actual value is table: 
```lua
{a=5, b="test", 2={}}
```

Then `value` is:
```json
{
    "a": {
        "type": "number",
        "short": "5"
    },
    "b": {
        "type": "string",
        "short": "test"
    },
    "1": {
        "type": "table",
        "short": "{...}"
    }
}
```

## Server-to-client

### Response signals

```json
{
    "type": "success|resumed"
}
```

### Pause event

```json
{
    "type": "paused",
    "?reason": "breakpoint|exception|step",
    "?message": "...",
    "?stack": [
        {
            "?function": "function name",
            "source": "source name",
            "what": "what",
            "line": 1,
            "locals": [
                "name": {
                    "type": "local type",
                    "index": 1,
                    "short": "short value",
                    "size": 0
                }
            ]
        }
    ]
}
```
