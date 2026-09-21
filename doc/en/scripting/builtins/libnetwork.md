# *network* library

A library for working with the network.

## HTTP requests

There is a configurable function `network.request` that allows performing HTTP requests with various methods (GET, POST, PUT, DELETE, etc.) and configuring headers, request body, timeout, and other parameters.

> To pass binary data in the request body, use a byte array (Bytearray) or a string. In `on_response`, the `body` string can be converted to a byte array using `Bytearray(response.body)`.

```lua
network.request(
    url: string,
    parameters: {
        -- Request method (GET, POST, PUT, DELETE, etc.)
        method: string,
        -- Request body as a string or Bytearray
        body: table|string,
        -- List of additional request headers
        headers: table<string>,
        -- Timeout in milliseconds
        timeout: int,
        -- Whether to verify the SSL certificate
        verify_ssl: boolean,
        -- Function called when a response is received
        on_response: function(response: {
            -- HTTP response status code
            status: int,
            -- Response body as a string
            body: string,
            -- List of response headers
            headers: table<string>
        }),
    }
)
```

### Simplified functions for GET and POST requests

```lua
-- Performs a GET request to the specified URL.
network.get(
    url: string,
    -- Function to call when response is received
    callback: function(string),
    -- Error handler
    [optional] onfailure: function(int, string),
    -- List of additional request headers
    [optional] headers: table<string>
)

-- Example:
network.get("https://api.github.com/repos/MihailRis/VoxelEngine-Cpp/releases/latest", function (s)
    print(json.parse(s).name) -- will output the name of the latest engine release
end)

-- A variant for binary files, with a byte array instead of a string in the response.
network.get_binary(
    url: string,
    callback: function(ByteArray),
    [optional] onfailure: function(int, string),
    [optional] headers: table<string>
)

-- Performs a POST request to the specified URL.
-- Currently, only `Content-Type: application/json` is supported
-- After receiving the response, passes the text to the callback function.
-- In case of an error, the HTTP response code will be passed to onfailure.
network.post(
    url: string,
    -- Request body as a table (will be converted to JSON) or string
    body: table|string,
    -- Function called when response is received
    callback: function(string),
    -- Error handler
    [optional] onfailure: function(int, string),
    -- List of additional request headers
    [optional] headers: table<string>
)
```

## TCP Connections

```lua
network.tcp_connect(
    -- Address
    address: string,
    -- Port
    port: int,
    -- Function called upon successful connection
    -- Sending will not work before connection
    -- Socket is passed as the only argument
    callback: function(Socket),
    -- Function called when a connection error occurs
    -- Arguments passed: socket and error text
    [optional] error_callback: function(Socket, string)
) -> Socket
```

Initiates TCP connection.

The Socket class has the following methods:

```lua
-- Sends a byte array
socket:send(table|ByteArray|string)

-- Reads the received data
socket:recv(
    -- Maximum size of the byte array to read
    length: int,
    -- Use table instead of Bytearray
    [optional] usetable: bool=false
) -> nil|table|Bytearray
-- Returns nil on error (socket is closed or does not exist).
-- If there is no data yet, returns an empty byte array.

-- Asynchronous version for use in coroutines.
-- Waits for the entire specified number of bytes to be received.
-- If socket closes, function works like socket:recv
socket:recv_async(
    -- Size of the byte array to read
    length: int,
    -- Use table instead of Bytearray
    [optional] usetable: bool=false
) -> nil|table|Bytearray

-- `peek` and `peek_async` are analogous to the `recv` and `recv_async` methods
-- with the exception that `peek` and `peek_async` do not advance the socket buffer position
-- This means they do not remove bytes from the socket, so the bytes can be received after
socket:peek(
    length: int,
    [optional] usetable: boolean=false
) -> nil|table|Bytearray

socket:peek_async(
    length: int,
    [optional] usetable: boolean=false
) -> nil|table|Bytearray

-- Closes the connection
socket:close()

-- Returns the number of data bytes available for reading
socket:available() -> int

-- Checks that the socket exists and is not closed.
socket:is_alive() -> bool

-- Checks if the connection is present (using socket:send(...) is available).
socket:is_connected() -> bool

-- Returns the address and port of the connection.
socket:get_address() -> string, int
```

```lua
-- Opens a TCP server.
network.tcp_open(
    -- Port
    port: int,
    -- Function called when connecting
    -- The socket of the connected client is passed as the only argument
    callback: function(Socket)
) -> ServerSocket
```

The SocketServer class has the following methods:

```lua
-- Closes the server, breaking connections with clients.
server:close()

-- Checks if the TCP server exists and is open.
server:is_open() -> bool

-- Returns the server port.
server:get_port() -> int
```

## HTTP Server

```lua
-- Opens an HTTP server on the given port.
network.http_open(
    -- Port
    port: int,
    -- HTTP request handler function
    handler: function(request),
    -- How long to wait for request:respond(...) before
    -- auto-sending 503. 0 means wait indefinitely.
    [optional] timeout_ms: int = 60000
) -> ServerSocket
```

The ServerSocket class for the HTTP server is identical to the TCP server's

The `handler` may respond in two ways:

* return a response table `{status: int, headers: table<string>, body: string|Bytearray}`
  (any field may be omitted; `status` defaults to `200`);
* or call `request:respond(status, body, headers)` itself, e.g. from a
  coroutine, for a delayed answer. In that case the handler's return value
  is ignored.

If the handler errors or does not respond within `timeout_ms`
(60 seconds by default), an `Internal Server Error` (500) or
`Service Unavailable` (503) response is sent automatically.

The `request` class has the following fields and methods:

```lua
request.method       -> string, e.g. "GET"
request.path         -> string, decoded path without the query string
request.query        -> string, raw query string (part after '?', if any)
request.headers      -> table<string>, "Name: value" entries
request.body         -> string|Bytearray
request.remote_addr  -> string
request.remote_port  -> int

-- Parses the request body as JSON.
request:json() -> any

-- Sends the response. May be called at most once, from anywhere
-- (including a coroutine, an `on_response` callback, etc.)
request:respond(
    [optional] status: int=200,
    [optional] body: string|Bytearray,
    [optional] headers: table<string>
)

-- Builds a JSON response table ready to be returned from a handler.
network.http_json(
    data: any,
    [optional] status: int=200,
    [optional] headers: table<string>
) --> table
```

### Router

For URL routing, `network.http_router()` provides a small helper.
Segments prefixed with `:` are captured and passed to the handler, in order.

```lua
local router = network.http_router()

router:get("/users/:id", function(request, id)
    return network.http_json({id = id})
end)

router:post("/users", function(request)
    local data = request:json()
    -- ...
    return {status = 201}
end)

network.http_open(8080, router)
```

`Router` methods: `get`, `post`, `put`, `delete`, `patch`, and the generic
`route(method, path, handler)`. Unmatched requests get a `404 Not Found`.

### Example

```lua
network.http_open(8080, function(request)
    if request.method == "GET" and request.path == "/status" then
        return network.http_json({ok = true, uptime = time.uptime()})
    end
    return {status = 404, body = "Not Found"}
end)
```

> The HTTP server supports HTTP/1.1 request/response bodies with
> `Content-Length`; chunked request bodies are not supported and will be
> rejected with `501 Not Implemented`. Every response closes the connection
> (no keep-alive).

## Analytics

```lua
-- Returns the approximate amount of data sent (including connections to localhost)
-- in bytes.
network.get_total_upload() -> int
-- Returns the approximate amount of data received (including connections to localhost)
-- in bytes.
network.get_total_download() -> int
```

## Other

```lua
-- Looks for a free port to use.
network.find_free_port() -> int or nil
```
