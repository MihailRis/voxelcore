# Библиотека *network*

Библиотека для работы с сетью.

## HTTP-Запросы

Существует настраиваемая функция `network.request`, которая позволяет выполнять HTTP-запросы с различными методами (GET, POST, PUT, DELETE и т.д.) и настраивать заголовки, тело запроса, таймаут и другие параметры.

> Для передачи двоичных данных в теле запроса, используйте массив байт (Bytearray) или строку. В `on_response` строку `body` можно преобразовать в массив байт с помощью `Bytearray(response.body)`.

```lua
network.request(
    url: string,
    parameters: {
        -- Метод запроса (GET, POST, PUT, DELETE и т.д.)
        method: string,
        -- Тело запроса в виде строки или массива байт (Bytearray)
        body: table|string,
        -- Список дополнительных заголовков запроса
        headers: table<string>,
        -- Таймаут в миллисекундах
        timeout: int,
        -- Проверять ли SSL-сертификат
        verify_ssl: boolean,
        -- Функция, вызываемая при получении ответа
        on_response: function(response: {
            -- HTTP-код ответа
            status: int,
            -- Тело ответа в виде строки
            body: string,
            -- Список заголовков ответа
            headers: table<string>
        }),
    }
)
```

### Упрощённые функции для GET и POST запросов

```lua
-- Выполняет GET запрос к указанному URL.
network.get(
    url: string,
    -- Функция, вызываемая при получении ответа
    callback: function(string),
    -- Обработчик ошибок
    [опционально] onfailure: function(int, string),
    -- Список дополнительных заголовков запроса
    [опционально] headers: table<string>
)

-- Пример:
network.get("https://api.github.com/repos/MihailRis/VoxelEngine-Cpp/releases/latest", function (s)
    print(json.parse(s).name) -- выведет имя последнего релиза движка
end)

-- Вариант для двоичных файлов, с массивом байт вместо строки в ответе.
network.get_binary(
    url: string,
    callback: function(Bytearray),
    [опционально] onfailure: function(int, Bytearray),
    [опционально] headers: table<string>
)

-- Выполняет POST запрос к указанному URL.
-- На данный момент реализована поддержка только `Content-Type: application/json`
-- После получения ответа, передаёт текст в функцию callback.
-- В случае ошибки в onfailure будет передан HTTP-код ответа.
network.post(
    url: string,
    -- Тело запроса в виде таблицы, конвертируемой в JSON или строки
    body: table|string,
    -- Функция, вызываемая при получении ответа
    callback: function(string),
    -- Обработчик ошибок
    [опционально] onfailure: function(int, string),
    -- Список дополнительных заголовков запроса
    [опционально] headers: table<string>
)
```

## TCP-Соединения

```lua
network.tcp_connect(
    -- Адрес
    address: string,
    -- Порт
    port: int,
    -- Функция, вызываемая при успешном подключении
    -- До подключения отправка работать не будет
    -- Как единственный аргумент передаётся сокет
    callback: function(Socket)
    -- Функция, вызываемая при ошибке подключения
    -- Как аргументы передаются сокет и текст ошибки
    [опционально] error_callback: function(Socket, string)
) -> Socket
```

Инициирует TCP подключение.

Класс Socket имеет следующие методы:

```lua
-- Отправляет массив байт
socket:send(table|Bytearray|string)

-- Читает полученные данные
socket:recv(
    -- Максимальный размер читаемого массива байт
    length: int, 
    -- Использовать таблицу вместо Bytearray
    [опционально] usetable: boolean=false
) -> nil|table|Bytearray
-- В случае ошибки возвращает nil (сокет закрыт или несуществует).
-- Если данных пока нет, возвращает пустой массив байт.

-- Асинхронный вариант для использования в корутинах.
-- Ожидает получение всего указанного числа байт.
-- При закрытии сокета работает как socket:recv
socket:recv_async(
    -- Размер читаемого массива байт
    length: int, 
    -- Использовать таблицу вместо Bytearray
    [опционально] usetable: boolean=false
) -> nil|table|Bytearray

-- `peek` и `peek_async` являются аналогами методов `recv` и `recv_async`
-- за тем исключением, что `peek` и `peek_async` не двигают позицию буфера сокета
-- Это означает, что они не удаляют байты из сокета, а значит байты могут быть прочитаны после
socket:peek(
    length: int,
    [опционально] usetable: boolean=false
) -> nil|table|Bytearray

socket:peek_async(
    length: int,
    [опционально] usetable: boolean=false
) -> nil|table|Bytearray

-- Оборачивает сокет в io_stream (см. ../io_stream.md)
socket:as_stream(
    [опционально] binary_mode: boolean=true
) -> io_stream

-- Закрывает соединение
socket:close()

-- Возвращает количество доступных для чтения байт данных
socket:available() -> int

-- Проверяет, что сокет существует и не закрыт.
socket:is_alive() -> boolean

-- Проверяет наличие соединения (доступно использование socket:send(...)).
socket:is_connected() -> boolean

-- Возвращает адрес и порт соединения.
socket:get_address() -> string, int

-- Возвращает состояние NoDelay
socket:is_nodelay() -> boolean

-- Устанавливает состояние NoDelay
socket:set_nodelay(state: boolean)
```

```lua
-- Открывает TCP-сервер.
network.tcp_open(
    -- Порт
    port: int,
    -- Функция, вызываемая при поключениях
    -- Как единственный аргумент передаётся сокет подключенного клиента
    callback: function(Socket)
) -> ServerSocket
```

Класс SocketServer имеет следующие методы:

```lua
-- Закрывает сервер, разрывая соединения с клиентами.
server:close()

-- Проверяет, существует и открыт ли TCP сервер.
server:is_open() -> boolean

-- Возвращает порт сервера.
server:get_port() -> int
```

## UDP-Датаграммы

```lua
network.udp_connect(
	address: string,
	port: int,
    -- Функция, вызываемая при получении датаграммы с указанного при открытии сокета адреса и порта
	datagramHandler: function(Bytearray),
	-- Функция, вызываемая после открытия сокета
	-- Опциональна, так как в UDP нет handshake
    [опционально] openCallback: function(WriteableSocket),
) -> WriteableSocket
```

Открывает UDP-сокет с привязкой к удалённому адресу и порту

Класс WriteableSocket имеет следующие методы:

```lua
-- Отправляет датаграмму на адрес и порт, заданные при открытии сокета
socket:send(table|Bytearray|string)

-- Закрывает сокет
socket:close()

-- Проверяет открыт ли сокет
socket:is_open() -> boolean

-- Возвращает адрес и порт, на которые привязан сокет
socket:get_address() -> string, int
```

```lua
network.udp_open(
	port: int,
	-- Функция, вызываемая при получении датаграмы
	-- В параметры передаётся адрес и порт отправителя, а также сами данные
	datagramHandler: function(address: string, port: int, data: Bytearray, server: DatagramServerSocket)
) -> DatagramServerSocket
```

Открывает UDP-сервер на указанном порту

Класс DatagramServerSocket имеет следующие методы:

```lua
-- Отправляет датаграмму на переданный адрес и порт
server:send(address: string, port: int, data: table|Bytearray|string)

-- Завершает принятие датаграмм
server:stop()

-- Проверяет возможность принятия датаграмм
server:is_open() -> boolean

-- Возвращает порт, который слушает сервер
server:get_port() -> int
```

## HTTP-Сервер

```lua
-- Открывает HTTP-сервер на указанном порту.
network.http_open(
    -- Порт
    port: int,
    -- Функция-обработчик HTTP запроса
    handler: function(request),
    -- Сколько ждать вызова request:respond(...), прежде чем
    -- автоматически отправить 503. 0 означает ждать бесконечно.
    [опционально] timeout_ms: int = 60000
) -> ServerSocket
```

Класс ServerSocket у HTTP-сервера идентичен классу TCP-сервера

Ответить на запрос `handler` может двумя способами:

* вернуть таблицу ответа `{status: int, headers: table<string,string>, body: string}`
  (любое поле можно опустить; `status` по умолчанию равен `200`);
* или самостоятельно вызвать `request:respond(status, body, headers)`,
  например из корутины, для отложенного ответа. В этом случае возвращаемое
  значение обработчика игнорируется.

Если обработчик выбросил ошибку или не ответил в течение `timeout_ms`
(по умолчанию 60 секунд), автоматически отправляется `Service Unavailable` (503).

Класс `request` содержит следующие поля и методы:

```lua
request.method       -> string, например "GET"
request.path         -> string, декодированный путь без строки запроса
request.query        -> string, необработанная строка запроса (часть после '?', если есть)
request.headers      -> table<string,string>, {["Имя"] = "значение", ...}
request.body         -> string|Bytearray
request.remote_addr  -> string
request.remote_port  -> int

-- Разбирает тело запроса как JSON.
request:json() -> any

-- Отправляет ответ. Может быть вызвана не более одного раза, из любого
-- места (в том числе из корутины, из callback'а `on_response` и т.д.)
request:respond(
    [опционально] status: int=200,
    [опционально] body: string|Bytearray,
    [опционально] headers: table<string,string>
)

-- Собирает таблицу JSON-ответа, готовую для возврата из обработчика.
network.http_json(
    data: any,
    [опционально] status: int=200,
    [опционально] headers: table<string,string>
) -> table
```

### Роутер

Для маршрутизации по URL есть небольшой помощник `network.http_router()`.
Сегменты пути с префиксом `:` захватываются и передаются в обработчик по
порядку.

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

Методы `Router`: `get`, `post`, `put`, `delete`, `patch`, а также общий
`route(method, path, handler)`. На запросы, для которых не нашлось
маршрута, отправляется `404 Not Found`.

### Пример

```lua
network.http_open(8080, function(request)
    if request.method == "GET" and request.path == "/status" then
        return network.http_json({ok = true, uptime = time.uptime()})
    end
    return {status = 404, body = "Not Found"}
end)
```

> HTTP-сервер поддерживает тела запросов и ответов HTTP/1.1 с заголовком
> `Content-Length`; тела запросов с `chunked`-кодировкой не поддерживаются
> и отклоняются с кодом `501 Not Implemented`. После каждого ответа
> соединение закрывается (без keep-alive).

## Аналитика

```lua
-- Возвращает приблизительный объем отправленных данных (включая соединения с localhost)
-- в байтах.
network.get_total_upload() -> int
-- Возвращает приблизительный объем полученных данных (включая соединения с localhost)
-- в байтах.
network.get_total_download() -> int
```

## Другое

```lua
-- Ищет свободный для использования порт.
network.find_free_port() -> int | nil
```
