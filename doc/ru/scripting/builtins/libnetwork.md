# Библиотека *network*

Библиотека для работы с сетью.

## HTTP-Запросы

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