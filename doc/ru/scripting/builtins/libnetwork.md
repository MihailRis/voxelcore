# Библиотека *network*

Библиотека для работы с сетью.

## HTTP-Запросы

```lua
-- Выполняет GET запрос к указанному URL.
network.get(
    url: str,
    -- Функция, вызываемая при получении ответа
    callback: function(str),
    -- Обработчик ошибок
    [опционально] onfailure: function(int, str),
    -- Список дополнительных заголовков запроса
    [опционально] headers: table<str>
)

-- Пример:
network.get("https://api.github.com/repos/MihailRis/VoxelEngine-Cpp/releases/latest", function (s)
    print(json.parse(s).name) -- выведет имя последнего релиза движка
end)

-- Вариант для двоичных файлов, с массивом байт вместо строки в ответе.
network.get_binary(
    url: str,
    callback: function(ByteArray),
    [опционально] onfailure: function(int, Bytearray),
    [опционально] headers: table<str>
)

-- Выполняет POST запрос к указанному URL.
-- На данный момент реализована поддержка только `Content-Type: application/json`
-- После получения ответа, передаёт текст в функцию callback.
-- В случае ошибки в onfailure будет передан HTTP-код ответа.
network.post(
    url: str,
    -- Тело запроса в виде таблицы, конвертируемой в JSON или строки
    body: table|str,
    -- Функция, вызываемая при получении ответа
    callback: function(str),
    -- Обработчик ошибок
    [опционально] onfailure: function(int, str),
    -- Список дополнительных заголовков запроса
    [опционально] headers: table<str>
)
```

## TCP-Соединения

```lua
network.tcp_connect(
    -- Адрес
    address: str,
    -- Порт
    port: int,
    -- Функция, вызываемая при успешном подключении
    -- До подключения отправка работать не будет
    -- Как единственный аргумент передаётся сокет
    callback: function(Socket)
) --> Socket
```

Инициирует TCP подключение.

Класс Socket имеет следующие методы:

```lua
-- Отправляет массив байт
socket:send(table|ByteArray|str)

-- Читает полученные данные
socket:recv(
    -- Максимальный размер читаемого массива байт
    length: int, 
    -- Использовать таблицу вместо Bytearray
    [опционально] usetable: bool=false
) -> nil|table|Bytearray
-- В случае ошибки возвращает nil (сокет закрыт или несуществует).
-- Если данных пока нет, возвращает пустой массив байт.

-- Закрывает соединение
socket:close()

-- Возвращает количество доступных для чтения байт данных
socket:available() --> int

-- Проверяет, что сокет существует и не закрыт.
socket:is_alive() --> bool

-- Проверяет наличие соединения (доступно использование socket:send(...)).
socket:is_connected() --> bool

-- Возвращает адрес и порт соединения.
socket:get_address() --> str, int
```

```lua
-- Открывает TCP-сервер.
network.tcp_open(
    -- Порт
    port: int,
    -- Функция, вызываемая при поключениях
    -- Как единственный аргумент передаётся сокет подключенного клиента
    callback: function(Socket)
) --> ServerSocket
```

Класс SocketServer имеет следующие методы:

```lua
-- Закрывает сервер, разрывая соединения с клиентами.
server:close()

-- Проверяет, существует и открыт ли TCP сервер.
server:is_open() --> bool

-- Возвращает порт сервера.
server:get_port() --> int
```

## UDP-Датаграммы

```lua
network.udp_connect(
	address: str,
	port: int,
    -- Функция, вызываемая при получении датаграммы с указанного при открытии сокета адреса и порта
	datagramHandler: function(Bytearray),
	-- Функция, вызываемая после открытия сокета
	-- Опциональна, так как в UDP нет handshake
    [опционально] openCallback: function(WriteableSocket),
) --> WriteableSocket
```

Открывает UDP-сокет с привязкой к удалённому адресу и порту

Класс WriteableSocket имеет следующие методы:

```lua
-- Отправляет датаграмму на адрес и порт, заданные при открытии сокета
socket:send(table|Bytearray|str)

-- Закрывает сокет
socket:close()

-- Проверяет открыт ли сокет
socket:is_open() --> bool

-- Возвращает адрес и порт, на которые привязан сокет
socket:get_address() --> str, int
```

```lua
network.udp_open(
	port: int,
	-- Функция, вызываемая при получении датаграмы
	-- В параметры передаётся адрес и порт отправителя, а также сами данные
	datagramHandler: function(address: str, port: int, data: Bytearray, server: DatagramServerSocket)
) --> DatagramServerSocket
```

Открывает UDP-сервер на указанном порту

Класс DatagramServerSocket имеет следующие методы:

```lua
-- Отправляет датаграмму на переданный адрес и порт
server:send(address: str, port: int, data: table|Bytearray|str)

-- Завершает принятие датаграмм
server:stop()

-- Проверяет возможность принятия датаграмм
server:is_open() --> bool

-- Возвращает порт, который слушает сервер
server:get_port() --> int
```

## Аналитика

```lua
-- Возвращает приблизительный объем отправленных данных (включая соединения с localhost)
-- в байтах.
network.get_total_upload() --> int
-- Возвращает приблизительный объем полученных данных (включая соединения с localhost)
-- в байтах.
network.get_total_download() --> int
```
