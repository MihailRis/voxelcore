# Библиотека *pack*

```lua
pack.is_installed(packid: string) -> boolean
```

Проверяет наличие установленного пака в мире

```lua
pack.data_file(packid: string, filename: string) -> string
-- и
pack.shared_file(packid: string, filename: string) -> string
```

Возвращает путь к файлу данных 
и создает недостающие директории в пути.

- Первый вариант возвращает: `world:data/packid/filename`
- Второй вариант возвращает: `config:packid/filename`

Примеры:
```lua
file.write(pack.data_file(PACK_ID, "example.txt"), text)
```
Для пака *containermod* запишет текст в файл `world:data/containermod/example.txt`.

Используйте для хранения данных в мире.

```lua
file.write(pack.shared_file(PACK_ID, "example.txt"), text)
```
Для пака *containermod* запишет текст в файл `config:containermod/example.txt`

Используйте для хранения данных общих для всех миров.

```lua
pack.get_folder(packid: string) -> string
```

Возвращает путь к папке установленного контент-пака.

```lua
pack.is_installed(packid: string) -> boolean
```

Проверяет наличие контент-пака в мире

```lua
pack.get_installed() -> table<string>
```

Возращает id всех установленных в мире контент-паков.

```lua
pack.get_available() -> table<string>
```

Возвращает id всех доступных, но не установленных в мире контент-паков.

```lua
pack.get_base_packs() -> table<string>
```

Возвращает id всех базовых паков (неудаляемых)

```lua
pack.get_info(packid: string) -> {
	id: string,
	title: string,
	creator: string,
	description: string,
	version: string,
    path: string,
	icon: string, -- отсутствует в headless режиме
	dependencies: table<string> -- опциональный
}
```

Возвращает информацию о паке (не обязательно установленном).
- icon - название текстуры предпросмотра (загружается автоматически)
- dependencies - строки в формате `{lvl}{id}`, где lvl:
	- `!` - required
	- `?` - optional
	- `~` - weak
    
	например `!teal`

Для получения информации о нескольких паках используйте таблицу id, чтобы не
производить сканирование для каждого пака:

```lua
pack.get_info(packids: table<string>) -> table<string, table>
```

```lua
pack.assemble(packids: table<string>) -> table<string>
```

Проверяет корректность конфигурации и добавляет зависимости, возвращая полную конфигурацию.

```lua
pack.request_writeable(packid: string, callback: function(string))
```

Запрашивает у пользователя право на модификацию пака. При подтвержении новая точка входа будет передана в callback.
