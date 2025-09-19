# Content-packs

Every content pack must have an ID following requirements:
- name can consist of Capital letters A-Z, lowercase letters a-z digits 0-9, and underscore '\_' signs. 
- the first character must not be a digit.
- name length must be in range \[2, 24\]

Content-pack folder having name same as ID may be created in *res/content*.
Content-pack folder must contain file **package.json** with following contents:

```json
{
	"id": "pack_id",
	"title": "pack name will be displayed in the content menu",
	"version": "content-pack version - major.minor",
	"creator": "content-pack creator",
	"description": "short description",
	"dependencies": [
		"pack",
		"dependencies"
	]
}
```

Dependency levels are indicated by prefixes in the name:
- '!' - required dependency
- '?' - optional dependency
- '~' - weak dependency
If prefix is not specified, '!' level will be used.

Example: '~randutil' - weak dependency 'randutil'.

Dependency version is indicated after '@' symbol and have operators to restrict acceptable versions.
If version is not specified, '\*' (any) version will be used.

Example: 'randutil@>=1.0' - dependency 'randutil' which requires version 1.0 or newer.

Example:
```json
{
    "id": "doors",
    "title": "DOORS",
    "creator": "MihailRis",
    "version": "1.0",
    "description": "doors test"
}
```

Content pack picture should be added as *icon.png* file. Recommended size: 128x128

See *res/content/base* as an example of content pack structure.


