# ![voxel-core](dev/VoxelCore.png) VoxelCore

## Latest release

- [Download](https://github.com/MihailRis/VoxelCore/releases/latest) | [Скачать](https://github.com/MihailRis/VoxelCore/releases/latest)
- [Documentation](https://github.com/MihailRis/VoxelCore/blob/release-0.28/doc/en/main-page.md) | [Документация](https://github.com/MihailRis/VoxelCore/blob/release-0.28/doc/ru/main-page.md)

## Build project in Linux

### Install libraries

#### Install EnTT

```sh
git clone https://github.com/skypjack/entt.git
cd entt/build
cmake -DCMAKE_BUILD_TYPE=Release -DENTT_INSTALL=on ..
sudo make install
```

> [!WARNING]
> If you are using ALT Linux, you should not use this EnTT installation method

#### ALT Linux based distros

```sh
su -
apt-get install entt-devel libglfw3-devel libGLEW-devel libglm-devel libpng-devel libvorbis-devel libopenal-devel libluajit-devel libstdc++13-devel-static libcurl-devel
```

#### Debian based distros

```sh
sudo apt install libglfw3-dev libglfw3 libglew-dev libglm-dev libpng-dev libopenal-dev libluajit-5.1-dev libvorbis-dev libcurl4-openssl-dev
```

> [!TIP]
> CMake missing LUA_INCLUDE_DIR and LUA_LIBRARIES fix:
>
> ```sh
> sudo ln -s /usr/lib/x86_64-linux-gnu/libluajit-5.1.a /usr/lib/x86_64-linux-gnu/liblua5.1.a
> sudo ln -s /usr/include/luajit-2.1 /usr/include/lua
> ```

#### RHEL based distros

```sh
sudo dnf install glfw-devel glfw glew-devel glm-devel libpng-devel libvorbis-devel openal-soft-devel luajit-devel libcurl-devel
```

#### Arch based distros

If you use X11

```sh
sudo pacman -S glfw-x11 glew glm libpng libvorbis openal luajit libcurl
```

If you use Wayland

```sh
sudo pacman -S glfw-wayland glew glm libpng libvorbis openal luajit libcurl
```

And you need entt. In yay you can use

```sh
yay -S entt
```

### Building engine with CMake

```sh
git clone --recursive https://github.com/MihailRis/VoxelCore.git
cd VoxelCore
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

## Building project in macOS

### Install libraries

```sh
brew install glfw3 glew glm libpng libvorbis lua luajit libcurl openal-soft skypjack/entt/entt
```

> [!TIP]
> If homebrew for some reason could not install the necessary packages:
> ```lua luajit openal-soft```, then download, install and compile them manually
> (Lua, LuaJIT and OpenAL).

### Building engine with CMake

```sh
git clone --recursive https://github.com/MihailRis/VoxelCore.git
cd VoxelCore
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

## Building in Windows

>[!NOTE]
> Requirement:
>
> vcpkg, CMake, Git
There are two options to use vcpkg:
1. If you have Visual Studio installed, most likely the **VCPKG_ROOT** environment variable will already exist in **Developer Command Prompt for VS**
2. If you want use **vcpkg**, install **vcpkg** from git to you system:
```PowerShell
cd C:/
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```
After installing **vcpkg**, setup env variable **VCPKG_ROOT** and add it to **PATH**:
```PowerShell
$env:VCPKG_ROOT = "C:\path\to\vcpkg"
$env:PATH = "$env:VCPKG_ROOT;$env:PATH"
```
>[!TIP]
>For troubleshooting you can read full [documentation](https://learn.microsoft.com/ru-ru/vcpkg/get_started/get-started?pivots=shell-powershell) for **vcpkg**

After installing **vcpkg** you can build project:
```PowerShell
git clone --recursive https://github.com/MihailRis/VoxelCore.git
cd VoxelCore
cmake --preset default-vs-msvc-windows
cmake --build --preset default-vs-msvc-windows
```

## Build using Docker

### Step 0. Install docker on your system

See <https://docs.docker.com/engine/install>

### Do you have Linux

### Step 1. Build docker container

```sh
docker build -t voxel-engine .
```

### Step 2. Build project using the docker container

```sh
docker run --rm -it -v$(pwd):/project voxel-engine bash -c "cmake -DCMAKE_BUILD_TYPE=Release -Bbuild && cmake --build build"
```

### Step 3. Run project using the docker container

```sh
docker run --rm -it -v$(pwd):/project -v/tmp/.X11-unix:/tmp/.X11-unix -v${XAUTHORITY}:/home/user/.Xauthority:ro -eDISPLAY --network=host voxel-engine ./build/VoxelEngine
```

### Do you have Windows

### Step 1. You need to install VcXsrv

### Step 2. Run VcXsrv with the command

```powershell
.\vcxsrv.exe :0 -multiwindow -ac
```

### Step 3. Build docker container

```powershell
docker build -t voxel-engine .
```

### Step 4. Build project using the docker container

```powershell
docker run --rm -it -v "${PWD}:/project" voxel-engine bash -c "cmake -DCMAKE_BUILD_TYPE=Release -Bbuild && cmake --build build"
```

### Step 5. Run project using the docker container

```powershell
docker run --rm -it -v "${PWD}:/project" -e DISPLAY=host.docker.internal:0.0 --network host voxel-engine ./build/VoxelEngine
```
