# ![voxel-core](dev/VoxelCore.png) VoxelCore

## Latest release

- [Download](https://github.com/MihailRis/VoxelCore/releases/latest) | [Скачать](https://github.com/MihailRis/VoxelCore/releases/latest)
- [Documentation](https://github.com/MihailRis/VoxelCore/blob/release-0.28/doc/en/main-page.md) | [Документация](https://github.com/MihailRis/VoxelCore/blob/release-0.28/doc/ru/main-page.md)

---

## Build project in Linux

### Install libraries

#### Install EnTT

```sh
git clone https://github.com/skypjack/entt.git
cd entt
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENTT_INSTALL=ON ..
sudo make install
```

> [!WARNING]
> If you are using ALT Linux, do **not** use this EnTT installation method.

#### ALT Linux based distros

```sh
su -
apt-get install entt-devel libglfw3-devel libGLEW-devel libglm-devel libpng-devel libvorbis-devel libopenal-devel libluajit-devel libstdc++13-devel-static libcurl-devel
```

#### Debian based distros

```sh
sudo apt install libglfw3 libglfw3-dev libglew-dev libglm-dev libpng-dev libopenal-dev libluajit-5.1-dev libvorbis-dev libcurl4-openssl-dev
```

> [!TIP]
> CMake missing `LUA_INCLUDE_DIR` and `LUA_LIBRARIES` fix:
>
> ```sh
> sudo ln -s /usr/lib/x86_64-linux-gnu/libluajit-5.1.a /usr/lib/x86_64-linux-gnu/liblua5.1.a
> sudo ln -s /usr/include/luajit-2.1 /usr/include/lua
> ```

#### RHEL based distros

```sh
sudo dnf install glfw-devel glew-devel glm-devel libpng-devel libvorbis-devel openal-soft-devel luajit-devel libcurl-devel
```

#### Arch based distros

If you use X11:

```sh
sudo pacman -S glfw-x11 glew glm libpng libvorbis openal luajit libcurl
```

If you use Wayland:

```sh
sudo pacman -S glfw-wayland glew glm libpng libvorbis openal luajit libcurl
```

And install EnTT:

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
cmake --build . --parallel
```

> [!TIP]
> Use `--parallel` to utilize all CPU cores during build.

---

## Building project in macOS

### Install libraries

```sh
brew install glfw3 glew glm libpng libvorbis lua luajit libcurl openal-soft skypjack/entt/entt
```

> [!TIP]
> If Homebrew fails to install `lua`, `luajit`, or `openal-soft`, download, compile, and install them manually.

### Building engine with CMake

```sh
git clone --recursive https://github.com/MihailRis/VoxelCore.git
cd VoxelCore
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --parallel
```

---

## Building in Windows

> [!NOTE]
> Requirements: **vcpkg**, **CMake**, **Git**, and **Visual Studio** (with C++ tools).

There are two options to use vcpkg:

1. If you have Visual Studio installed, the **VCPKG_ROOT** environment variable is often already set in the **Developer Command Prompt for VS**.
2. Otherwise, install **vcpkg** manually:

```powershell
cd C:\
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat
```

Then set the `VCPKG_ROOT` environment variable and add it to `PATH`:

```powershell
$env:VCPKG_ROOT = "C:\vcpkg"
$env:PATH = "$env:VCPKG_ROOT;$env:PATH"
```

> [!TIP]
> For troubleshooting, refer to the official [vcpkg documentation](https://learn.microsoft.com/ru-ru/vcpkg/get_started/get-started?pivots=shell-powershell).

After installing **vcpkg**, build the project:

```powershell
git clone --recursive https://github.com/MihailRis/VoxelCore.git
cd VoxelCore
cmake --preset default-vs-msvc-windows
cmake --build --preset default-vs-msvc-windows
```

> [!NOTE]
> Make sure your `CMakeUserPresets.json` (if used) contains the correct `VCPKG_ROOT` path.

---

## Build using Docker

> [!NOTE]
> First, install Docker Engine: [https://docs.docker.com/engine/install](https://docs.docker.com/engine/install)

### On Linux

#### Step 1. Build Docker image

```sh
docker build -t voxel-engine .
```

#### Step 2. Build project inside container

```sh
docker run --rm -it -v "$(pwd):/project" voxel-engine bash -c "cmake -DCMAKE_BUILD_TYPE=Release -Bbuild && cmake --build build --parallel"
```

#### Step 3. Run the application (requires X11 forwarding)

```sh
docker run --rm -it \
  -v "$(pwd):/project" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$XAUTHORITY:/home/user/.Xauthority:ro" \
  -e DISPLAY="$DISPLAY" \
  --network=host \
  voxel-engine ./build/VoxelEngine
```

### On Windows

> [!NOTE]
> You need an X server like **VcXsrv** to display the GUI.

#### Step 1. Install and run VcXsrv

Launch with:
```powershell
.\vcxsrv.exe :0 -multiwindow -ac
```

#### Step 2. Build Docker image

```powershell
docker build -t voxel-engine .
```

#### Step 3. Build project

```powershell
docker run --rm -it -v "${PWD}:/project" voxel-engine bash -c "cmake -DCMAKE_BUILD_TYPE=Release -Bbuild && cmake --build build --parallel"
```

#### Step 4. Run the application

```powershell
docker run --rm -it -v "${PWD}:/project" -e DISPLAY=host.docker.internal:0.0 --network=host voxel-engine ./build/VoxelEngine
```
