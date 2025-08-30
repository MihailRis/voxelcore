# Build docker container: docker build -t voxelcore .
# Build project: docker run --rm -it -v$(pwd):/project voxelcore bash -c "cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=TRUE -Bbuild && cmake --build build"
# Run project in docker: docker run --rm -it -v$(pwd):/project -v/tmp/.X11-unix:/tmp/.X11-unix -v${XAUTHORITY}:/home/user/.Xauthority:ro -eDISPLAY --network=host voxelcore ./build/VoxelEngine

FROM debian:bookworm-slim
LABEL Description="Docker container for building VoxelCore for Linux"

# Install dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    g++ \
    make \
    cmake \
    pkg-config \
    xauth \
    gdb \
    gdbserver \
    libglfw3-dev \
    libglfw3 \
    libglew-dev \
    libglew2.2 \
    libglm-dev \
    libpng-dev \
    libopenal-dev \
    libluajit-5.1-dev \
    libvorbis-dev \
    libcurl4-openssl-dev \
    ca-certificates \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Установка CMake >= 3.26 вручную
ARG CMAKE_VERSION=3.27.9
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.sh && \
    chmod +x cmake-${CMAKE_VERSION}-linux-x86_64.sh && \
    ./cmake-${CMAKE_VERSION}-linux-x86_64.sh --skip-license --prefix=/usr/local && \
    rm cmake-${CMAKE_VERSION}-linux-x86_64.sh


# Install EnTT
RUN git clone https://github.com/skypjack/entt.git && \
    cd entt/build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DENTT_INSTALL=on .. && \
    make install && \
    cd ../.. && rm -rf entt


# CMake missing LUA_INCLUDE_DIR and LUA_LIBRARIES fix:
RUN ln -s /usr/lib/x86_64-linux-gnu/libluajit-5.1.a /usr/lib/x86_64-linux-gnu/liblua5.1.a \
    && ln -s /usr/include/luajit-2.1 /usr/include/lua

# Install LuaJIT:
RUN git clone https://luajit.org/git/luajit.git \
    && cd luajit \
    && make && make install INSTALL_INC=/usr/include/lua \
    && cd .. && rm -rf luajit

# Create default user, due to:
#   - Build and test artifacts have user permissions and not root permissions
#   - Don't give root privileges from host to containers (security)
ARG USER=user
ARG UID=1000
ARG GID=1000
RUN useradd -m ${USER} --uid=${UID}
USER ${UID}:${GID}

# Project workspace
WORKDIR /project
