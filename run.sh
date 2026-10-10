#!/bin/bash



function delete {
    echo "[RUN SCRIPT] Delete build directory"
    rm -rf build
}


function build {
    echo "[RUN SCRIPT] Build project"
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build . -j$(nproc)
    cd ..
}


function rebuild {
    delete
    build
}


run=true
function norun {
    echo "[RUN SCRIPT] Build without run"
    run=
}


function test {
    echo "[RUN SCRIPT] Run tests"
    cd build
    rm -rf ./vctest-run
    mkdir ./vctest-run
    ./vctest/vctest --exe ./VoxelEngine --res res --user ./vctest-run --tests ../dev/tests --memchecker valgrind
    cd ..
    run=
}


function help {
    echo "[RUN SCRIPT] Usage: ./run [ARGUMENT]..."
    echo "[RUN SCRIPT] Arguments:"
    echo "[RUN SCRIPT]     -d, --delete     Delete build directory"
    echo "[RUN SCRIPT]     -b, --build      Build project"
    echo "[RUN SCRIPT]     -r, --rebuild    Rebuild project"
    echo "[RUN SCRIPT]     -R, --norun      Build without run"
    echo "[RUN SCRIPT]     -t, --test       Run tests (without run)"
    echo "[RUN SCRIPT]     -h, --help       Print this page"
}


while [ -n "$1" ]; do
    case "$1" in
        -d | --delete) delete ;;
        -b | --build) build ;;
        -r | --rebuild) rebuild ;;
        -R | --norun) norun ;;
        -t | --test) test ;;
        -h | --help) help
                     norun
                     break ;;
        *) echo "[RUN SCRIPT] Unknown argument: $1"
           help
           norun
           break ;;
    esac
    shift
done


if [[ $run ]]; then
    echo "[RUN SCRIPT] Run project"
    ./build/VoxelCore
fi
