#!/usr/bin/env bash

DYLIB_DIR="/usr/lib"

COMPILER="/usr/bin/c++"
CPP_FLAGS="-std=c++14 -Wall -Wextra -O3"
LD_FLAGS="-rdynamic"

BIN_DIR="bin"

INC_DIRS="-isystem /usr/include/eigen3 -isystem $HOME/.local/include -isystem /usr/include/bullet"
LIB_DIRS=""
LIBS="/usr/lib/libdart.so.6.3.0 -lassimp -lboost_system -lBulletCollision -lLinearMath -ldart-collision-bullet"

export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$DYLIB_DIR


function erun () {
    echo $@
    $@
}


function compile () {
    if [ -z "$2" ]; then
        DEF=""
    else
        DEF="-D$2"
    fi
    erun $COMPILER $CPP_FLAGS $LD_FLAGS $INC_DIRS $LIB_DIRS $LIBS $1/$1.cpp -o $BIN_DIR/$1 $DEF
}

function build () {
    if [ -d "$1" ]; then
        compile "$1" "$2"
    else
        printf "Error: could not find \"""$1""\"\n"
    fi
}

function run () {
    if [ -f "$BIN_DIR"/"$1" ]; then
        "$BIN_DIR"/"$1"
    else
        printf "Error: could not find \"""$BIN_DIR"/"$1""\"\n"
    fi
}

# Create a bin directory
mkdir -p $BIN_DIR


case "$1" in
    clean )
        rm -rf "$BIN_DIR"
        ;;

    build )
        build ${2%/} "$3"
        ;;

    run )
        run ${2%/}
        ;;
    brun )
        printf "*** Building ***\n"
        build ${2%/} "$3"

        if [ $? -eq 0 ]; then
            printf "*** Running ***\n"
            run ${2%/}
        fi
        ;;

    * )
        printf "Error: unknown command \"""$1""\"\n"
        ;;
esac


