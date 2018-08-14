#!/usr/bin/env bash

DYLIB_DIR="/usr/lib:/usr/lib/x86_64-linux-gnu/"

COMPILER="/usr/bin/c++"
CPP_FLAGS="-std=c++14 -Wall -Wextra -O3"
LD_FLAGS="-rdynamic"

BIN_DIR="bin"

INC_DIRS="-isystem /usr/include/eigen3 -isystem $HOME/.local/include -isystem /usr/include/bullet"
LIB_DIRS=""
# LIBS="/usr/lib/libdart.so.6.3.0 -lassimp -lboost_system -lBulletCollision -lLinearMath -ldart-collision-bullet"
LIBS="/usr/lib/libdart.so.6.4.0 \
      -lassimp -lboost_system \
      /usr/lib/x86_64-linux-gnu/libBulletCollision.so.2.83 \
      /usr/lib/x86_64-linux-gnu/libLinearMath.so.2.83 \
      /usr/lib/libdart-collision-bullet.so.6.4.0"

export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$DYLIB_DIR


function erun () {
    echo $@
    $@
}


function compile () {
    if [ -z "$2" ]; then
        DEF=""
        BIN_NAME=$BIN_DIR/$1
    else
        DEF="-D$2"
        BIN_NAME=$BIN_DIR/$1_vis
    fi


    # /usr/bin/c++ -isystem /usr/include/eigen3 -isystem $HOME/.local/include -isystem /usr/include/bullet -std=c++14 -Wall -Wextra -O3 -o ugv_fsm.cpp.o -c ugv_fsm.cpp
    # /usr/bin/c++ ugv_fsm.cpp.o -o ugv_fsm -rdynamic /usr/lib/libdart.so.6.3.0 -lassimp -lboost_system -lBulletCollision -lLinearMath -ldart-collision-bullet


    erun $COMPILER $INC_DIRS $CPP_FLAGS -o $BIN_NAME".cpp.o" -c $1/$1".cpp" $DEF
    erun $COMPILER $BIN_NAME".cpp.o" -o $BIN_NAME $LD_FLAGS $LIBS
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

    all )
        rm -rf "$BIN_DIR"
        mkdir -p $BIN_DIR

        build "ugv_fsm"
        build "ugv_fsm" "VISUALIZE"
        build "ugv_bnn"
        build "ugv_bnn" "VISUALIZE"
        build "ugv_bnn_twist"
        build "ugv_bnn_twist" "VISUALIZE"
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


