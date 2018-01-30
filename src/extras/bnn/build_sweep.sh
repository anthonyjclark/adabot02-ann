#!/usr/bin/env bash

COMPILER="clang++"
CPP_FLAGS="-std=c++14 -Wall -Wextra -O3"
INC_DIRS="-isystem/usr/local/include/eigen3"

mkdir -p bin

$COMPILER $CPP_FLAGS $INC_DIRS bnn_sweep.cpp -o bin/bnn_sweep
