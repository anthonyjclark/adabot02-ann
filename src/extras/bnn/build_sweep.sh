#!/usr/bin/env bash

COMPILER="clang++"
CPP_FLAGS="-std=c++14 -Wall -Wextra -O3"
INC_DIRS="-isystem/usr/local/include/eigen3"

$COMPILER $CPP_FLAGS $INC_DIRS bnn_sweep.cpp -o bnn_sweep
