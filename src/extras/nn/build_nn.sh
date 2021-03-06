#!/usr/bin/env bash

COMPILER="clang++"
# CPP_FLAGS="-std=c++14 -Wall -Wextra"
CPP_FLAGS="-std=c++14 -Wall -Wextra -O3"
INC_DIRS="-isystem/usr/local/include/eigen3"

$COMPILER $CPP_FLAGS $INC_DIRS nn_test.cpp -o nn_test
