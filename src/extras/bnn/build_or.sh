#!/usr/bin/env bash

COMPILER="clang++"
CPP_FLAGS="-std=c++14 -Wall -Wextra -O3"
INC_DIRS="-isystem/usr/local/include/eigen3"

$COMPILER $CPP_FLAGS $INC_DIRS or_obj_fcn.cpp -o or_obj_fcn
