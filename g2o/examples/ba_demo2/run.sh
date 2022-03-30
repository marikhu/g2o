#!/bin/bash 
set -e

cd build
cmake ..
make -j8
./ba_demo 2 0.1 1
#./ba_demo 0 0.0 1
