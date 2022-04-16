#!/bin/bash 
set -e

cd build
cmake ..
make -j8
#gdb --args ./ba_demo 2 0.1 1
#./ba_demo 0.5 0.05 1
./ba_demo 1 0.1 1
#./ba_demo 0 0 1
#./ba_demo 0 0 0
