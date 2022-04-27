#!/bin/bash 
set -e

sh compile.sh
cd build
rm -f input
ln -s ../input .
./ba_demo 1 0.1 1

#gdb --args ./ba_demo 2 0.1 1
#./ba_demo 0 0 1
#./ba_demo 0 0 0
