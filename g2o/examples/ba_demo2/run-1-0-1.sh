#!/bin/bash 
set -e

sh compile.sh
cd build
rm -f input
ln -s ../input .
./ba_demo	1	0	1	0	0	0
./ba_demo	1	0	1	0	0	1
./ba_demo	1	0	1	0	0	2
./ba_demo	1	0	1	0	0	3
./ba_demo	1	0	1	0	0	4
./ba_demo	1	0	1	0	0	5
./ba_demo	1	0	1	0	0	6
./ba_demo	1	0	1	0	0	7
./ba_demo	1	0	1	0	0	8
./ba_demo	1	0	1	0	0	9
./ba_demo	1	0	1	0	0	10
./ba_demo	1	0	1	0	0	11
./ba_demo	1	0	1	0	0	12
./ba_demo	1	0	1	0	0	13
./ba_demo	1	0	1	0	0	14
./ba_demo	1	0	1	0	0	15
./ba_demo	1	0	1	0	0	16
./ba_demo	1	0	1	0	0	17
./ba_demo	1	0	1	0	0	18
./ba_demo	1	0	1	0	0	19
./ba_demo	1	0	1	0	0	20
./ba_demo	1	0	1	0	0	21
./ba_demo	1	0	1	0	0	22
./ba_demo	1	0	1	0	0	23
./ba_demo	1	0	1	0	0	24
./ba_demo	1	0	1	0	0	25
./ba_demo	1	0	1	0	0	26
./ba_demo	1	0	1	0	0	27
./ba_demo	1	0	1	0	0	28

#gdb --args ./ba_demo 2 0.1 1
#./ba_demo 0 0 1
#./ba_demo 0 0 0
