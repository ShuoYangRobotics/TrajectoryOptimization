#!/bin/bash

currentdir=`pwd`

cd /home/biorobotics/optimization/robcogen/robcogen-0.5.1/exe/

printf '1\n4\n23\n28\n' |./robcogen.sh ${currentdir}/acrobot.kindsl ${currentdir}/acrobot.dtdsl

cd ${currentdir}

cp -r /tmp/gen .

cd gen/cpp

mkdir build

cd build

cmake -D EIGEN_ROOT=/usr/include/eigen3/ ..

make

sudo make install