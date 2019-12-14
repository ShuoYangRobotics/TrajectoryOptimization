TrajectoryOptimization

This repo will host my research code for studying trajectory optimization for different robotics systems.

My initial plan would be implement paper "trajectory optimization with implicit hard contacts" and study three tools mentioned in this paper:

1. [RobCoGen](https://robcogenteam.bitbucket.io/cpp.html)
2. [CppAD](https://coin-or.github.io/CppAD/doc/cppad.htm)
3. [Control Toolbox](https://ethz-adrl.github.io/ct/ct_doc/doc/html/index.html)

## acrobot
First system: acrobot with direct collocation

acrobot/model/run.sh contains a script that runs RobCoGen, copy the output to current folder and install necessary robot header and .so into /usr/local/include

RobCoGen has an option to generate URDF model, but the output model does not have visual component.

acrobot/test.cpp is used to test the model

To build the test.cpp, some cares need to be taken when writting the CMakeLists file to handle RPATH and eigen3 path.

## flickingFinger 

This is a implementation of trajectory generation algorithm described in paper "A Direct Method for Trajectory Optimization of Rigid Bodies Through Contact". 

## ifopt
Snopt is a closed source library written in Fortran, it has c/c++/matlab interfaces. Even c/c++ interface makes snopt easier to use, the problem construction is still cumbersome. Therefore, researchers at ETH made a tool called ifopt that provides an additional layer to allow user construct problems using Eigen linear algebra library, and then convert the problem into snopt. This folder contains some test code to use ifopt.

## acrobotOpt

After generating dynamic model of acrobot, install the model into /usr/local and install ifopt properly. Next step is to use direct collocation to generate trajectory for acrobot.