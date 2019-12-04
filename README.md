TrajectoryOptimization

This repo will host my research code for studying trajectory optimization for different robotics systems.

My initial plan would be implement paper "trajectory optimization with implicit hard contacts" and study three tools mentioned in this paper:

1. [RobCoGen](https://robcogenteam.bitbucket.io/cpp.html)
2. [CppAD](https://coin-or.github.io/CppAD/doc/cppad.htm)
3. [Control Toolbox](https://ethz-adrl.github.io/ct/ct_doc/doc/html/index.html)

First system: acrobot with direct collocation

acrobot/model/run.sh contains a script that runs RobCoGen, copy the output to current folder and install necessary robot header and .so into /usr/local/include

RobCoGen has an option to generate URDF model, but the output model does not have visual component.

acrobot/test.cpp is used to test the model

To build the test.cpp, some cares need to be taken when writting the CMakeLists file to handle RPATH and eigen3 path. 