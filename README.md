# E-Vortar

In order to run our code please install pre requirements as list below:
All the code was built and tested on Ubuntu 20.04 machine.

- ROS2 Foxy (Full build: Including Rviz)
- Ceres Solver == 2.0.0
- OpenCV >= 4.2.0

## Installing Ceres

I installed the [latest stable release](http://ceres-solver.org/ceres-solver-2.0.0.tar.gz) as mentioned in http://ceres-solver.org/installation.html. At step "make test", my computer got stuck at step 31 "dynamic_sparsity_test". This happened to both latest stable release and latest github repository. This bug was ignored, and I went to the next command "make install".

