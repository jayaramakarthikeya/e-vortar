# E-Vortar

In order to run our code please install pre requirements as list below:
All the code was built and tested on Ubuntu 20.04 machine.

- ROS2 Foxy (Full build: Including Rviz)
- Ceres Solver == 2.0.0
- OpenCV >= 4.2.0
- Boost (C++14 version)

## Installing OpenCV on ubuntu 20.04

```
sudo apt update && sudo apt install -y cmake g++ wget unzip
 
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip
 
# Create build directory and switch into it
mkdir -p build && cd build
 
# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x
 
# Build
cmake --build .
```



## Installing Ceres

I installed the [latest stable release](http://ceres-solver.org/ceres-solver-2.0.0.tar.gz) as mentioned in http://ceres-solver.org/installation.html. At step "make test", my computer got stuck at step 31 "dynamic_sparsity_test". This happened to both latest stable release and latest github repository. This bug was ignored, and I went to the next command "make install".

Prerequisites Install:
```
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev
```

Install Ceres:
```
tar zxf ceres-solver-2.0.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-2.0.0
make -j4
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
make install
```


## Building

After Successful installation we can processed building the code as follows:
```
mkdir ~/project_ws/src
cd ~/project_ws/src
git clone https://github.com/jayaramakarthikeya/e-vortar.git
cd ..
colcon build
```

In order to run just the kitti streamer we can run:
```
ros2 launch kitti_streamer kitti_streamer_node.launch.py
```



In order to run the full pipeline please run:
```
ros2 launch visual_odometry visual_odom.launch.py
```
Contact if there is any problem to:\
[jkarthik@seas.upenn.edu](jkarthik@seas.upenn.edu)\
[aakash24@seas.upenn.edu](aakash24@seas.upenn.edu)
