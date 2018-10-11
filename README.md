# ekf_bayesopt
Kalman Filter tuning using Bayesian Optimization

## Introduction
This package is written based on Prof.Nisar's matlab code for skyCrane machine and Prof.Simon's KF code.
This package will run bayesian opimization to optimize skyCrane's process noise or measurement or all of them.
EKF is implemented and the cost to bayesian optimization is J_NEES or J_NIS.

This system is also build on ROS so you also need install ROS.

## Install
### Install bayesian optimization
bayesian optimization is from its official lib.
```
sudo apt-get install libboost-dev python-dev python-numpy cmake cmake-curses-gui g++ cython octave-headers freeglut3-dev
git clone --single-branch -b bayesopt https://github.com/arpg/ekf_bayesopt.git
cd ekf_bayesopt/bayesopt
cmake .
make
sudo make install
```
Deatails can be seen at https://rmcantin.bitbucket.io/html/install.html

### Install ROS
recomment ROS kinetic version, please see http://wiki.ros.org/kinetic/Installation/Ubuntu 
### Install Eigen3
https://launchpad.net/ubuntu/+source/eigen3
### Install the bayesopt_ros package
```
git clone git://github.com/headmyshoulder/odeint-v2
```
This is ode integration lib using C++. Ode45 is used in matlab code, this lib can work the same as it. Just download it, you don't need build it. Details can be seen http://headmyshoulder.github.io/odeint-v2/index.html

```
git clone --single-branch -b master https://github.com/arpg/ekf_bayesopt.git
cp ekf_bayesopt/bayesopt_ros ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

