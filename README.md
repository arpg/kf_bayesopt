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
cd ekf_bayesopt
cmake .
make
sudo make install
```
Deatails can be seen at https://rmcantin.bitbucket.io/html/install.html
