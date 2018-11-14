# ekf_bayesopt
Kalman Filter tuning using Bayesian Optimization
If you meet any problems on installing or running the code please let me know!
## Introduction
This package is written based on Prof.Nisar's matlab code for skyCrane machine and Prof.Simon's KF code.
This package will run bayesian opimization to optimize skyCrane's process noise or measurement or all of them.
EKF is implemented and the cost to bayesian optimization is J_NEES or J_NIS.

This system is also build on ROS so you also need install ROS. With the help of ROS, bayesian optimization lib and skyCrane lib can communicate with each other more easily. At the same time, all the parameters of bayesian optimization and skyCrane can be modified in the yaml file.
The main structure can be seen from the following figure
![image](https://github.com/arpg/ekf_bayesopt/raw/master/Nodes.png)
skyCrane_EKF node will run simulator and estimator(EKF) accodring to different noise setting. It will output J_NEES or J_NIS(topic "cost") and publish to the skyCrane_bayesopt node. Acoording to the cost, skyCrane_bayesopt will run the bayesian optimization to optimize the cost, then it will generate the noise(topic "noise") to publish to the skyCrane_EKF and set the nosie. Then skyCrane_EKF nose can run the simulator and estimator agian and generate new cost.
To modify the parameters, please go to the `bayesopt_ros/config`. There are two yaml files `bayesParams.yaml` to modify the parameters of bayesopt and `vehicleParams` to modify the parameters of skyCrane. 

## Install
### Install bayesian optimization
bayesian optimization is from its official lib.
```
sudo apt-get install libboost-dev python-dev python-numpy cmake cmake-curses-gui g++ cython octave-headers freeglut3-dev
git clone --single-branch -b bayesopt https://github.com/arpg/ekf_bayesopt.git
cp ekf_bayesopt/bayesopt ~
rm -r -f ekf_bayesopt
cd ~/bayesopt
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

Delete the old ekf_bayesopt folder first. You should have already done it at the `Install bayesian optimization` part

```
git clone --single-branch -b master https://github.com/arpg/ekf_bayesopt.git
cp ekf_bayesopt/bayesopt_ros ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
## Run the code
**Run bayesian optimization for skyCrane problem**
open one terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch skycrane_bayesopt skyCrane_EKF.launch 
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch skycrane_bayesopt skyCrane_bayesopt.launch 
```
In 1D or 2D case, if you want to visualize the process, open another terminal

```
source ~/catkin_ws/devel/setup.bash
rosrun skycrane_bayesopt plotSurrogateAcquisition.py
```

**Test program**
1:test controller and check the state output
open one terminal 
```
roscore
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun skycrane_bayesopt plotStateHis.py
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch skycrane_bayesopt skyCrane_testController.launch
```

2: test model(simulator, estimator and controller). You need modify the specific estimator process noise you want to test in src/testProgram/testModel.cpp. Three are three lines
```
rV.pnoise[0] = 0.01;//0.01;//0.0001;
rV.pnoise[1] = 0.01;//0.01;//0.0001;
rV.pnoise[2] = 0.01;//0.01;//0.0001;
```
Then you need use catkin_make to recompile. After you sepcify the estimator process noise you want to test

```
roscore
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun skycrane_bayesopt plotStateHis.py
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch skycrane_bayesopt skyCrane_testModel.launch
```
3: test bayesopt and plot 1D 2D situation.
for 1D, use bayesopt lib's own example to visulize the surrogate model and acuisition fucntion. Will find the minimum of this function `(x-0.3)*(x-0.3) + sin(20*x)*0.2;`
open one terminal
```
roscore
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun skycrane_bayesopt plotSurrogateAcquisition.py
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun skycrane_bayesopt plot1DsurrogateModel
```

For2D visualization, we use example function `sqr(y-(5.1/(4*pi*pi))*sqr(x)+5*x/pi-6)+10*(1-1/(8*pi))*cos(x)+10;`
open one terminal
```
roscore
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun skycrane_bayesopt plotSurrogateAcquisition.py
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun skycrane_bayesopt plot2DsurrogateModel
```
