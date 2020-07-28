# kf_bayesopt
Kalman Filter tuning example using Bayesian Optimization
## Introduction
This package simulates 1D robot with linear kinematics model as decribed in our paper `<Weak in the NEES?: Auto-tuning Kalman Filters
with Bayesian Optimization>`

This system is also build on ROS so you also need install ROS. With the help of ROS, bayesian optimization lib and robot1d lib can communicate with each other more easily. At the same time, all the parameters of bayesian optimization and robot1d can be modified in the yaml file.
The main structure can be seen from the following figure
![image](https://github.com/arpg/ekf_bayesopt/raw/master/plot_example/Nodes.png)
robot1d_KF node will run simulator and estimator(KF) accodring to different noise setting. It will output J_NEES or J_NIS(topic "cost") and publish to the robot1d_bayesopt node. Acoording to the cost, robot1d_bayesopt will run the bayesian optimization to optimize the cost, then it will generate the noise(topic "noise") to publish to the robot1d_KF and set the nosie. Then robot1d_KF node can run the simulator and estimator agian and generate new cost.
To modify the parameters, please go to the `config`. There are two yaml files `bayesParams.yaml` to modify the parameters of bayesopt and `vehicleParams` to modify the parameters of robot1d.

## Dependencies
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

If you want to see the real-time surrogate function and acquisition function model, you can download my version (just add some plot API)
```
To be added
```

### ROS kinect or melodic
### Eigen3
### Install the bayesopt_ros package

```
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
cd src
git clone --single-branch -b master https://github.com/arpg/kf_bayesopt.git
cd ..
catkin_make
```
## Run the code
**Run bayesian optimization for robot1d problem** <br/>
open one terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch robot_kf_bayesopt robot1d_kf.launch 
```
open another terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch robot_kf_bayesopt robot1d_bayesopt.launch
```
In 1D or 2D case, if you download the bayesopt from my github, you can visualize the process in real time, open another terminal

```
source ~/catkin_ws/devel/setup.bash
rosrun robot_kf_bayesopt plot_surrogate_acquisition.py
```
For 1D optimization, you'll see something like this
![image](https://github.com/arpg/ekf_bayesopt/raw/master/plot_example/1d_opt_example.png).
In the example image, the upper one is surrogate model with mean, 95% upper bound, 95% lower bound and sample points. The lower one is acquisition function. you can see the legend to know the detail.  
For 2D optimization, you'll see something like this 
![image](https://github.com/arpg/ekf_bayesopt/raw/master/plot_example/2d_opt_example.png).
In this example image, you'll see 2d surrogate model and 2d acquisition function. No upper and lower bound is shown because the bound will cover the mean so I decide not adding them.
**Test program** <br/>
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
for 1D, use bayesopt lib's own example to visulize the surrogate model and acuisition fucntion. Will find the minimum of this function `(x-0.3)*(x-0.3) + sin(20*x)*0.2;` <br/>
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

For2D visualization, we use example function `sqr(y-(5.1/(4*pi*pi))*sqr(x)+5*x/pi-6)+10*(1-1/(8*pi))*cos(x)+10;`<br/>
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
4: plot 1D or 3D diagonal noise change vs cost. Which means if you want to check if the bayesopt works correctly, sometimes you need plot what the real function looks like.<br/>
similar as testModel example. You need go to src/testProgram/plotCost1D.cpp to modify where you want to start plot, how many points you want to plot, what's the interval between the plot and then save and `catkin_make`. <br/>
Then open one terminal
```
roscore
```
open another terminal
```
``
source ~/catkin_ws/devel/setup.bash
rosrun skycrane_bayesopt plotCost1D.py
```
open other terminal 
```
source ~/catkin_ws/devel/setup.bash
roslaunch skycrane_bayesopt skyCrane_plotCost1D.launch
```
Similar if you want to plot 2D process noise vs cost
