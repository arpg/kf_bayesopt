# kf_bayesopt
Kalman Filter tuning example using Bayesian Optimization. If you use the relates code, please consider citing
```
@inproceedings{chen2018weak,
  title={Weak in the NEES?: Auto-tuning Kalman filters with Bayesian optimization},
  author={Chen, Zhaozhong and Heckman, Christoffer and Julier, Simon and Ahmed, Nisar},
  booktitle={2018 21st International Conference on Information Fusion (FUSION)},
  pages={1072--1079},
  year={2018},
  organization={IEEE}
}
```
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
bayesian optimization is from my github https://github.com/zhaozhongch/bayesopt. (The official bayesopt will also work but no API for python and matlab plot)
```
sudo apt-get install libboost-dev python-dev python-numpy cmake cmake-curses-gui g++ cython octave-headers freeglut3-dev
git clone https://github.com/zhaozhongch/bayesopt.git
cd bayesopt
mkdir build
cd build
cmake ..
make
sudo make install
```
Deatails can be seen at https://rmcantin.bitbucket.io/html/install.html

### ROS indigo or kinect or melodic
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
In 1D or 2D case, **if you download the bayesopt from my github**, you can visualize the process in real time, open another terminal

```
source ~/catkin_ws/devel/setup.bash
rosrun robot_kf_bayesopt plot_surrogate_acquisition.py
```
For 1D optimization, you'll see something like this
![image](https://github.com/arpg/ekf_bayesopt/raw/master/plot_example/1d_opt_example.png)
In the example image, the upper one is surrogate model with mean, 95% upper bound, 95% lower bound and sample points. The lower one is acquisition function. you can see the legend to know the detail.  
For 2D optimization, you'll see something like this 
![image](https://github.com/arpg/ekf_bayesopt/raw/master/plot_example/2d_opt_example.png)
In this example image, you'll see 2d surrogate model and 2d acquisition function. No upper and lower bound is shown because the bound will cover the mean so I decide not to add them.  
What's more, we also provide matlab script to plot the bayesopt for 1D and 2D problem. For 1D and 2D optimization, every time the optimization is finished, you should be able to see some csv files in the bayesopt/bayesplot folder. With the matlab script, specify the location of the csv folder, output image folder and initial sample number, for 1D/2D optimization, you can get a plot like the following
![image](https://github.com/arpg/ekf_bayesopt/raw/master/plot_example/1d_matlab_plot.png)
![image](https://github.com/arpg/ekf_bayesopt/raw/master/plot_example/2d_matlab_plot.png)

## Parameters
The parameter of bayesopt should be considered from the bayesopt official website https://github.com/rmcantin/bayesopt. You can specify those parameters in `bayesParams.yaml`.
The parameters of the simulation robot are pretty strightforward if you read the paper and the parameters are stored in `vehicleParams.yaml`  
One thing you need to notice is that the `upper bound` and `lower bound`'s dimension in `bayesParams.yaml` must agree with the `optimizationChoice` parameter in `vehicleParams.yaml`.  
For example, the processnoise is 1d, if you choose `optimizationChoice` to be `processNoise` then the lower and upper bound should be just 1 dimension; if you choose `optimizationChoice` to be `all` then the lower and upper bound should be just 2 dimension.  
`CPUcoreNumber` decides how many threads you wan to run parallely, which you can choose according to the core number of your computer.
