# ds_motion_generator
---
This package provides a nodified version of DS motion generators. Type of DS:
- Analytically parameterized DS for simple motions used in [1], such as
  - linear motions, point-to-point oscillatory motions and swiping as below
  <p align="center">
  <img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/linearMotion.gif" width="200"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/PPOscilateMotion.gif" width="200"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/SwipeMotion.gif" width="200"></>
    
- Non-linear DS learned from demonstrations with the following approaches  
  - se-DS parametrization [2], we provide a couple of pre-learned models of curvilinear motion with different targets as used in [1]; e.g. Push-Down Motion, Curve-Left Motion, Curve-Right Motion and Free WS (workspace) motion
    <p align="center">
  <img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/seDS_pushDown.gif" width="200"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/seDS_curveLeft.gif" width="200"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/seDS_curveRight.gif" width="200"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/seDS_freeWS.gif" width="200">
</>
  - lpv-DS parametrization [3]**(Working on ...)**
  - lags-DS parametrization [4] **(TODO)**
  
This package was initially implemented by [Mahdi Khoramshahi](http://lasa.epfl.ch/people/member.php?SCIPER=217217) and has been extended and modified by [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387).  

## Installation
Do the following steps:
* In your catkin src directory clone the repository
```
$ git clone -b nadia https://github.com/epfl-lasa/ds_motion_generator.git
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$  wstool init
$  wstool merge ds_motion_generator/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro indigo 
```

## Usage
#### DS Configuration and Launch File Specifications
To test the **analytical DS** motions you can run any of the following launch file as the ones found in the ```launch/``` folder. For the linear DS, you should fill in the **DS configuration** file, which is a ```.yaml``` file stored in ```config/analytic_DS``` folder, the default is set to -y direction of motion. You can then test the launch file as follows:
```
$ roslaunch ds_motion_generator load_linearDS_motionGenerator.launch 
```
Some imporant things to fill in the launch file are the following:
1. the name of the input topic (potentially a position signal)
   1. the name of the output topic (potentially a desired velocity signal)
   1. the name of the topic for the filtered output.

The current topics are assigned assuming you are using the [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros) control interface and simulator (Gazebo). For the other analytical DS, the following launch files are provided:
```
$ roslaunch ds_motion_generator load_ppOscDS_motionGenerator.launch viz_DS_path:=true
```
The variable ```viz_DS_path``` defines if you want to visualize the integrated path of the DS. For the swipe motion the following launch file is provided:
```
$ roslaunch ds_motion_generator load_swipeDS_motionGenerator.launch direction:=right viz_DS_path:=true
```
The variable ```direction``` defines the direction of motion, which can be ```left/right```. 

--

To test the **learned DS** via se-DS parametrization, we provide the following launch file
```
$ roslaunch ds_motion_generator load_seDS_motionGenerator.launch DS_name:=free_ws viz_DS_path:=true
```
in which,apart from the input/output variables defined above, you must indicate the DS **DS configuration** file, which contains the the location of the GMM paremeters (i.e., a yaml file containing ``K`` (number of guassian), ``dim`` (the dimenstion input-output space), ``Priors``, ``Mu``, ``Sigma``). These files are stored in ```config/learned_DS``` folder. In this case the ``DS_name`` input variable is used to define the names of the pre-learned se-DS models, which are: ``push_down,Curve_go_right,Curve_go_left,free_ws``. To generate your own yaml file you can follow the ``demo_learn_seDS.m`` script in the [ds-opt](https://github.com/nbfigueroa/ds-opt) package.
  
--  
  

#### Dynamic re-configuration of DS/filter parameters
You can modify the filtering and some of the DS parameters dynamically (cfg folder)
1. Wn : speed of the filter
   1. fil_dx_lim : the limit on the velocity of the filter. (Note this is not the limit of the real velocity).
   1. fil_ddx_lim : the limit on the accelarion of the filter. (Same note)
   1. target_{x,y,z} : the position of the attractor.
   1. scaling : simply a multiplier to scale the computed velocities based on the DS of choice.
   1. trimmit : simply a threshold to limit the computed velocities based on the DS of choice.   

**References**     
> [1] Khoramshahi, M. and Billard, A. (2018) A Dynamical System Approach to Task-Adaptation in Physical Human-Robot Interaction. Under review in Autonomous Robots.    
> [2] Khansari Zadeh, S. M. and Billard, A. (2011) Learning Stable Non-Linear Dynamical Systems with Gaussian Mixture Models. IEEE Transaction on Robotics, vol. 27, num 5, p. 943-957.  
> [3] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL). Accepted.     
> [4] Figueroa, N. and Billard, A. (2018) Locally Active Globally Stable Dynamical Systems: Theory, Learning and Experiments. In preparation.

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

