# ds_motion_generator
---
This package provides a nodified version of DS motion generators. Type of DS:
- Analytically parameterized DS for simple motions used in [1], such as
  - linear motions, point-to-point oscillatory motions and swiping as below
  <p align="center"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/linearMotion.gif" width="180"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/PPOscilateMotion.gif" width="180"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/SwipeMotion.gif" width="180"></>
    
- Non-linear DS learned from demonstrations with the following approaches  
  - se-DS parametrization [2]. For testing, we provide a couple of pre-learned models of curvilinear motion with different targets as used in [1]; e.g. Push-Down Motion, Curve-Left Motion, Curve-Right Motion and Free WS (workspace) motion
    <p align="center"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/seDS_pushDown.gif" width="180"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/seDS_curveLeft.gif" width="180"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/seDS_curveRight.gif" width="180"><img src="https://github.com/epfl-lasa/ds_motion_generator/blob/nadia/img/seDS_freeWS.gif" width="180"></>
  - lpv-DS parametrization [3].  For testing, we provide a couple of pre-learned models of non-linear, non-monotic motions used in a variety of tasks in [3]; e.g. sink motion (inspection-line task), via-point motion (branding-line task), CShape motion (shelf-arranging task)
  
  **Add GIFs of execution**
 
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
### DS Configuration and Launch File Specifications
To test the **analytical DS** motions you can run any of the following launch file as the ones found in the ```launch/``` folder. For the linear DS, you should fill in the **DS configuration** file, which is a ```.yaml``` file stored in ```config/analytic_DS``` folder, the default is set to -y direction of motion. You can then test the launch file as follows:
```
$ roslaunch ds_motion_generator load_linearDS_motionGenerator.launch 
```
Some imporant things to fill in the launch file are the following:
- The name of the input topic (potentially a position signal)
- The name of the output topic (potentially a desired velocity signal)
- The name of the topic for the filtered output.

The current topics are assigned assuming you are using the [kuka-lwr-ros](https://github.com/epfl-lasa/kuka-lwr-ros) control interface and simulator (Gazebo). For the other analytical DS, the following launch files are provided:
```
$ roslaunch ds_motion_generator load_ppOscDS_motionGenerator.launch viz_DS_path:=true
```
The variable ```viz_DS_path``` defines if you want to visualize the integrated path of the DS. 

For the swipe motion the following launch file is provided:
```
$ roslaunch ds_motion_generator load_swipeDS_motionGenerator.launch direction:=right viz_DS_path:=true
```
The variable ```direction``` defines the direction of motion, which can be ```left/right```. 

---

To test the **learned DS** via se-DS [2] parametrization, we provide the following launch file
```
$ roslaunch ds_motion_generator load_seDS_motionGenerator.launch DS_name:=free_ws viz_DS_path:=true
```
in which,apart from the input/output variables defined above, you must indicate the **DS configuration** file, which should contain the following information in YAML format:
- ``K`` (number of guassian)
- ``dim`` (the dimenstion input-output space)
- ``Priors``
- ``Mu``
- ``Sigma``
- ``Attractor``  
Sometimes the parameters of ``Mu`` and ``Sigma`` might be too large; i.e. ``>1e6``. In these cases you can scale them and add the scaling values to the yaml file:
- ``Mu_scale``
- ``Sigma_scale``

This file is stored in the ```config/learned_DS/seDS``` folder. The input variable ``DS_name`` in the launch file is used to indicate the names of the pre-learned se-DS models, which are: ``push_down,Curve_go_right,Curve_go_left,free_ws``. 

To learn/test your own se-DS model, you must generate this yaml file, which you can do by following the ``demo_learn_seDS.m`` script in the [ds-opt](https://github.com/nbfigueroa/ds-opt) package [3].
  
---  

To test the **learned DS** via lpv-DS [2] parametrization, we provide the following launch file
```
$ roslaunch ds_motion_generator load_lpvDS_motionGenerator.launch DS_name:=via_point viz_DS_path:=true
```  


**Fill in information about LPV-DS parameters**

### Dynamic re-configuration of DS/filter parameters
Finally, once running the node for your desired DS, you can modify the filtering parameters and some of the DS parameters dynamically. The definition of the parameters that can be dynamically reconfigured is provided in the ``cfg/``. Following an example of the parameters you can reconfigure:
- Wn : speed of the filter
- fil_dx_lim : the limit on the velocity of the filter. (Note this is not the limit of the real velocity).
- fil_ddx_lim : the limit on the accelarion of the filter. (Same note)
- target_{x,y,z} : the position of the attractor.
- scaling : simply a multiplier to scale the computed velocities based on the DS of choice.
- trimmit : simply a threshold to limit the computed velocities based on the DS of choice.   

**References**     
> [1] Khoramshahi, M. and Billard, A. (2018) A Dynamical System Approach to Task-Adaptation in Physical Human-Robot Interaction. Autonomous Robots.    
> [2] Khansari Zadeh, S. M. and Billard, A. (2011) Learning Stable Non-Linear Dynamical Systems with Gaussian Mixture Models. IEEE Transaction on Robotics, vol. 27, num 5, p. 943-957.  
> [3] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL).     

This package was initially implemented by [Mahdi Khoramshahi](http://lasa.epfl.ch/people/member.php?SCIPER=217217) and has been extended and modified by [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387).  

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the European Communitys Horizon 2020 Research and Innovation pro-
gramme ICT-23-2014, grant agreement 644727-[Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon) and
643950-[SecondHands](https://secondhands.eu/).
