[![Build Status](https://travis-ci.org/epfl-lasa/ds_motion_generator.svg?branch=master)](https://travis-ci.org/epfl-lasa/ds_motion_generator)

# ds_motion_generator
---
This package provides a nodified version of DS motion generators. Type of DS:
- Analytically parameterized DS for simple motions used in [1], such as
  - linear motion
  - point-to-point motion
  - cyclic motion
- Non-linear DS learned from demonstrations with the following approaches  
  - se-DS parametrization [2]**(Cleaning-up ...)**
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
## Examples



## Usage

### Run a desired DS
1. Launch file: where you can define  
   1. the name of the input topic (potentially a position signal)
   1. the name of the output topic (potentially a desired velocity signal)
   1. the name of the topic for the filtered output.
   1. If DS is se-DS: the location to the GMM paramaters (i.e., a yaml file contating Prior, Mu, and Sigma)
   1. If DS is lpv-DS: the location to the GMM paramaters and system parameters (i.e., a yaml file contating Prior, Mu, and Sigma, A's and b's)
   
1. DS config file (config folder)
   1. where you provide the conventional Prior, Mu, Sigma,
   1. In addition, K (number of guassian) and dim (the dimenstion input-output space)
   1. WARNING: There is a transpose compared the previous version. In this version, each row (in Priors and Mu) indicates a guassian, and each column indicates a dimension. 

### Dynamic Re-configuration of DS Parameter
You can modify the filtering and scaling parameters dynamically (cfg folder)
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

