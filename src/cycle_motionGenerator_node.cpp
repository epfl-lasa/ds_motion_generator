/*
 * Copyright (C) 2016 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Mahdi Khoramshahi
 * email:   mahdi.khoramshahi@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the European Communitys Horizon 2020 Research and Innovation 
 * programme ICT-23-2014, grant agreement 643950-SecondHands.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "ros/ros.h"
#include "CycleMotionGenerator.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cycle_motion_genrator_node");

  ros::NodeHandle nh;
  double frequency = 300.0;


  // Parameters
  std::string input_topic_name;
  std::string output_topic_name;
  std::string output_filtered_topic_name;

  std::vector<double> CenterRotation;
  double radius;
  double RotationSpeed;
  double ConvergenceRate;


  if (!nh.getParam("input_topic_name", input_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("output_topic_name", output_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the output. ");
    // return -1;
  }

  if (!nh.getParam("output_filtered_topic_name", output_filtered_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the filtered output. ");
    // return -1;
  }

  if (!nh.getParam("CenterRotation", CenterRotation))   {
    ROS_ERROR("Couldn't retrieve the center of rotation. ");
    // return -1;
  }

  if (!nh.getParam("radius", radius))   {
    ROS_ERROR("Couldn't retrieve the radius of the rotation. ");
    // return -1;
  }

  if (!nh.getParam("RotationSpeed", RotationSpeed))  {
    ROS_ERROR("Couldn't retrieve the rotation speed.");
    // return -1;
  }

  if (!nh.getParam("ConvergenceRate", ConvergenceRate)) {
    ROS_ERROR("Couldn't retrieve the convergence speed. ");
    // return -1;
  }


  ROS_INFO("Starting the Motion generator...");


  CycleMotionGenerator cycle_motion_generator(nh,
      frequency,
      input_topic_name,
      output_topic_name,
      output_filtered_topic_name,
      CenterRotation,
      radius,
      RotationSpeed,
      ConvergenceRate);

  if (!cycle_motion_generator.Init()) {
    return -1;
  }
  else {
    cycle_motion_generator.Run();
  }


  return 0;
}