/*
 * Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Nadia Figueroa
 * email:   nadia.figueroafernandez@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the EU project Cogimon H2020-ICT-23-2014.
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
#include "lagsDSMotionGenerator.h"
#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lagsDS_motion_generator_node");

  ros::NodeHandle nh;
  double frequency = 500.0;

  // Parameters
  std::string input_topic_name;
  std::string input_target_topic_name;
  std::string output_topic_name;
  std::string output_filtered_topic_name;
  
  // DS Paramaters
  double K;
  double M;
  bool bPublish_DS_path (false);
  bool bDynamic_target (false);
  std::vector<double> Priors;
  std::vector<double> Mu;
  std::vector<double> Sigma;
  std::vector<double> A_g;
  std::vector<double> att_g;
  std::vector<double> A_l;
  std::vector<double> A_d;
  std::vector<double> att_l;
  std::vector<double> w_l;
  std::vector<double> b_l;
  double              scale;
  double              b_g;
  std::string         gpr_path;
  std::string         path_model;



  double path_offset;

  if (!nh.getParam("input_topic_name", input_topic_name))   {
    ROS_ERROR("Couldn't retrieve the topic name for the input. ");
    // return -1;
  }

  if (!nh.getParam("input_target_topic_name", input_target_topic_name))   {
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

  if (!nh.getParam("publish_DS_path", bPublish_DS_path))   {
    ROS_ERROR("Couldn't retrieve the publish DS path boolean. ");
    // return -1;
  }

  if (!nh.getParam("dynamic_target", bDynamic_target))   {
    ROS_ERROR("Couldn't retrieve the dynamic target boolean. ");
    // return -1;
  }

  if (!nh.getParam("K", K))   {
    ROS_ERROR("Couldn't retrieve the number of guassians. ");
    // return -1;
  }

  if (!nh.getParam("M", M))  {
    ROS_ERROR("Couldn't retrieve the dimension of the state space. ");
    // return -1;
  }

  if (!nh.getParam("Priors", Priors))   {
    ROS_ERROR("Couldn't retrieve Priors. Maybe you need to use [] in case of k=1");
    // return -1;
  }

  if (!nh.getParam("Mu", Mu))   {
    ROS_ERROR("Couldn't retrieve Mu. ");
    // return -1;
  }

  if (!nh.getParam("Sigma", Sigma))  {
    ROS_ERROR("Couldn't retrieve Sigma. ");
    // return -1;
  }

if (!nh.getParam("A_g", A_g))  {
        ROS_ERROR("Couldn't retrieve A_g. ");
        // ret = false;
    } else {
        cout << "A_g [0]: " << endl;
        for (int m = 0; m< int(M)*int(M); m++)
            cout << A_g.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("att_g", att_g))  {
        ROS_ERROR("Couldn't retrieve att_g. ");
        // ret = false;
    } else {
        cout << "att_g: " << endl;
        for (int m = 0; m< int(M); m++)
            cout << att_g.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("A_l", A_l))  {
        ROS_ERROR("Couldn't retrieve A_l. ");
        // ret = false;
    } else {
        cout << "A_l [0]: " << endl;
        for (int m = 0; m< int(M)*int(M); m++)
            cout << A_l.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("A_d", A_d))  {
        ROS_ERROR("Couldn't retrieve A_d. ");
        // ret = false;
    } else {
        cout << "A_d [0]: " << endl;
        for (int m = 0; m< int(M)*int(M); m++)
            cout << A_d.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("att_l", att_l))   {
        ROS_ERROR("Couldn't retrieve att_l. ");
        // ret = false;
    } else {
        cout << "att_l: " << endl;
        for (int m = 0; m< int(K)*int(M); m++)
            cout << att_l.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("w_l", w_l))   {
        ROS_ERROR("Couldn't retrieve w_l. ");
        // ret = false;
    } else {
        cout << "w_l: " << endl;
        for (int m = 0; m< int(K)*int(M); m++)
            cout << w_l.at(m) << " ";
        cout << endl;
    }

    if (!nh.getParam("b_l", b_l))   {
        ROS_ERROR("Couldn't retrieve b_l. ");
        // ret = false;
    } else {
        cout << "b_l: " << endl;
        for (int k = 0; k< int(K); k++)
            cout << b_l.at(k) << " ";
        cout << endl;
    }

    if (!nh.getParam("scale", scale))   {
        ROS_ERROR("Couldn't retrieve the scale. ");
        // ret = false;
    } else {
        cout << "scale: "<< scale << endl;
    }


    if (!nh.getParam("b_g", b_g))   {
        ROS_ERROR("Couldn't retrieve b_g. ");
        // ret = false;
    } else {
        cout << "b_g: "<< b_g << endl;
    }

    if (!nh.getParam("gpr_path", gpr_path))   {
        ROS_ERROR("Couldn't retrieve gpr_path. ");
        // ret = false;
    } else {
        cout << "gpr_path: "<< gpr_path << endl;
    }

  if (!nh.getParam("path_offset", path_offset))   {
    ROS_ERROR("Couldn't retrieve path_offset. ");
    // return -1;
  }

  if (bPublish_DS_path)
        ROS_INFO("Starting the lags-DS Motion generator... Publishing path in this node. ");
  else
      ROS_INFO("Starting the lags-DS Motion generator... NOT Publishing path in this node. ");

  lagsDSMotionGenerator lagsDS_motion_generator(nh, frequency,
                                        (int)K, (int)M, Priors, Mu, Sigma, 
                                        A_g, att_g, A_l, A_d, att_l, w_l, b_l, scale, b_g, gpr_path,
                                        input_topic_name,
                                        output_topic_name,
                                        output_filtered_topic_name,
                                        input_target_topic_name,
                                        bPublish_DS_path,
                                        bDynamic_target,
                                        path_offset);
  if (!lagsDS_motion_generator.Init()) {
    return -1;
  }
  else {
    lagsDS_motion_generator.Run();
  }

  return 0;
}
