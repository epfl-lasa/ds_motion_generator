#include "ros/ros.h"
#include "DSMotionGenerator.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_motion_genrator_node");

  ros::NodeHandle nh;
  double frequency = 500.0;


  // Parameters
  std::string input_topic_name;
  std::string output_topic_name;
  std::string output_filtered_topic_name;
  
  int K_gmm;
  int dim;
  std::vector<double> Priors;
  std::vector<double> Mu;
  std::vector<double> Sigma;
  std::vector<double> attractor;

  // double Wn;
  // std::vector<double> VelLimits;
  // std::vector<double> AccLimits;


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

  if (!nh.getParam("K", K_gmm))   {
    ROS_ERROR("Couldn't retrieve the number of guassians. ");
    // return -1;
  }

  if (!nh.getParam("dim", dim))  {
    ROS_ERROR("Couldn't retrieve the dimension of the state space. ");
    // return -1;
  }


  if (!nh.getParam("Priors", Priors))   {
    ROS_ERROR("Couldn't retrieve Priors. Maybe you need to use [] in case of k=1");
    // return -1;
  }

  // std::cout << Priors.size() << std::endl;
  if (!nh.getParam("Mu", Mu))   {
    ROS_ERROR("Couldn't retrieve Mu. ");
    // return -1;
  }

  if (!nh.getParam("Sigma", Sigma))  {
    ROS_ERROR("Couldn't retrieve Sigma. ");
    // return -1;
  }

  if (!nh.getParam("attractor", attractor))  {
    ROS_ERROR("Couldn't retrieve the attractor for the DS. ");
    // return -1;
  }


  ROS_INFO("Starting the Motion generator...");

  DSMotionGenerator ds_motion_generator(nh, frequency,
                                        K_gmm, dim, Priors, Mu, Sigma, attractor,
                                        input_topic_name,
                                        output_topic_name,
                                        output_filtered_topic_name);
  if (!ds_motion_generator.Init()) {
    return -1;
  }
  else {
    ds_motion_generator.Run();
  }


  return 0;
}
