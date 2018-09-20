#include "ros/ros.h"
#include "seDSMotionGenerator.h"


#include <vector>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "seDS_motion_generator_node");

  ros::NodeHandle nh;
  double frequency = 500.0;

  // Parameters
  std::string input_topic_name;
  std::string output_topic_name;
  std::string output_filtered_topic_name;
  
  int K_gmm;
  int dim;
  bool bPublish_DS_path (false);
  std::vector<double> Priors;
  std::vector<double> Mu;
  std::vector<double> Sigma;
  std::vector<double> attractor;
  double Mu_scale    (1.0);
  double Sigma_scale (1.0);


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

  if (!nh.getParam("publish_DS_path", bPublish_DS_path))   {
    ROS_ERROR("Couldn't retrieve the publish DS path boolean. ");
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

  if (!nh.getParam("Mu", Mu))   {
    ROS_ERROR("Couldn't retrieve Mu. ");
    // return -1;
  }

  if (!nh.getParam("Mu_scale", Mu_scale))   {
    ROS_ERROR("Couldn't retrieve Mu scaling factor. ");
    // return -1;
  }

  if (!nh.getParam("Sigma", Sigma))  {
    ROS_ERROR("Couldn't retrieve Sigma. ");
    // return -1;
  }

  if (!nh.getParam("Sigma_scale", Sigma_scale) ) {
    ROS_ERROR("Couldn't retrieve Sigma scaling factor. ");
    // return -1;
  }

  if (!nh.getParam("attractor", attractor))  {
    ROS_ERROR("Couldn't retrieve the attractor for the DS. ");
    // return -1;
  }

  if (bPublish_DS_path)
        ROS_INFO("Starting the se-DS Motion generator... Publishing path in this node. ");
  else
      ROS_INFO("Starting the se-DS Motion generator... NOT Publishing path in this node. ");


  ROS_INFO_STREAM("Mu_scaling: " << Mu_scale << " Sigma_scale: " << Sigma_scale);
  seDSMotionGenerator ds_motion_generator(nh, frequency,
                                        K_gmm, dim, Priors, Mu, Sigma,
                                        Mu_scale, Sigma_scale,
                                        attractor,
                                        input_topic_name,
                                        output_topic_name,
                                        output_filtered_topic_name,
                                        bPublish_DS_path);
  if (!ds_motion_generator.Init()) {
    return -1;
  }
  else {
    ds_motion_generator.Run();
  }

  return 0;
}
