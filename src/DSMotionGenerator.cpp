#include "DSMotionGenerator.h"





DSMotionGenerator::DSMotionGenerator(ros::NodeHandle &n,
                                     double frequency,
                                     std::string path_to_cds_gmm,
                                     double max_desired_vel)
	: nh_(n),
	  loop_rate_(frequency),
	  path_to_cds_gmm_(path_to_cds_gmm),
	  max_desired_vel_(max_desired_vel) {

	ROS_INFO_STREAM("Motion generator is created: " << nh_.getNamespace() << " freq: " << frequency_);
	ROS_INFO_STREAM("Getting CDS paramteres from: " << path_to_cds_gmm_);
	ROS_INFO_STREAM("Max desired velocity is set to: " << max_desired_vel_);

}


void DSMotionGenerator::Init() {


	pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>("/ds/desired_velocity", 1);
	sub_real_pose_ = nh_.subscribe("/ds/real_position" , 1000,
	                               &DSMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());


	while (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
	}
	ROS_INFO("The Motion generator is ready.");

}


void DSMotionGenerator::Run() {

}

void DSMotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

}


void DSMotionGenerator::ComputeDesiredVelocity(Eigen::VectorXd &desired_twist, Eigen::VectorXd &current_pose)
{

}
