#include "targetTracker.h"


TargetTracker::TargetTracker(ros::NodeHandle &n,
                             double frequency,
                             std::string base_topic_name,
                             std::string object_1_topic_name,
                             std::string object_2_topic_name,
                             std::string target_1_topic_name,
                             std::string target_2_topic_name,
                             std::string topic_target1_vel,
                             std::string topic_target2_vel)
	: nh_(n),
	  loop_rate_(frequency) {

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");


	sub_base_pose_ = nh_.subscribe( base_topic_name , 1,
	                                &TargetTracker::UpdateBasePosition, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_object1_pose_ = nh_.subscribe( object_1_topic_name , 1,
	                                   &TargetTracker::UpdateTarget_1_Position, this, ros::TransportHints().reliable().tcpNoDelay());
	sub_object2_pose_ = nh_.subscribe( object_2_topic_name , 1,
	                                   &TargetTracker::UpdateTarget_2_Position, this, ros::TransportHints().reliable().tcpNoDelay());

	pub_target1_pose_  = nh_.advertise<geometry_msgs::Pose>(target_1_topic_name, 1);
	pub_target2_pose_  = nh_.advertise<geometry_msgs::Pose>(target_2_topic_name, 1);

	pub_target1_velocity_  = nh_.advertise<geometry_msgs::Twist>(topic_target1_vel, 1);
	pub_target2_velocity_  = nh_.advertise<geometry_msgs::Twist>(topic_target2_vel, 1);


	base_position_.resize(3);
	target_1_position_.resize(3);
	target_2_position_.resize(3);
	target_1_velocity_.resize(3);
	target_2_velocity_.resize(3);

	while (nh_.ok()) {
		ros::spinOnce();
		loop_rate_.sleep();
	}

}


void TargetTracker::Run() {

	while (nh_.ok()) {

		ros::spinOnce();
		loop_rate_.sleep();
	}

}


void TargetTracker::UpdateBasePosition(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	base_position_[0] = msg->pose.position.x;
	base_position_[1] = msg->pose.position.y;
	base_position_[2] = msg->pose.position.z;

}


void TargetTracker::UpdateTarget_1_Position(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	geometry_msgs::PoseStamped msg_new;

	std::vector<double> target_old = target_1_position_;



	target_1_position_[0] = msg->pose.position.x - base_position_[0];
	target_1_position_[1] = msg->pose.position.y - base_position_[1];
	target_1_position_[2] = msg->pose.position.z - base_position_[2];

	msg_new.pose.position.x = target_1_position_[0];
	msg_new.pose.position.y = target_1_position_[1];
	msg_new.pose.position.z = target_1_position_[2];
	msg_new.header.frame_id = "world";

	pub_target1_pose_.publish(msg_new.pose);


	ros::Duration duration = loop_rate_.expectedCycleTime();
	for (int i = 0; i < 3; i++) {

		double v = (target_1_position_[i] - target_old[i]) / duration.toSec();
		target_1_velocity_[i] += 0.03 * (v - target_1_velocity_[i]);
	}


	geometry_msgs::Twist msg_vel;

	msg_vel.linear.x = target_1_velocity_[0];
	msg_vel.linear.y = target_1_velocity_[1];
	msg_vel.linear.z = target_1_velocity_[2];

	pub_target1_velocity_.publish(msg_vel);

}

void TargetTracker::UpdateTarget_2_Position(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	geometry_msgs::PoseStamped msg_new;

	std::vector<double> target_old = target_2_position_;



	target_2_position_[0] = msg->pose.position.x - base_position_[0];
	target_2_position_[1] = msg->pose.position.y - base_position_[1];
	target_2_position_[2] = msg->pose.position.z - base_position_[2];

	msg_new.pose.position.x = target_2_position_[0];
	msg_new.pose.position.y = target_2_position_[1];
	msg_new.pose.position.z = target_2_position_[2];
	msg_new.header.frame_id = "world";

	pub_target2_pose_.publish(msg_new.pose);


	ros::Duration duration = loop_rate_.expectedCycleTime();
	for (int i = 0; i < 3; i++) {

		double v = (target_2_position_[i] - target_old[i]) / duration.toSec();
		target_2_velocity_[i] += 0.03 * (v - target_2_velocity_[i]);
	}


	geometry_msgs::Twist msg_vel;

	msg_vel.linear.x = target_2_velocity_[0];
	msg_vel.linear.y = target_2_velocity_[1];
	msg_vel.linear.z = target_2_velocity_[2];

	pub_target2_velocity_.publish(msg_vel);

}