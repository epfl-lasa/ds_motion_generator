#include "targetTracker.h"


TargetTracker::TargetTracker(ros::NodeHandle &n,
                             double frequency,
                             std::string base_topic_name,
                             std::string object_1_topic_name,
                             std::string object_2_topic_name,
                             std::string target_1_topic_name,
                             std::string target_2_topic_name)
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

	base_position_.resize(3);

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
	msg_new.pose.position.x = msg->pose.position.x - base_position_[0];
	msg_new.pose.position.y = msg->pose.position.y - base_position_[1];
	msg_new.pose.position.z = msg->pose.position.z - base_position_[2];
	msg_new.header.frame_id = "world";

	pub_target1_pose_.publish(msg_new.pose);

}

void TargetTracker::UpdateTarget_2_Position(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	geometry_msgs::PoseStamped msg_new;
	msg_new.pose.position.x = msg->pose.position.x - base_position_[0];
	msg_new.pose.position.y = msg->pose.position.y - base_position_[1];
	msg_new.pose.position.z = msg->pose.position.z - base_position_[2];
	msg_new.header.frame_id = "world";

	pub_target2_pose_.publish(msg_new.pose);

}