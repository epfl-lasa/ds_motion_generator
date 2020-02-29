/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
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

#include "LinearMotionGenerator.h"

LinearMotionGenerator::LinearMotionGenerator(ros::NodeHandle &n,
        double frequency,
        std::string input_topic_name,
        std::string output_topic_name,
        std::string output_filtered_topic_name,
        std::string direction,
        double linearVelocity, 
        std::string world_frame_name)
	: nh_(n),
	  loop_rate_(frequency),
	  input_topic_name_(input_topic_name),
	  output_topic_name_(output_topic_name),
	  output_filtered_topic_name_(output_filtered_topic_name),
	  DIRECTION_(direction),
	  VELOCITY_(linearVelocity),
	  VELOCITY_offset_(0),
	  dt_(1 / frequency),
	  Wn_(0) , 
	  world_frame_name_(world_frame_name){

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool LinearMotionGenerator::Init() {

	real_pose_.Resize(3);
	desired_velocity_.Resize(3);


	// initializing the filter
	CCDyn_filter_.reset (new CDDynamics(3, dt_, Wn_));

	// we should set the size automagically
	velLimits_.Resize(3);
	CCDyn_filter_->SetVelocityLimits(velLimits_);

	accLimits_.Resize(3);
	CCDyn_filter_->SetAccelLimits(accLimits_);


	MathLib::Vector initial(3);

	initial.Zero();

	CCDyn_filter_->SetState(initial);
	CCDyn_filter_->SetTarget(initial);

	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}



bool LinearMotionGenerator::InitializeROS() {

	sub_real_pose_ = nh_.subscribe( input_topic_name_ , 1000,
	                                &LinearMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
	pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_topic_name_, 1);

	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("DS/DesiredPath", 1);

	msg_DesiredPath_.poses.resize(MAX_FRAME);

	dyn_rec_f_ = boost::bind(&LinearMotionGenerator::DynCallback, this, _1, _2);
	dyn_rec_srv_.setCallback(dyn_rec_f_);



	if (nh_.ok()) { // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Motion generator is ready.");
		return true;
	}
	else {
		ROS_ERROR("The ros node has a problem.");
		return false;
	}

}


void LinearMotionGenerator::Run() {

	while (nh_.ok()) {

		ComputeDesiredVelocity();

		PublishDesiredVelocity();

		PublishFuturePath();

		ros::spinOnce();

		loop_rate_.sleep();
	}
}

void LinearMotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;

}


void LinearMotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();


	desired_velocity_(0) = 0;
	desired_velocity_(1) = 0;
	desired_velocity_(2) = 0;

	if (DIRECTION_ == "-x") {
		desired_velocity_(0) = -1 * (VELOCITY_  + VELOCITY_offset_);
	}

	if (DIRECTION_ == "+x") {
		desired_velocity_(0) = +1 * (VELOCITY_  + VELOCITY_offset_);
	}

	if (DIRECTION_ == "-y") {
		desired_velocity_(1) = -1 * (VELOCITY_  + VELOCITY_offset_);
	}

	if (DIRECTION_ == "+y") {
		desired_velocity_(1) = +1 * (VELOCITY_  + VELOCITY_offset_);
	}

	if (DIRECTION_ == "-z") {
		desired_velocity_(2) = -1 * (VELOCITY_  + VELOCITY_offset_);
	}

	if (DIRECTION_ == "+z") {
		desired_velocity_(2) = +1 * (VELOCITY_  + VELOCITY_offset_);
	}



	if (std::isnan(desired_velocity_.Norm2())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.Zero();
	}


	msg_desired_velocity_.header.stamp = ros::Time::now();
	msg_desired_velocity_.twist.linear.x  = desired_velocity_(0);
	msg_desired_velocity_.twist.linear.y  = desired_velocity_(1);
	msg_desired_velocity_.twist.linear.z  = desired_velocity_(2);

	msg_desired_velocity_.twist.angular.x = 0;
	msg_desired_velocity_.twist.angular.y = 0;
	msg_desired_velocity_.twist.angular.z = 0;

	CCDyn_filter_->SetTarget(desired_velocity_);
	CCDyn_filter_->Update();
	CCDyn_filter_->GetState(desired_velocity_filtered_);

	msg_desired_velocity_filtered_.header.stamp = ros::Time::now();
	msg_desired_velocity_filtered_.twist.linear.x  = desired_velocity_filtered_(0);
	msg_desired_velocity_filtered_.twist.linear.y  = desired_velocity_filtered_(1);
	msg_desired_velocity_filtered_.twist.linear.z  = desired_velocity_filtered_(2);

	msg_desired_velocity_filtered_.twist.angular.x = 0;
	msg_desired_velocity_filtered_.twist.angular.y = 0;
	msg_desired_velocity_filtered_.twist.angular.z = 0;


	mutex_.unlock();

}


void LinearMotionGenerator::PublishDesiredVelocity() {

	pub_desired_twist_.publish(msg_desired_velocity_.twist);
	pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_.twist);

}


void LinearMotionGenerator::DynCallback(ds_motion_generator::LINEAR_paramsConfig &config, uint32_t level) {

	double  Wn = config.Wn;

	ROS_INFO("Reconfigure request. Updatig the parameters ...");


	Wn_ = config.Wn;
	CCDyn_filter_->SetWn(Wn_);

	double vlim = config.fil_dx_lim;

	velLimits_(0) = vlim;
	velLimits_(1) = vlim;
	velLimits_(2) = vlim;

	CCDyn_filter_->SetVelocityLimits(velLimits_);


	double alim = config.fil_ddx_lim;

	accLimits_(0) = alim;
	accLimits_(1) = alim;
	accLimits_(2) = alim;

	CCDyn_filter_->SetAccelLimits(accLimits_);


	VELOCITY_offset_ = config.Vel_offset;

}


void LinearMotionGenerator::PublishFuturePath() {


	// setting the header of the path
	msg_DesiredPath_.header.stamp = ros::Time::now();
	msg_DesiredPath_.header.frame_id = world_frame_name_;
	MathLib::Vector simulated_pose = real_pose_;
	MathLib::Vector simulated_vel;
	simulated_vel.Resize(3);

	for (int frame = 0; frame < MAX_FRAME; frame++)
	{
		simulated_vel(0) = 0;
		simulated_vel(1) = 0;
		simulated_vel(2) = 0;

		if (DIRECTION_ == "-x") {
			simulated_vel(0) = -1 * (VELOCITY_  + VELOCITY_offset_);
		}

		if (DIRECTION_ == "+x") {
			simulated_vel(0) = +1 * (VELOCITY_  + VELOCITY_offset_);
		}

		if (DIRECTION_ == "-y") {
			simulated_vel(1) = -1 * (VELOCITY_  + VELOCITY_offset_);
		}

		if (DIRECTION_ == "+y") {
			simulated_vel(1) = +1 * (VELOCITY_  + VELOCITY_offset_);
		}

		if (DIRECTION_ == "-z") {
			simulated_vel(2) = -1 * (VELOCITY_  + VELOCITY_offset_);
		}

		if (DIRECTION_ == "+z") {
			simulated_vel(2) = +1 * (VELOCITY_  + VELOCITY_offset_);
		}

		simulated_pose[0] +=  simulated_vel[0] * dt_ * 100;
		simulated_pose[1] +=  simulated_vel[1] * dt_ * 100;
		simulated_pose[2] +=  simulated_vel[2] * dt_ * 100;

		msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
		msg_DesiredPath_.poses[frame].header.frame_id = world_frame_name_;
		msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
		msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
		msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];

		pub_DesiredPath_.publish(msg_DesiredPath_);

		geometry_msgs::PointStamped msg;
		msg.header.frame_id = world_frame_name_;
		msg.header.stamp = ros::Time::now();
		msg.point.x = simulated_pose[0];
		msg.point.y = simulated_pose[1];
		msg.point.z = simulated_pose[2];

		pub_target_.publish(msg);


	}


}
