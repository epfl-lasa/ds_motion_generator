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

#include "PPOscilateMotionGenerator.h"

PPOscilateMotionGenerator::PPOscilateMotionGenerator(ros::NodeHandle &n,
        double frequency,
        std::string input_topic_name,
        std::string output_topic_name,
        std::string output_filtered_topic_name,
        double SwipeVelocity,
        double OrthogonalDamping,
        std::vector<double> Target_1,
        std::vector<double> Target_2,
        bool bPublish_DS_path, 
        std::string world_frame_name)
	: nh_(n),
	  loop_rate_(frequency),
	  input_topic_name_(input_topic_name),
	  output_topic_name_(output_topic_name),
	  output_filtered_topic_name_(output_filtered_topic_name),
	  dt_(1 / frequency),
	  Wn_(0),
	  SwipeVelocity_(SwipeVelocity),
	  OrthogonalDamping_(OrthogonalDamping),
	  vel_limit_(0.3),
	  Target_1_(Target_1),
	  Target_2_(Target_2),
	  SwipeVel_offset_(0),
      Orth_damp_scaling_(1),
      bPublish_DS_path_(bPublish_DS_path),
      world_frame_name_(world_frame_name){

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

bool PPOscilateMotionGenerator::Init() {

	real_pose_.Resize(3);
	desired_velocity_.Resize(3);
	target_1_offset_.Resize(3);
	target_2_offset_.Resize(3);

	target_pose_.Resize(3);
	target_pose_(0) = Target_1_[0];
	target_pose_(1) = Target_1_[1];
	target_pose_(2) = Target_1_[2];

	other_target_pose_.Resize(3);
	other_target_pose_(0) = Target_2_[0];
	other_target_pose_(1) = Target_2_[1];
	other_target_pose_(2) = Target_2_[2];


	PP_line_.Resize(3);
	PP_line_(0) = Target_1_[0] - Target_2_[0];
	PP_line_(1) = Target_1_[1] - Target_2_[1];
	PP_line_(2) = Target_1_[2] - Target_2_[2];

	TARGET_id = 1;

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



bool PPOscilateMotionGenerator::InitializeROS() {

	sub_real_pose_ = nh_.subscribe( input_topic_name_ , 1000,
	                                &PPOscilateMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
    pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);
    pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_topic_name_, 1);

	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("DS/DesiredPath", 1);

	msg_DesiredPath_.poses.resize(MAX_FRAME);

	dyn_rec_f_ = boost::bind(&PPOscilateMotionGenerator::DynCallback, this, _1, _2);
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


void PPOscilateMotionGenerator::Run() {

	while (nh_.ok()) {

		ComputeDesiredVelocity();

		PublishDesiredVelocity();

		PublishFuturePath();

		ros::spinOnce();

		loop_rate_.sleep();
	}
}

void PPOscilateMotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;

}


void PPOscilateMotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();




	if (TARGET_id == 1) {

		target_pose_(0) = Target_1_[0] + target_1_offset_(0);
		target_pose_(1) = Target_1_[1] + target_1_offset_(1);
		target_pose_(2) = Target_1_[2] + target_1_offset_(2);

		other_target_pose_(0) = Target_2_[0] + target_2_offset_(0);
		other_target_pose_(1) = Target_2_[1] + target_2_offset_(1);
		other_target_pose_(2) = Target_2_[2] + target_2_offset_(2);
	}
	else {
		target_pose_(0) = Target_2_[0] + target_2_offset_(0);
		target_pose_(1) = Target_2_[1] + target_2_offset_(1);
		target_pose_(2) = Target_2_[2] + target_2_offset_(2);

		other_target_pose_(0) = Target_1_[0] + target_1_offset_(0);
		other_target_pose_(1) = Target_1_[1] + target_1_offset_(1);
		other_target_pose_(2) = Target_1_[2] + target_1_offset_(2);

	}

	MathLib::Vector pose = real_pose_ - target_pose_;
	MathLib::Vector line = other_target_pose_ - target_pose_;
    double proj = pose(0) * line(0) + pose(1) * line(1) + pose(2) * line(2);


	MathLib::Vector pose_on_line = line / line.Norm() * proj;
	ROS_WARN_STREAM_THROTTLE(1, "target" << TARGET_id);
	MathLib::Vector pose_ortho_line = pose - pose_on_line;
	MathLib::Vector vel_on_line = pose_on_line / pose_on_line.Norm() * ( - SwipeVelocity_ - SwipeVel_offset_);
	MathLib::Vector vel_ortho_line =   pose_ortho_line * -(OrthogonalDamping_ * Orth_damp_scaling_) ;


	desired_velocity_ = vel_on_line + vel_ortho_line;

	if (desired_velocity_.Norm() > vel_limit_) {
		desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * vel_limit_;
	}

	if (pose.Norm() < line.Norm() / 10) {
		if (TARGET_id == 1) {
			TARGET_id = 2;
		}
		else {
			TARGET_id = 1;
		}
	}

	// ROS_WARN_STREAM("PP_line_ " << desired_velocity_(0) << "\t" << desired_velocity_(1) << "\t" << desired_velocity_(2) << "\t" << desired_velocity_(3) ) ;

	if (std::isnan(desired_velocity_.Norm2())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.Zero();
	}


	// For Kuka setting
    // msg_desired_velocity_.linear.x  = desired_velocity_(0);
    // msg_desired_velocity_.linear.y  = desired_velocity_(1);
	
	// For UR10 setting
    msg_desired_velocity_.linear.x  = -desired_velocity_(1);
    msg_desired_velocity_.linear.y  = desired_velocity_(0);

    msg_desired_velocity_.linear.z  = desired_velocity_(2);
    msg_desired_velocity_.angular.x = 0;
    msg_desired_velocity_.angular.y = 0;
    msg_desired_velocity_.angular.z = 0;

    CCDyn_filter_->SetTarget(desired_velocity_);
    CCDyn_filter_->Update();
    CCDyn_filter_->GetState(desired_velocity_filtered_);

    msg_desired_velocity_filtered_.linear.x  = desired_velocity_filtered_(0);
    msg_desired_velocity_filtered_.linear.y  = desired_velocity_filtered_(1);
    msg_desired_velocity_filtered_.linear.z  = desired_velocity_filtered_(2);

    msg_desired_velocity_filtered_.angular.x = 0;
    msg_desired_velocity_filtered_.angular.y = 0;
    msg_desired_velocity_filtered_.angular.z = 0;

	mutex_.unlock();

}


void PPOscilateMotionGenerator::PublishDesiredVelocity() {

	pub_desired_twist_.publish(msg_desired_velocity_);
	pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_);

}


void PPOscilateMotionGenerator::DynCallback(ds_motion_generator::PPOscilate_paramsConfig &config, uint32_t level) {

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

	target_1_offset_(0) = config.offset_1_x;
	target_1_offset_(1) = config.offset_1_y;
	target_1_offset_(2) = config.offset_1_z;

	target_2_offset_(0) = config.offset_2_x;
	target_2_offset_(1) = config.offset_2_y;
	target_2_offset_(2) = config.offset_2_z;

	// scaling_factor_ = config.scaling;
	vel_limit_   = config.trimming;

	SwipeVel_offset_ = config.SwipeVel_offset;
	Orth_damp_scaling_ = config.Orth_damp_scaling;

}


void PPOscilateMotionGenerator::PublishFuturePath() {

	geometry_msgs::PointStamped msg;

	msg.header.frame_id = world_frame_name_;
	msg.header.stamp = ros::Time::now();

	if (TARGET_id == 1) {
		msg.point.x = Target_1_[0] + target_1_offset_(0);
		msg.point.y = Target_1_[1] + target_1_offset_(1);
		msg.point.z = Target_1_[2] + target_1_offset_(2);
	}
	else {
		msg.point.x = Target_2_[0] + target_2_offset_(0);
		msg.point.y = Target_2_[1] + target_2_offset_(1);
		msg.point.z = Target_2_[2] + target_2_offset_(2);
	}
	pub_target_.publish(msg);

    if (bPublish_DS_path_){
        // setting the header of the path
        msg_DesiredPath_.header.stamp = ros::Time::now();
        msg_DesiredPath_.header.frame_id = world_frame_name_;

        MathLib::Vector simulated_pose = real_pose_;
        MathLib::Vector simulated_vel;
        simulated_vel.Resize(3);

        for (int frame = 0; frame < MAX_FRAME; frame++)
        {
            MathLib::Vector pose = simulated_pose - target_pose_;
            MathLib::Vector line = other_target_pose_ - target_pose_;

            double proj = pose(0) * line(0) + pose(1) * line(1) + pose(2) * line(2);
            MathLib::Vector pose_on_line = line / line.Norm() * proj;
            MathLib::Vector pose_ortho_line = pose - pose_on_line;
            MathLib::Vector vel_on_line = pose_on_line / pose_on_line.Norm() * ( - SwipeVelocity_ - SwipeVel_offset_);
            MathLib::Vector vel_ortho_line =   pose_ortho_line * -(OrthogonalDamping_ * Orth_damp_scaling_) ;
            simulated_vel = vel_on_line + vel_ortho_line;
            if (simulated_vel.Norm() > vel_limit_) {
                simulated_vel = simulated_vel / simulated_vel.Norm() * vel_limit_;
            }
            simulated_pose[0] +=  simulated_vel[0] * dt_ * 20;
            simulated_pose[1] +=  simulated_vel[1] * dt_ * 20;
            simulated_pose[2] +=  simulated_vel[2] * dt_ * 20;
            msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
            msg_DesiredPath_.poses[frame].header.frame_id = world_frame_name_;
            msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
            msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
            msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];
            pub_DesiredPath_.publish(msg_DesiredPath_);
        }
    }
}
