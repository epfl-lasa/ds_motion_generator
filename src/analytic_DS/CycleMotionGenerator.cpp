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

#include "CycleMotionGenerator.h"

CycleMotionGenerator::CycleMotionGenerator(ros::NodeHandle &n,
        double frequency,
        std::string input_topic_name,
        std::string output_topic_name,
        std::string output_filtered_topic_name,
        std::vector<double> CenterRotation,
        double radius,
        double RotationSpeed,
        double ConvergenceRate)
	: nh_(n),
	  loop_rate_(frequency),
	  input_topic_name_(input_topic_name),
	  output_topic_name_(output_topic_name),
	  output_filtered_topic_name_(output_filtered_topic_name),
	  dt_(1 / frequency),
	  Wn_(0),
	  Cycle_Target_(CenterRotation),
	  Cycle_radius_(radius),
	  Cycle_radius_scale_(1),
	  Cycle_speed_(RotationSpeed),
	  Cycle_speed_offset_(0),
	  Convergence_Rate_(ConvergenceRate),
	  Convergence_Rate_scale_(1),
	  Velocity_limit_(0) {

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}


bool CycleMotionGenerator::Init() {

	real_pose_.Resize(3);
	desired_pose_.Resize(3);
	desired_velocity_.Resize(3);

	target_pose_.Resize(3);
	target_pose_(0) = Cycle_Target_[0];
	target_pose_(1) = Cycle_Target_[1];
	target_pose_(2) = Cycle_Target_[2];

	object_position_.Resize(3);
	object_speed_.Resize(3);

	object_position_.Zero();
	object_speed_.Zero();

	target_offset_.Resize(3);


	if (Cycle_radius_ < 0) {
		ROS_ERROR("The radius cannot be negative!");
		return false;
	}

	if (Convergence_Rate_ < 0) {
		ROS_ERROR("The convergence rate cannot be negative!");
		return false;
	}



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

	_startThread = true;
    if(pthread_create(&_thread, NULL, &CycleMotionGenerator::startPathPublishingLoop, this))
    {
        throw std::runtime_error("Cannot create reception thread");  
    }



	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}



bool CycleMotionGenerator::InitializeROS() {

	sub_real_pose_ = nh_.subscribe( input_topic_name_ , 1,
	                                &CycleMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());

	if(_outputVelocity)
	{
		pub_desired_twist_ = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);
		pub_desiredOrientation_ = nh_.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
	}
	else
	{
		pub_desired_twist_ = nh_.advertise<geometry_msgs::Pose>(output_topic_name_, 1);
		
	}
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_topic_name_, 1);

	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("DS/DesiredPath", 1);

	///////
	sub_object_state_ = nh_.subscribe("test_polishing/object_state", 1000, &CycleMotionGenerator::UpdateObjectState, this, ros::TransportHints().reliable().tcpNoDelay());

	msg_DesiredPath_.poses.resize(MAX_FRAME);

	dyn_rec_f_ = boost::bind(&CycleMotionGenerator::DynCallback, this, _1, _2);
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

void CycleMotionGenerator::setDesiredOrientation(geometry_msgs::Quaternion msg) 
{
	msg_desired_orientation = msg;
}

void CycleMotionGenerator::Run() {

	while (nh_.ok()) {

		if(_firstPosition && _firstObject)
		{
			ComputeDesiredVelocity();

			PublishDesiredVelocity();

			// PublishFuturePath();			
		}

		ros::spinOnce();

		loop_rate_.sleep();
	}

	_startThread = false;
	pthread_join(_thread,NULL);

}

void CycleMotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;


	if(!_firstPosition)
	{
		desired_pose_ = real_pose_;
		std:cerr << desired_pose_(2) << " " <<  real_pose_(2) << std::endl;

		_firstPosition = true;
	}
}


void CycleMotionGenerator::UpdateObjectState(const std_msgs::Float64MultiArray::ConstPtr& msg) {

	target_pose_(0) = msg->data[0];
	target_pose_(1) = msg->data[1];
	target_pose_(2) = msg->data[2];
	object_speed_(0) = msg->data[3];
	object_speed_(1) = msg->data[4];
	object_speed_(2) = msg->data[5];

	if(!_firstObject)
	{
		_firstObject= true;
	}
}


void CycleMotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();

	MathLib::Vector pose, desired_velocity_des;
	pose.Resize(3);
	desired_velocity_des.Resize(3);
	if(_outputVelocity)
	{
		desired_velocity_ = getDesiredVelocity(real_pose_);
	}
	else
	{
		desired_velocity_ = getDesiredVelocity(desired_pose_);
	}


	desired_velocity_des = getDesiredVelocity(desired_pose_);

	if (std::isnan(desired_velocity_.Norm2())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.Zero();
	}


	if (desired_velocity_.Norm() > Velocity_limit_) {
		desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * Velocity_limit_;
	}


	if (desired_velocity_des.Norm() > Velocity_limit_) {
		desired_velocity_des = desired_velocity_des / desired_velocity_des.Norm() * Velocity_limit_;
	}

	float alpha = 0.5f;

    MathLib::Vector error;
    error.Resize(3);
    error = desired_pose_-real_pose_;
    for(int k =0; k < 3;k++)
    {
        desired_pose_(k) += dt_*((1-Alpha)*desired_velocity_(k)+Alpha*desired_velocity_des(k));
        // desired_pose_(k) += dt_*desired_velocity_des(k);
        // desired_pose_(k) = real_pose_(k)+dt_*((1-Alpha)*desired_velocity_(k)+Alpha*desired_velocity_des(k));
        // desired_pose_(k) = real_pose_(k)+dt_*(desired_velocity_(k));
        // desired_pose_(k) += dt_*desired_velocity_des(k);
    }

    msg_desired_velocity_.linear.x  = desired_velocity_(0);
    msg_desired_velocity_.linear.y  = desired_velocity_(1);
    msg_desired_velocity_.linear.z  = desired_velocity_(2);

    msg_desired_velocity_.angular.x = desired_pose_(0);
    msg_desired_velocity_.angular.y = desired_pose_(1);
    msg_desired_velocity_.angular.z = desired_pose_(2);

	msg_desired_pose_.position.x = desired_pose_(0);
	msg_desired_pose_.position.y = desired_pose_(1);
	msg_desired_pose_.position.z = desired_pose_(2);
	msg_desired_pose_.orientation.x = msg_desired_orientation.x;
	msg_desired_pose_.orientation.y = msg_desired_orientation.y;
	msg_desired_pose_.orientation.z = msg_desired_orientation.z;
	msg_desired_pose_.orientation.w = msg_desired_orientation.w;

	CCDyn_filter_->SetTarget(desired_velocity_);
	CCDyn_filter_->Update();
	CCDyn_filter_->GetState(desired_velocity_filtered_);

//	msg_desired_velocity_filtered_.header.stamp = ros::Time::now();
    msg_desired_velocity_filtered_.linear.x  = desired_velocity_filtered_(0);
    msg_desired_velocity_filtered_.linear.y  = desired_velocity_filtered_(1);
    msg_desired_velocity_filtered_.linear.z  = desired_velocity_filtered_(2);

    msg_desired_velocity_filtered_.angular.x = 0;
    msg_desired_velocity_filtered_.angular.y = 0;
    msg_desired_velocity_filtered_.angular.z = 0;


	mutex_.unlock();

}


MathLib::Vector CycleMotionGenerator::getDesiredVelocity(MathLib::Vector pose)
{
	MathLib::Vector velocity;
	velocity.Resize(3);

	pose = pose-target_pose_-target_offset_;

	double x_vel = 0;
	double y_vel = 0;
	double z_vel = - Convergence_Rate_ * Convergence_Rate_scale_ * pose(2);

	double R = sqrt(pose(0) * pose(0) + pose(1) * pose(1));
	double T = atan2(pose(1), pose(0));

	double Rdot = - Convergence_Rate_ * Convergence_Rate_scale_ * (R - Cycle_radius_ * Cycle_radius_scale_);
	double Tdot = Cycle_speed_ * (1+Cycle_speed_offset_);


	x_vel = Rdot * cos(T) - R * Tdot * sin(T);
	y_vel = Rdot * sin(T) + R * Tdot * cos(T);

	velocity(0) = x_vel+object_speed_(0);
	velocity(1) = y_vel+object_speed_(1);
	velocity(2) = z_vel+object_speed_(2);

	return velocity;
}


// void CycleMotionGenerator::ComputeDesiredVelocity() {

// 	mutex_.lock();

// 	MathLib::Vector pose;
// 	pose.Resize(3);
// 	if(_outputVelocity)
// 	{
// 		pose = real_pose_ - target_pose_  - target_offset_;
// 	}
// 	else
// 	{
// 		pose = desired_pose_ - target_pose_  - target_offset_;
// 	}

// 	double x_vel = 0;
// 	double y_vel = 0;
// 	double z_vel = - Convergence_Rate_ * Convergence_Rate_scale_ * pose(2) *0.5f;

// 	double R = sqrt(pose(0) * pose(0) + pose(1) * pose(1));
// 	double T = atan2(pose(1), pose(0));

// 	double Rdot = - Convergence_Rate_ * Convergence_Rate_scale_ * (R - Cycle_radius_ * Cycle_radius_scale_);
// 	double Tdot = Cycle_speed_ + Cycle_speed_offset_;


// 	x_vel = Rdot * cos(T) - R * Tdot * sin(T);
// 	y_vel = Rdot * sin(T) + R * Tdot * cos(T);

// 	// double alpha = Convergence_Rate_ * Convergence_Rate_scale_;
// 	// double omega = Cycle_speed_ + Cycle_speed_offset_;
// 	// double r = Cycle_radius_ * Cycle_radius_scale_;


// 	// x_vel = -alpha*(R-r) * cos(T) - R * omega * sin(T);
// 	// y_vel = -alpha*(R-r) * sin(T) + R * omega * cos(T);

// 	desired_velocity_(0) = x_vel+object_speed_(0);
// 	desired_velocity_(1) = y_vel+object_speed_(1);
// 	desired_velocity_(2) = z_vel+object_speed_(2);

// 	// desired pose
// 	MathLib::Vector ref_velocity;
// 	ref_velocity.Resize(3);

// 	if(_outputVelocity)
// 	{
// 		pose = desired_pose_ - target_pose_  - target_offset_;
// 		R = sqrt(pose(0) * pose(0) + pose(1) * pose(1));
// 		T = atan2(pose(1), pose(0));
// 		Rdot = - Convergence_Rate_ * Convergence_Rate_scale_ * (R - Cycle_radius_ * Cycle_radius_scale_);
// 		Tdot = Cycle_speed_ * (1+Cycle_speed_offset_);
// 		ref_velocity(0) = Rdot * cos(T) - R * Tdot * sin(T)+object_speed_(0);
// 		ref_velocity(1) = Rdot * sin(T) + R * Tdot * cos(T)+object_speed_(1);
// 		ref_velocity(2) = - Convergence_Rate_ * Convergence_Rate_scale_ * pose(2)+object_speed_(2);	

// 		// std::cerr << pose(0) << " " << pose(1) << " " << pose(2) << std::endl;

// 		if (ref_velocity.Norm() > Velocity_limit_) {
// 		ref_velocity = ref_velocity / ref_velocity.Norm() * Velocity_limit_;
// 		}

// 	}

// 	if (std::isnan(desired_velocity_.Norm2())) {
// 		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
// 		desired_velocity_.Zero();
// 	}

// 	if (desired_velocity_.Norm() > Velocity_limit_) {
// 		desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * Velocity_limit_;
// 	}


// 	// MathLib::Vector error_velocity;
// 	// error_velocity.Resize(3);
// 	// error_velocity.Norm();
// 	for(int k =0; k < 3;k++)
// 	{
// 		desired_pose_(k) += dt_*desired_velocity_(k);
// 	}

// 	// if(_outputVelocity)
// 	// {
// 	// 	for(int k =0; k < 3;k++)
// 	// 	{
// 	// 		desired_pose_(k) += dt_*ref_velocity(k);
// 	// 	}
// 	// }
// 	// else
// 	// {
// 	// 	for(int k =0; k < 3;k++)
// 	// 	{
// 	// 		desired_pose_(k) += dt_*desired_velocity_(k);
// 	// 	}
// 	// }



// 	msg_desired_velocity_.header.stamp = ros::Time::now();
// 	msg_desired_velocity_.twist.linear.x  = desired_velocity_(0);
// 	msg_desired_velocity_.twist.linear.y  = desired_velocity_(1);
// 	msg_desired_velocity_.twist.linear.z  = desired_velocity_(2);

// 	msg_desired_velocity_.twist.angular.x = desired_pose_(0);
// 	msg_desired_velocity_.twist.angular.y = desired_pose_(1);
// 	msg_desired_velocity_.twist.angular.z = desired_pose_(2);

// 	msg_desired_pose_.position.x = desired_pose_(0);
// 	msg_desired_pose_.position.y = desired_pose_(1);
// 	msg_desired_pose_.position.z = desired_pose_(2);
// 	msg_desired_pose_.orientation.x = msg_desired_orientation.x;
// 	msg_desired_pose_.orientation.y = msg_desired_orientation.y;
// 	msg_desired_pose_.orientation.z = msg_desired_orientation.z;
// 	msg_desired_pose_.orientation.w = msg_desired_orientation.w;

// 	CCDyn_filter_->SetTarget(desired_velocity_);
// 	CCDyn_filter_->Update();
// 	CCDyn_filter_->GetState(desired_velocity_filtered_);

// 	msg_desired_velocity_filtered_.header.stamp = ros::Time::now();
// 	msg_desired_velocity_filtered_.twist.linear.x  = desired_velocity_filtered_(0);
// 	msg_desired_velocity_filtered_.twist.linear.y  = desired_velocity_filtered_(1);
// 	msg_desired_velocity_filtered_.twist.linear.z  = desired_velocity_filtered_(2);

// 	msg_desired_velocity_filtered_.twist.angular.x = 0;
// 	msg_desired_velocity_filtered_.twist.angular.y = 0;
// 	msg_desired_velocity_filtered_.twist.angular.z = 0;


// 	mutex_.unlock();

// }


void CycleMotionGenerator::PublishDesiredVelocity() {

	if(_outputVelocity)
	{
        pub_desired_twist_.publish(msg_desired_velocity_);
		pub_desiredOrientation_.publish(msg_desired_orientation);
	}
	else
	{
		pub_desired_twist_.publish(msg_desired_pose_);
	}
    pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_);

}


void CycleMotionGenerator::DynCallback(ds_motion_generator::CYCLE_paramsConfig &config, uint32_t level) {

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

	target_offset_(0) = config.offset_x;
	target_offset_(1) = config.offset_y;
	target_offset_(2) = config.offset_z;

	Cycle_radius_scale_ = config.radius_scale;
	Cycle_speed_offset_ = config.Speed_offset;
	Convergence_Rate_scale_ = config.ConvergenceSpeed;
	Velocity_limit_ = config.vel_trimming;

	Alpha = config.Alpha;
	if (Cycle_radius_scale_ < 0) {
		ROS_ERROR("RECONFIGURE: The scaling factor for radius cannot be negative!");
	}

	if (Convergence_Rate_scale_ < 0) {
		ROS_ERROR("RECONFIGURE: The scaling factor for convergence rate cannot be negative!");
	}

	if (Velocity_limit_ < 0) {
		ROS_ERROR("RECONFIGURE: The limit for velocity cannot be negative!");
	}

}

void* CycleMotionGenerator::startPathPublishingLoop(void* ptr)
{
    reinterpret_cast<CycleMotionGenerator *>(ptr)->pathPublishingLoop(); 
}


void CycleMotionGenerator::pathPublishingLoop()
{
    while(_startThread)
    {
        if(_firstPosition)
        {
            PublishFuturePath();   
        }
    }
    std::cerr << "END path publishing thread" << std::endl;
}

void CycleMotionGenerator::PublishFuturePath() {

	geometry_msgs::PointStamped msg;

	msg.header.frame_id = "world";
	msg.header.stamp = ros::Time::now();
	msg.point.x = target_pose_[0] + target_offset_[0];
	msg.point.y = target_pose_[1] + target_offset_[1];
	msg.point.z = target_pose_[2] + target_offset_[2];

	pub_target_.publish(msg);

	// create a temporary message


	// setting the header of the path
	msg_DesiredPath_.header.stamp = ros::Time::now();
	msg_DesiredPath_.header.frame_id = "world";




	MathLib::Vector simulated_pose = real_pose_;
	MathLib::Vector simulated_vel;
	simulated_vel.Resize(3);

	for (int frame = 0; frame < MAX_FRAME; frame++)
	{



		MathLib::Vector pose = simulated_pose - target_pose_  - target_offset_;

		double R = sqrt(pose(0) * pose(0) + pose(1) * pose(1));
		double T = atan2(pose(1), pose(0));

		double Rdot = - Convergence_Rate_ * Convergence_Rate_scale_ * (R - Cycle_radius_ * Cycle_radius_scale_);
		double Tdot = Cycle_speed_ + Cycle_speed_offset_;

		simulated_vel(0) = Rdot * cos(T) - R * Tdot * sin(T);
		simulated_vel(1) = Rdot * sin(T) + R * Tdot * cos(T);



	// double alpha = Convergence_Rate_ * Convergence_Rate_scale_;
	// double omega = Cycle_speed_ + Cycle_speed_offset_;
	// double r = Cycle_radius_ * Cycle_radius_scale_;


	// simulated_vel(0) = -alpha*(R-r) * cos(T) - R * omega * sin(T);
	// simulated_vel(1) = -alpha*(R-r) * sin(T) + R * omega * cos(T);


		simulated_vel(2) = - Convergence_Rate_ * Convergence_Rate_scale_ * pose(2);;

		if (simulated_vel.Norm() > Velocity_limit_) {
			simulated_vel = simulated_vel / simulated_vel.Norm() * Velocity_limit_;
		}



		simulated_pose[0] +=  simulated_vel[0] * dt_ * 20;
		simulated_pose[1] +=  simulated_vel[1] * dt_ * 20;
		simulated_pose[2] +=  simulated_vel[2] * dt_ * 20;

		msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
		msg_DesiredPath_.poses[frame].header.frame_id = "world";
		msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
		msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
		msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];

		pub_DesiredPath_.publish(msg_DesiredPath_);


	}


}
