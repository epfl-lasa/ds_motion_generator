/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
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


#include "lagsDSMotionGenerator.h"

lagsDSMotionGenerator::lagsDSMotionGenerator(ros::NodeHandle &n,
                                     double frequency,
                                     int K, int M, std::vector<double> Priors, std::vector<double> Mu, std::vector<double> Sigma,
                                     std::vector<double>  A_g, std::vector<double>  att_g, std::vector<double>  A_l,
				                     std::vector<double>  A_d, std::vector<double>  att_l, std::vector<double>  w_l,
				                     std::vector<double>  b_l, double scale, double  b_g, std::string  gpr_path,
                                     std::string input_topic_name,
                                     std::string output_topic_name,
                                     std::string output_filtered_topic_name,
                                     std::string input_target_topic_name,
                                     bool bPublish_DS_path,
                                     bool bDynamic_target,
                                     double path_offset)
									: nh_(n),
									  loop_rate_(frequency),
									  K_(K), M_(M), Priors_(Priors), Mu_(Mu), Sigma_(Sigma),
									  A_g_(A_g), att_g_(att_g), A_l_(A_l), A_d_(A_d), 
									  att_l_(att_l), w_l_(w_l), b_l_(b_l), scale_(scale), 
									  b_g_(b_g),  gpr_path_(gpr_path),
									  input_topic_name_(input_topic_name),
									  output_topic_name_(output_topic_name),
									  output_filtered_topic_name_(output_filtered_topic_name),
									  dt_(1 / frequency),
									  Wn_(0),
									  scaling_factor_(5),
								      ds_vel_limit_(0.1),
								      bPublish_DS_path_(bPublish_DS_path),
								      bDynamic_target_(bDynamic_target),
								      input_target_topic_name_(input_target_topic_name),
								      path_offset_(path_offset){

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

lagsDSMotionGenerator::~lagsDSMotionGenerator(){
    ROS_INFO_STREAM("In destructor.. motion generator was killed! ");

}
bool lagsDSMotionGenerator::Init() {

	real_pose_.Resize(M_);
	desired_velocity_.Resize(M_);

	if (!InitializeDS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	if (!InitializeROS()) {
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}


bool lagsDSMotionGenerator::InitializeDS() {

	/* Checking Size of Variables */
	if (Priors_.size() != K_) {
		ROS_ERROR_STREAM("InitializeDS: " << K_ << " priors is expected while " << Priors_.size() << " is provided.");
		return false;
	}
	if (Mu_.size() != K_ * M_) {
		ROS_ERROR_STREAM("InitializeDS: " << K_ * M_ << " elements in Mu is expected while " << Mu_.size() << " is provided.");
		return false;
	}
	if (Sigma_.size() != K_ * M_ * M_ ) {
		ROS_ERROR_STREAM("InitializeDS: " << K_ * M_ * M_ << " elements in Sigma is expected while " << Sigma_.size() << " is provided.");
		return false;
	}

    if (A_g_.size() != K_ * M_ * M_ ) {
        ROS_ERROR_STREAM("InitializeDS: " << K_ * M_ * M_ << " elements in A is expected while " << A_g_.size() << " is provided.");
        return false;
    }
    if (att_g_.size() != M_) {
		ROS_ERROR_STREAM("InitializeDS: Please provide 6 elements for the attractor. It has " << att_g_.size() << " elements.");
		return false;
	}

    /* Instantiate lags-DS Model with parameters read from Yaml file via ROS Parameter Server*/
    LAGS_DS_.reset(new lagsDS(K_, M_, Priors_, Mu_, Sigma_, A_g_, att_g_, A_l_, A_d_, att_l_, w_l_, b_l_, scale_, b_g_, gpr_path_)); 


	target_offset_.Resize(M_);
	target_pose_.Resize(M_);

	for (int i = 0; i < att_g_.size(); i++) {
		target_pose_(i) = att_g_[i];
	}


	// initializing the filter
	CCDyn_filter_.reset (new CDDynamics(M_, dt_, Wn_));

	// we should set the size automagically
	velLimits_.Resize(M_);
	CCDyn_filter_->SetVelocityLimits(velLimits_);
    accLimits_.Resize(M_);

    qx = msg_real_pose_.orientation.x;
    qy = msg_real_pose_.orientation.y;
    qz = msg_real_pose_.orientation.z;
    qw = msg_real_pose_.orientation.w;
	CCDyn_filter_->SetAccelLimits(accLimits_);


	MathLib::Vector initial(M_);
	initial.Zero();
	CCDyn_filter_->SetState(initial);
	CCDyn_filter_->SetTarget(initial);
	return true;

}


bool lagsDSMotionGenerator::InitializeROS() {

    sub_real_pose_              = nh_.subscribe( input_topic_name_ , 1000, &lagsDSMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_desired_target_         = nh_.subscribe( input_target_topic_name_ , 1000, &lagsDSMotionGenerator::UpdateDynamicTarget, this, ros::TransportHints().reliable().tcpNoDelay());
    pub_desired_twist_          = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_topic_name_, 1);
    
    /* Doesn't seem to work */
    pub_tigger_passive_ds_      = nh_.advertise<std_msgs::Bool>("/lwr/joint_controllers/passive_ds_trigger", 1);


	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("DS/DesiredPath_1", 1);    
    msg_DesiredPath_.poses.resize(MAX_FRAME);

	dyn_rec_f_ = boost::bind(&lagsDSMotionGenerator::DynCallback, this, _1, _2);
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


void lagsDSMotionGenerator::Run() {

	while (nh_.ok()) {

        ComputeDesiredVelocity();
        PublishDesiredVelocity();
        PublishFuturePath();
		ros::spinOnce();

		loop_rate_.sleep();
	}
    nh_.shutdown();
}

void lagsDSMotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	if (M_== 3)
		real_pose_(2) = msg_real_pose_.position.z;
}

/* This should be removed for LAGS -- will not work with the current class */
void lagsDSMotionGenerator::UpdateDynamicTarget(const geometry_msgs::Point::ConstPtr& msg) {

    msg_desired_target_ = *msg;

    if (bDynamic_target_){
        target_pose_(0) = msg_desired_target_.x;
        target_pose_(1) = msg_desired_target_.y;
        if (M_== 3)
        	target_pose_(2) = msg_desired_target_.z;
    }

}

void lagsDSMotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();

    desired_velocity_ = LAGS_DS_->compute_fg(real_pose_, target_pose_- target_offset_);
    ROS_WARN_STREAM_THROTTLE(1, "Desired Velocities before limits:" << desired_velocity_(0) << " " << desired_velocity_(1));

	if (std::isnan(desired_velocity_.Norm2())) {
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		desired_velocity_.Zero();
	}

	desired_velocity_ = desired_velocity_ * scaling_factor_;

	if (desired_velocity_.Norm() > ds_vel_limit_) {
		desired_velocity_ = desired_velocity_ / desired_velocity_.Norm() * ds_vel_limit_;
	}

    pos_error_ = (real_pose_ - target_pose_ - target_offset_).Norm2();
    ROS_WARN_STREAM_THROTTLE(1, "Distance to attractor:" << pos_error_);
    if (pos_error_ < 1e-4){
        ROS_WARN_STREAM_THROTTLE(1, "[Attractor REACHED] Distance to attractor:" << pos_error_);
    }

    msg_desired_velocity_.linear.x  = desired_velocity_(0);
    msg_desired_velocity_.linear.y  = desired_velocity_(1);
    if (M_== 3){
    	msg_desired_velocity_.linear.z  = desired_velocity_(2);
    	ROS_WARN_STREAM_THROTTLE(1, "Desired Velocities:" << desired_velocity_(0) << " " << desired_velocity_(1) << " " << desired_velocity_(2));
    } else{
		msg_desired_velocity_.linear.z  = 0;
		ROS_WARN_STREAM_THROTTLE(1, "Desired Velocities:" << desired_velocity_(0) << " " << desired_velocity_(1));
    }
	msg_desired_velocity_.angular.x = 0;
	msg_desired_velocity_.angular.y = 0;
	msg_desired_velocity_.angular.z = 0;
    

    CCDyn_filter_->SetTarget(desired_velocity_);
    CCDyn_filter_->Update();
    CCDyn_filter_->GetState(desired_velocity_filtered_);

	msg_desired_velocity_filtered_.linear.x  = desired_velocity_filtered_(0);
	msg_desired_velocity_filtered_.linear.y  = desired_velocity_filtered_(1);
	if (M_== 3){
		msg_desired_velocity_filtered_.linear.z  = desired_velocity_filtered_(2);
		ROS_WARN_STREAM_THROTTLE(1, "Desired Filtered Velocities:" << desired_velocity_filtered_(0) << " " << desired_velocity_filtered_(1) << " " << desired_velocity_filtered_(2));
	}else{
		msg_desired_velocity_.linear.z  = 0;
		ROS_WARN_STREAM_THROTTLE(1, "Desired Filtered Velocities:" << desired_velocity_filtered_(0) << " " << desired_velocity_filtered_(1) );	
	}
	msg_desired_velocity_filtered_.angular.x = 0;
	msg_desired_velocity_filtered_.angular.y = 0;
	msg_desired_velocity_filtered_.angular.z = 0;
	mutex_.unlock();

}


void lagsDSMotionGenerator::PublishDesiredVelocity() {

	pub_desired_twist_.publish(msg_desired_velocity_);
	pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_);

}

void lagsDSMotionGenerator::DynCallback(ds_motion_generator::lagsDS_paramsConfig &config, uint32_t level) {

	double  Wn = config.Wn;

	ROS_INFO("Reconfigure request. Updating the parameters ...");


	Wn_ = config.Wn;
	CCDyn_filter_->SetWn(Wn_);

	double vlim = config.fil_dx_lim;

	velLimits_(0) = vlim;
	velLimits_(1) = vlim;
	if (M_==3)
		velLimits_(2) = vlim;

	CCDyn_filter_->SetVelocityLimits(velLimits_);


	double alim = config.fil_ddx_lim;

	accLimits_(0) = alim;
	accLimits_(1) = alim;
	if (M_==3)
		accLimits_(2) = alim;

	CCDyn_filter_->SetAccelLimits(accLimits_);

	target_offset_(0) = config.offset_x;
	target_offset_(1) = config.offset_y;
	if (M_==3)
		target_offset_(2) = config.offset_z;

	scaling_factor_ = config.scaling;
	ds_vel_limit_   = config.trimming;

}


void lagsDSMotionGenerator::PublishFuturePath() {

	geometry_msgs::PointStamped msg;
	msg.header.frame_id = "world";
	msg.header.stamp = ros::Time::now();
	msg.point.x = target_pose_[0] + target_offset_[0];
	msg.point.y = target_pose_[1] + target_offset_[1];
	if (M_ == 3)
		msg.point.z = target_pose_[2] + target_offset_[2];
	else
		msg.point.z = 0.25;
	pub_target_.publish(msg);

    if (bPublish_DS_path_){

        ROS_WARN_STREAM_THROTTLE(1, "Publishing Path...");
        
        // setting the header of the path
        msg_DesiredPath_.header.stamp = ros::Time::now();
        msg_DesiredPath_.header.frame_id = "world";

        MathLib::Vector simulated_pose = real_pose_;

        // Add offset to the path
        for (int m=1;m<M_;m++)
        	simulated_pose[m] = simulated_pose[m] + path_offset_;
        
        MathLib::Vector simulated_vel;
        simulated_vel.Resize(M_);
        for (int frame = 0; frame < MAX_FRAME; frame++)
        {
            // computing the next step based on the lagsDS model
            simulated_vel = LAGS_DS_->compute_fg(simulated_pose, target_pose_ - target_offset_);

            simulated_pose[0] +=  0.1* simulated_vel[0] * dt_ * 400;
            simulated_pose[1] +=  0.1* simulated_vel[1] * dt_ * 400;
            if (M_==3)
            	simulated_pose[2] +=  simulated_vel[2] * dt_ * 400;

            msg_DesiredPath_.poses[frame].header.stamp = ros::Time::now();
            msg_DesiredPath_.poses[frame].header.frame_id = "world";
            msg_DesiredPath_.poses[frame].pose.position.x = simulated_pose[0];
            msg_DesiredPath_.poses[frame].pose.position.y = simulated_pose[1];
            if (M_==3){
            	msg_DesiredPath_.poses[frame].pose.position.z = simulated_pose[2];
            }
            else{
            	msg_DesiredPath_.poses[frame].pose.position.z = 0.25;
            }

            pub_DesiredPath_.publish(msg_DesiredPath_);
        }

    }

}
