/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Authors: Mahdi Khoramshahi and Nadia Figueroa
 * email:   {mahdi.khoramshahi,nadia.figueroafernandez}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the European Communitys Horizon 2020 Research and Innovation 
 * programme ICT-23-2014, grant agreement 644727-Cogimon and 643950-SecondHands.
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

#include "seDSMotionGenerator.h"

seDSMotionGenerator::seDSMotionGenerator(ros::NodeHandle &n,
                                     double frequency,
                                     int K_gmm,
                                     int dim,
                                     std::vector<double> Priors,
                                     std::vector<double> Mu,
                                     std::vector<double> Sigma,
                                     double Mu_scale,
                                     double Sigma_scale,
                                     std::vector<double> attractor,
                                     std::string input_topic_name,
                                     std::string output_topic_name,
                                     std::string output_filtered_topic_name,
                                     std::string input_target_topic_name,
                                     bool bPublish_DS_path,
                                     bool bDynamic_target)
	: nh_(n),
	  loop_rate_(frequency),
	  K_gmm_(K_gmm),
	  dim_(dim),
	  Priors_(Priors),
	  Mu_(Mu),
	  Sigma_(Sigma),
      Mu_scale_(Mu_scale),
      Sigma_scale_(Sigma_scale),
	  attractor_(attractor),
	  input_topic_name_(input_topic_name),
	  output_topic_name_(output_topic_name),
	  output_filtered_topic_name_(output_filtered_topic_name),
	  dt_(1 / frequency),
	  Wn_(0),
      scaling_factor_(2),
      ds_vel_limit_(0.1),
      bPublish_DS_path_(bPublish_DS_path),
      input_target_topic_name_(input_target_topic_name),
      bDynamic_target_(bDynamic_target){

	ROS_INFO_STREAM("Motion generator node is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");
}

seDSMotionGenerator::~seDSMotionGenerator(){
    ROS_INFO_STREAM("In destructor.. motion generator was killed! ");

}
bool seDSMotionGenerator::Init() {

	real_pose_.Resize(3);
	desired_velocity_.Resize(3);

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


bool seDSMotionGenerator::InitializeDS() {

	if (Priors_.size() != K_gmm_) {
		ROS_ERROR_STREAM("InitializeDS: " << K_gmm_ << " priors is expected while " << Priors_.size() << " is provided.");
		return false;
	}

	if (Mu_.size() != K_gmm_ * dim_) {
		ROS_ERROR_STREAM("InitializeDS: " << K_gmm_ * dim_ << " elements in Mu is expected while " << Mu_.size() << " is provided.");
		return false;
	}

	if (Sigma_.size() != K_gmm_ * dim_ * dim_ ) {
		ROS_ERROR_STREAM("InitializeDS: " << K_gmm_ * dim_ * dim_ << " elements in Sigma is expected while " << Sigma_.size() << " is provided.");
		return false;
	}

    if (attractor_.size() != 6) {
		ROS_ERROR_STREAM("InitializeDS: Please provide 6 elements for the attractor. It has " << attractor_.size() << " elements.");
		return false;
	}

    /* Scale Mu and Sigma given scaling factor, default1=, but some models can have really high numbers */
    for (int i = 0; i < K_gmm_ * dim_; i++) {
        Mu_[i] = Mu_[i]*Mu_scale_;
    }

    for (int i = 0; i < K_gmm_ * dim_ * dim_; i++) {
        Sigma_[i] = Sigma_[i]*Sigma_scale_;
    }

    SED_GMM_.reset (new GMRDynamics(K_gmm_, dim_, dt_, Priors_, Mu_, Sigma_ ));
	SED_GMM_->initGMR(0, 2, 3, 5 );


	target_offset_.Resize(3);
	target_pose_.Resize(3);

    for (int i = 0; i < attractor_.size(); i++)
		target_pose_(i) = attractor_[i];


	// initializing the filter
	CCDyn_filter_.reset (new CDDynamics(3, dt_, Wn_));

	// we should set the size automagically
	velLimits_.Resize(3);
	CCDyn_filter_->SetVelocityLimits(velLimits_);

    accLimits_.Resize(3);    /* Set the desired orientation as the initial one */
    qx = msg_real_pose_.orientation.x;
    qy = msg_real_pose_.orientation.y;
    qz = msg_real_pose_.orientation.z;
    qw = msg_real_pose_.orientation.w;
	CCDyn_filter_->SetAccelLimits(accLimits_);


	MathLib::Vector initial(3);

	initial.Zero();

	CCDyn_filter_->SetState(initial);
	CCDyn_filter_->SetTarget(initial);
	return true;

}


bool seDSMotionGenerator::InitializeROS() {

    sub_real_pose_              = nh_.subscribe( input_topic_name_ , 1000, &seDSMotionGenerator::UpdateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_desired_target_         = nh_.subscribe( input_target_topic_name_ , 1000, &seDSMotionGenerator::UpdateDynamicTarget, this, ros::TransportHints().reliable().tcpNoDelay());
    pub_desired_twist_          = nh_.advertise<geometry_msgs::Twist>(output_topic_name_, 1);    
	pub_desired_twist_filtered_ = nh_.advertise<geometry_msgs::Twist>(output_filtered_topic_name_, 1);

    /* Doesn't see to work */
    pub_tigger_passive_ds_      = nh_.advertise<std_msgs::Bool>("/lwr/joint_controllers/passive_ds_trigger", 1);


	pub_target_ = nh_.advertise<geometry_msgs::PointStamped>("DS/target", 1);
	pub_DesiredPath_ = nh_.advertise<nav_msgs::Path>("DS/DesiredPath", 1);    
    msg_DesiredPath_.poses.resize(MAX_FRAME);

	dyn_rec_f_ = boost::bind(&seDSMotionGenerator::DynCallback, this, _1, _2);
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


void seDSMotionGenerator::Run() {

	while (nh_.ok()) {

        ComputeDesiredVelocity();
        PublishDesiredVelocity();
        PublishFuturePath();
		ros::spinOnce();

		loop_rate_.sleep();
	}
    nh_.shutdown();
}

void seDSMotionGenerator::UpdateRealPosition(const geometry_msgs::Pose::ConstPtr& msg) {

	msg_real_pose_ = *msg;

	real_pose_(0) = msg_real_pose_.position.x;
	real_pose_(1) = msg_real_pose_.position.y;
	real_pose_(2) = msg_real_pose_.position.z;
}


void seDSMotionGenerator::UpdateDynamicTarget(const geometry_msgs::Point::ConstPtr& msg) {

    msg_desired_target_ = *msg;

    if (bDynamic_target_){
        target_pose_(0) = msg_desired_target_.x;
        target_pose_(1) = msg_desired_target_.y;
        target_pose_(2) = msg_desired_target_.z;
    }

}

void seDSMotionGenerator::ComputeDesiredVelocity() {

	mutex_.lock();

    MathLib::Vector Trans_pose ; Trans_pose.Resize(3);
    Trans_pose = real_pose_ - target_pose_ - target_offset_;
    desired_velocity_ = SED_GMM_->getVelocity(Trans_pose);

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
    if (pos_error_ < 1e-4)
        ROS_WARN_STREAM_THROTTLE(1, "[Attractor REACHED] Distance to attractor:" << pos_error_);

    msg_desired_velocity_.linear.x  = desired_velocity_(0);
    msg_desired_velocity_.linear.y  = desired_velocity_(1);
    msg_desired_velocity_.linear.z  = desired_velocity_(2);
	msg_desired_velocity_.angular.x = 0;
	msg_desired_velocity_.angular.y = 0;
	msg_desired_velocity_.angular.z = 0;
    ROS_WARN_STREAM_THROTTLE(1, "Desired Velocities:" << desired_velocity_(0) << " " << desired_velocity_(1) << " " << desired_velocity_(2));

    CCDyn_filter_->SetTarget(desired_velocity_);
    CCDyn_filter_->Update();
    CCDyn_filter_->GetState(desired_velocity_filtered_);




	msg_desired_velocity_filtered_.linear.x  = desired_velocity_filtered_(0);
	msg_desired_velocity_filtered_.linear.y  = desired_velocity_filtered_(1);
	msg_desired_velocity_filtered_.linear.z  = desired_velocity_filtered_(2);
	msg_desired_velocity_filtered_.angular.x = 0;
	msg_desired_velocity_filtered_.angular.y = 0;
	msg_desired_velocity_filtered_.angular.z = 0;

    ROS_WARN_STREAM_THROTTLE(1, "Desired Filtered Velocities:" << desired_velocity_filtered_(0) << " " << desired_velocity_filtered_(1) << " " << desired_velocity_filtered_(2));

	mutex_.unlock();

}


void seDSMotionGenerator::PublishDesiredVelocity() {

	pub_desired_twist_.publish(msg_desired_velocity_);
	pub_desired_twist_filtered_.publish(msg_desired_velocity_filtered_);

}

void seDSMotionGenerator::DynCallback(ds_motion_generator::seDS_paramsConfig &config, uint32_t level) {

	double  Wn = config.Wn;

    ROS_INFO("Reconfigure request. Updating the parameters ...");


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

	scaling_factor_ = config.scaling;
	ds_vel_limit_   = config.trimming;

}


void seDSMotionGenerator::PublishFuturePath() {

	geometry_msgs::PointStamped msg;
	msg.header.frame_id = "world";
	msg.header.stamp = ros::Time::now();
	msg.point.x = target_pose_[0] + target_offset_[0];
	msg.point.y = target_pose_[1] + target_offset_[1];
	msg.point.z = target_pose_[2] + target_offset_[2];
	pub_target_.publish(msg);

    if (bPublish_DS_path_){

        ROS_WARN_STREAM_THROTTLE(1, "Publishing Path...");
        // setting the header of the path
        msg_DesiredPath_.header.stamp = ros::Time::now();
        msg_DesiredPath_.header.frame_id = "world";

        MathLib::Vector simulated_pose = real_pose_;
        MathLib::Vector simulated_vel;

        simulated_vel.Resize(3);

        for (int frame = 0; frame < MAX_FRAME; frame++)
        {
            // computing the next step based on the SED model
            // DesiredPos = Active_GMR->getNextState();


//            double simulated_pos_norm = simulated_pose.Norm2();

//            /* Apply stupid rotation around Z*/
//            double sin_angle = -1;   double cos_angle = 0;

//            MathLib::Vector3 a_hat;
//            a_hat(0) = simulated_vel(0)/simulated_pos_norm;
//            a_hat(1) = simulated_vel(1)/simulated_pos_norm;
//            a_hat(2) = simulated_vel(2)/simulated_pos_norm;


//            simulated_vel(0) = simulated_vel_norm *(a_hat(0)*cos_angle - a_hat(1)*sin_angle );
//            simulated_vel(1) = simulated_vel_norm *(a_hat(0)*sin_angle +  a_hat(1)*cos_angle);
//            simulated_vel(2) = simulated_vel_norm * a_hat(2);


            simulated_vel = SED_GMM_->getVelocity(simulated_pose - target_pose_ - target_offset_);

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

}
