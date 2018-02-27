/*
 * diana_online_walking.cpp
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#include "diana_online_walking_module/diana_online_walking.h"

using namespace diana;


DIANAOnlineWalking::DIANAOnlineWalking()
{
  mat_pelvis_to_rhip_ = robotis_framework::getTransformationXYZRPY(0, -0.105, 0, 0, 0, 0);
  mat_pelvis_to_lhip_ = robotis_framework::getTransformationXYZRPY(0,  0.105, 0, 0, 0, 0);

  mat_rhip_to_pelvis_ = robotis_framework::getTransformationXYZRPY(0,  0.105, 0, 0, 0, 0);
  mat_lhip_to_pelvis_ = robotis_framework::getTransformationXYZRPY(0, -0.105, 0, 0, 0, 0);

  balance_error_ = heroehs::BalanceControlError::NoError;
}

DIANAOnlineWalking::~DIANAOnlineWalking()
{

}

void DIANAOnlineWalking::initialize(double control_cycle_sec)
{
  diana_kd_ = new DIANAKinematicsDynamics(WholeBody);

  robotis_framework::Pose3D r_foot, l_foot, pelvis;
  r_foot.x = 0.0;    r_foot.y = -0.105;  r_foot.z = -0.55;
  r_foot.roll = 0.0; r_foot.pitch = 0.0; r_foot.yaw = 0.0;

  l_foot.x = 0.0;    l_foot.y = 0.105;   l_foot.z = -0.55;
  l_foot.roll = 0.0; l_foot.pitch = 0.0; l_foot.yaw = 0.0;

  pelvis.x = 0.0;    pelvis.y = 0.0;     pelvis.z = 0.0;
  pelvis.roll = 0.0; pelvis.pitch = 0.0; pelvis.yaw = 0;

  walking_pattern_.setInitialPose(r_foot, l_foot, pelvis);
  walking_pattern_.initialize(0.7, 1.6, control_cycle_sec);
}

void DIANAOnlineWalking::start()
{
  walking_pattern_.start();
}

void DIANAOnlineWalking::addStepData(robotis_framework::StepData& step_data)
{
  walking_pattern_.addStepData(step_data);
}

void DIANAOnlineWalking::eraseLastStepData()
{
  walking_pattern_.eraseLastStepData();
}

int  DIANAOnlineWalking::getNumofRemainingUnreservedStepData()
{
  return walking_pattern_.getNumofRemainingUnreservedStepData();
}

void DIANAOnlineWalking::getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition)
{
  walking_pattern_.getReferenceStepDatafotAddition(ref_step_data_for_addition);
}

void DIANAOnlineWalking::process()
{
  walking_pattern_.process();
  mat_g_to_pelvis_ = walking_pattern_.mat_g_to_pelvis_;
  mat_pelvis_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_pelvis_);
  mat_g_to_rfoot_ = walking_pattern_.mat_g_to_rfoot_;
  mat_g_to_lfoot_ = walking_pattern_.mat_g_to_lfoot_;

  rhip_to_rfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_rhip_to_pelvis_) * mat_pelvis_to_g_*mat_g_to_rfoot_);
  lhip_to_lfoot_pose_ = robotis_framework::getPose3DfromTransformMatrix((mat_lhip_to_pelvis_) * mat_pelvis_to_g_*mat_g_to_lfoot_);

  diana_kd_->calcInverseKinematicsForRightLeg(r_leg_out_angle_rad_, rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z,
      rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
  diana_kd_->calcInverseKinematicsForLeftLeg(l_leg_out_angle_rad_, lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z,
      lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);

  for(int i = 0; i < 6; i++)
  {
    out_angle_rad_[i+0] = r_leg_out_angle_rad_[i];
    out_angle_rad_[i+6] = l_leg_out_angle_rad_[i];
  }

  out_angle_rad_[0] = r_leg_out_angle_rad_[2];
  out_angle_rad_[1] = r_leg_out_angle_rad_[1];
  out_angle_rad_[2] = r_leg_out_angle_rad_[0];

  out_angle_rad_[6] = l_leg_out_angle_rad_[2];
  out_angle_rad_[7] = l_leg_out_angle_rad_[1];
  out_angle_rad_[8] = l_leg_out_angle_rad_[0];
}

bool DIANAOnlineWalking::isRunning()
{
  return walking_pattern_.isRunning();
}


void DIANAOnlineWalking::setCurrentIMUSensorOutput(double gyro_x, double gyro_y, double quat_x, double quat_y, double quat_z, double quat_w)
{
  imu_data_mutex_lock_.lock();

  current_gyro_roll_rad_per_sec_  = gyro_x;
  current_gyro_pitch_rad_per_sec_ = gyro_y;

  quat_current_imu_ = Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z);

//  mat_current_imu_ = (rot_x_pi_3d_ * quat_current_imu_.toRotationMatrix()) * rot_z_pi_3d_;
//
//  current_imu_roll_rad_  = atan2( mat_current_imu_.coeff(2,1), mat_current_imu_.coeff(2,2));
//  current_imu_pitch_rad_ = atan2(-mat_current_imu_.coeff(2,0), sqrt(robotis_framework::powDI(mat_current_imu_.coeff(2,1), 2) + robotis_framework::powDI(mat_current_imu_.coeff(2,2), 2)));

  imu_data_mutex_lock_.unlock();
}
