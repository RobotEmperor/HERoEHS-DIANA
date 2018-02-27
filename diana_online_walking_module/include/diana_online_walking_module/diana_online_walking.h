/*
 * diana_online_walking.h
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#ifndef DIANA_ONLINE_WALKING_MODULE_DIANA_ONLINE_WALKING_H_
#define DIANA_ONLINE_WALKING_MODULE_DIANA_ONLINE_WALKING_H_

#include "robotis_math/robotis_math.h"
#include "diana_kinematics_dynamics/kinematics_dynamics.h"
#include "heroehs_online_walking_pattern_generator/online_walking_pattern_generator.h"
#include "heroehs_pd_balance_controller/heroehs_pd_balance_controller.h"
#include "robotis_framework_common/singleton.h"

namespace diana
{

class DIANAOnlineWalking : public robotis_framework::Singleton<DIANAOnlineWalking>
{
public:
  DIANAOnlineWalking();
  virtual ~DIANAOnlineWalking();

  void initialize(double control_cycle_sec);
  void start();

  void process();
  bool isRunning();

  void addStepData(robotis_framework::StepData& step_data);
  void eraseLastStepData();
  int  getNumofRemainingUnreservedStepData();
  void getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition);

  Eigen::Matrix4d mat_pelvis_to_rhip_, mat_rhip_to_pelvis_;
  Eigen::Matrix4d mat_pelvis_to_lhip_, mat_lhip_to_pelvis_;
  Eigen::Matrix4d mat_g_to_pelvis_, mat_g_to_rfoot_, mat_g_to_lfoot_;
  Eigen::Matrix4d mat_pelvis_to_g_;

  robotis_framework::Pose3D rhip_to_rfoot_pose_, lhip_to_lfoot_pose_;

  double r_leg_out_angle_rad_[6];
  double l_leg_out_angle_rad_[6];
  double out_angle_rad_[12];

  heroehs::PDController leg_angle_feed_back_[12];

  // balance control
  int balance_error_;
  heroehs::BalanceControlUsingPDController balance_ctrl_;

//  // sensor value
//  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
//  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
//  double current_left_fx_N_,  current_left_fy_N_,  current_left_fz_N_;
//  double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;
private:
  heroehs::OnlineWalkingPatternGenerator walking_pattern_;
  DIANAKinematicsDynamics* diana_kd_;

};

}

#endif /* DIANA_ONLINE_WALKING_MODULE_DIANA_ONLINE_WALKING_H_ */
