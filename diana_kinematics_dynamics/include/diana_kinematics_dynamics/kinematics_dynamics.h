/*
 * kinematics_dynamics.h
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#ifndef DIANA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_
#define DIANA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_

#include <vector>

#include "diana_kinematics_dynamics/kinematics_dynamics_define.h"
#include "link_data.h"

namespace diana
{

enum TreeSelect
{
  Manipulation,
  Walking,
  WholeBody
};

class DIANAKinematicsDynamics
{

 public:
  DIANAKinematicsDynamics();
  ~DIANAKinematicsDynamics();
  DIANAKinematicsDynamics(TreeSelect tree);

//  std::vector<int> findRoute(int to);
//  std::vector<int> findRoute(int from, int to);

  double calcTotalMass(int joint_id);
  Eigen::MatrixXd calcMC(int joint_id);
  Eigen::MatrixXd calcCOM(Eigen::MatrixXd mc);

  void calcForwardKinematics(int joint_ID);

//  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
//  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
//  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
//                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);
//
//  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter,
//                             double ik_err);
//  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
//                             int max_iter, double ik_err);

//  // with weight
//  bool calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter,
//                             double ik_err, Eigen::MatrixXd weight);
//  bool calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
//                             int max_iter, double ik_err, Eigen::MatrixXd weight);

  bool calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
  bool calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch,
                                        double yaw);
  bool calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch,
                                       double yaw);

  LinkData *diana_link_data_[ ALL_JOINT_ID + 1];

  LinkData *getLinkData(const std::string link_name);
  LinkData *getLinkData(const int link_id);
  Eigen::MatrixXd getJointAxis(const std::string link_name);
  double getJointDirection(const std::string link_name);
  double getJointDirection(const int link_id);

  double thigh_length_m_;
  double calf_length_m_;
  double ankle_length_m_;
  double leg_side_offset_m_;
};

}



#endif /* DIANA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_H_ */
