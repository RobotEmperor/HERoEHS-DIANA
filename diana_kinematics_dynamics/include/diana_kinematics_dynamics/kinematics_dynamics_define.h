/*
 * kinematics_dynamics_define.h
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#ifndef DIANA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_
#define DIANA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_

namespace diana
{

#define MAX_JOINT_ID    (23)
#define ALL_JOINT_ID    (34)

#define MAX_ARM_ID      (3)
#define MAX_LEG_ID      (6)
//#define MAX_ITER        (5)

//#define ID_HEAD_END     (20)
#define ID_PELVIS         (29) //
//#define ID_TORSO        (29)

#define ID_R_ARM_START  (2)
#define ID_L_ARM_START  (1)
//#define ID_R_ARM_END    (21)
//#define ID_L_ARM_END    (22)

#define ID_R_LEG_START  (12)
#define ID_L_LEG_START  (11)
//#define ID_R_LEG_END    (31)
//#define ID_L_LEG_END    (30)

#define GRAVITY_ACCELERATION (9.8)

}




#endif /* DIANA_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_ */
