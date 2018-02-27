/*
 * diana_soccer_manager.cpp
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#include "robotis_controller/robotis_controller.h"
#include "diana_online_walking_module/online_walking_module.h"
#include "diana_base_module/base_module.h"


using namespace diana;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DIANA_SOCCER_Manager");
    ros::NodeHandle nh;

    ROS_INFO("manager->init");
    robotis_framework::RobotisController *controller = robotis_framework::RobotisController::getInstance();

    /* Load ROS Parameter */
    std::string offset_file = nh.param<std::string>("offset_file_path", "");
    std::string robot_file  = nh.param<std::string>("robot_file_path", "");

    std::string init_file   = nh.param<std::string>("init_file_path", "");

    /* gazebo simulation */
    controller->gazebo_mode_ = nh.param<bool>("gazebo", false);
    if(controller->gazebo_mode_ == true)
    {
        ROS_WARN("SET TO GAZEBO MODE!");
        std::string robot_name = nh.param<std::string>("gazebo_robot_name", "");
        if(robot_name != "")
            controller->gazebo_robot_name_ = robot_name;
    }

    if(robot_file == "")
    {
        ROS_ERROR("NO robot file path in the ROS parameters.");
        return -1;
    }

    if(controller->initialize(robot_file, init_file) == false)
    {
        ROS_ERROR("ROBOTIS Controller Initialize Fail!");
        return -1;
    }

    if(offset_file != "")
        controller->loadOffset(offset_file);

    sleep(1);
    controller->addMotionModule((robotis_framework::MotionModule*)OnlineWalkingModule::getInstance());
    controller->addMotionModule((robotis_framework::MotionModule*)BaseModule::getInstance());

    controller->startTimer();

    while(ros::ok())
    {
      usleep(1000*1000);
    }



    return 0;
}
