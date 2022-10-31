/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __ROBOTCONTROLLER_H__
#define __ROBOTCONTROLLER_H__

#include <Eigen/Core>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include "StateCommand.h"
#include "RestController.h"
#include "TrotGaitController.h"
#include "CrawlGaitController.h"
#include "StandController.h"

class RobotController
{
    public:

        // CrawlGaitController class constructor - set body and leg dimensions
        RobotController(const float body[], const float legs[]);

        // Main run function, return leg positions in the base_link_world frame
        Eigen::Matrix<float, 3, 4> run();
        
	// ROS imu callback
        void imu_orientation(const sensor_msgs::Imu::ConstPtr& msg);

    // ROS Joystick callback (for use of buttons to switch controllers)
        void joystick_command(const sensor_msgs::Joy::ConstPtr& msg);

    // ROS Bool callback (for use of buttons to switch controllers)
        void rest_command(const std_msgs::Bool::ConstPtr& msg);

        void trot_command(const std_msgs::Bool::ConstPtr& msg);

        void crawl_command(const std_msgs::Bool::ConstPtr& msg);

        void stand_command(const std_msgs::Bool::ConstPtr& msg);

	// ROS Keyboard callback
        void cmd_vel_command(const geometry_msgs::Twist::ConstPtr& msg);

        // change current controller if requested
        void change_controller();

        // robot's state
        State state;

    private:
        // variables
        float body[2];
        float legs[4];

        float delta_x;
        float delta_y;
        float x_shift_front;
        float x_shift_back;

        // rest controller
        RestController restController;

        // trot gait controller
        TrotGaitController trotGaitController;

        // crawl gait controller
        CrawlGaitController crawlGaitController;

        // stand controller
        StandController standController;

        Command command;

        // return default_stance
        Eigen::Matrix<float, 3, 4> default_stance();
};

#endif
