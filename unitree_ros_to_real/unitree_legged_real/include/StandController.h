/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __STANDCONTROLLER_H__
#define __STANDCONTROLLER_H__

#include <Eigen/Core>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include "StateCommand.h"

class StandController{
    public:
        // StandController class constructor - set default stance,
        StandController(Eigen::Matrix<float, 3, 4> def_stance);

		// ROS joystick callback - update state and other variables
        void updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
                State& state, Command& command);
      
        void updateVelCommand(const geometry_msgs::Twist::ConstPtr& msg,
                State& state, Command& command);
        
        // Main run function, return leg positions in the base_link_world frame
        Eigen::Matrix<float, 3, 4> run(State& state, Command& command);

    private:
        // robot's default stance
        Eigen::Matrix<float, 3, 4> def_stance;

        // return default_stance
        Eigen::Matrix<float, 3, 4> default_stance();

        // Controller step - return new leg positions
        Eigen::Matrix<float, 3, 4> step(State& state, Command& command);

        // FR leg position in the X direction 
        float FR_X;

        // FR leg position in the Y direction 
        float FR_Y;

        // FL leg position in the X direction 
        float FL_X;
        
        // FL leg position in the Y direction 
        float FL_Y;

        // F leg position in the Z direction
        float F_Z;

        // maximal leg reach
        float max_reach;
};

#endif
