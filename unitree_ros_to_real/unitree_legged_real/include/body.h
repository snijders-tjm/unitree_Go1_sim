/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

#include "ros/ros.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/HighState.h"
#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model {

extern unitree_legged_msgs::LowCmd lowCmd;
extern unitree_legged_msgs::LowState lowState, basePose;

extern ros::Publisher pub;
extern ros::Publisher highState_pub;

void paramInit();
void stand();
void motion_init();
void sendServoCmd();
void moveAllPosition(double* jointPositions, double duration);
double jointLinearInterpolation(double initPos, double targetPos, double rate);
}

#endif
