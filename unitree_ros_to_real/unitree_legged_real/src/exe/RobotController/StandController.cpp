#include "StandController.h"
#include "StateCommand.h"
#include <ros/ros.h>

///////////////////////////////////////////////////////////////////////////////
StandController::StandController(Eigen::Matrix<float, 3, 4> default_stance)
{
    def_stance = default_stance;

    FR_X = 0;
    FR_Y = 0;
    FL_X = 0;
    FL_Y = 0;
    F_Z = 0;
    max_reach = 0.08;
}

///////////////////////////////////////////////////////////////////////////////
void StandController::updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
        State& state, Command& command)
{
    // robot body local position
    state.body_local_position[0] = std::min(msg->axes[7] * 0.18, 0.08); // dpad up down

    FR_X = msg->axes[4]; // right joystick up down
    FR_Y = msg->axes[3]; // right joystick left right
    FL_X = msg->axes[1]; // left joystick up down
    FL_Y = msg->axes[0]; // left joystick left right
    F_Z = msg->axes[6]; // dpad left right


    if (msg->buttons[8]){
    ros::shutdown();
    }
}

void StandController::updateVelCommand(const geometry_msgs::Twist::ConstPtr& msg,
                State& state, Command& command){
    // robot body local position
    //state.body_local_position[0] = msg->axes[7] * 0.14; 

    //FR_X = msg->axes[1];
    //FR_Y = msg->axes[0];
    //FL_X = msg->axes[4];
    //FL_Y = msg->axes[3];

}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> StandController::default_stance()
{
    return def_stance;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> StandController::step(State& state, Command& command)
{
    Eigen::Matrix<float, 3, 4> temp = default_stance();
    for(int i = 0; i < 4; i++)
    {
        temp(2,i) = command.robot_height*1.2;//*4;
    }
    temp(0,0) += FR_X * max_reach;
    temp(0,1) += FL_X * max_reach;

    temp(1,0) += FR_Y * max_reach;
    temp(1,1) += FL_Y * max_reach;

    temp(2,0) += F_Z * max_reach;
    temp(2,1) += F_Z * max_reach;

    return temp;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> StandController::run(State& state, Command& command)
{
    state.foot_locations = step(state, command);
    return state.foot_locations;
}
