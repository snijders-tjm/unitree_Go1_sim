#include "RestController.h"
#include "StateCommand.h"
#include "Transformations.h"
#include <ros/ros.h>
#include <fstream>
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;
#define MAX_TILT 0.6
ofstream tunePID;

///////////////////////////////////////////////////////////////////////////////
RestController::RestController(Eigen::Matrix<float, 3, 4> default_stance)
: pid_controller(0.8, 0, 0)
// pid_controller(0.75, 2.29, 0.0)
// pid_controller(0.74, 2.49, 0.00)
//pid_controller(0.72, 0.99, 0.00)
{
    def_stance = default_stance;
    use_button = true;
    use_imu = false;
    max_tilt = MAX_TILT;
    roll_threshold = 0.03;
    pitch_threshold = 0.02;

    openCSV();
}

///////////////////////////////////////////////////////////////////////////////
void RestController::updateStateCommand(const sensor_msgs::Joy::ConstPtr& msg,
        State& state, Command& command)
{

    // robot body local position
//    state.body_local_position[0] = msg->axes[1] * 0.08;   // left joystick up down
//    state.body_local_position[1] = msg->axes[0] * 0.08;   // left joystick left right
//    state.body_local_position[2] = msg->axes[7] * 0.08;   // dpad up down
    Eigen::Vector3f vec1;
    vec1[0] = msg->axes[1] * 0.08;   // left joystick up down
    vec1[1] = msg->axes[0] * 0.08;   // left joystick left right
    vec1[2] = msg->axes[7] * 0.08;   // dpad up down

    Eigen::Vector3f lim;
    lim[0] = 0.03;
    lim[1] = 0.08;
    lim[2] = 0.08;
    for (int i = 0; i < 3; i++){
        if ((double) vec1[i] < -lim[i] ){
            cout << "bruh" << endl;
            state.body_local_position[i] = -lim[i];
        } else if ((double) vec1[i] > lim[i]){
            cout << "aha"<< endl;
            state.body_local_position[i] = (float) lim[i];
        }
        else{
           state.body_local_position[i] = vec1[i];
        }
    }

    // robot body local orientation
    state.body_local_orientation[0] = msg->axes[6] * 0.1; // dpad left right
    state.body_local_orientation[1] = msg->axes[4] * 0.5; // right joystick up down
    state.body_local_orientation[2] = msg->axes[3] * 0.1; // right joystick left right

    if(use_button){
        if(msg->buttons[7]){ // start button
            use_imu = !use_imu;
            use_button = false;
            if(use_imu){
                ROS_INFO("Rest Controller - Use roll/pitch compensation"
                        " : " "true");
            }
            else{
                reset_pid_controller();
                ROS_INFO("Rest Controller - Use roll/pitch compensation"
                        " : " "false");
            }
        }
    }

    if(!use_button)
    {
        if(!(msg->buttons[7])) // start button
            use_button = true;
    }
    if(msg->buttons[8])
    {
        closeCSV();
        ros::shutdown();
    }
}

void RestController::updateVelCommand(const geometry_msgs::Twist::ConstPtr& msg,
                State& state, Command& command) {

    // robot body local position
    state.body_local_position[0] = msg->linear.x * 0.04; 
    state.body_local_position[1] = msg->linear.y * 0.03; 
    state.body_local_position[2] = msg->linear.z * 0.03; 

    // robot body local orientation
    state.body_local_orientation[0] = msg->angular.x * 0.4;
    state.body_local_orientation[1] = msg->angular.y * 0.5;
    state.body_local_orientation[2] = msg->angular.z * 0.4;
	
}
	

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RestController::default_stance()
{
    return def_stance;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RestController::step(State& state, Command& command)
{
    Eigen::Matrix<float, 3, 4> temp = default_stance();
    for(int i = 0; i < 4; i++)
    {
        temp(2,i) = command.robot_height*0.9;//*4;
    }

    if(use_imu && (fabs(state.imu_roll)>=roll_threshold || fabs(state.imu_pitch)>=pitch_threshold))
    {
        Eigen::Vector2f compensation;
        compensation = pid_controller.run(state.imu_roll, state.imu_pitch);
        float roll_comp = -compensation[0];
        float pitch_comp = -compensation[1];

        //std::cout << "ROLL/PITCH vs R_COMP/P_COMP: " << roll_comp << "/" << pitch_comp << " vs " << state.imu_roll << "/" << state.imu_pitch << "\n";

        saveCSV(state, roll_comp, pitch_comp);

        bool bool_use = true;
        if(fabs(roll_comp) > max_tilt || fabs(pitch_comp) > max_tilt){
            bool_use = false;
        }
        if(bool_use)
        {
            // rotation matrix
            Eigen::Matrix3f rot = rotxyz(roll_comp, pitch_comp, 0);
            
            for(int leg_index = 0; leg_index < 4; leg_index++)
            {
                temp.col(leg_index) = rot * temp.col(leg_index); 
            }
        }
        else
        {
            use_imu = false;
            reset_pid_controller();
            ROS_INFO("Rest Controller - Use roll/pitch compensation"
                    " : " "false -> Crossed max tilt boundary");
        }
    }

    return temp;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> RestController::run(State& state, Command& command)
{
    state.foot_locations = step(state, command);
    return state.foot_locations;
}

///////////////////////////////////////////////////////////////////////////////
void RestController::reset_pid_controller()
{
    pid_controller.reset();
}

////////////////////////////////////////////////////////////////////////////////
void RestController::saveCSV(State& state, float rollcomp, float pitchcomp)
{
    tunePID << ros::Time::now() << ";" << rollcomp << ";" << pitchcomp << ";" << state.imu_roll << ";" << state.imu_pitch << "\n";
}

///////////////////////////////////////////////////////////////////////////////
void RestController::openCSV()
{
    tunePID.open("tunePID.csv");
    tunePID << "t;rollcomp;pitchcomp;state.imu_roll;state.imu_pitch\n";
}

///////////////////////////////////////////////////////////////////////////////
void RestController::closeCSV()
{
    tunePID.close();
}
