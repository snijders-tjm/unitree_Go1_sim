#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include "InverseKinematics.h"
#include "Transformations.h"
#include "RobotController.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include "body.h"
#include <fstream>

#define RATE 60

using namespace std;
using namespace unitree_model;

bool start_up = true;

class multiThread
{
public:
    multiThread(string rname){
        robot_name = rname;
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    {
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;

        lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z;

    }

    void FRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].q = msg.q;
        lowState.motorState[0].dq = msg.dq;
        lowState.motorState[0].tauEst = msg.tauEst;
    }

    void FRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].q = msg.q;
        lowState.motorState[1].dq = msg.dq;
        lowState.motorState[1].tauEst = msg.tauEst;
    }

    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].q = msg.q;
        lowState.motorState[2].dq = msg.dq;
        lowState.motorState[2].tauEst = msg.tauEst;
    }

    void FLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].q = msg.q;
        lowState.motorState[3].dq = msg.dq;
        lowState.motorState[3].tauEst = msg.tauEst;
    }

    void FLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].q = msg.q;
        lowState.motorState[4].dq = msg.dq;
        lowState.motorState[4].tauEst = msg.tauEst;
    }

    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].q = msg.q;
        lowState.motorState[5].dq = msg.dq;
        lowState.motorState[5].tauEst = msg.tauEst;
    }

    void RRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].q = msg.q;
        lowState.motorState[6].dq = msg.dq;
        lowState.motorState[6].tauEst = msg.tauEst;
    }

    void RRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].q = msg.q;
        lowState.motorState[7].dq = msg.dq;
        lowState.motorState[7].tauEst = msg.tauEst;
    }

    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].q = msg.q;
        lowState.motorState[8].dq = msg.dq;
        lowState.motorState[8].tauEst = msg.tauEst;
    }

    void RLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].q = msg.q;
        lowState.motorState[9].dq = msg.dq;
        lowState.motorState[9].tauEst = msg.tauEst;
    }

    void RLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].q = msg.q;
        lowState.motorState[10].dq = msg.dq;
        lowState.motorState[10].tauEst = msg.tauEst;
    }

    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].q = msg.q;
        lowState.motorState[11].dq = msg.dq;
        lowState.motorState[11].tauEst = msg.tauEst;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }

private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub;
    string robot_name;
};

int main(int argc, char* argv[])
{
    // robot body dimensions - body length, body width
    const float body_dimensions[] = {0.3762, 0.0935}; // 2*leg offset x, 2*leg offset y

    // robot leg dimensions - l1, l2, l3, l4
    const float leg_dimensions[] = {0.0, 0.08, 0.213, 0.213};

    // robot joint desired angles
    double joint_angles[12];
    
    // ROS node initialization
    ros::init(argc, argv, "Robot_Controller");
    ros::NodeHandle n;

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;

    multiThread listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(0); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    // RobotController
    RobotController go1(body_dimensions, leg_dimensions);

    ros::Subscriber dsa = n.subscribe("/trunk_imu", 1,
            &RobotController::imu_orientation, &go1);

    ros::Subscriber dsd = n.subscribe("/twist_mux/cmdvel", 1,
            &RobotController::cmd_vel_command, &go1);

    ros::Subscriber dsd2 = n.subscribe("/go1_joy/joy_ramped", 1,
            &RobotController::joystick_command, &go1);
// ---------------------------------------------------------------------
    // axes length = 8: 0: left joystick (L=1; R=-1)
    //                  1: left joystick (Up=1;Down=-1)
    //                  2: left (big) trigger (press=-1;release=1)
    //                  3-5 : vice versa for right side
    //                  6: dpad (left=1;right=-1;none=0)
    //                  7: dpad (up=1;down=-1;none=0) <-- press hard
    // button length = 11 (binary, i.e., press=1, no press = 0)
    //                  0: A-button
    //                  1: B-button
    //                  2: X-button
    //                  3: Y-button
    //                  4: left (rear) button
    //                  5: right (rear) button
    //                  6: back button
    //                  7: start button
    //                  8: Xbox button
    //                  9: press left joystick
    //                  10: press right joystick
// -----------------------------------------------------------------------

    ros::Subscriber dsde = n.subscribe("rest_cmd", 1,
            &RobotController::rest_command, &go1);

    ros::Subscriber dsdf = n.subscribe("trot_cmd", 1,
            &RobotController::trot_command, &go1);

    ros::Subscriber dsdg = n.subscribe("crawl_cmd", 1,
            &RobotController::crawl_command, &go1);

    ros::Subscriber dsdh = n.subscribe("stand_cmd", 1,
            &RobotController::stand_command, &go1);



    // Gazebo command publishers
    ros::Publisher lowState_pub; //for rviz visualization
    // ros::Rate loop_rate(1000);
    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);

    unitree_model::motion_init();
    // Inverse Kinematics
    InverseKinematics go1_IK(body_dimensions, leg_dimensions);
    usleep(3000000); // must wait 300ms, to get first state

    long long time_ms = 0;  //time, ms
    ofstream timePoseData;
    timePoseData.open("RPYTommy_" + robot_name + "_gazeboRPY.csv");
    timePoseData << "t_ms;t_ros_secs;roll;pitch;rollref;pitchref;thighFR;thighFL;RRthigh;RLthigh \n";

    // main while loop rate
    ros::Rate loop_rate(RATE);
    while(ros::ok())
    {
        // new leg positions
        Eigen::Matrix<float, 3, 4> leg_positions = go1.run();
        go1.change_controller();

        // body local position
        float dx = go1.state.body_local_position[0];
        float dy = go1.state.body_local_position[1];
        float dz = go1.state.body_local_position[2];

        //std::cout << "local body pos: " << go1.state.body_local_position[0] << ", " << go1.state.body_local_position[1] << ", " << go1.state.body_local_position[2] << "\n";

        // body local orientation
        float roll = go1.state.body_local_orientation[0];
        float pitch = go1.state.body_local_orientation[1];
        float yaw = go1.state.body_local_orientation[2];

        //std::cout << "local body orien: " << go1.state.body_local_orientation[0] << ", " << go1.state.body_local_orientation[1] << ", " << go1.state.body_local_orientation[2] << "\n";

        // inverse kinematics -> joint angles
        std::vector<double> angles = go1_IK.inverse_kinematics(leg_positions,
                dx, dy, dz, roll, pitch, yaw);

        // publish joint angle commands
        for(int i = 0; i < 12; i++)
        {
            joint_angles[i] = angles[i];

            if(isnan(angles[i]))
            {
                //std::cout << "Nu in MAIN CPP WHILE LOOP MET ANGLES" << angles[i] << "\n";
                angles[i] = lowState.motorState[i].q; //NOTE: MOET JOINT_ANGLES[i] ZIJN!!!
//                publishers[i].publish(command_message);
            }
            lowCmd.motorCmd[i].q = joint_angles[i];
        }
        double secs = ros::Time::now().toSec();
        timePoseData << time_ms << ";" << secs << ";" << go1.state.imu_roll << ";" << go1.state.imu_roll << ";" <<  0 << ";" << 0 << ";" << angles[1] << ";" << angles[4] << ";" << angles[7] << ";" << angles[10] << "\n";
        unitree_model::sendServoCmd();
        lowState_pub.publish(lowState); // For updating rviz in gazebo
        time_ms += 1;

        // spin
        ros::spinOnce();
        
        // sleep
        loop_rate.sleep();
    }
    timePoseData.close();
    return 0;
}
