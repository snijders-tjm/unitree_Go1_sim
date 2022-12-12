#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include "InverseKinematics.h"
#include "Transformations.h"
#include "RobotController.h"
#include <geometry_msgs/WrenchStamped.h>
#include <vector>
#include <string>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include "body.h"
#include <fstream>

using namespace UNITREE_LEGGED_SDK;

using namespace std;
using namespace unitree_model;

bool start_up = true;

class multiThread
{
public:
    multiThread(string rname){
        robot_name = rname;
        sub_low = nm.subscribe("low_state", 1, &multiThread::lowStateCallback, this);
//        imu_sub = nm.subscribe("imu", 1, &multiThread::imuCallback, this);
//        servo_sub[0] = nm.subscribe("low_state", 1, &multiThread::FRhipCallback, this);
//        servo_sub[1] = nm.subscribe("low_state", 1, &multiThread::FRthighCallback, this);
//        servo_sub[2] = nm.subscribe("low_state", 1, &multiThread::FRcalfCallback, this);
//        servo_sub[3] = nm.subscribe("low_state", 1, &multiThread::FLhipCallback, this);
//        servo_sub[4] = nm.subscribe("low_state", 1, &multiThread::FLthighCallback, this);
//        servo_sub[5] = nm.subscribe("low_state", 1, &multiThread::FLcalfCallback, this);
//        servo_sub[6] = nm.subscribe("low_state", 1, &multiThread::RRhipCallback, this);
//        servo_sub[7] = nm.subscribe("low_state", 1, &multiThread::RRthighCallback, this);
//        servo_sub[8] = nm.subscribe("low_state", 1, &multiThread::RRcalfCallback, this);
//        servo_sub[9] = nm.subscribe("low_state", 1, &multiThread::RLhipCallback, this);
//        servo_sub[10] = nm.subscribe("low_state", 1, &multiThread::RLthighCallback, this);
//        servo_sub[11] = nm.subscribe("low_state", 1, &multiThread::RLcalfCallback, this);
    }

    void lowStateCallback (const unitree_legged_msgs::LowState &msg){
        lowState = msg;
    }
//    void imuCallback(const unitree_legged_msgs::IMU & msg)
//    {
//        lowState.imu.quaternion[0] = msg.quaternion[0];
//        lowState.imu.quaternion[1] = msg.quaternion[1];
//        lowState.imu.quaternion[2] = msg.quaternion[2];
//        lowState.imu.quaternion[3] = msg.quaternion[3];

//        lowState.imu.gyroscope[0] = msg.gyroscope[0];
//        lowState.imu.gyroscope[1] = msg.gyroscope[1];
//        lowState.imu.gyroscope[2] = msg.gyroscope[2];

//        lowState.imu.accelerometer[0] = msg.accelerometer[0];
//        lowState.imu.accelerometer[1] = msg.accelerometer[1];
//        lowState.imu.accelerometer[2] = msg.accelerometer[2];

//    }

//    void FRhipCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        start_up = false;
//        lowState.motorState[0].mode = msg.mode;
//        lowState.motorState[0].q = msg.q;
//        lowState.motorState[0].dq = msg.dq;
//        lowState.motorState[0].tauEst = msg.tauEst;
//    }

//    void FRthighCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.motorState[1].mode = msg.mode;
//        lowState.motorState[1].q = msg.q;
//        lowState.motorState[1].dq = msg.dq;
//        lowState.motorState[1].tauEst = msg.tauEst;
//    }

//    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.motorState[2].mode = msg.mode;
//        lowState.motorState[2].q = msg.q;
//        lowState.motorState[2].dq = msg.dq;
//        lowState.motorState[2].tauEst = msg.tauEst;
//    }

//    void FLhipCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        start_up = false;
//        lowState.motorState[3].mode = msg.mode;
//        lowState.motorState[3].q = msg.q;
//        lowState.motorState[3].dq = msg.dq;
//        lowState.motorState[3].tauEst = msg.tauEst;
//    }

//    void FLthighCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.motorState[4].mode = msg.mode;
//        lowState.motorState[4].q = msg.q;
//        lowState.motorState[4].dq = msg.dq;
//        lowState.motorState[4].tauEst = msg.tauEst;
//    }

//    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.motorState[5].mode = msg.mode;
//        lowState.motorState[5].q = msg.q;
//        lowState.motorState[5].dq = msg.dq;
//        lowState.motorState[5].tauEst = msg.tauEst;
//    }

//    void RRhipCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        start_up = false;
//        lowState.motorState[6].mode = msg.mode;
//        lowState.motorState[6].q = msg.q;
//        lowState.motorState[6].dq = msg.dq;
//        lowState.motorState[6].tauEst = msg.tauEst;
//    }

//    void RRthighCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.motorState[7].mode = msg.mode;
//        lowState.motorState[7].q = msg.q;
//        lowState.motorState[7].dq = msg.dq;
//        lowState.motorState[7].tauEst = msg.tauEst;
//    }

//    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.motorState[8].mode = msg.mode;
//        lowState.motorState[8].q = msg.q;
//        lowState.motorState[8].dq = msg.dq;
//        lowState.motorState[8].tauEst = msg.tauEst;
//    }

//    void RLhipCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        start_up = false;
//        lowState.motorState[9].mode = msg.mode;
//        lowState.motorState[9].q = msg.q;
//        lowState.motorState[9].dq = msg.dq;
//        lowState.motorState[9].tauEst = msg.tauEst;
//    }

//    void RLthighCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.motorState[10].mode = msg.mode;
//        lowState.motorState[10].q = msg.q;
//        lowState.motorState[10].dq = msg.dq;
//        lowState.motorState[10].tauEst = msg.tauEst;
//    }

//    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.motorState[11].mode = msg.mode;
//        lowState.motorState[11].q = msg.q;
//        lowState.motorState[11].dq = msg.dq;
//        lowState.motorState[11].tauEst = msg.tauEst;
//    }
// Unknown how to obtain FootForce data appropriately

//    void FRfootCallback(const unitree_legged_msgs::LowState& msg)
//    {
//        lowState.eeForce[0].x = msg.wrench.force.x;
//        lowState.eeForce[0].y = msg.wrench.force.y;
//        lowState.eeForce[0].z = msg.wrench.force.z;
//        lowState.footForce[0] = msg.wrench.force.z;
//    }

//    void FLfootCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.eeForce[1].x = msg.wrench.force.x;
//        lowState.eeForce[1].y = msg.wrench.force.y;
//        lowState.eeForce[1].z = msg.wrench.force.z;
//        lowState.footForce[1] = msg.wrench.force.z;
//    }

//    void RRfootCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.eeForce[2].x = msg.wrench.force.x;
//        lowState.eeForce[2].y = msg.wrench.force.y;
//        lowState.eeForce[2].z = msg.wrench.force.z;
//        lowState.footForce[2] = msg.wrench.force.z;
//    }

//    void RLfootCallback(const unitree_legged_msgs::MotorState& msg)
//    {
//        lowState.eeForce[3].x = msg.wrench.force.x;
//        lowState.eeForce[3].y = msg.wrench.force.y;
//        lowState.eeForce[3].z = msg.wrench.force.z;
//        lowState.footForce[3] = msg.wrench.force.z;
//    }

private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, sub_low;
    string robot_name;
};

int main(int argc, char **argv)
{
    // robot body dimensions - body length, body width
    const float body_dimensions[] = {0.3762, 0.0935}; // 2*leg offset x, 2*leg offset y

    // robot leg dimensions - l1, l2, l3, l4
    const float leg_dimensions[] = {0.0, 0.08, 0.213, 0.213};

    // robot joint desired angles
    double joint_angles[12];

    ros::init(argc, argv, "low_level_code_tommy");

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(60);

    cout << "Initializing program..." << endl;

    multiThread listen_publish_obj("go1");
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

    int rate_count = 0;
    int rate_count2 = 0;
    int rate_count3 = 0;
    float qInit[12] = {0};
    float qDes[12] = {0};
    float Kp[12] = {0};
    float Kd[12] = {0};

    int count = 0;

    ros::Publisher pub = n.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
///////////////////////////////////////////////////////////
    lowCmd.head[0] = 0xFE;
    lowCmd.head[1] = 0xEF;
    lowCmd.levelFlag = LOWLEVEL;
    for (int i=0;i < 12;i++){
        lowCmd.motorCmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
    }

    float targetPos[12] = {0.0, 1.27, -2.69, -0.0, 1.27, -2.69,
                      0.0, 1.27, -2.69, -0.0, 1.27, -2.69};
    float stand[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};

    // Inverse Kinematics
    InverseKinematics go1_IK(body_dimensions, leg_dimensions);
    usleep(3000000); // must wait 300ms, to get first state

    long long time_ms = 0;  //time, ms
    ofstream timePoseData;
    timePoseData.open("RPYTommy_TUNING.csv");

    timePoseData << "time";
    for (int i=0;i<12;i++){
        timePoseData << ";q" << i << ";qref" << i ;
    }
    timePoseData << "\n";

    while (ros::ok())
    {
        count++;
        if( count >= 0){
            // first, get record initial position
            if( count >= 0 && count < 10){
                cout << "count 0-10" << endl;
                for(int j=0; j<12; j++){
                    qInit[j] = lowState.motorState[j].q;
                }
            }
            // second, move to the origin point of a sine movement with Kp Kd
            if( count >= 10 && count < 400){
                cout << "count 10-400" << endl;
                rate_count++;
                double rate = rate_count/200.0;                       // needs count to 200
                for(int i=0;i<12;i++){
                    if (i%3==0){ // hips
                        Kp[i]=15;
                        Kd[i]=5;
                        }
                    else if(i%3==1){//thighs
                        Kp[i]=15;
                        Kd[i]=5;
                    }
                    else{ //calfs
                        Kp[i]=15;
                        Kd[i]=5;
                    }
                    qDes[i] = jointLinearInterpolation(qInit[i], targetPos[i], rate);
                }
            }
            // last, do sine wave
            if( count >= 400 && count < 800){
                cout << "count 400+" << endl;
                rate_count2++;
                double rate = rate_count2/200.0;                       // needs count to 200
                for(int i=0;i<12;i++){// hips
                    if (i%3==0){
                        Kp[i]=15;
                        Kd[i]=5;
                    }
                    else if(i%3==1){//thighs
                        Kp[i]=15;
                        Kd[i]=5;
                    }
                    else{//calfs
                        Kp[i]=15;
                        Kd[i]=5;
                    }
                    qDes[i] = jointLinearInterpolation(targetPos[i], stand[i], rate);
                }
            }

            if( count >= 800){
                cout << "count 400+" << endl;
                rate_count3++;
                double rate = rate_count3/200.0;                       // needs count to 200
                for(int i=0;i<12;i++){// hips
                    if (i%3==0){
                        Kp[i]=11;
                        Kd[i]=3;
                    }
                    else if(i%3==1){//thighs
                        Kp[i]=9;
                        Kd[i]=3;
                    }
                    else{//calfs
                        Kp[i]=14;
                        Kd[i]=4;
                    }
                    qDes[i] = jointLinearInterpolation(stand[i], targetPos[i], rate);
                }
            }
            timePoseData << count;
            for (int i=0; i < 12;i++){
                lowCmd.motorCmd[i].q = qDes[i];
                lowCmd.motorCmd[i].dq = 0;
                lowCmd.motorCmd[i].Kp = Kp[i];
                lowCmd.motorCmd[i].Kd = Kd[i];
                timePoseData << ";" << lowState.motorState[i].q << ";" << lowCmd.motorCmd[i].q;
            }
            timePoseData << "\n";

            for (int i=0;i<12;i++){
                if (i%3==0){
                    continue;
                }
                else{
                    lowCmd.motorCmd[i].tau = 0.0f;
                }
            }
        }

        pub.publish(lowCmd);

        ros::spinOnce();
        loop_rate.sleep();
    }
    timePoseData.close();
    return 0;
}
