/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "Transformations.h"
#include "InverseKinematics.h"
#include <Eigen/Geometry>
#include <vector>
#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

///////////////////////////////////////////////////////////////////////////////
InverseKinematics::InverseKinematics(const float body_dimensions[],
        const float leg_dimensions[])
{
    // body dimensions
    body_length = body_dimensions[0];
    body_width = body_dimensions[1];

    // leg dimensions
    a1 = leg_dimensions[0]; // hip length -> set to 0
    d2 = leg_dimensions[1]; // thigh offset -> set to 0.08
    a3 = leg_dimensions[2]; // thigh length -> set to 0.213
    a4 = leg_dimensions[3]; // calf length -> 0.213
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float, 3, 4> InverseKinematics::get_local_positions(
        Eigen::Matrix<float, 3, 4> leg_positions, float dx, float dy,
        float dz, float roll, float pitch, float yaw)
{
    // make a 4x4 leg position matrix (4 x leg position vector)
    Eigen::Matrix4f leg_positions_matrix;
    leg_positions_matrix.block<3,4>(0,0) = leg_positions;
    for(int i = 0; i < 4; i++)
    {
        leg_positions_matrix(3,i) = 1.0;
    }

    // Transformations matrix, base_link_world => base_link
    Eigen::Matrix4f T_blwbl = homog_transform(dx, dy, dz, roll, pitch, yaw);

    // Transformation matrix, base_link_world => FR1
    Eigen::Matrix4f T_blwFR1 = T_blwbl * homog_transform(
        0.5 * body_length, -0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);

//    Eigen::Matrix4f T_blwFR1 = T_blwbl * homog_transform(
//        0.5 * body_length, -0.5 * body_width, 0.0,
//        0, 0, 0.0);

    
    // Transformation matrix, base_link_world => FL1
    Eigen::Matrix4f T_blwFL1 = T_blwbl * homog_transform(
        0.5 * body_length, 0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);

//    Eigen::Matrix4f T_blwFL1 = T_blwbl * homog_transform(
//        0.5 * body_length, 0.5 * body_width, 0.0,
//        0, 0, 0.0);


    // Transformation matrix, base_link_world => RR1
    Eigen::Matrix4f T_blwRR1 = T_blwbl * homog_transform(
        -0.5 * body_length, -0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);

//    Eigen::Matrix4f T_blwRR1 = T_blwbl * homog_transform(
//        -0.5 * body_length, -0.5 * body_width, 0.0,
//        0, 0, 0.0);

    
    // Transformation matrix, base_link_world => RL1
    Eigen::Matrix4f T_blwRL1 = T_blwbl * homog_transform(
        -0.5 * body_length, 0.5 * body_width, 0.0,
        PI/2, -PI/2, 0.0);

//    Eigen::Matrix4f T_blwRL1 = T_blwbl * homog_transform(
//        -0.5 * body_length, 0.5 * body_width, 0.0,
//        0, 0, 0.0);


    // apply transformations
    // FR
    leg_positions_matrix.col(0) = homog_transform_inverse(T_blwFR1) *
                leg_positions_matrix.col(0);

    // FL
    leg_positions_matrix.col(1) = homog_transform_inverse(T_blwFL1) *
                leg_positions_matrix.col(1);
    
    // RR
    leg_positions_matrix.col(2) = homog_transform_inverse(T_blwRR1) *
                leg_positions_matrix.col(2);

    // RL
    leg_positions_matrix.col(3) = homog_transform_inverse(T_blwRL1) *
                leg_positions_matrix.col(3);

    return leg_positions_matrix.block<3,4>(0,0); 
}

///////////////////////////////////////////////////////////////////////////////
std::vector<double> InverseKinematics::inverse_kinematics(
    Eigen::Matrix<float, 3, 4> leg_positions, float dx, float dy,
    float dz, float roll, float pitch, float yaw)
{
    Eigen::Matrix<float, 3, 4> positions = get_local_positions(
        leg_positions, dx, dy, dz, roll, pitch, yaw);

    //std::cout << "Leg positions: " << positions << std::endl;

    std::vector<double> angles;
    // [x,y,z] here = [z y x] from Rviz frames

    for(int i = 0; i < 4; i++)
    {
        double x = positions.col(i)[0];
        double y = positions.col(i)[1];
        double z = positions.col(i)[2];

        double F = sqrt(x*x + y*y - d2*d2);
        double G = F - a1;
        double H = sqrt(G*G + z*z);

        if (F>1){
            F=1;
        }
        else if (F<-1){
            F=-1;
        }
        double theta1 = atan2(y, x) + atan2(F, d2 * pow((-1), i));
        //std::cout << "Leg " << i << ", x y z: " << x << "; " << y <<"; " << z << std::endl;
        //std::cout << "Leg " << i << ", theta1 : " << theta1 << std::endl;

        double D = (H*H - a3*a3 - a4*a4) / (2*a3*a4);
        if (D>1){
            D=1;
        }
        else if (D<-1){
            D=-1;
        }
        
        double theta4 = -atan2((sqrt(1-D*D)), D);

        double theta3 = atan2(z,G) - atan2(a4 * sin(theta4),
                a3 + a4 * cos(theta4));

        angles.push_back(theta1);
        angles.push_back(theta3);
        angles.push_back(theta4);
    }

    return angles;
}

//
std::vector<double> InverseKinematics::inverse_kinematics2(
    Eigen::Matrix<float, 3, 4> leg_positions, float dx, float dy,
    float dz, float roll, float pitch, float yaw)
{
    Eigen::Matrix<float, 3, 4> positions = get_local_positions(
        leg_positions, dx, dy, dz, roll, pitch, yaw);

    //std::cout << "Leg positions: " << positions << std::endl;

    std::vector<double> angles;
    Eigen::RowVector3f y_offset;
    Eigen::Vector3f xyz, posi;

    for(int i = 0; i < 4; i++)
    {
        float bruh = d2 * pow((-1), i);
        y_offset = Eigen::RowVector3f(bruh,0,0);
        double x = positions.col(i)[0];
        double y = positions.col(i)[1];
        double z = positions.col(i)[2];

        double D = (pow(x,2) + pow(y,2) + pow(z,2) - pow(d2,2) - pow(a3,2) - pow(a4,2))/(2*a3*a4);

        if (D>1){
            D=1;
        }
        else if (D<-1){
            D=-1;
        }

        double theta4 =  -atan2((sqrt(1-pow(D,2))), D);
        double prot = pow(x,2)+pow(y,2)-pow(d2,2);

        if (prot<0) {prot=0;}

        double theta3 = atan2(z,prot)-atan2(a4*sin(theta4),a3+a4*cos(theta4));
        double theta1 = atan2(y,x)+atan2(sqrt(prot),bruh);
//        double F = sqrt(x*x + y*y - d2*d2);
//        double G = F - a1;
//        double H = sqrt(G*G + z*z);

//        double theta1 = atan2(y, x) + atan2(F, d2 * pow((-1), i));
//        std::cout << "Leg " << i << ", x y z: " << x << "; " << y <<"; " << z << std::endl;

//        float theta = theta1;
//        posi = rotz(theta) * Eigen::Vector3f(positions.col(i)[0],positions.col(i)[1],positions.col(i)[2]);
//        xyz = y_offset * rotz(theta);


//        double x2 = posi[0] - xyz[0];
//        double y2 = posi[1] - xyz[1];
//        double z2 = posi[2] - xyz[2];

//        std::cout << "Leg " << i << ", x2 y2 z2: " << x2 << ", " << y2 << ", " << z2 << std::endl;
//        std::cout << "Leg " << i << ", theta1 : " << theta1 << std::endl;
//        F = sqrt(x*x + y*y-(d2*d2));
//        G = F - a1;
//        H = sqrt(G*G + z*z);

//        double F2 = sqrt(x2*x2 + y2*y2);
//        double G2 = F2 - a1;
//        double H2 = sqrt(G2*G2 + z2*z2);
//        double D2 = (H2*H2 - a3*a3 - a4*a4) / (2*a3*a4);

//        double D = (H*H - a3*a3 - a4*a4) / (2*a3*a4);

//        double theta4alt = -atan2((sqrt(1-D2*D2)), D2);

//        double theta3alt = atan2(z2,G2) - atan2(a4 * sin(theta4alt),
//                a3 + a4 * cos(theta4alt));

//        double theta4 = -atan2((sqrt(1-D*D)), D);

//        double theta3 = atan2(z,G) - atan2(a4 * sin(theta4),
//                a3 + a4 * cos(theta4));

        angles.push_back(theta1);
        angles.push_back(theta3);
        angles.push_back(theta4);
    }

    return angles;
}
