/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __TRANSFORMATIONS_H__
#define __TRANSFORMATIONS_H__

#include <Eigen/Core>

#define PI 3.1415926535897


// Rotation around x axis
Eigen::Matrix3f rotx(float alpha);

// Rotation around y axis
Eigen::Matrix3f roty(float beta);

// Rotation around z axis
Eigen::Matrix3f rotz(float gamma);

// Rotation around x axis -> y axis -> z axis
Eigen::Matrix3f rotxyz(float alpha, float beta, float gamma);

// Transformation along the x, y, and z axis
Eigen::Matrix4f homog_transxyz(float dx, float dy, float dz);

// 4x4 general transformation matrix
Eigen::Matrix4f homog_transform(float dx, float dy, float dz,
                                float alpha, float beta, float gamma);

// Inverse of a general 4x4 transformation matrix
Eigen::Matrix4f homog_transform_inverse(Eigen::Matrix4f matrix);

#endif
