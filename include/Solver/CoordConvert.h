#ifndef MAIN_CONVERT_H
#define MAIN_CONVERT_H

#pragma once
#include <iostream>
#include <math.h>
#include "eigen3/Eigen/Dense"

/**
 *  @brief  获取旋转矩阵 Z-Y-X 顺序旋转
 */
Eigen::Matrix<float, 3, 3> getRotMat(float yaw, float roll, float pitch)
{
    Eigen::Matrix<float, 3, 3> X;
    X << cos(pitch), -sin(pitch), 0,
        sin(pitch), cos(pitch), 0,
        0, 0, 1;

    Eigen::Matrix<float, 3, 3> Y;
    Y << cos(roll), 0, sin(roll),
        0, 1, 0,
        -sin(roll), 0, cos(roll);

    Eigen::Matrix<float, 3, 3> Z;
    Z << 1, 0, 0,
        0, cos(yaw), -sin(yaw),
        0, sin(yaw), cos(yaw);
    return (X * Y * Z).transpose();
}

inline Eigen::Vector3d camera2world(float x, float y, float z, Eigen::Matrix<float, 3, 3> R)
{
    return R * Eigen::Vector3d(x, y, z);
}

#endif // MAIN_CONVERT_H
