#ifndef TOOLS_H
#define TOOLS_H

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
using namespace cv;

/**
 *  @brief  Gamma算法，用于调整图片的明暗程度
 *  @param  src 原图像
 *  @param  dst 输出图像
 *  @param  fGamma  gamma参数，(大于1.0时整体变亮，小于1.0时整体变暗)
 */
void Gamma(Mat src, Mat &dst, float fGamma);

float calDistance(cv::Point2f pt1, cv::Point2f pt2);


/**
 *  @brief  通过四元数转换坐标系
 */
    cv::Point3f camera2world(Eigen::Quaternionf q, cv::Point3f point, cv::Point3f trans_offset = cv::Point3f(0,0,0));

/**
 *  @brief  世界坐标系转换到相机坐标系
 */
    cv::Point3f world2camera(Eigen::Quaternionf q, cv::Point3f point, cv::Point3f trans_offset = cv::Point3f(0,0,0));

/**
 *  @brief  规范旋转矩形, height > width
 */
    void regularRotated(cv::RotatedRect &r);

#endif