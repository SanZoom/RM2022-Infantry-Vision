#ifndef SOLVER_H
#define SOLVER_H

#include <opencv2/opencv.hpp>
#include <const.h>

// 解算参数
typedef struct SolverParam
{   
    cv::Mat camera_matrix;                  // 相机参数
    cv::Mat dist_coeffs;                    // 畸变矩阵
    float small_armor_boxes_real_height;    // 小装甲板的高
    float small_armor_boxes_real_width;     // 小装甲板的宽
    float big_armor_boxes_real_height;      // 大装甲板的高
    float big_armor_boxes_real_width;       // 大装甲板的宽
    float rune_armor_boxes_real_height;     // 能量机关装甲板的高
    float rune_armor_boxes_real_width;      // 能量机关装甲板的宽
    SolverParam();
}SolverParam;

class Solver
{
private:
    std::vector<cv::Point3f>  points_3D;    // 装甲板世界坐标（人为设置）
    std::vector<cv::Point2f>  points_2D;    // 装甲板二维坐标
public:
    SolverParam param;                      // 解算参数
    float yaw;                              // 侧航角（x/z）
    float pitch;                            // 俯仰角（y/z）
    float distance;                         // 距离
    cv::Mat rVec;                           // 旋转矩阵
    cv::Mat tVec;                           // 平移矩阵
    cv::Point2f center_point;               // 目标装甲板的中心点坐标
    cv::Point3f point_3D;                   // 相机坐标系下目标点
    
    typedef ArmorType BoxSize;
    BoxSize box_size;
    

private:

/**
 * @brief 设置2D点集
 * @param _points_2D 进行PNP解算的四个角点
*/
    void set2DPoints(const std::vector<cv::Point2f> &_points_2D);

/**
  * @brief 设置3D点集
  * @param _box_size  装甲板的类型
 */
    void set3DPoints(const BoxSize &_box_size);

public:
/**
  * @brief 进行PNP解算
  * @param _points_2D 进行PNP解算的四个角点
  * @param _box_size  装甲板的类型，不明确时，默认值为小装甲板
 */
    bool solve(const std::vector<cv::Point2f> &_points_2D ,const BoxSize &_box_size);

    Solver()=default;
};


#endif