#ifndef RUNEBOX_H
#define RUNEBOX_H

#include <chrono>
#include "Tools.h"
#include "Mat_time.h"
#include <iostream>

using namespace std;
using namespace cv;

typedef struct RuneParam
{
    float approx_epsilon;                       // 多边形拟合参数

    float blue_brightness_thresh;
    float blue_color_thresh;

    float red_brightness_thresh;
    float red_color_thresh;

    float blue_red_diff;

    float min_armor_area;                       // 装甲板面积
    float max_armor_area;
    float min_vane_area;                        // 扇叶面积
    float max_vane_area;
    float min_armor_vane_ratio;                 // 装甲板和扇叶面积比
    float max_armor_vane_ratio;
    float armor_min_lw_ratio;                   // 装甲板长宽比
    float armor_max_lw_ratio;
    float vane_min_lw_ratio;                    // 扇叶长宽比
    float vane_max_lw_ratio;
    float vane_min_contour_rrect_ratio;         // 扇叶轮廓面积 / 外接矩形面积
    float vane_max_contour_rrect_ratio;
    
    float min_target_vane_ratio;                // 目标装甲板和扇叶面积比
    float max_target_vane_ratio;
    float target_vane_min_contour_rrect_ratio;  // 目标装甲扇叶轮廓面积 / 外接矩形面积
    float target_vane_max_contour_rrect_ratio;

    float min_shot_armor_vane_ratio;            // 已激活目标装甲板与扇叶的轮廓面积比
    float max_shot_armor_vane_ratio;
    float min_shot_vane_contour_rrect_ratio;    // 已激活目标扇叶轮廓面积 / 外接矩形面积
    float max_shot_vane_contour_rrect_ratio;

    float min_R_area_ratio;                     // R标志面积与装甲板面积最小比
    float max_R_to_predict_distance_ratio;      // 预估圆心与R的距离 / 装甲板长度

    float max_diff_distance_ratio;

    RuneParam();
}RuneParam;

class Vane
{
public:
    int hull_num;               // 轮廓角点数
    RotatedRect rrect;
    vector<Point> contour;
    float c_area;               // 轮廓面积

public:
    ~Vane() = default;
    Vane() = default;
    Vane(vector<Point> vane_contour, float epsilon);
};

class RuneArmor
{
public:
    RotatedRect rrect;
    vector<Point> contour;
    Vane vane;                                  // 装甲板所在扇叶

    float c_area;                               // 轮廓面积
    float area_ratio;                           // 内外轮廓面积比
    Point2f r_direction;                        // 半径方向，圆心指向装甲板

    double roll_angle;                          // PnP解算得到的rVec roll
    double angle;                               // 角度，单位：弧度    
    GyroPose gyro_pose;                         // 时间戳

    cv::Point2f circle_center;                  // 能量机关圆心

private:
/**
 *  @brief  将装甲板的RotatedRect规范为width > height， 角度范围为[0~180°]
 */
    void regularRotated(cv::RotatedRect &r);

public:
    ~RuneArmor() = default;
    RuneArmor() = default;
    RuneArmor(vector<Point> armor_contour, vector<Point> vane_contour, float epsilon, GyroPose _gyro_pose);

/**
 *  @brief  判断是否为同一块装甲板
 */
    bool ifSameArmor(RuneArmor armor_2);

/**
 *  @brief  获取四个角点，顺时针顺序
 */ 
    void getPoints(vector<cv::Point2f>& pts);
    bool operator>(RuneArmor);
    bool operator<(RuneArmor);
};


#endif