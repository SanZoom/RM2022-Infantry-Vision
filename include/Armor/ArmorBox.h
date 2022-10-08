#ifndef ARMORBOX_H_
#define ARMORBOX_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include "const.h"
#include "Tools.h"

typedef struct ArmorParam
{
    // 灯条筛选参数
    float lights_angle_max_dif;         // 灯条对角度差最大值
    float lights_length_max_ratio;      // 灯条对长度比最大值
    float lights_Y_max_ratio;           // Y坐标差 / 平均长度
    float armor_width_height_min_ratio; // 装甲板宽高比最小值
    float armor_width_height_max_ratio; // 装甲板宽高比最大值
    float armor_max_angle;              // 装甲板最大倾斜角度
    float armor_inside_thresh;          // 装甲板区域内的最大亮度

    bool erase_wrong_armor;        // 是否滤除错误数字目标
    float armor_number_confidence; // 分类器置信度

    float light_height_width_min_ratio; // 灯条高宽比最小值
    float light_height_width_max_ratio; // 灯条高宽比最大值
    float light_size_area_min_ratio;    // 灯条最小凸包比
    float light_angle_to_vertigal_max;  // 灯条角度与90°差值的最大值

    float blue_red_diff; // B - R
    float red_blue_diff; // R - B

    float blue_light_threshold; // 蓝色灯条所设阈值
    float red_light_threshold;  // 红色灯条所设阈值

    float same_lights_max_dis;       // 判断相同灯条的距离差值
    float same_armors_max_dis_ratio; // 判断相同装甲板的距离差值

    float box_height_enlarge; // 装甲板高度放大系数

    ArmorParam();
} ArmorParam;

/********************* 灯条类定义　************************/
class LightBlob
{
public:
    cv::RotatedRect rect; // 灯条的旋转矩形
    cv::Point up;
    cv::Point down;

public:
    /**
     * @brief 构造函数
     * @param r 旋转矩形
     */
    LightBlob(cv::RotatedRect &r);
    LightBlob() = default;

    /**
     * @brief  析构函数
     */
    ~LightBlob();

private:
    /**
     *  @brief  将灯条的RotatedRect规范为height > width， 角度范围为[0~180°]
     */
    void regularRotated(cv::RotatedRect &r);
};
typedef std::vector<LightBlob> LightBlobs;

/********************* 装甲板类定义　************************/
class ArmorBox
{
public:
    cv::RotatedRect rect;   // 装甲板旋转矩形
    cv::Rect box;           // 装甲板正接矩形
    LightBlobs light_Blobs; // 装甲板的左右灯条 [0]左 [1]右

    float anglediff;

    ArmorType type;     // 装甲板大小类型
    int id;             // 装甲板id
    float confidence;   // 数字识别结果的置信度
    uint32_t timestamp; // 时间戳

public:
    void getPoints(std::vector<cv::Point2f> &);

    /**
     * @brief 构造函数
     * @param blobs 装甲板的一对灯条
     * @param box_id  装甲板id
     */
    ArmorBox(const LightBlob &left = LightBlob(), const LightBlob &right = LightBlob(), uint32_t timestamp = 0, int box_id = 10);

    /**
     * @brief  对装甲板优先级比较
     */
    bool operator>(const ArmorBox &box) const;

    ~ArmorBox();

private:
    /**
     *  @brief  将装甲板的RotatedRect规范为height > width， 角度范围为[0~180°]
     */
    void regularRotated(cv::RotatedRect &r);
};
typedef std::vector<ArmorBox> ArmorBoxes;

#endif
