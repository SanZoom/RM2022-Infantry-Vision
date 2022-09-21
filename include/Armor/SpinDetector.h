#ifndef SPINDETECTOR_H
#define SPINDETECTOR_H

#include "const.h"
#include "Tools.h"
#include "rm_types.h"
#include "Mat_time.h"
#include "ArmorBox.h"
#include <vector>

using namespace std;

#define ARMOR_WINDOW_LENGTH 200
#define ARMOR_TO_JUDGE_LENGTH  2

class SpinDetector{
private:
    vector<ArmorBox> armor_info_window;         // 滑动窗口
    vector<cv::Point3f> coord_info_window; 
    vector<uint32_t> spin_timestamp_list;       // 跳变帧序列
    // const double shoot_time = 0.05;          // 发弹区间
    float last_yaw_diff;
    vector<float> yaw_window;

public:
    cv::Point3f mean_point;
    RM::spin_param param;                       // 参数
    bool is_spin;
    bool spin_once_flag;
    bool fire_flag;

private:
    /**
     *  @brief  判断是符否合陀螺状态规定
     */
    bool ifLegal();

public:
    /**
     *  @brief  清空状态
     */
    void reset();

    /**
     *  @brief  判断目标是否处于陀螺状态
     *  @param  coord   坐标
     *  @param  time    陀螺仪时间戳
     *  @return 是否进入反小陀螺策略
     */
    bool judgeSpin(ArmorBox armor, bool if_spin=false);

    bool judgeSpin(float yaw, uint32_t timestamp);

    /**
     *  @brief  添加滤波结果的三维坐标
     */
    void pushCoord(cv::Point3f coord);

    /**
     *  @brief  获取坐标均值
     *  @param  coord   结果参数，坐标均值
     *  @return 是否成功获取坐标均值
     */
    bool solverSpin(cv::Point3f &coord);

    SpinDetector();
    ~SpinDetector() = default;
};


#endif