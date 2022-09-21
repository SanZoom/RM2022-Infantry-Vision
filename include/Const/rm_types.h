#ifndef RM_TYPE_H
#define RM_TYPE_H
#include <opencv2/opencv.hpp>

namespace RM
{
typedef struct coord_time
{
    cv::Point2f coord;
    uint32_t time;

    coord_time(cv::Point2f coord = cv::Point2f(), uint32_t time = 0)
    {
        this->coord = coord;
        this->time = time;
    }
}coord_time;

typedef struct spin_param
{
    float min_x_delta_ratio;
    float max_x_delta_ratio;
    float min_y_delta_ratio;
    float max_y_delta_ratio;
    float min_yaw_diff;
    int spin_timeout;          // 超时时间

    int min_spin_count;        // 进入反陀螺最小计数值
    
    spin_param()
    {
        cv::FileStorage fs("../Configure/Settings.xml", cv::FileStorage::READ);
        fs["spin_timeout"] >> spin_timeout;
        fs["min_spin_count"] >> min_spin_count;
        fs["min_x_delta_ratio"] >> min_x_delta_ratio;
        fs["max_x_delta_ratio"] >> max_x_delta_ratio;
        fs["min_y_delta_ratio"] >> min_y_delta_ratio;
        fs["max_y_delta_ratio"] >> max_y_delta_ratio;
        fs["min_yaw_diff"] >> min_yaw_diff;
        fs.release();
    }
}spin_param;

typedef struct timeout_clk
{
    uint32_t start_time;
    bool is_started;

    timeout_clk(uint32_t lost_timestamp = 0, bool is_started = false)
    {
        start_time = lost_timestamp;
        this->is_started = is_started;
    }

    void set_start(uint32_t lost_timestamp)
    {
        start_time = lost_timestamp;
        is_started = true;
    }
}timeout_clk;

}; 




#endif