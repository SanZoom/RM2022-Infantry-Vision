/*------------------------------------------------------
brief   :   带有时间戳的Mat类，时间标准: chrono::steady_clock
author  :   XDU Robomaster shanzoom
date    :   2021.11.16 
---------------------------------------------------------*/
#ifndef MAT_TIME_H
#define MAT_TIME_H

#include <opencv2/opencv.hpp>
#include <chrono>
using namespace std;

typedef struct GyroPose{
    float q_0 = 0;
    float q_1 = 0;
    float q_2 = 0;
    float q_3 = 0;

    uint32_t timestamp = 0;    
    chrono::steady_clock::time_point local_timestamp;
    chrono::steady_clock::duration time_delay = chrono::milliseconds(0);

    GyroPose(){
        q_0 = 0;
        q_1 = 0;
        q_2 = 0;
        q_3 = 0;
        timestamp = 0;
        local_timestamp = chrono::steady_clock::now() - time_delay;
    };
    GyroPose(float *imu_data, uint32_t t, chrono::steady_clock::time_point local = chrono::steady_clock::now())
    {
        q_0 = imu_data[0];
        q_1 = imu_data[1];
        q_2 = imu_data[2];
        q_3 = imu_data[3];
        
        timestamp = t;
        local_timestamp = local - time_delay;
    }
    
}GyroPose;

class Mat_time : public cv::Mat
{
private:
    chrono::steady_clock::duration time_delay = chrono::milliseconds(4);

public:
    using Image_time = chrono::steady_clock::time_point;

    Image_time produced_time;   // 图片产生时间
    Image_time done_time;       // 图片处理完成时间
    float process_time;         // 图片处理时间，单位: ms

    GyroPose gyro_pose;         // 陀螺仪数据

    using cv::Mat::Mat;

    /**
     *  @brief  设置产生时间
     */
    void setProducedTime();

    /**
     *  @brief  设置处理完成时间
     */
    void setDoneTime();

    /**
     *  @brief  获取处理过程时间
     *  @return 处理过程时间，单位ms
     */
    float getProcessTime();

    /**
     *  @brief  重写copyTo，以完成数据复制
     */
    void copyTo(Mat_time&frame);
};

#endif