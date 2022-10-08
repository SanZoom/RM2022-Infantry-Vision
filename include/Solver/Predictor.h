/* --------------------------------------------
 * Brief:   使用世界坐标进行预测的卡尔曼滤波器,
 * Author:  Shan Zoom   ——XDU IRobot
 * Date:    2022.3.4
 * -------------------------------------------- */
#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <opencv2/opencv.hpp>
#include "Tools.h"
#include "Mat_time.h"
#include "SpinDetector.h"

class CoordPredictor
{
private:
    std::shared_ptr<cv::KalmanFilter> KF;
    const int measure_num = 3;
    const int state_num = 6;
    double bullet_speed = 15.0;
    uint32_t last_t = 0;

    double delay_time = 0.019; // 发弹延迟、通信延迟等较为固定延迟
    double k = 0.0402;         // 阻力系数
    double g = 9.75;           // 重力加速度
    int iter_num = 40;

    double bs_coeff = 0.9; // 初始弹速系数

public:
    cv::Point3f cam2gyro_offset; // 陀螺仪到相机的平移偏移量
    cv::Point3f gun2cam_offset;  // 枪口相对于相机的平移偏移量
    cv::Point2f pitch_time;      // 弹道补偿抬枪角度以及子弹飞行时间
    cv::Point3f world_coord;
    cv::Point3f correct_coord;

public:
    /**
     *  @brief  计算KF->correct后下一点位置
     *  @param  result KF->correct的结果
     *  @param  time    图片处理延迟 + 子弹飞行时间
     */
    cv::Point3f predictNextpoint(cv::Mat result, float time)
    {
        float t = time;
        float x = result.at<float>(0, 0) + t * result.at<float>(1, 0);
        float y = result.at<float>(2, 0) + t * result.at<float>(3, 0);
        float z = result.at<float>(4, 0) + t * result.at<float>(5, 0);
        return cv::Point3f(x, y, z);
    }

    /**
     *  @brief  计算子弹飞行时间
     *  @param  angle   pitch角
     *  @param  correct_state   KF得出的状态值
     *  @param  T   n-1状态中子弹飞行时间
     *  @param  dt  图片处理延迟
     *  @return 返回子弹在绝对坐标系中飞行的时间；
     */
    double getflytime(double angle, cv::Point3f correct_spin, double T, double dt)
    {
        double x = sqrt(correct_spin.x * correct_spin.x + correct_spin.y * correct_spin.y) / 1000.0;

        return (exp(k * x) - 1.0) / (k * bs_coeff * bullet_speed * cos(angle)); //- dt - delay_time;
    }
    /**
     *  @brief  重力补偿；
     *  @param  bullet_speed    弹速
     *  @param  correct_spin    KF得出的坐标状态值
     *  @param  dt  图片处理时间
     * @return  pitch, time
     */
    cv::Point2d compensate(cv::Point3f correct_spin, double dt)
    {
        correct_spin -= gun2cam_offset;
        double dy, angle, y_actual;
        double t_actual = 0.0;
        double y_temp = correct_spin.z / 1000.0;
        double y = y_temp;
        double x = sqrt(correct_spin.x * correct_spin.x + correct_spin.y * correct_spin.y) / 1000.0;

        for (int i = 0; i < iter_num; i++)
        {
            angle = atan2(y_temp, x);
            t_actual = getflytime(angle, correct_spin, t_actual, dt);
            y_actual = double(bs_coeff * bullet_speed * sin(angle) * t_actual - g * t_actual * t_actual / 2.0);
            dy = y - y_actual;
            y_temp += dy;
            if (abs(dy) < 0.001)
                break;
        }
        return cv::Point2d((angle) / M_PI * 180.0, t_actual);
    }

    void setBS_coeff(cv::Point3f coord)
    {
        if (bullet_speed < 16)
        {
            bs_coeff = 0.85;
        }
        else if (bullet_speed > 17 && bullet_speed < 20)
        {
            bs_coeff = 0.95;
            if (coord.z > 300)
                bs_coeff *= 0.98;
        }
        else if (bullet_speed > 28 && bullet_speed < 32)
        {
            bs_coeff = 1.00;
            if (coord.z > 250)
                bs_coeff *= 0.90;
            if (coord.z > 1000)
                bs_coeff *= 0.88;
        }
        else
            bs_coeff = 0.9;
    }

public:
    /**
     *  @brief  初始化状态值
     *  @param  coord   相机坐标系下目标的坐标
     *  @param  gyro_pose   IMU数据
     */
    void initState(cv::Point3f coord, GyroPose gyro_pose, bool spin_once_flag = false);

    /**
     *  @brief  根据世界坐标进行预测, 对距离滤波
     *  @param  coord   相机坐标系下目标的坐标
     *  @param  gyro_pose   IMU数据
     *  @return 世界坐标系预测点
     */
    cv::Point3f predict(cv::Point3f coord, SpinDetector &spin_detector, GyroPose gyro_pose);

    /**
     *  @brief  根据世界坐标进行预测, 对距离滤波
     *  @param  world_point 绝对坐标
     */
    cv::Point3f predict(cv::Point3f world_point, uint32_t timestamp);

    cv::Point3f predict(cv::Point3f world_point, SpinDetector &spin_detector, uint32_t timestamp);

    /**
     *  @brief  设置弹速等级
     *  @param  bulletspeed 电控发来的弹速等级
     */
    void setBulletSpeed(int bulletspeed);

    CoordPredictor();
    ~CoordPredictor() = default;
};

#endif