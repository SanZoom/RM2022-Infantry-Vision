#ifndef IMAGEPROGRESS_H
#define IMAGEPROGRESS_H

#include "mercure_driver.h"
#include "ArmorDetector.h"
#include "Solver.h"
#include "Predictor.h"
#include "Debug.h"
#include "DebugParam.h"
#include "CanSerial.h"
#include "Rune/RuneDetector.h"
#include "Rune/Fitting.h"
#include <thread>
#include <mutex>
#include <condition_variable>

#define GYRO_BUFFER_NUM     15
#define FRAME_BUFFER_NUM    6

class ImageProgress
{
private:
    camera::MercureDriver cap;  // 大恒相机

    /*---线程控制量---*/
    mutex frame_mutex; // 线程锁
    mutex gyro_mutex;
    mutex send_mutex;
    mutex rune_mutex;
    mutex fitting_mutex;
    condition_variable fitting_condition;
    unique_lock<std::mutex> fitting_lck;

    int ProSegament = 0;
    int GyroSegament = 0;
    int RuneSegament = 0;

    /*---规则参数---*/
    uint16_t _bullet_speed = 15;    // 收到的弹速
    COLOR _enemy_color;             // 敌方颜色
    Mode _mode = Mode::NORMAL;      // 模式 1：常规， 2：大符，3：小符
    // Mode _mode = Mode::RUNE;
    bool allow_rune = false;
    bool allow_normal_rune = false;

    /*---共享资源池---*/
    Mat_time cap_frame;    // 缓存图片
    std::queue<Mat_time> frame_buffer;

    /*---大小符资源池---*/
    RuneArmor last_rune_armor;
    RuneArmor new_rune_armor;
    GyroPose rune_gyro_pose;
    std::vector<cv::Point2f> rune_next_pos;     // 能量机关下一位置

    /*---发送数据---*/
    float send_pitch_yaw[2];    // pitch yaw
    uint32_t send_time;         // IMU时间
    ArmorState armor_state = ArmorState::LOST;
    int send_target_id = 1;

    /*---接收数据---*/
    GyroPose newest_pose;               // 最新陀螺仪数据
    std::vector<GyroPose> gyro_buffer;  // 陀螺仪数据Buffer

    CanSerial can_serial;

    DebugParam debug_param;             // Debug参数

public:
    /**
     *  @brief  从相机读取图片
     */
    void ImageProducer();

    /**
     *  @brief  对图片进行识别处理
     */
    void ImageConsumer2();

    /**
     *  @brief  常规模式检测
     */
    void NormalConsumer(ArmorDetector &detector, Mat_time frame, std::chrono::steady_clock::time_point &last_time);

    /**
     *  @brief  打符模式检测
     */
    void RuneConsumer(RuneDetector &detctor, Mat_time frame, std::chrono::steady_clock::time_point &last_time);

    /**
     *  @brief  大符运动拟合，角度解算
     */
    void Fitting();

    /**
     *  @brief  从can读取数据
     */
    void ReadIMU2();

    /**
     *  @brief  数据发送线程can
     */
    void SendData2();

    /**
     *  @brief  时间同步,给frame附上陀螺仪数据
     *  @return 返回判断插值是否成功
     */
    bool InterGyroPose(Mat_time &frame);
    
    /**
     *  @brief  用于自瞄模式切换
     *  @param  receive_mode    收到的请求指令
     *  @return 如果发生切换了，返回true
     */
    bool switchMode(Mode receive_mode)
    {
        if (receive_mode == _mode)
        {
            _mode = receive_mode;
            return false;
        }
        else
        {
            _mode = receive_mode;
            return true;
        }
    }

    void VideoSave();
    ImageProgress();
    ~ImageProgress() = default;
};

#endif
