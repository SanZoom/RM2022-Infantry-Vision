#ifndef CAN_SERIAL_H
#define CAN_SERIAL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <chrono>
#include "const.h"
#include <iostream>

#define CAN_NAME "can0"
#define STATE_SEND_ID 0x106
#define DATA_SEND_ID 0x108

#define IMU_TIME_RECEIVE_ID 0x100    // 时间ID
#define IMU_RECEIVE_ID 0x0FF         // 四元数ID
#define MODE_RECEIVE_ID 0x110        // 模式ID
#define BS_RECEIVE_ID 0x125          // 弹速等级ID
#define ROBOT_RECEIVE_ID 0x129       // 机器人状态信息ID
#define COMPETITION_RECEVIE_ID 0x12B // 比赛信息ID

class CanSerial
{
private:
    int socket_fd;
    struct sockaddr_can addr;
    struct ifreq interface_request;

    u_char PY_data[4]; // pitch 和 yaw

    std::chrono::steady_clock::time_point last_send_state = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_send_data = std::chrono::steady_clock::now();

public:
    /**
     *  @brief  自瞄状态与目标发送接口API
     *  @param  state   识别状态
     *  @param  target_id   目标ID
     */
    int send_state(const ArmorState state, const int target_id, const bool if_shoot = true);

    /**
     *  @brief  自瞄数据发送接口API
     *  @param  pitch_yaw   pitch: 0     yaw: 1
     *  @param  time    所使用的IMU数据的时间
     */
    int send_data(float *pitch_yaw, u_int32_t time);

    /**
     *  @brief  接收数据，根据id区分数据包，需要大于1000Hz频率接收
     *  @return error_code
     */
    int can_receive(uint &id, u_char *buf, u_char &dlc);

    /**
     *  @brief  转化接收的IMU四元数数据
     *  @param  IMU_data    两帧数据拼接
     */
    void transformIMU(u_char *IMU_data, float *imu_data);

    void testTransformIMU(u_char *IMU_data, float *imu_data);

    /**
     *  @brief  转化接收的IMU 时间数据
     */
    void transformIMU_Time(u_char *IMU_time, u_int32_t &time);

    CanSerial();
    ~CanSerial() = default;

    enum CanError
    {
        SUCCESS,
        DLC_ERROR,
        WRITE_ERROR,
        READ_ERROR,
        TIME_ERROR
    };

private:
    /**
     *  @brief  将角度数据转换为uint8以发送
     */
    void transformData(float *data);

    /**
     * @brief   发送数据
     * @param   id  数据对应的ID
     * @param   buf 数据，长度小于等于8
     * @param   dlc 数据长度
     * @return  error_code
     */
    int can_send(uint id, u_char *buf, u_char dlc);
};

#endif
