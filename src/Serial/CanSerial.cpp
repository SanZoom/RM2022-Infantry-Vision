#include "CanSerial.h"
#include <math.h>

int CanSerial::send_state(const ArmorState state, const int target_id, bool if_shoot)
{
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_send_state).count() < 4)
        return TIME_ERROR;
    last_send_state = std::chrono::steady_clock::now();

    u_char buf[2] = {0};

    switch (target_id)
    {
    case 0: // 基地
        buf[1] = 128;
        break;
    case 1: // 英雄
        buf[1] = 1;
        break;
    case 2: // 工程
        buf[1] = 2;
        break;
    case 3: // 步兵
        buf[1] = 4;
        break;
    case 4:
        buf[1] = 8;
        break;
    case 5:
        buf[1] = 16;
        break;
    case 6: // 哨兵
        buf[1] = 64;
        break;
    case 7: // 前哨站
        buf[1] = 32;
        break;
    default:
        buf[1] = 0;
        break;
    }

    switch (state)
    {
    case ArmorState::LOST:
    // case ArmorState::FINDING:
        buf[0] = 0;
        buf[1] = 0;
        break;
    
    default:
        buf[0] = 3;
        break;
    }
    if (if_shoot)
        buf[0] |= 0x02;
    else 
        buf[0] &= 0xFD;

    return can_send(STATE_SEND_ID, buf, 2);
}

int CanSerial::send_data(float *pitch_yaw, u_int32_t time)
{
    if (std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - last_send_data).count() < 800)
        return TIME_ERROR;
    last_send_data = std::chrono::steady_clock::now();
    
    transformData(pitch_yaw);
    u_char buf[8] = {0};
    memcpy(buf, PY_data, 4);
    memcpy(&buf[4], &time, 4);

    return can_send(DATA_SEND_ID, buf, 8);
}

void CanSerial::transformData(float *data)
{
    int16_t pitch = (int16_t)(data[0] * 32768.0 / 180.0);
    int16_t yaw = (int16_t)(-data[1] * 32768.0 / 180.0);

    for (int i = 0; i < 4; i++) PY_data[i] = 0;

    PY_data[0] |= pitch;
    PY_data[1] |= (pitch >> 8);
    PY_data[2] |= yaw;
    PY_data[3] |= (yaw >> 8);
}

void CanSerial::transformIMU(u_char *IMU_data, float *imu_data)
{
    memcpy(imu_data, IMU_data, sizeof(u_char) * 16);
}

void CanSerial::testTransformIMU(u_char *IMU_data, float *imu_data)
{
    int16_t q[4] = {0};
    memcpy(q, IMU_data, 8);
    
    float sum = 0.0;
    for (int i = 0; i < 4; i++)
    {
        imu_data[i] = (float)q[i] / 32768.0;
        sum += imu_data[i] * imu_data[i];
    }
    for (int i = 0; i < 4; i++)
    {
        imu_data[i] /= sqrt(sum);
    }
}

void CanSerial::transformIMU_Time(u_char *IMU_time, uint32_t &time)
{
    time = 0;
    memcpy(&time, IMU_time, 4);
}

int CanSerial::can_send(uint id, u_char *buf, u_char dlc)
{
    if (dlc > 8)    return DLC_ERROR;

    struct can_frame send_frame;

    send_frame.can_id = id;
    send_frame.can_dlc = dlc;

    for (int i = 0; i < (int)dlc; i++)
        send_frame.data[i] = buf[i];
    
    int t = write(socket_fd, &send_frame, sizeof(send_frame));
    if (t > 0)  return SUCCESS;
    return WRITE_ERROR;
}

int CanSerial::can_receive(uint &id, u_char *buf, u_char &dlc)
{
    struct can_frame frame;
    int t = read(socket_fd, &frame, sizeof(frame));
    if (t <= 0) return READ_ERROR;

    id = frame.can_id;
    dlc = frame.can_dlc;

    memcpy(buf, frame.data, dlc);
    return SUCCESS;
}

CanSerial::CanSerial()
{
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    // 配置 Socket CAN 为非阻塞IO
    int flags=fcntl(socket_fd, F_GETFL, 0); 
    flags |= O_NONBLOCK;
    fcntl(socket_fd, F_SETFL, flags);

    // 指定can设备
    strcpy(interface_request.ifr_name, CAN_NAME);
    ioctl(socket_fd, SIOCGIFINDEX, &interface_request);
    addr.can_family = AF_CAN;
    addr.can_ifindex = interface_request.ifr_ifindex;
    
    // 将套接字与can设备绑定
    bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr));
}

