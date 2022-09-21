#ifndef SERIAL_H_
#define SERIAL_H_
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <stdexcept>
#include <exception>
#include <memory.h>

using namespace std;

#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_SPEED 460800
#define DEFAULT_DATABITS 8
#define DEFAULT_STOPBITS 1
#define DEFAULT_PARITY 'N'
#define READ_BYTE 16
#define SEND_BYTE 8

// #define DEBUG_COMMANDS
// #define DEBUG_READ
// #define DEBUG_SERIAL

const uint8_t  SERIAL_SOF = (uint8_t)0x66;
const uint8_t  SERIAL_EOF = (uint8_t)0x88;
typedef enum{
   //没识别到不发子弹
   NO_FIND_NO_SHOOT=0x00,
   //识别到不发射子弹
   FIND_NO_SHOOT=0x01,
   //识别到且发射子弹
   FIND_SHOOT=0x02,
   //没解算完，与上次指令相同
   NO_END_SOLVER=0x03,
}SHOOT_ECODE;
struct SendData
{
    const uint8_t SOF=SERIAL_SOF;
    uint8_t Data[6];                   
    const uint8_t END=SERIAL_EOF;
    // 是否找到装甲板
    // pitch 高位
    // pitch 低位
    // yaw  高位
    // yaw  低位
    // 求和校验位
};

struct ReceiveData{

    const uint8_t SOF=SERIAL_SOF;
    uint8_t CMD[14];
    const uint8_t END=SERIAL_EOF;
    /*--------------------------------
    0:  相关标志信息
    1:  _roll，低位
    2:  roll_，高位
    3:  _pitch，低位
    4:  pitch_，高位
    5:  _yaw，低位
    6:  yaw_，高位
    7:  pitch角速度，低位
    8:  pitch角速度，高位
    9:  yaw角速度，低位
    10: yaw角速度，高位
    11\12\13: 时间ms，L\M\H，共计24位表示

    0:  
    b4~0: 枪口射速
        4'b0001     1       10m/s
        4'b0010     2       12m/s
        4'b0011     3       14m/s
        4'b0100     4       15m/s
        4'b0101     5       16m/s
        4'b0110     6       18m/s
        4'b0111     7       22m/s
        4'b1000     8       30m/s
    b6~5: 比赛信息
        2'b01       1       当前为红方
        2'b10       2       当前为蓝方
    b8~7: 任务信息
        2'b01       1       普通模式
        2'b10       2       打符模式
    ----------------------------------*/


    void init()
    {
        for (int i = 0; i < READ_BYTE - 2; i++)
            CMD[i] = 0x00;
    }
};

class Serial{
public:
    //！ 错误代码
    enum ErrorCode
    {
        SYSTEM_ERROR = -1,
        OK = 0,                         // 一切正常
        PORT_OCCUPIED = 1,              // 关闭串口失败
        READ_WRITE_ERROR = 2,           // 写入失败
        TIME_OUT = 3                    // 接受超时
    };

private:

    SendData send_data;                 // 发送的数据  
    ReceiveData receive_data;           // 接受的数据
    
    int fd = -1;                        // 串口权柄
    int speed, databits, stopbits;      // 波特率、数据位、停止位
    char parity;                        // 奇偶校验位

    ErrorCode  error_code;              // 错误代码
    const int bullet_speed[8] = {10, 12, 14, 15, 16, 18, 22, 30};

public:
/**
 * @brief 发送数据
 * @return error_code
*/
    int send(float*, uint8_t);

/**
 *  @brief  发送数据（debug）
 */
    int debugsend();

/**
 *  @brief  接收数据（debug）
 */
    int debugreceive();
/**
 * @brief 打开串口
 * @return error_code
*/
    int openPort();

/**
 * @brief 关闭串口
 * @return error_code
*/
    int closePort();

/**
 * @brief 返回串口是否打开
 * @return bool
*/
    bool isOpened() const;

/**
 * @brief 接受数据
 * @return error_code
*/
    int receive();

/**
 *  @brief  接收数据的对外API
 *  @return commands 中的数据顺序为 pitch、yaw、roll、time
 */
    int receive(float *commands, int& bullet_speed, int& color, int& mode);
/**
 *  @brief  转化接收到的数据，需要与电控协调
 *  @param  commands    转化后得到的数据
 */
    int transformCMD(float *commands);

/**
 * @brief 构造串口
*/
    Serial();

private:
    /**
     *  @brief   设置波特率
     */
    int setSpeed();

    /**
     *  @brief  设置数据位、停止位以及奇偶校验位
     */
    int setBits();

    /**
     *  @brief  将数据转换为uchar型以供发送
     */
    int transformData(float* data);

    int uart_set();
};

#endif