#include "Serial.h"

Serial::Serial()
{
    speed = DEFAULT_SPEED;
    databits = DEFAULT_DATABITS;
    stopbits = DEFAULT_STOPBITS;
    parity = DEFAULT_PARITY;
}

int Serial::openPort()
{
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        cout << "Open USB0 Failed" << endl;
        cout << "Now Try to Open USB1...";
        fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
        if (fd == -1)
        {
            cout << " Also Failed!" << endl;
            return SYSTEM_ERROR;
        }
        
    }
    cout << " Successfully!" << endl;
    // if (!setSpeed())
    // {
    //     cout << "Set Speed Failed!" << endl;
    //     return SYSTEM_ERROR;
    // }
    // if (!setBits())
    // {
    //     cout << "Set Bits Failed!" << endl;
    //     return SYSTEM_ERROR;
    // }
    if (!uart_set())
        return error_code = ErrorCode::SYSTEM_ERROR;
    cout << "Open Serial Port Succuessfully!" << endl; 
    return OK;
}

int Serial::uart_set()
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("Set Fail");
        return error_code = ErrorCode::SYSTEM_ERROR;
    }
    bzero( &newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (databits)
    {
    case 7: 
      newtio.c_cflag |= CS7; 
      break; 
    case 8: 
      newtio.c_cflag |= CS8; 
      break; 
    default:
        break;
    }
    switch (parity)
    {
    case 'o':
    case 'O': 
        newtio.c_cflag |= PARENB; 
        newtio.c_cflag |= PARODD; 
        newtio.c_iflag |= (INPCK | ISTRIP); 
        break; 
    case 'e':
    case 'E': 
        newtio.c_iflag |= (INPCK | ISTRIP); 
        newtio.c_cflag |= PARENB; 
        newtio.c_cflag &= ~PARODD; 
        break;
    case 'n':
    case 'N': 
        newtio.c_cflag &= ~PARENB; 
        break;
    
    default:
        break;
    }
    switch (speed)
    {
    case 2400: 
      cfsetispeed(&newtio, B2400); 
      cfsetospeed(&newtio, B2400); 
      break; 
     case 4800: 
      cfsetispeed(&newtio, B4800); 
      cfsetospeed(&newtio, B4800); 
      break; 
     case 9600: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
      break; 
     case 115200: 
      cfsetispeed(&newtio, B115200); 
      cfsetospeed(&newtio, B115200); 
      break; 
     case 460800: 
      cfsetispeed(&newtio, B460800); 
      cfsetospeed(&newtio, B460800); 
      break; 
    case 921600: 
      cfsetispeed(&newtio, B921600); 
      cfsetospeed(&newtio, B921600); 
      break; 
     default: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
     break; 
    }
    switch (stopbits)
    {
    case 1:
        newtio.c_cflag &=  ~CSTOPB; 
        break;
    case 2:
        newtio.c_cflag |=  CSTOPB; 
        break;
    default:
        newtio.c_cflag &=  ~CSTOPB; 
        break;
    }
    if (tcsetattr(fd, TCSANOW, &newtio) != 0)
    {
        cout << "Set Failed" << endl;
        return 0;
    }
    else
    {
        cout << "Set Done!" << endl;
        return 1;
    }

}   

int Serial::setSpeed()
{
    int speed_arr[] = {B115200,B460800, B38400, B19200, B9600, B4800, B2400};
    int name_arr[] = {115200,460800, 38400, 19200, 9600, 4800, 2400};
    
    int index;

    struct termios termios_p;
    tcgetattr(fd, &termios_p);

    for (index = 0; index < sizeof(speed_arr)/sizeof(int); index++)
    {
        if (speed == name_arr[index])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&termios_p, speed_arr[index]);
            cfsetospeed(&termios_p, speed_arr[index]);

            if (tcsetattr(fd, TCSANOW, &termios_p) != 0)
            {
                cout << "Speed tcsetattr error" << endl;
                return 0;
            }

            tcflush(fd, TCIOFLUSH);
            return 1;
        }
    }

    cout << "No Matched Speed!" << endl;
    return 0;
}

int Serial::setBits()
{
    struct termios termios_p;
    tcgetattr(fd, &termios_p);

    termios_p.c_cflag |= (CLOCAL | CREAD);  // 接收数据
    termios_p.c_cflag &= ~CSIZE;            // 取消数据位掩码

    // 设置数据位
    switch (databits)
    {
    case 8:
        termios_p.c_cflag |= CS8;
        break;
    case 7:
        termios_p.c_cflag |= CS7;
        break;
    case 6:
        termios_p.c_cflag |= CS6;
        break;
    default:
        cout << "No matched databits, so use CS8" << endl;
        termios_p.c_cflag |= CS8;
        break;
    }

    // 设置校验位
    switch (parity) 
    {
    case 'n':
    case 'N':
        termios_p.c_cflag &= ~PARENB;   /* 关闭校验码的生成和检测 */
        termios_p.c_iflag &= ~INPCK;    /* 关闭对接收到的字符进行奇偶校验 */
        break;
    
    case 'o':
    case 'O':
        termios_p.c_cflag |= (PARENB | PARODD); /* 设置为奇校验 */
        termios_p.c_iflag |= INPCK; 
        break;

    case 'e':
    case 'E':
        termios_p.c_cflag |= PARENB;    /* 设置为偶校验 */
        termios_p.c_cflag &= ~PARODD;
        termios_p.c_iflag |= INPCK;
        break;
    
    case 's':
    case 'S':
        termios_p.c_cflag &= ~PARENB;
        termios_p.c_cflag &= ~INPCK;
        break;

    default:
        perror("Unsupported parity\n");
        return 0;
        break;
    }

    //设置停止位
    switch (stopbits)   
    {
    case 1:
        termios_p.c_cflag &= ~CSTOPB;   /* 选择不使用两位停止位 */
        break;
    
    case 2:
        termios_p.c_cflag |= CSTOPB;
        break;

    default:
        perror("Unsupported stop bits\n");
        return 0;
        break;
    }

    termios_p.c_cflag &= ~(INLCR | ICRNL | IXON);
    termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termios_p.c_oflag &= ~OPOST;
    termios_p.c_oflag &= ~(ONLCR | OCRNL);
    termios_p.c_iflag &= ~(ICRNL | INLCR);
    termios_p.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcflush(fd, TCIFLUSH);  
    termios_p.c_cc[VTIME] = 1;      // 阻塞时有效
    termios_p.c_cc[VMIN] = 1;       

    if (tcsetattr(fd, TCSANOW, &termios_p) != 0)
    {
        perror("Set Bit Fail");
        return 0;
    }

    return 1;
}

bool Serial::isOpened() const
{
    return fd != -1;
}

int Serial::send(float* data, uint8_t flag)
{
    tcflush(fd, TCOFLUSH);

    // 将数据转换为 u_char
    send_data.Data[0] = flag;
    transformData(data);
    
    if(write(fd, &send_data, sizeof(send_data)) == -1)  return error_code = READ_WRITE_ERROR;

    return error_code =  OK;
}

int Serial::debugsend()
{
    struct testData
    {
        uint8_t a = 0x66;
        uint8_t b = 0x17;
        uint8_t c = 0x11;
        uint8_t d = 0x18;
        uint8_t e = 0x22;
        uint8_t f = 0x56;
        uint8_t g = 0x75;
        uint8_t h = 0x01;
        uint8_t i = 0x13;
        uint8_t j = 0x07;
        uint8_t k = 0x01;
        uint8_t l = 0x17;
        uint8_t o = 0x00;
        uint8_t q = 0x00;
        uint8_t m = (uint8_t)96;
        uint8_t n = 0x88;
    }data_t;
    
    tcflush(fd, TCOFLUSH);
    int c;
    if((c = write(fd, &data_t, sizeof(data_t))) == -1)  return error_code = READ_WRITE_ERROR;
    
    return error_code =  OK;
}

int Serial::receive()
{
    int fail_times = 0;
    uint8_t commands[READ_BYTE] = {0};
    uint8_t buffer[4*READ_BYTE] = {0};
    memset(&receive_data, 0, sizeof(receive_data));
    int readCount = -1;
    while (fail_times < 3)
    {
        // int readCount = -1;
        readCount = -1;
        readCount = read(fd, (uint8_t*)(&buffer), READ_BYTE);
        // cout << readCount << endl;
        if (readCount == READ_BYTE)
        {
            for (int j = 0; j < READ_BYTE; j++)
            {
                commands[j] = buffer[j];
            }
            break;
        }
        this_thread::sleep_for(chrono::milliseconds(1));
        fail_times ++;
    }
    
    int i = 0;
    
    if (fail_times >= 3)
    {
        cout << "Read Time Out!" << endl;
        return error_code = TIME_OUT;
    }  
    while (i < READ_BYTE)
    {
        if (commands[i%READ_BYTE] == SERIAL_SOF && commands[(i+READ_BYTE-1)%READ_BYTE] == SERIAL_EOF)
        {
            for (int j = 0; j < READ_BYTE - 2; j++, i++)
                receive_data.CMD[j] = commands[(i+1)%READ_BYTE];
            return error_code=OK;
        }
        i++;
    }  
    
    cout << "CMD SOF or EOF Wrong!" << endl;
    cout << "SOF: " << (int)commands[0] << "     " << "EOF: " << (int)commands[READ_BYTE-1] << endl;
    cout << "ReadCount: " << readCount << endl;
    cout << endl;
    return error_code = READ_WRITE_ERROR;
}

int Serial::receive(float *commands, int& bulletspeed, int& color, int& mode)
{
    error_code = ErrorCode(Serial::receive());
    if (error_code != OK)   return error_code;

    error_code = ErrorCode(Serial::transformCMD(commands));

    int speed_flag = (int)(receive_data.CMD[0] & 0x0F) - 1;
    bulletspeed = bullet_speed[speed_flag];
    color = (int)(receive_data.CMD[0] & 0x30 >> 4);
    mode = (int)(receive_data.CMD[0] >> 6);

#ifdef DEBUG_COMMANDS
    cout << "CMD: [" << commands[0] << ", " << commands[1] << ", " << commands[2] <<
     ", " << commands[3] << ", " << commands[4] << "," << commands[5] <<"]" <<endl;
#endif
    return error_code;
}


int Serial::transformCMD(float* commands)
{
    int16_t roll, pitch, yaw, pitch_speed, yaw_speed;
    uint32_t ms;

    roll = receive_data.CMD[1];
    roll = ((int16_t)receive_data.CMD[2] << 8 | roll);

    pitch = receive_data.CMD[3];
    pitch = ((int16_t)receive_data.CMD[4] << 8 | pitch);

    yaw = receive_data.CMD[5];
    yaw = ((int16_t)receive_data.CMD[6] << 8 | yaw);

    pitch_speed = receive_data.CMD[7];
    pitch_speed = ((int16_t)receive_data.CMD[8] << 8 | pitch_speed);

    yaw_speed = receive_data.CMD[9];
    yaw_speed = ((int16_t)receive_data.CMD[10] << 8 | yaw_speed);

    ms = (uint32_t)receive_data.CMD[11];
    ms = ((uint32_t)receive_data.CMD[12] << 8 | ms);
    ms = ((uint32_t)receive_data.CMD[13] << 16 | ms);

    commands[0] = (float)pitch * 180.0 / 32767.0;  // 取负
    commands[1] = (float)yaw * 180.0 / 32767.0;
    commands[2] = (float)roll * 180.0 / 32767.0;
    commands[3] = (float)ms / 1000.0;   // 转换为s单位
    commands[4] = (float)pitch_speed * 2000.0 / 32767.0;
    commands[5] = (float)yaw_speed * 2000.0 / 32767.0;
    return OK;
}



int Serial::transformData(float* data)
{
    int16_t pitch, yaw;
    u_char pitch_, _pitch, yaw_, _yaw;
    u_char check;   // 校验位

    pitch = (int16_t)(data[0] * 32768.0 / 180.0);
    pitch_ = pitch >> 8;
    _pitch = pitch & 0xFF;

    yaw = (int16_t)(-data[1] * 32768.0 /180.0);     // 取反
    yaw_ = yaw >> 8;
    _yaw = yaw & 0xFF;

    check = (u_char)((_pitch + pitch_ + _yaw + yaw_)%10);
    send_data.Data[1] = (u_char)pitch_;
    send_data.Data[2] = (u_char)_pitch;
    send_data.Data[3] = (u_char)yaw_;
    send_data.Data[4] = (u_char)_yaw;
    send_data.Data[5] = (u_char)check;

#ifdef DEBUG_SERIAL
    // cout << "Send: [" << (int)send_data.Data[1] << ", " << (int)send_data.Data[2] << ", " <<  (int)send_data.Data_3 
    // << ", " << (int)send_data.Data_4 << ", " << (int)send_data.Data_5 << "]" << endl;
    cout << "Send: [";
    for (int i = 0; i < SEND_BYTE - 2; i++)
    {
        cout << (int)send_data.Data[i] << ", ";
    }
    cout << "]"<< endl;
#endif
    return OK;
}

int Serial::closePort()
{
    close(fd);
    return OK;
}

int Serial::debugreceive()
{
    uint8_t commands[SEND_BYTE];
    if (read(fd, commands, sizeof(commands)) == -1)
        return READ_WRITE_ERROR;

    for (int i = 0; i < SEND_BYTE; i++)
        cout << (int)commands[i] << ", ";
    cout << endl;
    return OK;
}