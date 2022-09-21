#include "ImageProgress.h"

ImageProgress::ImageProgress()
{
    fitting_lck = std::unique_lock<std::mutex>(fitting_mutex);

    int color;
    cv::FileStorage fs("../Configure/Settings.xml", cv::FileStorage::READ);
    fs["enemy_color"] >> color;
    fs.release();
    _enemy_color = (COLOR)color;
}

void ImageProgress::ImageProducer()
{
    if (this->_mode == Mode::RUNE || this->_mode == Mode::NORMAL_RUNE)
        cap.resetParam("../Configure/RuneParam.xml");
    while (1)
    {
        Mat_time frame;
        try
        {
            frame.create(ACQ_FRAME_HEIGHT, ACQ_FRAME_WIDTH, CV_8UC3);
            cap >> frame;
        }
        catch(const cv::Exception e)
        {
            std::cerr << e.what() << '\n';
            continue;
        }
        if (frame.empty())
            continue;
        frame_mutex.lock();
        frame_buffer.push(frame);
        while (frame_buffer.size() > FRAME_BUFFER_NUM)
        {
            frame_buffer.pop();
        }
        frame_mutex.unlock();

        this_thread::sleep_for(chrono::milliseconds(1));
    }
}

void ImageProgress::ImageConsumer2()
{
    ArmorDetector detector;
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    RuneDetector rune_detector;

    while (1)
    {
        Mat_time frame;
        if (debug_param.debug_no_gyro) // 无IMU模式
        {
            while (frame_buffer.empty())
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            frame_mutex.lock();
            frame = frame_buffer.front();
            frame.gyro_pose.timestamp = (double)cv::getTickCount() / cv::getTickFrequency() * 1000.0;
            frame_buffer.pop();
            frame_mutex.unlock();
        }
        else
            while (!InterGyroPose(frame))
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (_mode == Mode::NORMAL)
            NormalConsumer(detector, frame, last_time);
        else
            RuneConsumer(rune_detector, frame, last_time);

        if (_mode == Mode::NORMAL)
        {
            armor_state = detector.state;
            send_target_id = detector.target.id;
        }
        else
            armor_state = rune_detector.state;
    }
}

void ImageProgress::NormalConsumer(ArmorDetector &detector, Mat_time frame, std::chrono::steady_clock::time_point &last_time)
{
    detector.setEnemyColor(_enemy_color);
    detector.predictor.setBulletSpeed(_bullet_speed);

    cv::Point2f pitch_yaw;
    // 检测装甲板
    detector.run(frame, pitch_yaw);

    send_mutex.lock();
    send_pitch_yaw[0] = pitch_yaw.x;
    send_pitch_yaw[1] = pitch_yaw.y;
    send_time = frame.gyro_pose.timestamp;

    armor_state = detector.state;
    send_target_id = detector.target.id;

    send_mutex.unlock();

    if (debug_param.debug_speed)
    {
        std::cout << "Tick: " << std::chrono::duration_cast<chrono::microseconds>(std::chrono::steady_clock::now() - last_time).count()
                  << endl;
        std::cout << endl;
        last_time = std::chrono::steady_clock::now();
    }

    if (debug_param.debug)
    {
        show_armor_result(frame, detector, debug_param);

        if (waitKey(1) == 'q')
            exit(0);
    }
}

void ImageProgress::RuneConsumer(RuneDetector &detector, Mat_time frame, std::chrono::steady_clock::time_point &last_time)
{
    detector.setEnemyColor(_enemy_color);
    detector.run(frame);

    rune_mutex.lock();
    RuneSegament = 1;
    armor_state = detector.state;
    new_rune_armor = RuneArmor(detector.target);
    rune_gyro_pose = GyroPose(frame.gyro_pose);
    rune_mutex.unlock();

    if (debug_param.debug_speed)
    {
        std::cout << "Tick: " << std::chrono::duration_cast<chrono::microseconds>(std::chrono::steady_clock::now() - last_time).count()
                  << endl;
        std::cout << endl;
        last_time = std::chrono::steady_clock::now();
    }

    if (debug_param.debug)
    {
        show_armor_result(frame, detector, rune_next_pos, debug_param);
        if (waitKey(1) == 'q')
            exit(0);
    }
}

void ImageProgress::Fitting()
{
    Solver solver;
    CoordPredictor predictor;
    FitTool fittool;
    while (true)
    {
        if (_mode == Mode::NORMAL)
        {
            fittool.clearData();
            fitting_condition.wait(fitting_lck);
            send_mutex.lock();
            send_pitch_yaw[0] = 0;
            send_pitch_yaw[1] = 0;
            send_mutex.unlock();
        }

        switch (_mode)
        {
        case RUNE:
            cout << "RUNE" << endl;
            break;
        
        case NORMAL_RUNE:
            cout << "NORMAL RUNE" << endl;
            break;

        default:
            cout << "WRONG" << endl;
            break;
        }
        while (RuneSegament <= 0)
        {
            this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        rune_mutex.lock();
        RuneSegament--;
        RuneArmor fit_new = new_rune_armor;
        GyroPose fit_gyro_pose = rune_gyro_pose;
        fit_new.gyro_pose = rune_gyro_pose;
        new_rune_armor = RuneArmor();
        rune_gyro_pose = GyroPose();
        rune_mutex.unlock();

        rune_next_pos.clear();        
        if (fittool.run(fit_new, rune_next_pos, armor_state, _mode))
        {
            solver.solve(rune_next_pos, ArmorType::RUNE_ARMOR);

            // 抬枪补偿
            Eigen::Quaternionf q(fit_gyro_pose.q_0, fit_gyro_pose.q_1, fit_gyro_pose.q_2, fit_gyro_pose.q_3);
            cv::Point3f world_coord = camera2world(q, solver.point_3D);

            predictor.setBulletSpeed(30);
            predictor.setBS_coeff(world_coord);
            cv::Point2f pitch_time = predictor.compensate(world_coord, 0.005);
            // cout << "SolvePnP: " << solver.point_3D << endl;
            // cout << "Pitch Time: " << pitch_time << endl;
            cout << "World Coord: " << world_coord << endl;
            send_mutex.lock();
            send_pitch_yaw[0] = -solver.pitch - pitch_time.x + atan(world_coord.z / sqrt(world_coord.x * world_coord.x + world_coord.y * world_coord.y)) * 180.0 / CV_PI + 1.0;
            send_pitch_yaw[1] = solver.yaw - 0.8;
            cout <<"Pitch: " << send_pitch_yaw[0] << "Yaw: " << send_pitch_yaw[1] << endl;
            send_time = fit_new.gyro_pose.timestamp;
            send_mutex.unlock();

            fittool.printResult();
        }
    }
}

bool ImageProgress::InterGyroPose(Mat_time &frame)
{
    while (gyro_buffer.size() < GYRO_BUFFER_NUM)
        this_thread::sleep_for(chrono::milliseconds(1));
    gyro_mutex.lock();
    GyroPose temp_gyro_buffer[GYRO_BUFFER_NUM];
    for (int i = 0; i < GYRO_BUFFER_NUM; i++)
        temp_gyro_buffer[i] = gyro_buffer[i];
    gyro_mutex.unlock();

    while (frame_buffer.empty())
    {
        this_thread::sleep_for(chrono::milliseconds(1));
    }
    frame_mutex.lock();

    while (!frame_buffer.empty())
    {
        Mat_time temp = frame_buffer.front();
        frame_buffer.pop();

        int64 delta[GYRO_BUFFER_NUM] = {0};
        for (int i = 0; i < GYRO_BUFFER_NUM; i++)
            delta[i] = chrono::duration_cast<chrono::microseconds>(temp_gyro_buffer[i].local_timestamp - temp.produced_time).count();
        if (delta[0] > 0) // 图像远滞后于陀螺仪数据
        {
        }
        else if (delta[GYRO_BUFFER_NUM - 1] >= 0)
        {
            int min_index = 0;
            int64 min_value = abs(delta[0]);
            for (int i = 1; i < GYRO_BUFFER_NUM; i++)
            {
                if (abs(delta[i]) < min_value)
                {
                    min_value = abs(delta[i]);
                    min_index = i;
                }
            }
            frame = temp;
            frame.gyro_pose = temp_gyro_buffer[min_index];

            frame_mutex.unlock();
            return true;
        }
        else if (delta[GYRO_BUFFER_NUM - 1] < 0)
        {
            if (delta[GYRO_BUFFER_NUM - 1] > -1000)
            {
                frame = temp;
                frame.gyro_pose = temp_gyro_buffer[GYRO_BUFFER_NUM - 1];

                frame_mutex.unlock();
                return true;
            }

            frame_mutex.unlock();
            return false;
        }
    }
    frame_mutex.unlock();
    return false;
}

void ImageProgress::ReadIMU2()
{
    CanSerial::CanError state;
    std::chrono::steady_clock::time_point last_read_time = std::chrono::steady_clock::now();

    float imu_data[4] = {0};
    uint32_t timestamp = 0, temp_timestamp = 0;
    auto rune_signal_first_get_timestamp = chrono::steady_clock::now(); // 接收到打符信号后x秒再退出

    while (true)
    {
        uint id = 0;
        u_char dlc = 0;
        u_char buf[8] = {0};
        uint16_t competition_remain_time;

        if (debug_param.debug_serial_frequency_read)
        {
            std::cout << "ReadFreq: " << std::chrono::duration_cast<chrono::microseconds>(std::chrono::steady_clock::now() - last_read_time).count()
                      << endl;
            last_read_time = std::chrono::steady_clock::now();
        }
        state = (CanSerial::CanError)can_serial.can_receive(id, buf, dlc);
        switch (id)
        {
        case IMU_TIME_RECEIVE_ID:
            if (dlc == 4)
            {
                can_serial.transformIMU_Time(buf, temp_timestamp);
            }
            else 
            {
                cout << "err" << endl;
            }
            if (temp_timestamp < 5000000)
            {
                if ((double)temp_timestamp - (double)timestamp < - 10 && timestamp != 0)
                {
                    cout << "TEMP: "<<  temp_timestamp << endl;
                    cout << "time: " << timestamp << endl;
                    exit(0);
                }
                timestamp = temp_timestamp;
            }
            else
            {
                timestamp = timestamp + 1;
            }
            break;

        case IMU_RECEIVE_ID:
            can_serial.testTransformIMU(buf, imu_data);
            gyro_mutex.lock();
            newest_pose = GyroPose(imu_data, timestamp);
            gyro_buffer.push_back(newest_pose);
            while (gyro_buffer.size() > GYRO_BUFFER_NUM)
                gyro_buffer.erase(gyro_buffer.begin());
            gyro_mutex.unlock();

            if (debug_param.debug_gyro_data)
            {
                std::cout << "四元数：" << std::endl;
                std::cout << "q0: " << newest_pose.q_0 << "     ";
                std::cout << "q1: " << newest_pose.q_1 << endl;
                std::cout << "q2: " << newest_pose.q_2 << "     ";
                std::cout << "q3: " << newest_pose.q_3 << endl;
                std::cout << "Time: " << newest_pose.timestamp << endl;
                std::cout << endl;
            }

            break;

        case BS_RECEIVE_ID:
            _bullet_speed = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
            break;

        case MODE_RECEIVE_ID:
            if (debug_param.debug_no_competition)
                break;
            if ((buf[0] & 0x10) == 0x10)
            {
                // cout << "NORAML_RUNE" << endl;
                if (switchMode(Mode::NORMAL_RUNE))
                {
                    cap.resetParam("../Configure/RuneParam.xml");
                    fitting_condition.notify_all();
                }
            }
            else if ((buf[0] & 0x20) == 0x20)
            {
                // cout << "RUNE" << endl;
                if (switchMode(Mode::RUNE))
                {
                    cap.resetParam("../Configure/RuneParam.xml");
                    fitting_condition.notify_all();
                }
            }
            else
            {
                // cout << "NORAML" << endl;
                if (switchMode(Mode::NORMAL))
                {
                    cap.resetParam("../Configure/Settings.xml");
                }
            }
            // cout << "RECEIVE MODE: " << (int)(buf[0]) << endl;    // 测试模式
            break;

        case ROBOT_RECEIVE_ID:
            if (dlc != 6)
                break;
            if (buf[0] < 10)
                _enemy_color = COLOR::BLUE;
            else if (buf[0] > 100)
                _enemy_color = COLOR::RED;

            break;

        case COMPETITION_RECEVIE_ID:
            competition_remain_time = 0;
            memcpy(&competition_remain_time, &buf[1], 2);
            allow_normal_rune = (competition_remain_time < 370 && competition_remain_time > 239);
            allow_rune = (competition_remain_time < 190);
            cout << "competition_remain_time: " << competition_remain_time << endl;
            break;

        default:
            break;
        }
        this_thread::sleep_for(chrono::microseconds(50));
    }
}

void ImageProgress::SendData2()
{
    CanSerial::CanError state;
    chrono::steady_clock::time_point serial_last_send = chrono::steady_clock::now();

    while (true)
    {
        send_mutex.lock();
        can_serial.send_data(send_pitch_yaw, this->send_time);
        send_mutex.unlock();

        if (state != CanSerial::TIME_ERROR && debug_param.debug_serial_frequency_send)
        {
            std::cout << "Send Freq: " << chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - serial_last_send).count() << endl;
            serial_last_send = chrono::steady_clock::now();
        }
        can_serial.send_state(armor_state, send_target_id);
        this_thread::sleep_for(chrono::microseconds(400));
    }
}

void ImageProgress::VideoSave()
{
    // auto t = int64(cv::getTickCount() / cv::getTickFrequency()) % 1000;
    // string filename = "./video_" + to_string(t) + ".avi";
    // cv::VideoWriter video_writer(filename, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, Size(ACQ_FRAME_WIDTH, ACQ_FRAME_HEIGHT));
    // while (true)
    // {
    //     while (frame_buffer.empty())
    //         this_thread::sleep_for(chrono::milliseconds(30));
    //     frame_mutex.lock();
    //     Mat frame = frame_buffer.front();
    //     frame_mutex.unlock();
        // video_writer << frame;
    //     this_thread::sleep_for(chrono::milliseconds(1));
    // }
}