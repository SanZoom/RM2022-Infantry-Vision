#include "Predictor.h"

CoordPredictor::CoordPredictor()
{
    cv::FileStorage fs("../Configure/Settings.xml", cv::FileStorage::READ);
    cv::Mat processNoise, measurementNoise;
    cv::Mat t;
    fs["kalman_Q"] >> processNoise;
    fs["kalman_R"] >> measurementNoise;
    fs["cam2gyro_offset"] >> cam2gyro_offset;
    fs["gun2cam_offset"] >> gun2cam_offset;
    fs.release();

    KF = std::make_shared<cv::KalmanFilter>(state_num, measure_num, 0);

    cv::Mat H = (cv::Mat_<float>(measure_num, state_num) << 1, 0, 0, 0, 0, 0, 
                                                            0, 0, 1, 0, 0, 0, 
                                                            0, 0, 0, 0, 1, 0);

    KF->processNoiseCov = processNoise;                        // Q 过程噪声
    KF->measurementNoiseCov = measurementNoise;                // R 测量噪声
    cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));     // P 卡尔曼增益
    cv::setIdentity(KF->transitionMatrix, cv::Scalar::all(1)); // F 状态转移矩阵
    KF->measurementMatrix = H;
}

cv::Point3f CoordPredictor::predict(cv::Point3f coord, SpinDetector &spin_detector, GyroPose gyro_pose)
{
    Eigen::Quaternionf q(gyro_pose.q_0, gyro_pose.q_1, gyro_pose.q_2, gyro_pose.q_3);
    
    cout << "SOLVE_PNP: " << coord << endl;
    world_coord = camera2world(q, coord, cam2gyro_offset);
    return predict(world_coord, spin_detector, gyro_pose.timestamp);
}

cv::Point3f CoordPredictor::predict(cv::Point3f coord, SpinDetector &spin_detector, uint32_t timestamp)
{
    cv::Mat correct_state;
    float dt = (float)(timestamp - last_t) / 1000.0;
    last_t = timestamp;
    cout << "SOLVE_WORLD: " << world_coord << endl;
    cv::Mat measurement = (cv::Mat_<float>(measure_num, 1) << world_coord.x, 
                                                              world_coord.y, 
                                                              world_coord.z);

    KF->transitionMatrix.at<float>(0, 1) = dt;
    KF->transitionMatrix.at<float>(2, 3) = dt;
    KF->transitionMatrix.at<float>(4, 5) = dt;

    KF->predict();

    correct_state = KF->correct(measurement);
    correct_coord = cv::Point3f(correct_state.at<float>(0, 0), correct_state.at<float>(2, 0), correct_state.at<float>(4, 0));
    spin_detector.pushCoord(correct_coord);
    bool spin_flag = spin_detector.solverSpin(correct_coord);

    setBS_coeff(correct_coord);

    pitch_time = compensate(correct_coord, dt);
    cout << correct_state << endl;
    cout << "Pitch: " << pitch_time.x << "Time: " << pitch_time.y << endl;
    cout << "BS: " << bullet_speed << endl;
    if (spin_flag)
    {
        cout << "SPIN_FLAG" << endl;
        return correct_coord;
    }
    cv::Point3f world_next = predictNextpoint(correct_state, pitch_time.y + dt + delay_time);
    return world_next;
}


void CoordPredictor::initState(cv::Point3f coord, GyroPose gyro_pose, bool spin_once_flag)
{
    Eigen::Quaternionf q(gyro_pose.q_0, gyro_pose.q_1, gyro_pose.q_2, gyro_pose.q_3);
    cv::Point3f world_coord = camera2world(q, coord, cam2gyro_offset);
    if (spin_once_flag)
    {
        KF->statePost = (cv::Mat_<float>(state_num, 1) << world_coord.x,
                                                        KF->statePost.at<float>(1, 0),
                                                        world_coord.y,
                                                        KF->statePost.at<float>(3, 0),
                                                        world_coord.z,
                                                        KF->statePost.at<float>(5, 0));
    }
    else
    { 
        KF->statePost = (cv::Mat_<float>(state_num, 1) << world_coord.x,
                                                        0,
                                                        world_coord.y,
                                                        0,
                                                        world_coord.z,
                                                        0);
        last_t = gyro_pose.timestamp;
    }
}

void CoordPredictor::setBulletSpeed(int bs)
{
    bullet_speed = (double)bs;
}
