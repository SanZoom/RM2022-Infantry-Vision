// #include "EKF.h"
// #include "opencv2/core/eigen.hpp"

// EKF_CTRV::EKF_CTRV(const std::string &filename)
// {
//     cv::Mat processNoiseCov, measurementNoiseCov;
//     Eigen::Matrix process;
//     Eigen::Matrix measurement;
//     cv::FileStorage fs(filename, cv::FileStorage::READ);

//     fs.release();

//     cv::cv2eigen(processNoiseCov, process);
//     cv::cv2eigen(measurementNoiseCov, measurement);
// }