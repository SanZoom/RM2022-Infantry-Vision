#ifndef RM_EKF_H
#define RM_EKF_H

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/core.hpp>

template <int state_num, int measure_num, typename T = double>
class ExtendKalmanFilter
{
public:
    ExtendKalmanFilter() : j_transitionMatrix(Eigen::Matrix<T, state_num, state_num>::Identity()),
                           j_measuremenMatrix(Eigen::Matrix<T, measure_num, state_num>::Identity()),
                           measurementNoiseCov(Eigen::Matrix<T, measure_num, measure_num>::Identity()),
                           erroCovPost(Eigen::Matrix<T, state_num, state_num>::Identity()),
                           erroCovPre(Eigen::Matrix<T, state_num, state_num>::Identity()),
                           processNoiseCov(Eigen::Matrix<T, state_num, state_num>::Identity()),
                           gain(Eigen::Matrix<T, state_num, measure_num>::Zero()),
                           statePost(Eigen::Matrix<T, state_num, 1>::Zero()),
                           statePre(Eigen::Matrix<T, state_num, 1>::Zero())
    {
    }
    ~ExtendKalmanFilter() {}

    template <class Func>
    Eigen::Matrix<T, state_num, 1> predict(Func &&predictFunc)
    {
        // set auto cal jet matrix for  statePose to statePre
        ceres::Jet<T, state_num> state_post_jet[state_num];
        for (int i = 0; i < state_num; i++)
        {
            state_post_jet[i].a = statePost[i];
            state_post_jet[i].v[i] = 1;
        }
        ceres::Jet<T, state_num> state_pre_jet[state_num];
        predictFunc(state_post_jet, state_pre_jet);
        for (int i = 0; i < state_num; i++)
        {
            // get statePre
            statePre[i] = state_pre_jet[i].a;
            // get the jet for A(transitionMatrix)
            j_transitionMatrix.block(i, 0, 1, state_num) = state_pre_jet[i].v.transpose();
        }
        // P= JA*P'*JA.trans()
        erroCovPre = j_transitionMatrix * erroCovPost * j_transitionMatrix.transpose() + processNoiseCov;
        return statePre;
    }

    template <class Func>
    Eigen::Matrix<T, state_num, 1> correct(Eigen::Matrix<T, measure_num, 1> measure, Func &&measureFunc)
    {
        ceres::Jet<double, state_num> state_pre_jet[state_num];
        for (int i = 0; i < state_num; i++)
        {
            state_pre_jet[i].a = statePre[i];
            state_pre_jet[i].v[i] = 1;
        }
        ceres::Jet<double, state_num> state_mse_jet[measure_num];

        measureFunc(state_pre_jet, state_mse_jet);
        //  cal h(x_pre)
        Eigen::Matrix<T, measure_num, 1> temp_msu; // h(statePre)
        for (int i = 0; i < measure_num; i++)
        {
            temp_msu[i] = state_mse_jet[i].a;
            j_measuremenMatrix.block(i, 0, 1, state_num) = state_mse_jet[i].v.transpose();
        }
        Eigen::Matrix<T, measure_num, measure_num> temp1 = (j_measuremenMatrix * erroCovPre * j_measuremenMatrix.transpose() + measurementNoiseCov).inverse();
        // K=P*JH*(JH*P*JH.trans() + R)^-1
        gain = erroCovPre * j_measuremenMatrix.transpose() * temp1;
        // X'= X+ K*(Z- H(X))
        statePost = statePre + gain * (measure - temp_msu);

        Eigen::Matrix<T, state_num, state_num> I = Eigen::Matrix<T, state_num, state_num>::Identity();
        // P' = (I-K*H)*P
        erroCovPost = (I - gain * j_measuremenMatrix) * erroCovPre;
        return statePost;
    }

    Eigen::Matrix<T, state_num, state_num> j_transitionMatrix;      // JA
    Eigen::Matrix<T, measure_num, state_num> j_measuremenMatrix;    // JH
    Eigen::Matrix<T, measure_num, measure_num> measurementNoiseCov; // R
    Eigen::Matrix<T, state_num, measure_num> gain;                  // K
    Eigen::Matrix<T, state_num, state_num> erroCovPost;             // P'
    Eigen::Matrix<T, state_num, state_num> erroCovPre;              // P
    Eigen::Matrix<T, state_num, state_num> processNoiseCov;         // Q
    Eigen::Matrix<T, state_num, 1> statePost;                       // X'
    Eigen::Matrix<T, state_num, 1> statePre;                        // X
};

// 输入绝对系坐标，用于反陀螺
class EKF_CTRV
{
private:
    ExtendKalmanFilter<5, 2> ekf;
    uint32_t last_timestamp;

public:
    /* data */
    struct PredictFunction
    {
        double d_time;

        template <class T>
        void operator()(T *x, T *y) //定义预测模型
        {
            // CTRV 模型  x=[x,y,v,angle,d_angle]
            if (x[4].a)
            {
                y[0] = x[2] / x[4] * sin(x[4] * d_time + x[3]) - x[2] / x[4] * sin(x[3]) + x[0];
                y[1] = -x[2] / x[4] * cos(x[4] * d_time + x[3]) + x[2] / x[4] * cos(x[3]) + x[1];
                y[2] = x[2];
                y[3] = x[4] * d_time + x[3];
                y[4] = x[4];
            }
            else
            {
                y[0] = x[2] * cos(x[3]) * d_time + x[0];
                y[1] = x[2] * sin(x[3]) * d_time + x[1];
                y[2] = x[2];
                y[3] = x[4] * d_time + x[3];
                y[4] = x[4];
            }
        }
        PredictFunction() : d_time(0.01){}
    };
    struct MeasureFunction
    {
        template <class T>
        void operator()(T *x, T *y) //定义观测模型
        {
            // H=[1,0,0,0,0
            //    0,1,0,0,0]
            y[0] = x[0];
            y[1] = x[1];
        }
        MeasureFunction() {}
    };

    PredictFunction pre_func;
    MeasureFunction mea_func;
    EKF_CTRV(const std::string &filename = "../Configure/EKF_CTRV.xml");

    /**
     *  @brief  初始化
     *  @param  world_coord 绝对系坐标
     *  @param  timestamp   时间戳
     */
    void init(cv::Point3f world_coord, uint32_t timestamp);

    /**
     *  @brief  预测更新
     *  @param  world_coord 绝对系坐标
     *  @param  timestamp   时间戳
     */
    void predict(cv::Point3f world_coord, uint32_t timestamp);

    ~EKF_CTRV() {};
};

#endif