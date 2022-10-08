#ifndef ARMORDETECTOR_H_
#define ARMORDETECTOR_H_

#include <string>
#include "ArmorBox.h"
#include "CV_Classifier.h"
#include "classfier.h"
#include "Solver.h"
#include "Predictor.h"
#include "Mat_time.h"
#include "Tools.h"
#include "DebugParam.h"
#include "SpinDetector.h"

class ArmorDetector
{
private:
    ArmorParam param;           // 装甲板限制参数
    Classifier classifier;      // 手写 CNN 神经网络分类
    SpinDetector spin_detector; // 反小陀螺

    COLOR enemy_color; // 敌方颜色,RED,BLUE,OTHER_CLORO

    /*---ROI---*/
    float roi_enlarge;               // ROI放大倍数
    int lost_count = 0;              // 丢失目标次数
    Rect roi_rect;                   // roi区域矩形
    Rect roi_temp;                   // 暂存上一次识别到的装甲板的roi
    float roi_width_to_height_ratio; // roi区域水平方向放大程度

    cv::Point3f last_world_point; // 上次目标的绝对坐标
    int use_last_count = 0;       // 装甲板类型掉帧处理

    DebugParam debug_param; // Debug参数

public:
    ArmorBox target;          // 目标装甲板
    ArmorBox last_target;     // 上次识别到的目标
    ArmorBoxes _armor_boxes;  // 目标装甲板集
    LightBlobs _light_blobs;  // 目标灯条集
    cv::Mat roi;              // 图像ROI区域
    cv::Point offset;         // ROI偏移量
    Mat_time src;             // 原图像
    Solver solver;            // PNP解算器
    CoordPredictor predictor; // KF预测以及弹道补偿

    ArmorState state = ArmorState::LOST; // 识别的状态(丢失、首次、持续)
    bool if_shoot = true;                // 是否发弹

    float pitch;
    float yaw;
    cv::Point2f cp_2D; // 预测后的相机2D点

private:
    /**
     * @brief 设置ROI区域(水平方向扩大程度大于垂直方向)
     * @param  src 相机得到的图片
     * @param  tracking_rect 需要取的roi区域，默认为空时取原始图像区域
     */
    bool setRoi(Mat_time _src, cv::Rect &tracking_rect);

    /**
     * @brief   在图像上寻找灯条
     * @param   light_blobs 得到的灯条集
     */
    bool findLightBlobs(LightBlobs &light_blobs);

    /**
     * @brief   筛选灯条
     * @param   contours 灯条轮廓
     * @param   rect 灯条的旋转矩形
     */
    bool filterLightBlob(const std::vector<cv::Point> &contour);

    /**
     * @brief  匹配所有灯条，得出伪装甲板集
     * @param  light_blobs 筛选后得到的灯条集
     * @param  armor_boxes 得到的装甲板集
     */
    bool matchArmorBoxes(LightBlobs &light_blobs, ArmorBoxes &armor_boxes);

    /**
     * @brief  检验两灯条是否满足构造成装甲板的要求
     * @param  light_blob_i  需要匹配的灯条i
     * @param  light_blob_j  需要匹配的灯条j
     */
    bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j);

    /**
     * @brief   对装甲板集进行数字识别
     * @param   armor_boxes 经过灯条匹配后得到的装甲板集
     */
    bool getArmorNum(ArmorBoxes &armor_boxes);

    /**
     * @brief   判断是否两灯条间还有灯条
     */
    bool isBadArmor(int i, int j, const LightBlobs &lightblobs);

    /**
     * @brief   对输入分类器的装甲板正接矩形矩形调整
     */
    void adjustBox(cv::Rect &r);

    /**
     * @brief   将装甲板排序，得出最佳击打目标
     * @param   boxes   得到的目标装甲板集
     */
    bool getBestArmor(ArmorBoxes &boxes);

    /**
     *  @brief  判断是否为"老"装甲板
     */
    bool ifOldArmor();

    /**
     * @brief   解算并预测装甲板的pitch和yaw
     * @param   target  目标装甲板
     * @param   gyro_pose   陀螺仪姿态
     * @return  (Pitch, Yaw)
     */
    cv::Point2f getPitchYaw(ArmorBox target, GyroPose gyro_pose);

    /**
     *  @brief  根据绝对坐标得出pitch和yaw，用于掉帧处理
     *  @param  world_point 世界坐标（上一帧）
     *  @param  gyro_pose   陀螺仪数据（当前帧）
     *  @return  (Pitch, Yaw)
     */
    cv::Point2f getPitchYaw(cv::Point3f world_point, GyroPose gyro_pose);

    /**
     *  @brief  根据predict绝对坐标得出pitch和yaw
     *  @param  world_predict   预测后的世界坐标
     *  @param  gyro_pose   陀螺仪数据
     *  @return (Pitch, Yaw)
     */
    cv::Point2f calPredict(cv::Point3f world_predict, GyroPose gyro_pose);

    /**
     *  @brief  设置ROI
     *  @param  ratio   放大系数
     */
    void get_roi(float ratio)
    {
        if (ratio <= 1.0)
        {
            roi_rect = Rect();
            roi_temp = Rect();
            roi = src;
            offset = Point2f(0, 0);
        }
        else
        {
            roi_rect.x = roi_temp.x - roi_temp.width / 2.0 * (ratio * roi_width_to_height_ratio - 1.0);
            roi_rect.y = roi_temp.y - roi_temp.height / 2.0 * (ratio - 1.0);
            roi_rect.width = roi_temp.width * ratio * roi_width_to_height_ratio;
            roi_rect.height = roi_temp.height * ratio;
            roi_rect &= Rect(Point2f(0, 0), Size(src.cols, src.rows));
            roi = src(roi_rect);
            offset = roi_rect.tl();
        }
    }

    /**
     * @brief  得出目标装甲板
     * @param  box 得到的目标装甲板
     */
    bool findArmorBox(ArmorBox &box, uint32_t time_);

    /**
     * @brief  得出目标装甲板集
     * @param  boxes 得到的目标装甲板集
     */
    bool findArmorBoxes(ArmorBoxes &boxes);

public:
    /**
     *  @brief  对外API
     *  @param  _src    输入的原图像
     *  @param  pitch_yaw   解算预测得出的pitch_yaw
     *  @return 是否成功检测
     */
    bool run(Mat_time _src, cv::Point2f &pitch_yaw);

    void setEnemyColor(COLOR color)
    {
        enemy_color = color;
    }

    /**
     * @brief   构造检测器
     * @param   _CNN  分类器标志符
     */
    ArmorDetector();

    ~ArmorDetector() = default;
};

#endif