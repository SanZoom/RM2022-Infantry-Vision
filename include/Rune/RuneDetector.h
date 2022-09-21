#ifndef RUNEDETECTOR_H
#define RUNEDETECTOR_H

#include "RuneBox.h"
#include "Solver.h"
#include "Mat_time.h"
#include "const.h"
#include "DebugParam.h"

class RuneDetector
{
private:
    Mat_time src;
    Mat bin;

    RuneParam param;
    DebugParam debug_param;

    COLOR enemy_color;
    vector<RuneArmor> rune_armors;

    int lost_times = 0; 

public:
    RuneArmor target;
    RuneArmor last_target;
    vector<cv::Point2f> nextPos;

    ArmorState state = ArmorState::LOST;

    Solver solver;      // PnP

public:
    RuneDetector();
    ~RuneDetector() = default;

    /**
     *  @brief  API
     *  @param  _src 输入的图像
     *  @return 是否检测到装甲板
     */
    bool run(Mat_time _src);

    void setEnemyColor(COLOR color)
    {
        if (color == COLOR::BLUE)
            enemy_color = COLOR::RED;
        else if (color == COLOR::RED)
            enemy_color = COLOR::BLUE;
    }

    /**
     *  @brief  计算瞬时角速度 (armor_1 - armor_2)
     *  @param  armor_1 新目标
     *  @param  armor_2 老目标
     *  @return 角速度，单位 弧度/秒
     */
    double calAngleSpeed(RuneArmor armor_1, RuneArmor armor_2);

private:
    /**
     *  @brief  亮度阈值+颜色通道相减，预处理图像
     *  @param  frame 原图像
     *  @param  bin 二值图
     */
    void preDeal(Mat frame);

    /**
     *  @brief  找到装甲板并区分
     *  @param  bin 二值图
     */
    bool findRuneArmor();

    /**
     *  @brief  选择待激活目标
     *  @param  candidate 候选装甲板
     */
    bool chooseTarget(vector<RuneArmor> candidate);

    /**
     *  @brief  约束条件确认待激活目标
     *  @param  t_armor 待确认目标
     */
    bool isTarget(RuneArmor t_armor, cv::Mat binary = cv::Mat());

    /**
     *  @brief  约束条件确认已激活目标
     *  @param  t_armor 待确认目标
     */
    bool isActivated(RuneArmor t_armor);

    /**
     *  @brief  测算圆心R
     *  @param  装甲板目标
     */
    bool calCenter(RuneArmor &t_armor);

    /**
     *  @brief  判断是否为last_target
     */
    bool ifOldArmor();

    /**
     *  @brief  设置跟踪状态设置
     */
    void setFoundState()
    {
        lost_times = 0;

        switch (state)
        {
        case ArmorState::LOST:
        case ArmorState::FINDING:
            state = ArmorState::FIRST;
            break;
        case ArmorState::FIRST:
        case ArmorState::SHOOT:
            if (ifOldArmor())
            {
                state = ArmorState::SHOOT;
            }
            else
                state = ArmorState::FIRST;
            break;
        default:
            break;
        }
    }
};

#endif