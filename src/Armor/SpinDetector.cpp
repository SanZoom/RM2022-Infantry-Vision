#include "SpinDetector.h"

void SpinDetector::reset()
{
    armor_info_window.clear();
    spin_timestamp_list.clear();
    coord_info_window.clear();
    yaw_window.clear();
    is_spin = false;
    fire_flag = true;

    mean_point = cv::Point3f(0, 0, 0);
    last_yaw_diff = 0;
}

bool SpinDetector::judgeSpin(float yaw, uint32_t timestamp)
{
    spin_once_flag = false;
    if (!spin_timestamp_list.empty())
    {
        if (timestamp - spin_timestamp_list[spin_timestamp_list.size() - 1] > param.spin_timeout)
            reset();
    }
    yaw_window.push_back(yaw);

    // 判断是否为跳变帧
    if (yaw_window.size() >= ARMOR_TO_JUDGE_LENGTH)
    {
        if (abs(yaw_window[yaw_window.size() - 1] - yaw_window[yaw_window.size() - 2]) > param.min_yaw_diff)
        {
            spin_once_flag = true;
            spin_timestamp_list.push_back(timestamp);
            if (spin_timestamp_list.size() > param.min_spin_count && (signbit(last_yaw_diff) == signbit(yaw_window[yaw_window.size() - 1] - yaw_window[yaw_window.size() - 2])))
            {
                is_spin = true;
            }
            last_yaw_diff = yaw_window[yaw_window.size() - 1] - yaw_window[yaw_window.size() - 2];
        }
    }
    while (yaw_window.size() > ARMOR_WINDOW_LENGTH)
    {
        yaw_window.erase(yaw_window.begin());
    }
    
    // 判断是否可以开火
    if (is_spin)
    {
        double spin_T = 0.0;
        fire_flag = false;
        if (spin_timestamp_list.size() >= 2)
        {
            for (int i = 0; i < spin_timestamp_list.size() - 1; i++)
            {
                spin_T += (spin_timestamp_list[i + 1] - spin_timestamp_list[i]);
            }
            spin_T /= spin_timestamp_list.size();
            cout << "spin T: " << (double)spin_T / 1000.0 << "s" << endl;
            if ((double)(timestamp - spin_timestamp_list[spin_timestamp_list.size() - 1]) / spin_T < 0.3)
            {
                fire_flag = true;
            }
        }
    }
    
    
    return is_spin;
}

bool SpinDetector::judgeSpin(ArmorBox armor, bool if_spin)
{
    spin_once_flag = false;
    if (!spin_timestamp_list.empty())
    {
        if (armor.timestamp - spin_timestamp_list[spin_timestamp_list.size() - 1] > param.spin_timeout)
            reset();
    }

    armor_info_window.push_back(armor);
    if (armor_info_window.size() >= ARMOR_TO_JUDGE_LENGTH)
    {
        if (ifLegal() || if_spin)
        {
            spin_once_flag = true;
            spin_timestamp_list.push_back(armor.timestamp);
            is_spin = (spin_timestamp_list.size() > param.min_spin_count);
        }
    }

    while (armor_info_window.size() > ARMOR_WINDOW_LENGTH)
    {
        armor_info_window.erase(armor_info_window.begin());
    }

    return is_spin;
}

bool SpinDetector::ifLegal()
{
    if (armor_info_window.size() < 2)
        return false;
    ArmorBox ct1 = armor_info_window[armor_info_window.size() - 1];
    ArmorBox ct2 = armor_info_window[armor_info_window.size() - 2];
    cv::Point2f coord_delta = ct1.rect.center - ct2.rect.center;

    float y_delta = fabs(coord_delta.y);
    float x_delta = fabs(coord_delta.x);
    float light_height = ct1.box.height;

    return (x_delta / light_height > param.min_x_delta_ratio &&
            x_delta / light_height < param.max_x_delta_ratio &&
            y_delta / light_height > param.min_y_delta_ratio &&
            y_delta / light_height < param.max_y_delta_ratio);
}

bool SpinDetector::solverSpin(cv::Point3f &coord)
{
    if (armor_info_window.size() < ARMOR_WINDOW_LENGTH || !is_spin)
        return false;
    
    float mean_distance = 0;
    for (int i = 0; i < coord_info_window.size(); i++)
    {
        mean_distance += sqrt(coord_info_window[i].x * coord_info_window[i].x + coord_info_window[i].y + coord_info_window[i].y);
    }
    mean_distance /= (float)coord_info_window.size();

    coord = cv::Point3f(0, 0, 0);
    int count = 0;
    for (int i = 0; i < coord_info_window.size(); i++)
    {
        if (sqrt(coord_info_window[i].x * coord_info_window[i].x + coord_info_window[i].y + coord_info_window[i].y) < mean_distance);
        {
            count++;
            coord += coord_info_window[i];
        }
    }
    coord /= (float)count;
    double spin_T = 0.0;
    for (int i = 0; i < spin_timestamp_list.size() - 1; i++)
    {
        spin_T += (spin_timestamp_list[i + 1] - spin_timestamp_list[i]);
    }
    spin_T /= spin_timestamp_list.size();
    cout << "spin T: " << (double)spin_T / 1000.0 << "s" << endl;

    return true;
}

void SpinDetector::pushCoord(cv::Point3f coord)
{
    coord_info_window.push_back(coord);
    if (coord_info_window.size() > ARMOR_WINDOW_LENGTH)
    {
        coord_info_window.erase(coord_info_window.begin());
    }
}

SpinDetector::SpinDetector()
{
    reset();
}