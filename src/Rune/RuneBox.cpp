#include "Rune/RuneBox.h"

bool RuneArmor::operator>(RuneArmor armor_2)
{
    // 待激活目标，角点数多，内外轮廓面积比较大
    if (vane.hull_num > armor_2.vane.hull_num)
        return true;
    else
    {
        return area_ratio > armor_2.area_ratio;
    }
}

bool RuneArmor::operator<(RuneArmor armor_2)
{
    return !(*this > armor_2);
}


void RuneArmor::regularRotated(cv::RotatedRect &r)
{
    if (r.size.width > r.size.height)
    {
        swap<float>(r.size.width, r.size.height);
        r.angle = 
            r.angle >= 0.0f ? r.angle - 90.0f : r.angle + 90.f;
    }
    
    if (r.angle < 0)    r.angle += 180.0f;
}

RuneArmor::RuneArmor(vector<Point> armor_contour, vector<Point> vane_contour, float epsilon, GyroPose _gyro_pose)
{
    gyro_pose = _gyro_pose;
    vane = Vane(vane_contour, epsilon);
    contour = armor_contour;
    rrect = minAreaRect(armor_contour);
    regularRotated(rrect);
    c_area = contourArea(armor_contour);
    area_ratio = c_area / vane.c_area;
    r_direction = rrect.center - vane.rrect.center;
    angle = atan2(r_direction.y, r_direction.x);

    // 根据比例计算
    // a = 1 / 3.3;
    // OB = (1 / a) * OP - (1 / a - 1) * OA
    circle_center = vane.rrect.center * 3.3 - 2.3 * rrect.center;

}

void RuneArmor::getPoints(vector<cv::Point2f>& armor_points)
{
    cv::Point2f pts[4];
    rrect.points(pts);
    float max_value = 0.0;
    int max_i = 0;
    for (int i = 0; i < 4; i++)
    {
        cv::Point2f delta_ = pts[i] - pts[(i + 1) % 4];
        if (max_value > delta_.x * r_direction.x + delta_.y * r_direction.y)
        {
            max_i = i;
            max_value =  delta_.x * r_direction.x + delta_.y * r_direction.y;
        }
    }

    for (int i = max_i + 1; i < max_i + 5; i++)
    {
        armor_points.push_back(pts[i % 4]);
    }
}

Vane::Vane(vector<Point> vane_contour, float epsilon)
{
    contour = vane_contour;
    rrect = minAreaRect(contour);
    vector<Point> hull;
    approxPolyDP(contour, hull, epsilon, true);
    hull_num = hull.size();
    c_area = contourArea(vane_contour);
}

RuneParam::RuneParam()
{
    FileStorage fs("../Configure/RuneParam.xml", FileStorage::READ);
    
    fs["approx_epsilon"] >> approx_epsilon;
    fs["blue_brightness_thresh"] >> blue_brightness_thresh;
    fs["blue_color_thresh"] >> blue_color_thresh;
    fs["red_brightness_thresh"] >> red_brightness_thresh;
    fs["red_color_thresh"] >> red_color_thresh;
    fs["blue_red_diff"] >> blue_red_diff;
    fs["min_armor_area"] >> min_armor_area;
    fs["max_armor_area"] >> max_armor_area;
    fs["min_vane_area"] >> min_vane_area;
    fs["max_vane_area"] >> max_vane_area;
    fs["min_armor_vane_ratio"] >> min_armor_vane_ratio;
    fs["max_armor_vane_ratio"] >> max_armor_vane_ratio;
    fs["min_target_vane_ratio"] >> min_target_vane_ratio;
    fs["max_target_vane_ratio"] >> max_target_vane_ratio;
    fs["armor_min_lw_ratio"] >> armor_min_lw_ratio;
    fs["armor_max_lw_ratio"] >> armor_max_lw_ratio;
    fs["vane_min_lw_ratio"] >> vane_min_lw_ratio;
    fs["vane_max_lw_ratio"] >> vane_max_lw_ratio;
    fs["vane_min_contour_rrect_ratio"] >> vane_min_contour_rrect_ratio;
    fs["vane_max_contour_rrect_ratio"] >> vane_max_contour_rrect_ratio;
    fs["target_vane_min_contour_rrect_ratio"] >> target_vane_min_contour_rrect_ratio;
    fs["target_vane_max_contour_rrect_ratio"] >> target_vane_max_contour_rrect_ratio;
    fs["min_shot_armor_vane_ratio"] >> min_shot_armor_vane_ratio;
    fs["max_shot_armor_vane_ratio"] >> max_shot_armor_vane_ratio;
    fs["min_shot_vane_contour_rrect_ratio"] >> min_shot_vane_contour_rrect_ratio;
    fs["max_shot_vane_contour_rrect_ratio"] >> max_shot_vane_contour_rrect_ratio;
    fs["min_R_area_ratio"] >> min_R_area_ratio;
    fs["max_R_to_predict_distance_ratio"] >> max_R_to_predict_distance_ratio;
    fs["max_diff_distance_ratio"] >> max_diff_distance_ratio;
    fs.release();
}