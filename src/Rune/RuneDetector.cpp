#include "Rune/RuneDetector.h"

bool RuneDetector::run(Mat_time frame)
{
    frame.copyTo(src);
    target = RuneArmor();
    preDeal(frame);
    if (findRuneArmor())
    {
        calCenter(target);
        setFoundState();

        std::vector<cv::Point2f> pts;
        target.getPoints(pts);

        last_target = target;
        return true;
    }
    if (lost_times < 100 && state != ArmorState::LOST)
    {
        state = ArmorState::FINDING;
        lost_times++;
        cout << "Lost Times: " << lost_times << endl;
    }
    else
    {
        lost_times = 0;
        state = ArmorState::LOST;
    }
    return false;
}

void RuneDetector::preDeal(Mat frame)
{
    Mat gray, gaussian;
    vector<Mat> bgr;

    cvtColor(frame, gray, COLOR_BGR2GRAY);
    split(frame, bgr);

    Mat gray_bin, color_bin;
    if (enemy_color == COLOR::BLUE)
    {
        threshold(gray, gray_bin, param.blue_brightness_thresh, 255, THRESH_BINARY);
        subtract(bgr[0], bgr[2], color_bin);
        threshold(color_bin, color_bin, param.blue_color_thresh, 255, THRESH_BINARY);
    }
    else
    {
        threshold(gray, gray_bin, param.red_brightness_thresh, 255, THRESH_BINARY);
        subtract(bgr[2], bgr[0], color_bin);
        threshold(color_bin, color_bin, param.red_color_thresh, 255, THRESH_BINARY);
    }
    bin = gray_bin & color_bin;

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(bin, bin, element);
    if (debug_param.debug_show_bin)
    {
        imshow("Binary", bin);
        if (waitKey(1) == 'q')
            exit(0);
    }
}

bool RuneDetector::findRuneArmor()
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    rune_armors.clear();

    findContours(bin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++)
    {
        // 面积、长宽比、角点数
        double son_area = contourArea(contours[i]);
        if (son_area < 20)
            continue;

        RotatedRect son_box = minAreaRect(contours[i]);
        regularRotated(son_box);

        float length = son_box.size.height > son_box.size.width ? son_box.size.height : son_box.size.width;
        float width = son_box.size.height < son_box.size.width ? son_box.size.height : son_box.size.width;
        if (length / width < param.armor_min_lw_ratio ||
            length / width > param.armor_max_lw_ratio ||
            son_box.size.area() < param.min_armor_area ||
            son_box.size.area() > param.max_armor_area )
            continue;

        vector<Point> son_hull;
        approxPolyDP(contours[i], son_hull, param.approx_epsilon * 4.0, true);
        if (son_hull.size() < 4 || son_hull.size() > 10)
            continue;
        
        // 父轮廓遍历
        // 面积比、长宽比、角点数、凸包面积比
        for (int j = hierarchy[i][3]; j < contours.size() && j > -1; j = hierarchy[j][3])
        {
            float father_area = contourArea(contours[j]);
            if (son_area / father_area < param.min_armor_vane_ratio ||
                son_area / father_area > param.max_armor_vane_ratio)
                continue;

            vector<Point> father_hull;
            approxPolyDP(contours[j], father_hull, param.approx_epsilon, true);
            if (father_hull.size() < 6)
                continue;
            RotatedRect father_box = minAreaRect(contours[j]);
            length = father_box.size.height > father_box.size.width ? father_box.size.height : father_box.size.width;
            width = father_box.size.height < father_box.size.width ? father_box.size.height : father_box.size.width;
            if (length / width < param.vane_min_lw_ratio ||
                length / width > param.vane_max_lw_ratio ||
                father_area / father_box.size.area() < param.vane_min_contour_rrect_ratio ||
                father_area / father_box.size.area() > param.vane_max_contour_rrect_ratio ||
                father_box.size.area() < param.min_vane_area ||
                father_box.size.area() > param.max_vane_area)
                continue;

            rune_armors.push_back(RuneArmor(contours[i], contours[j], param.approx_epsilon, src.gyro_pose));
            break;
        }
    }
    
    if (rune_armors.size() == 0)
    {
        return false;
    }
    
    return chooseTarget(rune_armors);
}

bool RuneDetector::chooseTarget(vector<RuneArmor> candidate)
{
    auto cmp = [](RuneArmor a, RuneArmor b) -> bool
    {
        return a > b;
    };
    sort(candidate.begin(), candidate.end(), cmp);
    if (!isTarget(candidate[0]))
        return false;

    target = candidate[0];
    return true;
}

bool RuneDetector::isTarget(RuneArmor _armor, cv::Mat binary)
{
    if (_armor.area_ratio > param.max_armor_vane_ratio ||
        _armor.area_ratio < param.min_armor_vane_ratio ||
        _armor.vane.hull_num < 10 ||
        _armor.vane.hull_num > 80 ||
        _armor.vane.c_area / _armor.vane.rrect.size.area() < param.target_vane_min_contour_rrect_ratio ||
        _armor.vane.c_area / _armor.vane.rrect.size.area() > param.target_vane_max_contour_rrect_ratio)
    {
        return false;
    }
    return true;
}

bool RuneDetector::isActivated(RuneArmor t_armor)
{
    if (t_armor.area_ratio < param.min_shot_armor_vane_ratio ||
        t_armor.area_ratio > param.max_shot_armor_vane_ratio ||
        t_armor.vane.c_area / t_armor.vane.rrect.size.area() < param.min_shot_vane_contour_rrect_ratio ||
        t_armor.vane.c_area / t_armor.vane.rrect.size.area() > param.max_shot_vane_contour_rrect_ratio ||
        t_armor.vane.hull_num < 6 || t_armor.vane.hull_num > 30)
        return false;
    return true;
}


bool RuneDetector::calCenter(RuneArmor &t_armor)
{
    
    cv::Point2f temp_circle_center = cv::Point2f(t_armor.circle_center);
    float R_roi = t_armor.rrect.size.width > t_armor.rrect.size.height? t_armor.rrect.size.width  : t_armor.rrect.size.height;
    R_roi /= 1.7;
    cv::Rect2f R_roi_rect = cv::Rect2f(temp_circle_center.x - R_roi, temp_circle_center.y - R_roi, 2 * R_roi, 2 * R_roi);
    R_roi_rect &= cv::Rect2f(cv::Point2f(0, 0), cv::Point2f(bin.cols, bin.rows));
   
    if (R_roi_rect.width <= 10 || R_roi_rect.height <= 10)    return false;
    cv::Mat temp_R_roi = bin(R_roi_rect);

    if (debug_param.debug_show_bin)
    {
        cv::imshow("R_Bin", temp_R_roi);
        if (cv::waitKey(1) == 'q')  exit(0);
    }

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(temp_R_roi, contours, hierarchy, cv::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, cv::Point2f(R_roi_rect.x, R_roi_rect.y));
    if (contours.size() <= 0)
        return false;

    int max_i = -1;
    double max_area = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area / t_armor.c_area < param.min_R_area_ratio)
            continue;
        if (area > max_area)
        {
            max_area = area;
            max_i = i;
        }
    }
    if (max_i < 0)    return false;
    cv::RotatedRect R_rrect = minAreaRect(contours[max_i]);
    if (calDistance(t_armor.circle_center, R_rrect.center) / R_roi > param.max_R_to_predict_distance_ratio)
        return false;

    t_armor.circle_center = R_rrect.center;
    return true;
}

bool RuneDetector::ifOldArmor()
{
    if (last_target.rrect.boundingRect().empty())   return false;

    float length = target.rrect.size.width > target.rrect.size.height? target.rrect.size.width : target.rrect.size.height;
    if (calDistance(target.rrect.center, last_target.rrect.center) / length > param.max_diff_distance_ratio)
        return false;
    return true;
}


RuneDetector::RuneDetector()
{
    int color;
    cv::FileStorage fs("../Configure/Settings.xml", cv::FileStorage::READ);
    fs["enemy_color"] >> color;
    fs.release();
    setEnemyColor((COLOR)color);
}