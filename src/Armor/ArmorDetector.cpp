#include "ArmorDetector.h"
#include <chrono>

bool ArmorDetector::setRoi(Mat_time _src, cv::Rect &tracking_rect)
{
    _src.copyTo(src);
    if (tracking_rect.empty())
    {
        if (roi_temp.empty())
        {
            roi = src;
            offset = Point(0, 0);
            lost_count = 0;
            state = ArmorState::LOST;

            return true;
        }
        if (lost_count < 130)
        {
            get_roi(roi_enlarge);
            lost_count++;
            state = ArmorState::FINDING;
            return true;
        }
        else
        {
            roi = src;
            roi_rect = Rect();
            roi_temp = Rect();
            offset = Point(0, 0);
            lost_count = 0;
            state = ArmorState::LOST;
            return true;
        }
    }

    roi_temp = Rect(tracking_rect);
    get_roi(roi_enlarge);
    lost_count = 0;
    return true;
}

bool ArmorDetector::findLightBlobs(LightBlobs &light_blobs)
{
    light_blobs.clear();
    if (roi.empty())
        return false;

    cv::Mat guassian;
    cv::Mat gray;

    cvtColor(roi, gray, COLOR_BGR2GRAY);

    vector<cv::Mat> bgr;
    split(roi, bgr);

    /*-----------预处理得二值图-----------*/
    cv::Mat gray_binary, color_binary, binary;
    if (enemy_color == RED)
    {
        threshold(gray, gray_binary, param.red_light_threshold, 255, THRESH_BINARY);

        subtract(bgr[2], bgr[0], color_binary);
        threshold(color_binary, color_binary, param.red_blue_diff, 255, THRESH_BINARY);

        binary = gray_binary & color_binary;
    }
    else
    {
        threshold(gray, gray_binary, param.blue_light_threshold, 255, THRESH_BINARY);

        subtract(bgr[0], bgr[2], color_binary);
        threshold(color_binary, color_binary, param.blue_red_diff, 255, THRESH_BINARY);

        binary = gray_binary & color_binary;
    }
    cv::Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(binary, binary, MORPH_CLOSE, element);

    /*-----------寻找并筛选灯条轮廓-----------*/
    vector<vector<Point>> contours;
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE, offset);

    if (debug_param.debug_show_bin)
    {
        imshow("binaray", binary);
        waitKey(1);
    }

    for (int i = 0; i < contours.size(); i++)
    {
        if (!filterLightBlob(contours[i]))
            continue;

        RotatedRect rrect = minAreaRect(contours[i]);
        LightBlob temp = LightBlob(rrect);

        light_blobs.push_back(temp);
    }

    if (light_blobs.size() < 2)
    {
        return false;
    }

    if (debug_param.debug_show_light)
    {
        Mat_time light;
        src.copyTo(light);
        for (int i = 0; i < light_blobs.size(); i++)
        {
            Point2f pts[4];
            light_blobs[i].rect.points(pts);
            for (int i = 0; i < 4; i++)
            {
                line(light, pts[i % 4], pts[(i + 1) % 4], Scalar(0, 0, 255), 1);
            }
        }

        imshow("light", light);
    }

    return true;
}

bool ArmorDetector::filterLightBlob(const std::vector<cv::Point> &contour)
{
    if (contourArea(contour) < 5)
        return false;

    RotatedRect light_rrect = minAreaRect(contour);
    LightBlob temp(light_rrect);
    light_rrect = temp.rect;

    // 筛选条件：面积，凸包比，垂直程度，高宽比
    if (contourArea(contour) / light_rrect.size.area() < param.light_size_area_min_ratio ||
        (light_rrect.angle < 90 && light_rrect.angle > param.light_angle_to_vertigal_max) ||
        (light_rrect.angle > 90 && 180 - light_rrect.angle > param.light_angle_to_vertigal_max) ||
        light_rrect.size.height / light_rrect.size.width < param.light_height_width_min_ratio)
        return false;

    return true;
}

bool ArmorDetector::matchArmorBoxes(LightBlobs &light_blobs, ArmorBoxes &armor_boxes)
{
    armor_boxes.clear();

    auto cmp = [](LightBlob a, LightBlob b) -> bool
    {
        return a.rect.center.x < b.rect.center.x;
    };
    sort(light_blobs.begin(), light_blobs.end(), cmp);

    for (int i = 0; i < light_blobs.size() - 1; i++)
    {
        for (int j = i + 1; j < light_blobs.size(); j++)
        {
            if (!isCoupleLight(light_blobs[i], light_blobs[j]))
                continue;
            if (isBadArmor(i, j, light_blobs))
                continue;

            armor_boxes.push_back(ArmorBox(light_blobs[i], light_blobs[j]));
        }
    }

    if (armor_boxes.empty())
    {
        return false;
    }

    return getArmorNum(armor_boxes);
}

bool ArmorDetector::isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j)
{
    float width = light_blob_j.rect.center.x - light_blob_i.rect.center.x;
    float height = (light_blob_j.rect.size.height + light_blob_i.rect.size.height) / 2.0;
    float angle_i = light_blob_i.rect.angle > 90 ? 180.0 - light_blob_i.rect.angle : light_blob_i.rect.angle;
    float angle_j = light_blob_j.rect.angle > 90 ? 180.0 - light_blob_j.rect.angle : light_blob_j.rect.angle;

    // 匹配灯条：角度差、长度比、高度差、组成的宽高比，滤除/\ \/
    if (fabs(angle_i - angle_j) > param.lights_angle_max_dif ||
        max(light_blob_i.rect.size.height, light_blob_j.rect.size.height) / min(light_blob_i.rect.size.height, light_blob_j.rect.size.height) > param.lights_length_max_ratio ||
        fabs(light_blob_i.rect.center.y - light_blob_j.rect.center.y) / (height * cos(angle_i / 180.0 * M_PI)) > param.lights_Y_max_ratio ||
        width / height > param.armor_width_height_max_ratio ||
        width / height < param.armor_width_height_min_ratio ||
        (fabs(light_blob_i.rect.angle - light_blob_j.rect.angle) >= 90 && fabs(light_blob_i.rect.angle - light_blob_j.rect.angle) < 160)) // 滤除 /\ 和 \/ 灯条
        return false;

    return true;
}

bool ArmorDetector::isBadArmor(int i, int j, const LightBlobs &lightblobs)
{
    float y_min = lightblobs[i].up.y < lightblobs[j].up.y ? lightblobs[i].up.y : lightblobs[j].up.y;
    float y_max = lightblobs[i].down.y > lightblobs[j].down.y ? lightblobs[i].down.y : lightblobs[j].down.y;

    ArmorBox temp(lightblobs[i], lightblobs[j]);
    if (fabs(temp.rect.angle) > param.armor_max_angle)
        return true;

    for (int m = i + 1; m < j; m++)
    {
        float light_min = lightblobs[m].up.y;
        float light_max = lightblobs[m].down.y;
        float light_center = lightblobs[m].rect.center.y;

        if ((light_min > y_min && light_min < y_max) ||
            (light_center > y_min && light_center < y_max) ||
            (light_max > y_min && light_max < y_max) ||
            (light_min < y_min && light_max > y_max))
            return true;
    }
    //  滤除类如窗户形成的伪装甲板，即装甲板区域过亮
    Mat temp_inarmor = src(temp.box & Rect(Point(0, 0), src.size()));
    cvtColor(temp_inarmor, temp_inarmor, COLOR_BGR2GRAY);
    if (mean(temp_inarmor)[0] > param.armor_inside_thresh)
        return true;
    return false;
}

// 使用CNN分类
bool ArmorDetector::getArmorNum(ArmorBoxes &armor_boxes)
{
    Mat temp;
    bool all_wrong_flag = true;      // 是否全为WRONG装甲板
    for (auto &armor : armor_boxes)
    {
        adjustBox(armor.box);
        temp = src(Rect2d(armor.box));
        resize(temp, temp, Size(28, 28), cv::INTER_LINEAR);
        Gamma(temp, temp, 0.6);
        cvtColor(temp, temp, COLOR_BGR2GRAY);
        armor.id = classifier(temp);
        armor.confidence = 1.0;

        if (debug_param.debug_classifier)
            imshow("ID", temp);

        switch (armor.id)
        {
            case 0:
            case 1:
            case 6:
            case 7:
                all_wrong_flag = false;
                armor.type = ArmorType::BIG;
                break;
            
            case 2:
            case 3:
            case 4:
            case 5:
                all_wrong_flag = false;
                armor.type = ArmorType::SMALL;
                break;

            // case 3:
            // case 4:
            // case 5:
            //     all_wrong_flag = false;

            default:
                break;
        }
        
    }
    return !(param.erase_wrong_armor && all_wrong_flag);
}

bool ArmorDetector::getBestArmor(ArmorBoxes &boxes)
{
    auto cmp = [](ArmorBox a, ArmorBox b)
    {
        return a > b;
    };

    sort(boxes.begin(), boxes.end(), cmp);
    return true;
}

bool ArmorDetector::findArmorBox(ArmorBox &box, uint32_t time_)
{
    box = ArmorBox();
    target = ArmorBox();
    _armor_boxes.clear();
    _light_blobs.clear();

    ArmorBoxes armor_boxes;
    if (!findLightBlobs(_light_blobs))
        return false;
    if (!matchArmorBoxes(_light_blobs, armor_boxes))
        return false;
    if (!getBestArmor(armor_boxes))
        return false;

    _armor_boxes = armor_boxes;

    target = armor_boxes[0];
    cout << "ID: " << target.id << endl;
    target.timestamp = time_;
    box = armor_boxes[0];

    if (ifOldArmor())
        state = ArmorState::SHOOT;
    else
        state = ArmorState::FIRST;

    last_target = target;

    return true;
}

cv::Point2f ArmorDetector::getPitchYaw(ArmorBox temp_target, GyroPose gyro_pose)
{
    vector<Point2f> _pts;
    temp_target.getPoints(_pts);
    solver.solve(_pts, temp_target.type);
    
    Eigen::Quaternionf q(gyro_pose.q_0, gyro_pose.q_1, gyro_pose.q_2, gyro_pose.q_3);
    cv::Point3f world_coord = camera2world(q, solver.point_3D, predictor.cam2gyro_offset);
    float temp_yaw = atan(world_coord.x / world_coord.z) * 180.0 / CV_PI;
    if (state == ArmorState::FIRST)
        predictor.initState(solver.point_3D, gyro_pose);
    cv::Point3f world_predict = predictor.predict(solver.point_3D, spin_detector, gyro_pose);

    last_world_point = predictor.world_coord;
    return calPredict(world_predict, gyro_pose);
}

cv::Point2f ArmorDetector::getPitchYaw(cv::Point3f world_point, GyroPose gyro_pose)
{
    cv::Point3f world_predict = predictor.predict(world_point, spin_detector, gyro_pose.timestamp);
    return calPredict(world_predict, gyro_pose);
}

cv::Point2f ArmorDetector::calPredict(cv::Point3f world_predict, GyroPose gyro_pose)
{
    // 固定坐标系转回相机坐标系
    cv::Point3f cp_predict = world2camera(Eigen::Quaternionf(gyro_pose.q_0, gyro_pose.q_1, gyro_pose.q_2, gyro_pose.q_3),
     world_predict,
     predictor.cam2gyro_offset);
    

    // 将3D点投影回2D(用于直观测试)
    vector<cv::Point3d> origin_point;
    origin_point.push_back(cv::Point3f(0, 0, 0));
    vector<cv::Point2d> cp_2Ds;
    cv::Mat t(cp_predict);
    cv::projectPoints(origin_point, solver.rVec, t, solver.param.camera_matrix, solver.param.dist_coeffs, cp_2Ds);
    cp_2D = cp_2Ds[0];

    // 相对相机偏角
    yaw = atan(cp_predict.x / cp_predict.z) * 180.0 / CV_PI - 0.3;
    float cp_pitch = atan(cp_predict.y / cp_predict.z) * 180.0 / CV_PI;

    // 绝对坐标Pitch
    float targetPitch = atan(world_predict.z / sqrt(world_predict.x * world_predict.x + world_predict.y * world_predict.y)) * 180.0 / CV_PI;

    // 抬枪补偿
    pitch = cp_pitch - predictor.pitch_time.x + targetPitch + 1.0;
    std::cout << "targetPitch: " << targetPitch << endl;
    std::cout << "resultPitch: " << pitch << endl;
    return cv::Point2f(pitch, yaw);
}

bool ArmorDetector::run(Mat_time _src, cv::Point2f &pitch_yaw)
{
    ArmorBox temp_target;
    setRoi(_src, target.box);
    if (!findArmorBox(temp_target, _src.gyro_pose.timestamp))
    {
        if (state != ArmorState::LOST)
        {
            pitch_yaw = getPitchYaw(last_world_point, _src.gyro_pose);
        }
        return true;
    }

    pitch_yaw = getPitchYaw(temp_target, _src.gyro_pose);
    spin_detector.pushCoord(predictor.world_coord);
    return true;
}

ArmorDetector::ArmorDetector()
{
    FileStorage fs("../Configure/Settings.xml", FileStorage::READ);
    int color;
    fs["enemy_color"] >> color;
    fs["roi_width_to_height_ratio"] >> roi_width_to_height_ratio;
    fs["roi_enlarge"] >> roi_enlarge;
    enemy_color = (COLOR)color;
    fs.release();
    classifier = Classifier("../Configure/param/");
}

void ArmorDetector::adjustBox(Rect &r)
{
    r.y -= r.height / 2.0 * param.box_height_enlarge;
    r.height *= 2.0 * param.box_height_enlarge;
    r &= Rect(Point(0, 0), src.size());
}

bool ArmorDetector::ifOldArmor()
{
    if (last_target.box.empty() || state == ArmorState::LOST)
        return false;

    // 针对装甲板大小类型的突变
    if (lost_count < 70 && last_target.type != target.type && use_last_count < 70)
    {
        cout << target.type << " " << last_target.type << endl;
        target = last_target;
        use_last_count++;
        cout << "USE LAST" << endl;
        return true;
    }
    else
        use_last_count = 0;

    cv::Point2f delta = target.rect.center - last_target.rect.center;
    float distance = sqrt(delta.x * delta.x + delta.y * delta.y);

    if (last_target.id == target.id ||
        distance / target.box.height < param.same_armors_max_dis_ratio)
        return true;

    return false;
}
