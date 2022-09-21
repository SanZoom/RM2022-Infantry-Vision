#include "ArmorBox.h"

using namespace cv;

/*---------------ArmorParam----------------*/
ArmorParam::ArmorParam()
{
    FileStorage fs("../Configure/Settings.xml", FileStorage::READ);

    fs["lights_angle_max_dif"] >> lights_angle_max_dif;
    fs["lights_length_max_ratio"] >> lights_length_max_ratio;
    fs["lights_Y_max_ratio"] >> lights_Y_max_ratio;
    fs["armor_width_height_min_ratio"] >> armor_width_height_min_ratio;
    fs["armor_width_height_max_ratio"] >> armor_width_height_max_ratio;
    fs["armor_max_angle"] >> armor_max_angle;
    fs["armor_inside_thresh"] >> armor_inside_thresh;

    fs["armor_number_confidence"] >> armor_number_confidence;
    fs["erase_wrong_armor"] >> erase_wrong_armor;

    fs["light_height_width_min_ratio"] >> light_height_width_min_ratio;
    fs["light_height_width_max_ratio"] >> light_height_width_max_ratio;
    fs["light_size_area_min_ratio"] >> light_size_area_min_ratio;
    fs["light_angle_to_vertigal_max"] >> light_angle_to_vertigal_max;

    fs["blue_red_diff"] >> blue_red_diff;
    fs["red_blue_diff"] >> red_blue_diff;

    fs["blue_light_threshold"] >> blue_light_threshold;
    fs["red_light_threshold"] >> red_light_threshold;

    fs["same_lights_max_dis"] >> same_lights_max_dis;
    fs["same_armors_max_dis_ratio"] >> same_armors_max_dis_ratio;
    fs["box_height_enlarge"] >> box_height_enlarge;

    fs.release();
}

/*---------------LightBlob----------------*/
LightBlob::LightBlob(cv::RotatedRect &r)
{
    rect = RotatedRect(r);
    regularRotated(rect);

    float x = rect.center.x;
    float y = rect.center.y;
    float angle = rect.angle;
    float height = rect.size.height / 2.0;

    // cmath 的三角函数参数为弧度！！！
    if (angle < 90)
    {
        up = cv::Point(x + height * sin(angle / 180.0 * M_PI), y - height * cos(angle / 180.0 * M_PI));
        down = cv::Point(x - height * sin(angle / 180.0 * M_PI), y + height * cos(angle / 180.0 * M_PI));
    }
    else
    {
        angle = 180 - angle;
        up = cv::Point(x - height * sin(angle / 180.0 * M_PI), y - height * cos(angle / 180.0 * M_PI));
        down = cv::Point(x + height * sin(angle / 180.0 * M_PI), y + height * cos(angle / 180.0 * M_PI));
    }
}

void LightBlob::regularRotated(RotatedRect &r)
{
    if (r.size.width > r.size.height)
    {
        swap<float>(r.size.width, r.size.height);
        r.angle =
            r.angle >= 0.0f ? r.angle - 90.0f : r.angle + 90.f;
    }

    if (r.angle < 0)
        r.angle += 180.0f;
}

/*---------------ArmorBox----------------*/
ArmorBox::ArmorBox(const LightBlob &left, const LightBlob &right, uint32_t timestamp, int box_id)
{

    light_Blobs.push_back(left);
    light_Blobs.push_back(right);

    Point center = (left.rect.center + right.rect.center) / 2.0;
    float width, height, angle;
    width = sqrt(pow(right.rect.center.x - left.rect.center.x, 2) + pow(right.rect.center.y - left.rect.center.y, 2));
    height = max(left.rect.size.height, right.rect.size.height);
    angle = atan2(right.rect.center.y - left.rect.center.y, right.rect.center.x - left.rect.center.x) * 180 / CV_PI;

    float angle_l = left.rect.angle;
    float angle_r = right.rect.angle;

    if (angle_l > 90)
        angle_l = 180 - angle_l;
    if (angle_r > 90)
        angle_r = 180 - angle_r;

    anglediff = fabs(angle_l - angle_r);

    rect = RotatedRect(center, Size(width, height), angle);

    // 初步判断装甲板大小
    if (rect.size.width / rect.size.height > 2.5)
        type = ArmorType::BIG;
    else
        type = ArmorType::SMALL;

    box = Rect(center - Point(width / 2.0, height / 2.0), Size(width, height));

    id = box_id;
    this->timestamp = timestamp;
}

// 1-e^(-x)
bool ArmorBox::operator>(const ArmorBox &armor_2) const
{
    // 分类结果比较
    if ((id >= 8 && armor_2.id < 8) ||
        (id != 1 && armor_2.id == 1) ||
        (id == 2 && armor_2.id != 2))
        return false;
    else if ((type < 8 && armor_2.id >= 8) ||
             (id == 1 && armor_2.id != 1) ||
             (id != 2 && armor_2.id == 2))
        return true;

    cv::Point2f delta = rect.center - armor_2.rect.center;

    // 水平位置相近，滤除高度较小的目标
    // if (abs(delta.y / delta.x) > 50 && delta.y < 0)
    //     return true;
    // else if (abs(delta.y / delta.x) > 50 && delta.y > 0)
    //     return false;


    cv::Point2f center(IMAGE_WIDTH / 2.0, IMAGE_HEIGHT / 2.0);
    float distance_score = calDistance(center, rect.center), distance_score2 = calDistance(center, armor_2.rect.center);

    return distance_score < distance_score2;
}

void ArmorBox::getPoints(std::vector<cv::Point2f> &pts)
{
    pts.clear();

    pts.push_back(light_Blobs[0].up);
    pts.push_back(light_Blobs[1].up);
    pts.push_back(light_Blobs[1].down);
    pts.push_back(light_Blobs[0].down);
}

ArmorBox::~ArmorBox(){};
LightBlob::~LightBlob(){};