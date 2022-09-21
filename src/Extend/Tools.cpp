#include "Tools.h"

void Gamma(Mat src, Mat &dst, float fGamma)
{
    unsigned char lut[256];

    for (int i = 0; i < 256; i++)
    {
        float normalize = (float)(i/255.0);
        lut[i] = saturate_cast<uchar>(pow(normalize, fGamma) * 255.0f);
    }

    src.copyTo(dst);
    MatIterator_<Vec3b> it,end;
    for (it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++)
    {
        (*it)[0] = lut[((*it)[0])];
        (*it)[1] = lut[((*it)[1])];
        (*it)[2] = lut[((*it)[2])];
    }
    
}

float calDistance(cv::Point2f pt1, cv::Point2f pt2)
{
    cv::Point2f dis = pt1 - pt2;
    return sqrt(pow(dis.x,2)+pow(dis.y,2));
}


/**      Z
 *       |   X
 *       |  /   
 *       | /
 * Y_____|/
 *      
 * IMU 的世界坐标系
 */
cv::Point3f camera2world(Eigen::Quaternionf q1, cv::Point3f point, cv::Point3f trans_offset)
{
    point += trans_offset;
    Eigen::Quaternionf p(0, point.z, -point.x, -point.y);

    Eigen::Quaternionf result = q1 * p *q1.inverse();
    return cv::Point3f(result.x(), result.y(), result.z());
}

cv::Point3f world2camera(Eigen::Quaternionf q1, cv::Point3f point, cv::Point3f trans_offset)
{
    Eigen::Quaternionf p(0, point.x, point.y, point.z);

    Eigen::Quaternionf result = q1.inverse() * p * q1;
    return cv::Point3f(-result.y(), -result.z(), result.x()) - trans_offset;
}

cv::Mat getRotatedMask(cv::RotatedRect rotatedRect, cv::Size size)
{
    cv::Mat mask(size, CV_8UC1, Scalar(0));
    
    cv::Point2f pts[4];
    float angle = rotatedRect.angle * CV_PI / 180.0;
    rotatedRect.points(pts);

    cv::Point2f down_point = pts[0];
    float width = rotatedRect.size.width, height = rotatedRect.size.height;
    for (int i = down_point.x; i < down_point.x + width; i++)
    {
        for (int j = down_point.y; j > down_point.y - height; j--)
        {
            cv::Point2f point_in_rect(i, j);
            float radius = calDistance(point_in_rect, down_point);
            cv::Point2f relative_pos = point_in_rect - down_point;
            float relative_angle = atan2(relative_pos.y, relative_pos.x);
            float next_angle = relative_angle + angle;
            
            cv::Point2f next_pos(cv::Point2f(cos(next_angle) * radius, sin(next_angle) * radius) + down_point);
            mask.at<u_char>(next_pos) = 255;
        }
    }
    return mask;
}

void regularRotated(cv::RotatedRect &r)
{
    if (r.size.width > r.size.height)
    {
        swap<float>(r.size.width, r.size.height);
        r.angle = 
            r.angle >= 0.0f ? r.angle - 90.0f : r.angle + 90.f;
    }
    
    if (r.angle < 0)    r.angle += 180.0f;
}