#include "Debug.h"

void show_armor_result(Mat src, ArmorDetector detector, DebugParam debug_param)
{
    Mat show_img = src.clone();
    if (debug_param.show_armor && detector.state != ArmorState::LOST)
    {
        ArmorBox temp = detector.target;
        vector<Point2f> pts;
        Point2f armor_center;
        temp.getPoints(pts);
        for (int i = 0; i < pts.size(); i++)
        {
            line(show_img, pts[i], pts[(i + 1) % pts.size()], Scalar(0, 0, 255));
        }

        if (debug_param.show_pnp)
        {
            float pitch, yaw;
            pitch = detector.pitch;
            yaw = detector.yaw;
            string text = "Pitch: " + to_string(pitch) + " Yaw: " + to_string(yaw);
            if (debug_param.show_distance)
            {
                text += "\nDistance: " + to_string(detector.solver.distance);
            }
            putText(show_img, text, temp.rect.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        }
        if (debug_param.show_ID)
        {
            int id = temp.id;
            string text_id = "ID: " + to_string(id);
            putText(show_img, text_id, temp.box.tl(), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        }
        if (debug_param.show_predict)
        {
            cv::circle(show_img, detector.cp_2D, 5, Scalar(0, 255, 0), -1);
        }
    }
    else if (debug_param.show_multi_armor && detector.state != ArmorState::LOST)
    {
        ArmorBoxes temp = detector._armor_boxes;
        Mat armors_img = src.clone();

        for (int i = 0; i < temp.size(); i++)
        {
            vector<Point2f> pts;
            temp[i].getPoints(pts);
            for (int j = 0; j < pts.size(); j++)
                line(armors_img, pts[j], pts[(j + 1) % pts.size()], Scalar(0, 0, 255));
        }
        imshow("Debug Armors", armors_img);
        waitKey(1);
    }
    circle(show_img, detector.last_target.rect.center, 5, Scalar(0, 0, 255), -1);
    circle(show_img, Point(640, 360), 5, Scalar(0, 0, 255), -1);
    namedWindow("Debug Armor", WINDOW_NORMAL);
    resizeWindow("Debug Armor", Size(640, 360));
    imshow("Debug Armor", show_img);
    if (waitKey(1) == 'q')
        exit(0);
}

void show_armor_result(Mat src, ArmorDetector detector, VideoWriter &video_writer, DebugParam debug_param)
{
    Mat show_img = src.clone();
    if (debug_param.show_armor && detector.state != ArmorState::LOST)
    {
        ArmorBox temp = detector.target;
        vector<Point2f> pts;
        temp.getPoints(pts);
        for (int i = 0; i < pts.size(); i++)
            line(show_img, pts[i], pts[(i + 1) % pts.size()], Scalar(0, 0, 255));
        if (debug_param.show_pnp)
        {
            float pitch, yaw;
            pitch = detector.pitch;
            yaw = detector.yaw;
            string text = "Pitch: " + to_string(pitch) + " Yaw: " + to_string(yaw);
            if (debug_param.show_distance)
            {
                text += "\nDistance: " + to_string(detector.solver.distance);
            }
            putText(show_img, text, temp.rect.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        }
        if (debug_param.show_ID)
        {
            int id = temp.id;
            string text_id = "ID: " + to_string(id);
            putText(show_img, text_id, temp.box.tl(), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        }
    }
    else if (debug_param.show_multi_armor && detector.state != ArmorState::LOST)
    {
        ArmorBoxes temp = detector._armor_boxes;
        Mat armors_img = src.clone();

        for (int i = 0; i < temp.size(); i++)
        {
            vector<Point2f> pts;
            temp[i].getPoints(pts);
            for (int j = 0; j < pts.size(); j++)
                line(armors_img, pts[j], pts[(j + 1) % pts.size()], Scalar(0, 0, 255));
        }
        imshow("Debug Armors", armors_img);
        waitKey(1);
    }
    namedWindow("Debug Armor", WINDOW_NORMAL);
    video_writer << show_img;
    imshow("Debug Armor", show_img);
    if (waitKey(1) == 'q')
        exit(0);
}

void show_armor_result(Mat src, RuneDetector detector, std::vector<cv::Point2f> rune_next_pos, DebugParam debug_param)
{
    Mat show_img = src.clone();
    if (debug_param.show_armor && detector.state != ArmorState::LOST)
    {
        RuneArmor temp = detector.target;
        vector<Point2f> pts;
        Point2f armor_center;

        temp.getPoints(pts);
        for (int i = 0; i < pts.size(); i++)
        {
            line(show_img, pts[i], pts[(i + 1) % pts.size()], Scalar(0, 0, 255));
            circle(show_img, pts[i], 2 * (i + 1), Scalar(0, 0, 255), -1);
        }
        circle(show_img, temp.circle_center, calDistance(temp.circle_center, temp.rrect.center), Scalar(255, 0, 0));    // 大符轨迹
        
        if (debug_param.show_pnp)
        {
            float pitch, yaw;
            pitch = detector.solver.pitch;
            yaw = detector.solver.yaw;
            string text = "Pitch: " + to_string(pitch) + " Yaw: " + to_string(yaw);
            if (debug_param.show_distance)
            {
                text += "\nDistance: " + to_string(detector.solver.distance);
            }
            putText(show_img, text, temp.rrect.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
        }

        if (debug_param.show_predict)
        {
            for (int i = 0; i < rune_next_pos.size(); i++)
            {
                line(show_img, rune_next_pos[i], rune_next_pos[(i + 1) % rune_next_pos.size()], Scalar(0, 0, 255));
            }
        }

        
    }
    circle(show_img, Point(640, 360), 5, Scalar(0, 0, 255), -1);
    namedWindow("Debug Armor", WINDOW_NORMAL);
    resizeWindow("Debug Armor", Size(640, 360));
    imshow("Debug Armor", show_img);
    if (waitKey(1) == 'q')
        exit(0);
}

DebugParam::DebugParam()
{
    cv::FileStorage fs("../Configure/Debug.xml", cv::FileStorage::READ);
    fs["debug"] >> debug;
    fs["debug_no_gyro"] >> debug_no_gyro;
    fs["debug_serial_frequency_send"] >> debug_serial_frequency_send;
    fs["debug_serial_frequency_read"] >> debug_serial_frequency_read;
    fs["debug_gyro_data"] >> debug_gyro_data;
    fs["debug_speed"] >> debug_speed;

    fs["show_armor"] >> show_armor;
    fs["show_multi_armor"] >> show_multi_armor;
    fs["show_pnp"] >> show_pnp;
    fs["show_distance"] >> show_distance;
    fs["show_ID"] >> show_ID;
    fs["show_predict"] >> show_predict;
    fs["debug_show_bin"] >> debug_show_bin;
    fs["debug_show_light"] >> debug_show_light;
    fs["debug_classifier"] >> debug_classifier;
    fs["debug_no_competition"] >> debug_no_competition;
    fs.release();
}