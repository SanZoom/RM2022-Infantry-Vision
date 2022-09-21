#ifndef DEBUG_H
#define DEBUG_H
#include "DebugParam.h"
#include "ArmorDetector.h"
#include "RuneDetector.h"

// VIDEO_WIRTER
// #define VIDEO_WRITER_RESULT

void show_armor_result(Mat src, ArmorDetector detector, DebugParam param);
void show_armor_result(Mat src, ArmorDetector detector, VideoWriter& video_writer, DebugParam param);
void show_armor_result(Mat src, RuneDetector detctor, std::vector<cv::Point2f> rune_next_pos, DebugParam param);
#endif
