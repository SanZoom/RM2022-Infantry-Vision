#ifndef DEBUG_PARAM_H
#define DEBUG_PARAM_H

#include <opencv2/opencv.hpp>

class DebugParam{
public:
    bool debug = true;
    bool debug_no_gyro = false;
    bool debug_no_competition = false;
    bool debug_serial_frequency_send = false;
    bool debug_serial_frequency_read = false;
    bool debug_gyro_data = false;
    bool debug_speed = false;

    bool show_armor = true;
    bool show_multi_armor = false;
    bool show_pnp = true;
    bool show_distance = true;
    bool show_ID = true;
    bool show_predict = true;

    bool debug_show_bin = false;
    bool debug_show_light = false;
    bool debug_classifier = false;
public:
    DebugParam();
    ~DebugParam() = default;
};

#endif