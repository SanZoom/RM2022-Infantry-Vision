#include <Mat_time.h>

void Mat_time::setProducedTime()
{
    produced_time = chrono::steady_clock::now() - time_delay;
}

void Mat_time::setDoneTime()
{
    done_time = chrono::steady_clock::now();
    if (produced_time.time_since_epoch().count() != 0)
        process_time = 
            chrono::duration_cast<chrono::microseconds>(done_time - produced_time).count()/1000.0/1000.0;
}

float Mat_time::getProcessTime()
{
    return process_time;
}

// void Mat_time::setProducedTime(Image_time _produced_time)
// {
//     produced_time = _produced_time;
// }

// void Mat_time::setDoneTime(Image_time _done_time)
// {
//     done_time = _done_time;
//     if (produced_time.time_since_epoch().count() != 0)
//         process_time = 
//             chrono::duration_cast<chrono::microseconds>(done_time - produced_time).count()/1000.0/1000.0;
// }

// float Mat_time::getProcessTime(const Mat_time& first, const Mat_time& second)
// {
//     process_time = 
//         chrono::duration_cast<chrono::microseconds>(second.done_time - first.produced_time).count()/1000.0/1000.0;
//     return process_time;
// }

// double duration(chrono::steady_clock::time_point begin)
// {
//     return double(chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - begin).count()/1000.0);
// }

// double duration(chrono::steady_clock::time_point timestamp, chrono::steady_clock::time_point begin)
// {
//     return double(chrono::duration_cast<chrono::milliseconds>(timestamp - begin).count()/1000.0);
// }

// chrono::microseconds time_transform(double time)
// {
//     long long int t = (long long int)(time*1000*1000);
//     return chrono::microseconds(t);
// }

void Mat_time::copyTo(Mat_time &frame)
{
    Mat::copyTo(frame);
    frame.produced_time = produced_time;
    frame.done_time = done_time;
    frame.process_time = process_time;
    frame.gyro_pose = gyro_pose;
}