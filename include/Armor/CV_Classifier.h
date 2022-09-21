#ifndef CV_CLASSIFIER_H
#define CV_CLASSIFIER_H

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include "const.h"

using namespace cv;
using namespace std;

class CV_Classifier
{
private:
    dnn::Net net;       // 当前使用pytorch-> onnx
    Size input_size;

    int softmax(const Mat&, Mat&);
public:
    /**
     *  @brief  对装甲板数字进行识别分类
     *  @param  frame   需要识别的图片
     *  @param  result  识别结果
     *  @return 返回置信度
     */
    double predict(Mat frame, int &result);

    CV_Classifier() = default;
    CV_Classifier(string path, Size _input = Size(32,32));
    
    ~CV_Classifier() = default;

};

#endif