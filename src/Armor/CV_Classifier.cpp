#include "CV_Classifier.h"

CV_Classifier::CV_Classifier(string path, Size _input)
{
    net = dnn::readNetFromONNX(path);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA); 
    input_size = _input;
    Mat _temp = Mat::zeros(Size(32,32), CV_32FC1);
    _temp = dnn::blobFromImage(_temp);
    net.setInput(_temp);
    net.forward();
}

double CV_Classifier::predict(Mat frame, int &result)
{
    Mat pre;
    cvtColor(frame, pre, COLOR_BGR2GRAY);

    Mat score, soft_score;
    pre = dnn::blobFromImage(pre, 1.0/255.0, input_size);
    net.setInput(pre);
    score = net.forward();
    
    softmax(score, soft_score);
    Point maxclass;
    double maxValue;
    minMaxLoc(score, NULL, &maxValue, NULL, &maxclass);

    result = (int)maxclass.x;
    return maxValue;
}

int CV_Classifier::softmax(const Mat& src, Mat& dst)
{
    float max = 0.0;
	float sum = 0.0;

	max = *max_element(src.begin<float>(), src.end<float>());
	cv::exp((src - max), dst);
	sum = cv::sum(dst)[0];
	dst /= sum;
	
	return 0;
}