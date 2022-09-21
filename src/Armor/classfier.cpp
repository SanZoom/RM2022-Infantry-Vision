//
// Created by xinyang on 19-4-19.
//
// 对本文件的大致描述请看classifier.h

#include "classfier.h"
#include <cstdio>
#include <iostream>

vector<vector<MatrixXd>> Classifier::load_conv_w(const string &file)
{
    vector<vector<MatrixXd>> result;
    FILE *fp = fopen(file.data(), "r");
    // FILE* fp;
    // fopen(&fp, file.data(), "r");
    int errNum = 0;
    if (fp == nullptr)
    {
        // errNum = errno;
        // cout << errno << endl;
        cout << "conv_w open fail!" << endl;
        //LOGE("%s ", file.data());
        state = false;
        return result;
    }
    int channel_in, channel_out, row, col;
    fscanf(fp, "%d %d %d %d", &channel_in, &channel_out, &row, &col);
    for (int o = 0; o < channel_in; o++)
    {
        vector<MatrixXd> sub;
        for (int i = 0; i < channel_out; i++)
        {
            MatrixXd f(row, col);
            for (int r = 0; r < row; r++)
            {
                for (int c = 0; c < col; c++)
                {
                    fscanf(fp, "%lf", &f(r, c));
                }
            }
            sub.emplace_back(f);
        }
        result.emplace_back(sub);
    }

    return result;
}

vector<double> Classifier::load_conv_b(const string &file)
{
    vector<double> result;
    FILE *fp = fopen(file.data(), "r");
    // FILE* fp;
    // fopen_s(&fp, file.data(), "r");
    if (fp == nullptr)
    {
        cout << "conv_b open fail!" << endl;

        //LOGE("%s open fail!", file.data());
        state = false;
        return result;
    }
    int len;
    fscanf(fp, "%d", &len);
    for (int i = 0; i < len; i++)
    {
        double v;
        fscanf(fp, "%lf", &v);
        result.emplace_back(v);
    }
    return result;
}

MatrixXd Classifier::load_fc_w(const string &file)
{
    FILE *fp = fopen(file.data(), "r");
    if (fp == nullptr)
    {
        cout << "fc_w open fail!" << endl;
        state = false;
        return MatrixXd::Zero(1, 1);
    }
    int row, col;


    fscanf(fp, "%d %d", &row, &col);
    MatrixXd mat(row, col);

    for (int r = 0; r < row; r++)
    {
        for (int c = 0; c < col; c++)
        {
            fscanf(fp, "%lf", &mat(r, c));
        }
    }

    return mat;
}

VectorXd Classifier::load_fc_b(const string &file)
{
    FILE *fp = fopen(file.data(), "r");
    if (fp == nullptr)
    {
        cout << "fc_b open fail!" << endl;
        // LOGE("%s open fail!", file.data());
        state = false;
        return VectorXd::Zero(1, 1);
    }
    int row;
    fscanf(fp, "%d", &row);
    VectorXd vec(row, 1);
    for (int r = 0; r < row; r++)
    {
        fscanf(fp, "%lf", &vec(r));
    }
    return vec;
}

MatrixXd Classifier::softmax(const MatrixXd &input)
{
    MatrixXd tmp = input.array() - input.maxCoeff();
    return (tmp.array().exp() / tmp.array().exp().sum()).log();
}

vector<vector<MatrixXd>> Classifier::max_pool(const vector<vector<MatrixXd>> &input, int size)
{
    vector<vector<MatrixXd>> output;
    for (int sample = 0; sample < input.size(); sample++)
    {
        vector<MatrixXd> sub;
        for (int channel = 0; channel < input[0].size(); channel++)
        {
            MatrixXd tmp(input[0][0].rows() / size, input[0][0].cols() / size);
            for (int row = 0; row < input[0][0].rows() / size; row++)
            {
                for (int col = 0; col < input[0][0].cols() / size; col++)
                {
                    double max = 0;
                    for (int x = 0; x < size; x++)
                    {
                        for (int y = 0; y < size; y++)
                        {
                            if (max < input[sample][channel](row * size + x, col * size + y))
                            {
                                max = input[sample][channel](row * size + x, col * size + y);
                            }
                        }
                    }
                    tmp(row, col) = max;
                }
            }
            sub.emplace_back(tmp);
        }
        output.emplace_back(sub);
    }
    return output;
}

vector<vector<MatrixXd>> Classifier::mean_pool(const vector<vector<MatrixXd>> &input, int size)
{
    vector<vector<MatrixXd>> output;
    for (int sample = 0; sample < input.size(); sample++)
    {
        vector<MatrixXd> sub;
        for (int channel = 0; channel < input[0].size(); channel++)
        {
            MatrixXd tmp(input[0][0].rows() / size, input[0][0].cols() / size);
            for (int row = 0; row < input[0][0].rows() / size; row++)
            {
                for (int col = 0; col < input[0][0].cols() / size; col++)
                {
                    double val = 0;
                    for (int x = 0; x < size; x++)
                    {
                        for (int y = 0; y < size; y++)
                        {
                            val += input[sample][channel](row * size + x, col * size + y);
                        }
                    }
                    tmp(row, col) = val / size / size;
                }
            }
            sub.emplace_back(tmp);
        }
        output.emplace_back(sub);
    }
    return output;
}

vector<vector<MatrixXd>> Classifier::apply_bias(const vector<vector<MatrixXd>> &input, const vector<double> &bias)
{
    assert(input[0].size() == bias.size()); //断言
    vector<vector<MatrixXd>> result;
    for (int samples = 0; samples < input.size(); samples++)
    {
        vector<MatrixXd> sub;
        for (int channels = 0; channels < input[0].size(); channels++)
        {
            MatrixXd mat = input[samples][channels].array() + bias[channels];
            sub.emplace_back(mat);
        }
        result.emplace_back(sub);
    }
    return result;
}

MatrixXd Classifier::relu(const MatrixXd &input)
{
    return input.unaryExpr([](double val) {
        return (val > 0) ? (val) : (0);
    });
}

MatrixXd Classifier::leaky_relu(const MatrixXd &input, float alpha)
{
    return input.unaryExpr([&](double val) {
        return (val > 0) ? (val) : (alpha * val);
    });
}

vector<vector<MatrixXd>> Classifier::relu(const vector<vector<MatrixXd>> &input)
{
    vector<vector<MatrixXd>> result;
    for (int samples = 0; samples < input.size(); samples++)
    {
        vector<MatrixXd> sub;
        for (int channels = 0; channels < input[0].size(); channels++)
        {
            sub.emplace_back(relu(input[samples][channels]));
        }
        result.emplace_back(sub);
    }
    return result;
}

vector<vector<MatrixXd>> Classifier::leaky_relu(const vector<vector<MatrixXd>> &input, float alpha)
{
    vector<vector<MatrixXd>> result;
    for (int samples = 0; samples < input.size(); samples++)
    {
        vector<MatrixXd> sub;
        for (int channels = 0; channels < input[0].size(); channels++)
        {
            sub.emplace_back(leaky_relu(input[samples][channels], alpha));
        }
        result.emplace_back(sub);
    }
    return result;
}

vector<vector<MatrixXd>> Classifier::pand(const vector<vector<MatrixXd>> &input, int val)
{
    vector<vector<MatrixXd>> result;
    for (int sample = 0; sample < input.size(); sample++)
    {
        vector<MatrixXd> sub;
        for (int channels = 0; channels < input[0].size(); channels++)
        {
            MatrixXd mat = MatrixXd::Zero(input[0][0].rows() + 2 * val, input[0][0].cols() + 2 * val);
            mat.block(val, val, input[0][0].rows(), input[0][0].cols()) = input[sample][channels];
            sub.emplace_back(mat);
        }
        result.emplace_back(sub);
    }
    return result;
}

MatrixXd Classifier::conv(const MatrixXd &filter, const MatrixXd &input)
{
    /*
    int result_rows = input.rows() - filter.rows() + 1;
    int result_cols = input.cols() - filter.cols() + 1;
    */
    int result_rows = (input.rows() - filter.rows()) + 1;
    int result_cols = (input.cols() - filter.cols()) + 1;
    MatrixXd result(result_rows, result_cols);
    for (int row = 0; row < result_rows; row++)
    {
        for (int col = 0; col < result_cols; col++)
        {
            double val = 0;
            for (int x = 0; x < filter.cols(); x++)
            {
                for (int y = 0; y < filter.cols(); y++)
                {
                    val += input(row + x, col + y) * filter(x, y);
                }
            }
            result(row, col) = val;
            //            result(row, col) = (input.block(row, col, size, size).array() * input.array()).sum();
        }
    }
    return result;
}

vector<vector<MatrixXd>> Classifier::conv2(const vector<vector<MatrixXd>> &filter, const vector<vector<MatrixXd>> &input)
{
    if (filter.size() != input[0].size())
    {
        cout << "shape du not match, which is filter.size=" << filter.size() << "input[0].size()=" << input[0].size() << endl;
        //  LOGE("shape du not match, which is filter.size=%d, input[0].size()=%d", filter.size(), input[0].size());
        exit(-1);
    }
    vector<vector<MatrixXd>> result;
    int result_rows = input[0][0].rows() - filter[0][0].rows() + 1;
    int result_cols = input[0][0].cols() - filter[0][0].cols() + 1;
    for (int col = 0; col < input.size(); col++)
    {
        vector<MatrixXd> sub;
        for (int row = 0; row < filter[0].size(); row++)
        {
            MatrixXd val = MatrixXd::Zero(result_rows, result_cols);
            for (int x = 0; x < filter.size(); x++)
            {
                val += conv(filter[x][row], input[col][x]); //
            }
            sub.emplace_back(val);
        }
        result.emplace_back(sub);
    }
    return result;
}

MatrixXd Classifier::flatten(const vector<vector<MatrixXd>> &input)
{
    int cnt_1 = 0;
    int ones = input[0][0].rows() * input[0][0].cols();
    int channels = input[0].size();
    int samples = input.size();
    int row = input[0][0].rows();
    int col = input[0][0].cols();
    MatrixXd output(channels * ones, samples);

    for (int s = 0; s < samples; s++)
    {
        for (int i = 0; i < channels; i++)
        {
            for (int r = 0, cnt = 0; r < row; r++)
            {
                for (int c = 0; c < col; c++)
                {

                    output(cnt_1++, s) = input[s][i](r, c);
                    //cout << input[s][i](r, c) << " ";
                }
            }
        }
    }
    //cout << "s" << samples << endl;
    return output;
}

Classifier::Classifier(const string &folder) : state(true)
{
    conv1_w = load_conv_w(folder + "conv1_w");
    conv1_b = load_conv_b(folder + "conv1_b");
    conv2_w = load_conv_w(folder + "conv2_w");
    conv2_b = load_conv_b(folder + "conv2_b");
    fc1_w = load_fc_w(folder + "fc1_w");
    fc1_b = load_fc_b(folder + "fc1_b");
    fc2_w = load_fc_w(folder + "fc2_w");
    fc2_b = load_fc_b(folder + "fc2_b");
    if (state)
    {
        cout << "classifier Load para success!" << endl;
        // LOGM("");
    }
}

MatrixXd Classifier::calculate(const vector<vector<MatrixXd>> &input)
{
    vector<vector<MatrixXd>> conv1_result = relu(apply_bias(conv2(conv1_w, input), conv1_b));        //加上偏置值，然后对其使用ReLU激活函数
    vector<vector<MatrixXd>> pool1_result = max_pool(conv1_result, 2);                               //chi hua(jianji)均值池化：mean-pooling——选取指定区域内数值的平均值来代表整片区域
    vector<vector<MatrixXd>> conv2_result = relu(apply_bias(conv2(conv2_w, pool1_result), conv2_b)); //zai fu pian zhi,zai ji huo
    vector<vector<MatrixXd>> pool2_result = max_pool(conv2_result, 2);                               //zai chi hua
    
    MatrixXd flattened = flatten(pool2_result);                                                      // zhan cheng yi wei
    MatrixXd y1 = fc1_w * flattened;                                                                 //全连接层
    y1.colwise() += fc1_b;                                                                           //colwise()每一列
    MatrixXd fc1 = relu(y1);
    MatrixXd y2 = fc2_w * fc1;
    y2.colwise() += fc2_b;
    MatrixXd fc2 = softmax(y2);
    return fc2;
}

Classifier::operator bool() const
{
    return state;
}

int Classifier::operator()(const cv::Mat &image)
{
    MatrixXd gray;
    cv2eigen(image, gray);
    gray /= 255.0;
    vector<MatrixXd> sub = {gray}; //b,g,r
    vector<vector<MatrixXd>> in = {sub};
    MatrixXd result = calculate(in);
    MatrixXd::Index minRow, minCol;
    result.maxCoeff(&minRow, &minCol); //返回矩阵中最大值的位置
    if (result(minRow, minCol) > -10 )
    { ////0.50
        return minRow;
    }
    else
    {
        return -1;
    }
}