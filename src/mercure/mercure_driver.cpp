
/*-------------------------------------------------------------
brief  :  Mercure Camera driver
author :  XDU Robomaster, skystarry
date   :  2019.05.29
---------------------------------------------------------------*/
#include "mercure_driver.h"

namespace camera
{
    MercureDriver::MercureDriver() : status_(GX_STATUS_SUCCESS),
                                     device_(nullptr),
                                     pFrameBuffer_(nullptr)
    {
        init_sdk();
        LoadParam();
        rgbImagebuf_ = new uint8_t[ACQ_FRAME_HEIGHT * ACQ_FRAME_WIDTH * 3];

        status_ = GXStreamOn(device_);

        if (status_ != GX_STATUS_SUCCESS)
        {
            if (rgbImagebuf_ != nullptr)
            {
                delete[] rgbImagebuf_;
                rgbImagebuf_ = nullptr;
            }
            GXCloseDevice(device_);
            device_ = nullptr;
            GXCloseLib();
        }
    }

    void MercureDriver::resetParam(const std::string & file_name)
    {
        status_ = GXStreamOff(device_);
        status_ = GXCloseDevice(device_);
        status_ = init_sdk();
        LoadParam(file_name);
        status_ = GXStreamOn(device_);
    }

    GX_STATUS MercureDriver::init_sdk()
    {
        uint32_t ui32DeviceNum = 0;

        status_ = GXInitLib();
        if (status_ != GX_STATUS_SUCCESS)
        {
            return status_;
        }

        status_ = GXUpdateDeviceList(&ui32DeviceNum, 1000);
        if (status_ != GX_STATUS_SUCCESS)
        {
            GXCloseLib();
            return status_;
        }

        if (ui32DeviceNum <= 0)
        {
            GXCloseLib();
            return status_;
        }

        status_ = GXOpenDeviceByIndex(1, &device_);

        if (status_ != GX_STATUS_SUCCESS)
        {
            GXCloseLib();
            return status_;
        }
        GetVision();

        status_ = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        status_ = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);

        status_ = GXSetAcqusitionBufferNumber(device_, ACQ_BUFFER_NUM);
        status_ = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        status_ = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);

        // support
        status_ = GXSetInt(device_, GX_INT_WIDTH, ACQ_FRAME_WIDTH);
        status_ = GXSetInt(device_, GX_INT_HEIGHT, ACQ_FRAME_HEIGHT);
        status_ = GXSetInt(device_, GX_INT_OFFSET_X, (1920 - ACQ_FRAME_WIDTH) / 2);
        status_ = GXSetInt(device_, GX_INT_OFFSET_Y, (1200 - ACQ_FRAME_HEIGHT) / 2);

        return status_;
    }

    void MercureDriver::LoadParam(const std::string & file_name)
    {
        cv::FileStorage fs(file_name, cv::FileStorage::READ);
        if (!fs.isOpened())
            cout << "WRONG" << endl;

        fs["exposure_auto"] >> param_.exp_auto_;
        fs["exposure_time"] >> param_.exp_time_;
        if (param_.exp_auto_)
        {
            status_ = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);

            status_ = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, 5000);
            status_ = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, 20000);
        }
        else
        {
            status_ = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);

            status_ = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, param_.exp_time_ - 200);
            status_ = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, param_.exp_time_ + 200);
            status_ = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, param_.exp_time_);
        }
        fs["w_auto"] >> param_.w_auto_;
        fs["w_red"] >> param_.w_red_;
        fs["w_green"] >> param_.w_green_;
        fs["w_blue"] >> param_.w_blue_;
        if (param_.w_auto_)
        {
            status_ = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        }
        else
        {
            status_ = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);

            status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
            status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, param_.w_red_);

            status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
            status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, param_.w_green_);

            status_ = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
            status_ = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, param_.w_blue_);
        }

        fs["gain_auto"] >> param_.gain_auto_;
        fs["gain"] >> param_.gain_;

        if (param_.gain_auto_)
        {
            status_ = GXSetEnum(device_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
            status_ = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MIN, 0);
            status_ = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MAX, 10);
        }
        else
        {
            status_ = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);

            status_ = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MIN, 0);
            status_ = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MAX, 10);
            status_ = GXSetFloat(device_, GX_FLOAT_GAIN, param_.gain_);
        }
        fs.release();
    }

    void MercureDriver::GetVision()
    {
        size_t nSize = 0;
        GXGetStringLength(device_, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
        char *pszVendorName = new char[nSize];
        GXGetString(device_, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);

        delete[] pszVendorName;
        pszVendorName = NULL;

        GXGetStringLength(device_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
        char *pszModelName = new char[nSize];
        GXGetString(device_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);

        delete[] pszModelName;
        pszModelName = NULL;

        GXGetStringLength(device_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
        char *pszSerialNumber = new char[nSize];
        GXGetString(device_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);

        delete[] pszSerialNumber;
        pszSerialNumber = NULL;

        GXGetStringLength(device_, GX_STRING_DEVICE_VERSION, &nSize);
        char *pszDeviceVersion = new char[nSize];
        GXGetString(device_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);

        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;
    }

    // 可以记录开始时间以及结束时间
    void MercureDriver::operator>>(cv::Mat &Image)
    {

        GXDQBuf(device_, &pFrameBuffer_, 1000);

        try
        {
            DxRaw8toRGB24(pFrameBuffer_->pImgBuf,
                          rgbImagebuf_,
                          pFrameBuffer_->nWidth,
                          pFrameBuffer_->nHeight,
                          RAW2RGB_NEIGHBOUR,
                          DX_PIXEL_COLOR_FILTER(4), // DX_PIXEL_COLOR_FILTER(colorfilter_),
                          false);

            memcpy(Image.data, rgbImagebuf_, pFrameBuffer_->nHeight * pFrameBuffer_->nWidth * 3);

            GXQBuf(device_, pFrameBuffer_);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            Image = cv::Mat();
        }
    }

    void MercureDriver::operator>>(Mat_time &Image)
    {

        GXDQBuf(device_, &pFrameBuffer_, 1000);

        try
        {
            DxRaw8toRGB24(pFrameBuffer_->pImgBuf,
                          rgbImagebuf_,
                          pFrameBuffer_->nWidth,
                          pFrameBuffer_->nHeight,
                          RAW2RGB_NEIGHBOUR,
                          DX_PIXEL_COLOR_FILTER(4), // DX_PIXEL_COLOR_FILTER(colorfilter_),
                          false);

            memcpy(Image.data, rgbImagebuf_, pFrameBuffer_->nHeight * pFrameBuffer_->nWidth * 3);

            GXQBuf(device_, pFrameBuffer_);
            Image.setProducedTime();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            Image = Mat_time();
        }
    }

    MercureDriver::~MercureDriver()
    {
        status_ = GXStreamOff(device_);
        if (status_ != GX_STATUS_SUCCESS)
        {
            delete[] rgbImagebuf_;
            rgbImagebuf_ = nullptr;
            GXCloseDevice(device_);
            device_ = nullptr;
            GXCloseLib();
        }

        delete[] rgbImagebuf_;
        rgbImagebuf_ = nullptr;

        status_ = GXCloseDevice(device_);
        if (status_ != GX_STATUS_SUCCESS)
        {
            device_ = nullptr;
            GXCloseLib();
        }
        status_ = GXCloseLib();
        if (status_ != GX_STATUS_SUCCESS)
        {
        }
    }

} // namespace camera