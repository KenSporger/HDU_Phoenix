#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include <typeinfo>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "camera.h"
#include "ArmorDector.h"
#include "general.h"
#include "ArmorDector.h"
#include "opencv_extend.h"
#include "serial.h"
#include "log.h"
#include "Buff.h"

#define IMAGE_CENTER_X 327
#define IMAGE_CENTER_Y 230
#define FOCUS_PIXAL 1269
#define PI (3.14159265459)
// 0 : armor 
// 1 : buff
#define DETECT_MODE 1

cv::Mat img = cv::Mat(480, 640, CV_8UC3, (0, 0, 0));
cv::VideoCapture video;
ArmorDetector Arm;
Serial serial;

static bool sendTarget(Serial &serial, float x, float y)
{
    static short x_tmp, y_tmp, z_tmp;
    uint8_t buff[10];

    union f_data {
        float temp;
        unsigned char fdata[4];
    } float_data_x, float_data_y;

    float_data_x.temp = x;
    float_data_y.temp = y;

    buff[0] = 's';
    buff[1] = static_cast<char>(float_data_x.fdata[0]);
    buff[2] = static_cast<char>(float_data_x.fdata[1]);
    buff[3] = static_cast<char>(float_data_x.fdata[2]);
    buff[4] = static_cast<char>(float_data_x.fdata[3]);
    buff[5] = static_cast<char>(float_data_y.fdata[0]);
    buff[6] = static_cast<char>(float_data_y.fdata[1]);
    buff[7] = static_cast<char>(float_data_y.fdata[2]);
    buff[8] = static_cast<char>(float_data_y.fdata[3]);
    buff[9] = 'e';

    return serial.WriteData(buff, sizeof(buff));
}

bool sendBoxPosition(cv::Point point)
{
    float dx = point.x - IMAGE_CENTER_X;
    float dy = point.y - IMAGE_CENTER_Y;
    float yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
    float pitch = atan(dy / FOCUS_PIXAL) * 180 / PI;
    DLOG_INFO << "  "
              << " yaw: " << yaw << " pitch " << pitch;
    return sendTarget(serial, yaw, pitch);
}

int main(int argc, char const *argv[])
{
    if(VIDEO_TYPE == 0)
        video.open("../video/buff.mp4");
    else if (VIDEO_TYPE == 1) 
        video.open("../video/wind.mp4");
    // bool err = serial.InitPort();
    GLogWrapper glog(argv[0]);
    // while (err)
    // {
    //     DLOG_WARNING << "can't open dev/usb";
    // }

    DLOG_INFO << "arm begin";
    Detect detect;
    Mat dst;
    int cnt = 0;
    double s1 = 0;
    double s2 = 0;
    while (1)
    {
        video >> img;
        if (img.empty())
        {
            cout << "can't read frame!" << endl;
            return -1;
        }
        cv::resize(img, img, cv::Size(640, 480));
        if (img.size().width != 640 || img.size().height != 480)
        {
            LOG_ERROR << "size error";
            continue;
        }

        if (DETECT_MODE == 0)
        {
            Arm.loadImg(img);
            Arm.setEnemyColor(BLUE);
            int find_flag = Arm.detect();

            if (find_flag != 0)
            {
                std::vector<cv::Point2f> Points = Arm.getArmorVertex();
                cv::Point aimPoint;
                aimPoint.x = aimPoint.y = 0;

                for (const auto &point : Points)
                {
                    aimPoint.x += point.x;
                    aimPoint.y += point.y;
                }
                aimPoint.x = aimPoint.x / 4;
                aimPoint.y = aimPoint.y / 4;

                sendBoxPosition(aimPoint);
            }
            else
            {
                DLOG_INFO << "can't find enemy";
            }
        }
        else if (DETECT_MODE == 1)
        {
                
            if (VIDEO_TYPE == 0 && cnt >= 300 && cnt % 50 ==0)
            {
                s1 = rand()%50;
                s2 = rand()%50;
            }
            else if (VIDEO_TYPE == 1 && cnt >= 100 && cnt % 50 ==0)
            {
                s1 = rand()%50;
                s2 = rand()%50;
            }
            Mat affineM(2, 3, CV_64FC1, Scalar(0));
            affineM.at<double>(0, 0) = 1;
            affineM.at<double>(0, 1) = 0;
            affineM.at<double>(0, 2) = s1;
            affineM.at<double>(1, 0) = 0;
            affineM.at<double>(1, 1) = 1;
            affineM.at<double>(1, 2) = s2;
            warpAffine(img, dst, affineM, img.size());
            cnt++;
            detect.detect_new(dst);
        }
        cv::imshow("src1", dst);
        waitKey(0);

    }
}
