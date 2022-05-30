# pragma once

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "helper.h"
#define CV_COLOR_RED cv::Scalar(0,0,255)       //纯红
#define CV_COLOR_GREEN cv::Scalar(0,255,0)        //纯绿
#define CV_COLOR_BLUE cv::Scalar(255,0,0)       //纯蓝
#define CV_COLOR_BLACK cv::Scalar(0,0,0) //黑色
#define CV_COLOR_WHITE cv::Scalar(255,255,255) //白色
#define CV_COLOR_DARKGRAY cv::Scalar(150,150,150) //深灰色
#define CV_COLOR_GRAY cv::Scalar(220,220,220) //浅灰色
#define CV_COLOR_DARKRED cv::Scalar(0,0,210) //深红色
#define CV_COLOR_ORANGERED cv::Scalar(0,69,255)     //橙红色
#define CV_COLOR_CHOCOLATE cv::Scalar(30,105,210) //巧克力
#define CV_COLOR_GOLD cv::Scalar(10,215,255) //金色
#define CV_COLOR_YELLOW cv::Scalar(0,255,255)     //纯黄色
#define CV_COLOR_OLIVE cv::Scalar(0,128,128) //橄榄色
#define CV_COLOR_LIGHTGREEN cv::Scalar(144,238,144) //浅绿色
#define CV_COLOR_DARKCYAN cv::Scalar(139,139,0)     //深青色
#define CV_COLOR_SKYBLUE cv::Scalar(230,216,173) //天蓝色
#define CV_COLOR_INDIGO cv::Scalar(130,0,75) //藏青色
#define CV_COLOR_PURPLE cv::Scalar(128,0,128)     //紫色
#define CV_COLOR_PINK cv::Scalar(203,192,255) //粉色
#define CV_COLOR_DEEPPINK cv::Scalar(147,20,255) //深粉色
#define CV_COLOR_VIOLET cv::Scalar(238,130,238)     //紫罗兰
class AxisPicture {
public:

    AxisPicture() = default;
    ~AxisPicture() = default;
    void Init(int type, const std::string str, const std::string xl, const std::string yl, double x, double y, double s);
    void Save(const std::string path, const std::string suffix);
    void stop() {trigger = false;}
    void Reset();
    void Trigger();
    void DrawPoint(double x_, double y_, cv::Scalar color);
    void DrawLine(double x1_, double y1_, double x2_, double y2_, cv::Scalar color);
    void DrawCar(double x_, double y_, double theta, double w, double l, cv::Scalar color);
private:
    int check_x(const int x);
    int check_y(const int y);
    std::mutex mutex;

    cv::Mat img;
    cv::Mat origin;
    std::string name, ylabel, xlabel;
    double scale;
    int width;
    int height;
    int offset_x, offset_y;
    int margin;
    double line_size;
    double point_size;

    bool trigger = false;
    bool init = false;
};