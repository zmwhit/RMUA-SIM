#include "visualization/axis_picture.h"
using namespace std;
void AxisPicture::Init(int type, const string str, const string xl, const string yl, 
                       double x, double y, double s) {
    if (init) return;
    name = str;
    xlabel = xl;
    ylabel = yl;
    scale = s;
    if (type == 1) {//一四象限
        offset_x = static_cast<int>(0.1*x*scale);
        offset_y = static_cast<int>(y*scale);
        width = static_cast<int>(x*scale + 2*offset_x);
        height = static_cast<int>(2*y*scale);
    } else if (type == 2) {//一象限
        offset_x = static_cast<int>(0.1*x*scale);
        offset_y = static_cast<int>(0.95*y*scale);
        width = static_cast<int>(x*scale + 2*offset_x);
        height = static_cast<int>(y*scale);
    }
    line_size = scale/100;
    point_size = scale/100;
    margin = scale/100; 
    img = cv::Mat(height, width, CV_8UC3, CV_COLOR_WHITE);
    Reset();
    trigger = true;
    init = true;
    // std::thread(&AxisPicture::Trigger, this).detach();
}
void AxisPicture::Trigger() {
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
    while(trigger) {
        // cv::imshow(name, img);
        Save(Helper::default_pic_path, "");
        // cv::waitKey(0);  
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
int AxisPicture::check_x(const int x) {
    return std::max(std::min(x, width), 0);
}
int AxisPicture::check_y(const int y) {
    return std::max(std::min(y, height), 0);
}
void AxisPicture::DrawCar(double x_, double y_, double a, double w, double l, cv::Scalar color) {
    if (!init) return;
    std::vector<std::vector<double>> rectangle = {{l/2, w/2}, {l/2,-w/2}, {-l/2, -w/2}, {-l/2, w/2}};
    std::vector<std::vector<int>> points(5, std::vector<int>(2));
    for (int i = 0; i < rectangle.size(); ++i) {
        int x = (x_ + rectangle[i][0]*std::cos(a) - rectangle[i][1]*std::sin(a))*scale + offset_x;
        int y = -(y_ + rectangle[i][0]*std::sin(a) + rectangle[i][1]*std::cos(a))*scale + offset_y;
        points[i][0] = x;
        points[i][1] = y;
    }
    points.back() = points[0];
    mutex.lock();
    for (int i = 0; i < 4; ++i) {
        int x1 = check_x(points[i][0]);
        int y1 = check_y(points[i][1]);
        int x2 = check_x(points[i+1][0]);
        int y2 = check_y(points[i+1][1]);       
        cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, 2, cv::LINE_8, 0);
    }
    mutex.unlock();
}
void AxisPicture::DrawPoint(double x_, double y_, cv::Scalar color) {
    if (!init) return;
    int x = check_x(x_*scale + offset_x);
    int y = check_y(-y_*scale + offset_y);
    mutex.lock();
    cv::circle(img, cv::Point(x, y), point_size, color, point_size, cv::LINE_8, 0);
    mutex.unlock();
}
void AxisPicture::DrawLine(double x1_, double y1_, double x2_, double y2_, cv::Scalar color) {
    if (!init) return;
    int x1 = check_x(x1_*scale + offset_x);
    int x2 = check_x(x2_*scale + offset_x);
    int y1 = check_y(-y1_*scale + offset_y);
    int y2 = check_y(-y2_*scale + offset_y);
    mutex.lock();
    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, line_size, cv::LINE_8, 0);
    mutex.unlock();
}
void AxisPicture::Save(const string path, const string suffix) {
    cv::imwrite(path +"/" + name + suffix + ".jpg", img);
}
void AxisPicture::Reset() {
    img = cv::Mat(height, width, CV_8UC3, CV_COLOR_WHITE);
    double x1, x2, y1, y2;
    x1 = 0; 
    x2 = width - margin;
    y1 = y2 = offset_y;
    cv::arrowedLine(img, cv::Point(x1, y1), cv::Point(x2, y2), CV_COLOR_DARKRED, line_size, cv::LINE_8, 0, 0.01);
    x1 = x2 = offset_x; 
    y1 = height;
    y2 = margin;
    cv::arrowedLine(img, cv::Point(x1, y1), cv::Point(x2, y2), CV_COLOR_DARKRED, line_size, cv::LINE_8, 0, 0.01);
    x1 = width*0.02;
    y1 = height*0.05;
    x2 = width*0.95;
    y2 = offset_y+5*point_size;
    cv::putText(img, ylabel, cv::Point(x1, y1), cv::FONT_HERSHEY_COMPLEX, 0.5, CV_COLOR_BLACK, 1.0);
    cv::putText(img, xlabel, cv::Point(x2, y2), cv::FONT_HERSHEY_COMPLEX, 0.5, CV_COLOR_BLACK, 1.0);
}