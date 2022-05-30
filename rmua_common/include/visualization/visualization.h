#pragma once
#include <thread>
#include <mutex>

#include "ros_headers.h"
#include "helper.h"
enum color {
    RED,
    GREEN,
    BLUE,
    PURPLE,
    ORANGE,
    YELLOW,
    MAGENTA,
    PINK,
    CYAN,
    LIME_GREEN,
    WHITE,
    GREY,
    DARK_GREY,
    BLACK
};
class Visualization {
public:
    Visualization() = default;
    ~Visualization() = default;
    static std_msgs::ColorRGBA getColor(int num, double alpha);
    void drawOneCar(int id, geometry_msgs::PoseStamped& pose, double t,
                    double length, double width, double height, double alpha, color c);
    void drawCarOnTrajectory(int id, std::vector<geometry_msgs::PoseStamped>& poses, std::vector<double> times, 
                            double length, double width, double height, double min_alpha, color c);
    void drawLine(int id, std::vector<geometry_msgs::PoseStamped>& poses, double height, double size, color c, double alpha);
    void drawBalls(int id, std::vector<geometry_msgs::PoseStamped>& poses, double size, color c, double alpha);
    void drawRoundsOnLine(int id, std::vector<geometry_msgs::PoseStamped>& poses, double size, color c, double alpha);
    void drawRoundsOnPolygon(int id, std::vector<geometry_msgs::PoseStamped>& poses, double height, double size, color c, double alpha);
    void drawPolygon(int id, geometry_msgs::Polygon& polygon, double height, double size, color c, double alpha);
    void drawPolygon(int id, std::vector<geometry_msgs::PoseStamped>& poses, double height, double size, color c, double alpha);
    void Init(ros::NodeHandle& node, std::string topic);
    void Trigger();
    void Clear();
    void Add(visualization_msgs::Marker& msg);
private:
    std::string frame = "map";
    std::mutex mutex;

    ros::Publisher vis_pub;
    visualization_msgs::MarkerArray arr;
};

