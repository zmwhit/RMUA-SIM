#include "visualization/visualization.h"

void Visualization::drawOneCar(int id, geometry_msgs::PoseStamped& pose, double t,
                double length, double width, double height, double alpha, color c) {
    std::vector<std::vector<double>> list_rectangle = {{length, width}, {-length, width}, 
                                                            {-length, -width}, {length, -width}};
    std::vector<std::vector<double>> list_triangle = {{0, width/2}, {0, -width/2}, {length/2, 0}}; 

    visualization_msgs::Marker msg;     
    msg.id = id;
    msg.action = visualization_msgs::Marker::ADD;
    // msg.lifetime = ros::Duration(2.0);
    msg.pose = pose.pose;
    msg.pose.position.z = t;
    msg.header.frame_id = frame;
    msg.ns = "TRIANGLE_LIST";
    msg.type = visualization_msgs::Marker::TRIANGLE_LIST;

    geometry_msgs::Point point;
    for(size_t i = 0; i < 3; ++i) {
        point.x = list_triangle[i][0];
        point.y = list_triangle[i][1];
        point.z = height + 0.001;
        msg.points.emplace_back(point);
    }
    msg.color = getColor(color::YELLOW, 0.5);
    msg.scale.x = msg.scale.y = msg.scale.z = 1.0;
    Add(msg);

    msg.id = id + 1;
    msg.ns = "CUBE";
    msg.type = visualization_msgs::Marker::CUBE;
    msg.pose.position.z += height/2;
    msg.points.clear();
    msg.scale.x = length;
    msg.scale.y = width;
    msg.scale.z = height;
    msg.color = getColor(c, alpha);   
    Add(msg);  
}
void Visualization::drawCarOnTrajectory(int id, std::vector<geometry_msgs::PoseStamped>& poses, std::vector<double> times, 
                                        double length, double width, double height, double min_alpha, color c) {   
    double al = min_alpha;
    double dal = (1.0-min_alpha)/poses.size();
    for(size_t i = 0; i < poses.size(); ++i) {
        drawOneCar(id + 2*i, poses[i], times[i], length, width, height, al, c);         
        // al -= dal;
    }
}
std_msgs::ColorRGBA Visualization::getColor(int num, double alpha) {
    std_msgs::ColorRGBA result;
    switch(num) {
        case color::RED://RED
            result.r = 0.8;
            result.g = 0.1;
            result.b = 0.1;
            result.a = alpha;
            break;
        case color::GREEN://GREEN
            result.r = 0.1;
            result.g = 0.8;
            result.b = 0.1;
            result.a = alpha;
            break;
        case color::BLUE://BLUE
            result.r = 0.1;
            result.g = 0.1;
            result.b = 0.8;
            result.a = alpha;
            break;
        case color::GREY://GREY
            result.r = 0.9;
            result.g = 0.9;
            result.b = 0.9;
            result.a = alpha;
            break;
        case color::DARK_GREY://DARK_GREY
            result.r = 0.6;
            result.g = 0.6;
            result.b = 0.6;
            result.a = alpha;
            break;
        case color::WHITE://WHITE
            result.r = 1.0;
            result.g = 1.0;
            result.b = 1.0;
            result.a = alpha;
            break;
        case color::ORANGE://ORANGE
            result.r = 1.0;
            result.g = 0.5;
            result.b = 0.0;
            result.a = alpha;
            break;
        case color::BLACK://BLACK
            result.r = 0.0;
            result.g = 0.0;
            result.b = 0.0;
            result.a = alpha;
            break;
        case color::YELLOW://YELLOW
            result.r = 1.0;
            result.g = 1.0;
            result.b = 0.0;
            result.a = alpha;
            break;
        case color::PINK://PINK
            result.r = 1.0;
            result.g = 0.4;
            result.b = 1;
            result.a = alpha;
            break;
        case color::LIME_GREEN://LIME_GREEN
            result.r = 0.6;
            result.g = 1.0;
            result.b = 0.2;
            result.a = alpha;
            break;
        case color::PURPLE://PURPLE
            result.r = 0.597;
            result.g = 0.0;
            result.b = 0.597;
            result.a = alpha;
            break;
        case color::CYAN://CYAN
            result.r = 0.0;
            result.g = 1.0;
            result.b = 1.0;
            result.a = alpha;
            break;
        case color::MAGENTA://MAGENTA
            result.r = 1.0;
            result.g = 0.0;
            result.b = 1.0;
            result.a = alpha;
            break;
    }
    return result;
}
void Visualization::drawLine(int id, std::vector<geometry_msgs::PoseStamped>& poses, double height, double size, color c, double alpha) {
    if (poses.empty()) return;
    visualization_msgs::Marker msg; 
    msg.id = id;
    msg.header.seq = 0;
    msg.header.frame_id = frame;
    msg.header.stamp = ros::Time::now();
    msg.action = visualization_msgs::Marker::MODIFY;
    msg.ns = "LINE_STRIP";
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.pose.orientation.w = 1.0;
    geometry_msgs::Point point;
    for(size_t i = 0; i < poses.size(); ++i) {
        point.x = poses[i].pose.position.x;
        point.y = poses[i].pose.position.y;
        point.z = poses[i].pose.position.z + height;
        msg.points.emplace_back(point);
    }
    msg.scale.x = size;
    msg.scale.y = msg.scale.z = 0;
    msg.color = getColor(c, alpha);
    Add(msg);
}
void Visualization::drawBalls(int id, std::vector<geometry_msgs::PoseStamped>& poses, double size, color c, double alpha) {
    visualization_msgs::Marker msg; 
    msg.id = id;
    msg.header.frame_id = frame;
    msg.header.stamp = ros::Time::now();
    msg.action = visualization_msgs::Marker::MODIFY;
    msg.ns = "SPHERE_LIST";
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    msg.pose.orientation.w = 1.0;
    

    geometry_msgs::Point point;
    for(size_t i = 0; i < poses.size(); ++i) {
        point.x = poses[i].pose.position.x;
        point.y = poses[i].pose.position.y;
        point.z = poses[i].pose.position.z;
        msg.points.emplace_back(point);
    }
    msg.scale.x = msg.scale.y = msg.scale.z = size;
    msg.color = getColor(c, alpha);
    Add(msg);
}
void Visualization::drawRoundsOnLine(int id, std::vector<geometry_msgs::PoseStamped>& poses, double size, color c, double alpha) {
    drawBalls(id, poses, size*1.5, color::DARK_GREY, alpha);    
    drawLine(id + 1, poses, 0, size, c, alpha);                 
}
void Visualization::drawPolygon(int id, geometry_msgs::Polygon& polygon, double height, double size, color c, double alpha) {
    std::vector<geometry_msgs::PoseStamped> poses;
    std::vector<geometry_msgs::Point32> points = polygon.points;
    points.emplace_back(polygon.points.front());
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.w  = 1.0;
    std::for_each(points.begin(), points.end(), [&](geometry_msgs::Point32& point){
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        poses.emplace_back(pose);
    });
    drawLine(id, poses, height, size, c, alpha);
}
void Visualization::drawPolygon(int id, std::vector<geometry_msgs::PoseStamped>& poses, double height, double size, color c, double alpha) {
    poses.emplace_back(poses.front());
    drawLine(id, poses, height, size, c, alpha);
}
void Visualization::drawRoundsOnPolygon(int id, std::vector<geometry_msgs::PoseStamped>& poses, double height, double size, color c, double alpha) {
    poses.emplace_back(poses.front());
    drawBalls(id, poses, size*1.5, color::DARK_GREY, alpha);    
    drawLine(id + 1, poses, height, size, c, alpha); 
}
void Visualization::Init(ros::NodeHandle& node, std::string topic) {
    vis_pub = node.advertise<visualization_msgs::MarkerArray>(topic, 1);
}
void Visualization::Trigger() {
    mutex.lock();
    vis_pub.publish(arr);
    arr.markers.clear();
    mutex.unlock();
}    
void Visualization::Clear() {
    mutex.lock();
    arr.markers.clear();
    visualization_msgs::Marker msg;
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.ns = "clear";
    arr.markers.emplace_back(msg);
    vis_pub.publish(arr);
    arr.markers.clear();
    mutex.unlock();
}
void Visualization::Add(visualization_msgs::Marker& msg) {
    mutex.lock();
    arr.markers.emplace_back(msg);
    mutex.unlock();
}

