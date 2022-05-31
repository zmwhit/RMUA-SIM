
#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include <yaml-cpp/yaml.h>
#include "ros_headers.h"
#include "math/polygon2d.h"
#include "helper.h"

#include <Eigen/Eigen>
#include <Eigen/Core>
nav_msgs::OccupancyGrid static_map, dilate_map, origin_map, dynamic_map;
std::vector<geometry_msgs::PoseStamped> dynamic_obs;
void PoseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg, int id) {
    dynamic_obs[id] = *msg;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "map_manager");
    ros::NodeHandle nh("~");
    ros::Publisher static_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/static_map", 1);
    ros::Publisher dilate_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/dilate_map", 1);
    ros::Publisher dynamic_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/dynamic_map", 1);
    ros::Publisher static_wall_pub = nh.advertise<visualization_msgs::MarkerArray>("/static_wall", 1);
    std::vector<ros::Subscriber> pose_sub_set_;
    for (int i = 0; i < 4; ++i) {
        std::string topic = "/robot_" + std::to_string(i) + "/position";
        ros::Subscriber pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(topic, 5, boost::bind(&PoseCallBack, _1, i));
        pose_sub_set_.emplace_back(pose_sub_);
    }
    dynamic_obs.resize(4, Helper::getIdentityPoseStamp());

    std::string path = ros::package::getPath("rmua_simulator")+"/map/map.yaml";
    YAML::Node data = YAML::LoadFile(path);
    static_map.header.frame_id = data["GridMap"]["frame_id"].as<std::string>();
    double res = data["GridMap"]["resolution"].as<double>();
    int map_length = std::ceil(data["GridMap"]["length"].as<double>()/res);
    int map_width = std::ceil(data["GridMap"]["width"].as<double>()/res);
    static_map.info.resolution = res;
    static_map.info.height = map_width;//y
    static_map.info.width = map_length;//x
    static_map.info.origin.orientation.w = 1.0;
    std::vector<math::Polygon2d> static_obs, dilate_obs;
    std::vector<double> obs_height;
    // int n = data["Polygons"].size();
    // for (int i = 0; i < n; ++i) {
    //     std::vector<math::Vec2d> points;
    //     int m = data["Polygons"][i]["polygon"].size();
    //     for (int j = 0; j < m; ++j) {
    //         double x = data["Polygons"][i]["polygon"][j]["x"].as<double>();
    //         double y = data["Polygons"][i]["polygon"][j]["y"].as<double>();
    //         points.emplace_back(x, y);
    //     }
    //     static_obs.emplace_back(points);
    // }
    int n = data["Rectangles"].size();
    for (int i = 0; i < n; ++i) {
        double x = data["Rectangles"][i]["rectangle"]["x"].as<double>();
        double y = data["Rectangles"][i]["rectangle"]["y"].as<double>();
        double a = data["Rectangles"][i]["rectangle"]["a"].as<double>()/180*M_PI;
        double l = data["Rectangles"][i]["rectangle"]["l"].as<double>();
        double w = data["Rectangles"][i]["rectangle"]["w"].as<double>();
        double h = data["Rectangles"][i]["rectangle"]["h"].as<double>();
        math::Vec2d center(x, y);
        std::vector<math::Vec2d> points{{l/2, w/2}, {-l/2, w/2}, {-l/2, -w/2}, {l/2, -w/2}};
        for (int j = 0; j < 4; ++j) {
            points[j] = points[j].rotate(a) + center;
        }
        static_obs.emplace_back(points);
        obs_height.emplace_back(h);
    }
    int obs_num = static_obs.size();
    double dilate_radius = data["GridMap"]["dilate_radius"].as<double>();
    for (int i = 0; i < obs_num; ++i) {
        dilate_obs.emplace_back(static_obs[i].ExpandEdgeByDistance(dilate_radius, res));
    }
    int cnt = 0;
    static_map.data.resize(static_map.info.height * static_map.info.width, 0);
    dilate_map = static_map;
    for (int i = 0; i < static_map.info.height; ++i) {
        for (int j = 0; j < static_map.info.width; ++j) {
            double x = (j + 0.5)*static_map.info.resolution;
            double y = (i + 0.5)*static_map.info.resolution;
            math::Vec2d point(x, y);
            for (int k = 0; k < obs_num; ++k) {
                if (static_obs[k].IsPointIn(point)) {
                    static_map.data[cnt] = (int8_t)100;
                    break;
                }
            }
            cnt ++;
        }
    }
    cnt = 0;
    for (int i = 0; i < dilate_map.info.height; ++i) {
        for (int j = 0; j < dilate_map.info.width; ++j) {
            double x = (j + 0.5)*dilate_map.info.resolution;
            double y = (i + 0.5)*dilate_map.info.resolution;
            math::Vec2d point(x, y);
            for (int k = 0; k < obs_num; ++k) {
                if (dilate_obs[k].IsPointIn(point)) {
                    dilate_map.data[cnt] = (int8_t)100;
                    break;
                }
            }
            cnt ++;
        }
    }
    origin_map = dilate_map;

    //only for visualization
    visualization_msgs::MarkerArray walls;
    visualization_msgs::Marker wall;
    wall.action = visualization_msgs::Marker::MODIFY;
    wall.header.frame_id = "map";
    wall.ns = "LINE_LIST";
    wall.type = visualization_msgs::Marker::LINE_LIST;
    wall.scale.x = static_map.info.resolution*1.1;
    wall.scale.y = static_map.info.resolution*1.1;
    wall.scale.z = static_map.info.resolution*1.1;
    wall.color.a = 1.0;
    wall.color.r = wall.color.g = wall.color.b = 0.3;
    wall.pose = static_map.info.origin;
    cnt = 0;
    geometry_msgs::Point p;
    int wall_x = 0.3/res;
    int wall_y = 0.3/res;
    for (int i = 0; i < static_map.info.height; ++i) {
        for (int j = 0; j < static_map.info.width; ++j) {
            double x = (j + 0.5)*static_map.info.resolution;
            double y = (i + 0.5)*static_map.info.resolution;
            math::Vec2d point(x, y);
            for (int k = 0; k < obs_num; ++k) {
                if (static_obs[k].IsPointIn(point)) {
                    p.x = x;
                    p.y = y;
                    p.z = 0;
                    wall.points.emplace_back(p);
                    p.z = obs_height[k];
                    wall.points.emplace_back(p);  
                    break;
                }
            }
            cnt ++;
        }
    } 
    int outside_y_min = -wall_y;
    int outside_y_max = (int)static_map.info.height + wall_y;
    int outside_x_min = -wall_x;
    int outside_x_max = (int)static_map.info.width + wall_x;
    for (int i = outside_y_min; i < outside_y_max; ++i) {
        for (int j = outside_x_min; j < outside_x_max; ++j) {
            if (i < 0 || i >= (int)static_map.info.height || j < 0 || j >= (int)static_map.info.width) {
                p.x = (j + 0.5)*static_map.info.resolution;
                p.y = (i + 0.5)*static_map.info.resolution;
                p.z = 0;
                wall.points.emplace_back(p);
                p.z = 0.2;
                wall.points.emplace_back(p);   
            }
        }
    }
    walls.markers.emplace_back(wall);
    wall.ns = "CUBE";
    wall.type = visualization_msgs::Marker::CUBE;
    wall.id ++;
    wall.scale.x = static_map.info.width * static_map.info.resolution;
    wall.scale.y = static_map.info.height * static_map.info.resolution;
    wall.scale.z = 0.01;
    wall.color.a = 1.0;
    wall.color.r = wall.color.g = wall.color.b = 0.0;
    wall.pose.position.x = wall.scale.x/2;
    wall.pose.position.y = wall.scale.y/2;
    wall.pose.position.z = 0;
    wall.color.a = 0.1;
    wall.points.clear();
    walls.markers.emplace_back(wall);


    ros::Rate r(10);
    while(ros::ok()) {
        dynamic_map = origin_map;
        cnt = 0;
        for (int i = 0; i < dynamic_map.info.height; ++i) {
            for (int j = 0; j < dynamic_map.info.width; ++j) {
                double x = (j + 0.5)*dynamic_map.info.resolution;
                double y = (i + 0.5)*dynamic_map.info.resolution;
                for (int k = 1; k < 4; ++k) {
                    double x_ = dynamic_obs[k].pose.position.x;
                    double y_ = dynamic_obs[k].pose.position.y;
                    if (x_ > 0 && y > 0 && std::hypot(x - x_, y - y_) < 0.7) {
                        dynamic_map.data[cnt] = (int8_t)100;
                    }
                }
                cnt ++;
            }
        }
        static_map_pub.publish(static_map);
        dilate_map_pub.publish(dilate_map);
        dynamic_map_pub.publish(dynamic_map);

        static_wall_pub.publish(walls);
        ros::spinOnce();
        r.sleep();
    }


}