#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <math.h>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include "ros_headers.h"

#include "CSVWriter.h"
namespace Helper {
static std::string default_csv_path = "/home/zmw/motion_planning/RMUASimulator/src/rmua_planning/scripts/data";
static std::string default_pic_path = "/home/zmw/motion_planning/RMUASimulator/src/rmua_planning/scripts/pic";

double NormalizeAngle(const double angle);
double WrapAngle(const double angle);

double getDistance(const geometry_msgs::PoseStamped& p, const geometry_msgs::PoseStamped& q);
double getDistance(const std::vector<geometry_msgs::PoseStamped>& path);
double getDistance(const std::vector<Eigen::Vector2d>& path);

double getRelativeAngle(const geometry_msgs::PoseStamped& p, const geometry_msgs::PoseStamped& q);
                                
std::vector<geometry_msgs::PoseStamped> PathTransform(const std::vector<Eigen::Vector2d>& path);
std::vector<Eigen::Vector2d> PathTransform(const std::vector<geometry_msgs::PoseStamped>& path);

geometry_msgs::Pose getIdentityPose();   
geometry_msgs::PoseStamped getIdentityPoseStamp();



void SaveVector(std::string file_path, std::string file_name, std::string first, std::string last, std::vector<double>& vec);
void SaveWaveForm(std::string file_path, std::string file_name, std::string first, std::string last, std::vector<double>& time, std::vector<double>& data);
void SaveWaveForm(std::string file_path, std::string file_name, std::string first, std::string last, double dt, std::vector<double>& data);
void SavePath(std::string file_path, std::string file_name, std::vector<geometry_msgs::PoseStamped>& path);
}

