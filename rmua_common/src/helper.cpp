#include "helper.h"
namespace Helper {

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double getRelativeAngle(const geometry_msgs::PoseStamped& p, 
                        const geometry_msgs::PoseStamped& q) {
  return std::atan2(p.pose.position.y - q.pose.position.y,
                    p.pose.position.x - q.pose.position.x);
}


std::vector<geometry_msgs::PoseStamped> PathTransform(const std::vector<Eigen::Vector2d>& path) {
  std::vector<geometry_msgs::PoseStamped> result;
  geometry_msgs::PoseStamped pp;
  pp.pose.orientation.w  = 1.0;
  for_each(path.begin(), path.end(), [&](const Eigen::Vector2d& p){
    pp.pose.position.x = p(0);
    pp.pose.position.y = p(1);
    pp.pose.position.z = 0.0;
    result.emplace_back(pp);
  });
  return result;
}
std::vector<Eigen::Vector2d> PathTransform(const std::vector<geometry_msgs::PoseStamped>& path) {
  std::vector<Eigen::Vector2d> result;
  Eigen::Vector2d pp;
  for_each(path.begin(), path.end(), [&](const geometry_msgs::PoseStamped& p){
    pp(0) = p.pose.position.x;
    pp(1) = p.pose.position.y;
    result.emplace_back(pp);
  });
  return result;
}
double getDistance(const std::vector<Eigen::Vector2d>& path) {
  double dis = 0;
  if (path.size() < 2) return dis;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    dis += (path[i], path[i+1]).norm();
  }
  return dis;
}
double getDistance(const std::vector<geometry_msgs::PoseStamped>& path) {
  double dis = 0;
  if (path.size() < 2) return dis;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    dis += getDistance(path[i], path[i+1]);
  }
  return dis;
}
double getDistance(const geometry_msgs::PoseStamped& p, const geometry_msgs::PoseStamped& q) {
    double dx = p.pose.position.x - q.pose.position.x;
    double dy = p.pose.position.y - q.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
}


geometry_msgs::PoseStamped getIdentityPoseStamp() {
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  // Position
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  // Orientation on place
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;
  return pose;
}
geometry_msgs::Pose getIdentityPose() {
  geometry_msgs::Pose pose;
  // Position
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  // Orientation on place
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  return pose;
}

void SaveVector(std::string file_path, std::string file_name, std::string first, std::string last, std::vector<double>& vec) {
  CSVWriter csv(","); 
  csv.newRow() << first << last;
  for (auto& data : vec) {
    csv.newRow() << data;
  }
  csv.writeToFile(file_path + "/" + file_name + ".csv");
}
void SaveWaveForm(std::string file_path, std::string file_name, std::string first, std::string last, std::vector<double>& time, std::vector<double>& data) {
  if (data.size() > time.size())  return;
  CSVWriter csv(","); 
  csv.newRow() << first << last;
  for (size_t i = 0; i < time.size(); ++i) {
    csv.newRow() << time[i] << data[i];
  }
  csv.writeToFile(file_path + "/" + file_name + ".csv");
}
void SaveWaveForm(std::string file_path, std::string file_name, std::string first, std::string last, double dt, std::vector<double>& data) {
  CSVWriter csv(","); 
  csv.newRow() << first << last;
  for (size_t i = 0; i < data.size(); ++i) {
    csv.newRow() << i*dt << data[i];
  }
  csv.writeToFile(file_path + "/" + file_name + ".csv");
}
void SavePath(std::string file_path, std::string file_name, std::vector<geometry_msgs::PoseStamped>& path) {
  CSVWriter csv(","); 
  csv.newRow() << "x" << "y" << "a";
  for (auto& pose : path) {
    csv.newRow() << pose.pose.position.x << pose.pose.position.y << tf::getYaw(pose.pose.orientation);  
  }
  csv.writeToFile(file_path + "/" + file_name + ".csv");
}



std::vector<geometry_msgs::PoseStamped> ReadPath(std::string file_path, std::string file_name) {
  std::vector<geometry_msgs::PoseStamped> path;
  std::vector<std::vector<double>> csv_table;
  std::ifstream fp(file_path + "/" + file_name + ".csv");
  if (!fp) {
      std::cout << " load csv file failed : " << file_name  << " ... " << std::endl;
      return path;
  }
  int count = 0;
  while(!fp.eof()) {
      std::string data;
      std::getline(fp, data);
      if (data == "x,y,a")  continue;
      std::istringstream stream(data);
      std::vector<double> splitted_data;
      std::string buffer;
      while(std::getline(stream, buffer, ',')){
          splitted_data.push_back(stod(buffer));
      }   
      csv_table.emplace_back(splitted_data);
  }
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "map";
  for (auto& pose : csv_table) {
    p.pose.position.x = pose[0];
    p.pose.position.y = pose[1];
    p.pose.orientation = tf::createQuaternionMsgFromYaw(pose[2]);
    path.emplace_back(p);

  }
  return path;
}

}
