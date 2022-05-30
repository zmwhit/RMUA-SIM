/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "a_star_planner.h"

AStarPlanner::AStarPlanner(const nav_msgs::OccupancyGrid& map) {
    map_ = map;
    cost_ = map_.data.data();
    res_ = map_.info.resolution;
    gridmap_height_ = map_.info.height;
    gridmap_width_ = map_.info.width;


    inaccessible_cost_ = 100;
    goal_search_tolerance_ = 1;
}

AStarPlanner::~AStarPlanner(){
    cost_ = nullptr;
}
bool AStarPlanner::CheckBound(const int x, const int y) {
    if (x < 0 || x >= gridmap_width_ || y < 0 || y >= gridmap_height_) 
        return false;
    return true;
}
bool AStarPlanner::World2Map(const double x, const double y, int& x_, int& y_) {
    x_ = std::round(x/res_);
    y_ = std::round(y/res_);
    return CheckBound(x_, y_);
}
bool AStarPlanner::Map2World(const int x, const int y, double& x_, double& y_) {
    x_ = (x + 0.5)*res_;
    y_ = (y + 0.5)*res_;
}
int AStarPlanner::GetCost(const int x, const int y) {
    int index = GetIndex(x, y);
    if (!CheckBound(x, y))
        index = inaccessible_cost_;
    return cost_[index];
}
int AStarPlanner::GetIndex(const int x, const int y) {
    return y*gridmap_width_ + x;
}
void AStarPlanner::Index2Cells(const int index, int& x, int & y) {
    y = index/gridmap_width_;
    x = index%gridmap_width_;
}
bool AStarPlanner::Plan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                        std::vector<geometry_msgs::PoseStamped> &path) {

    int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
    int valid_goal[2];
    unsigned  int shortest_dist = std::numeric_limits<int>::max();
    bool goal_valid = false;
    path.clear();
    double t1 = std::clock();
    if (!World2Map(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
        return false;
    }
    if (!World2Map(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        return false;
    }
    if (GetCost(goal_x,goal_y)<inaccessible_cost_){
        valid_goal[0] = goal_x;
        valid_goal[1] = goal_y;
        goal_valid = true;
    } else {
        tmp_goal_x = goal_x;
        tmp_goal_y = goal_y - goal_search_tolerance_;

        while(tmp_goal_y <= goal_y + goal_search_tolerance_){
            tmp_goal_x = goal_x - goal_search_tolerance_;
            while(tmp_goal_x <= goal_x + goal_search_tolerance_){
                unsigned char cost = GetCost(tmp_goal_x, tmp_goal_y);
                int dist = abs(static_cast<int>(goal_x - tmp_goal_x)) + abs(static_cast<int>(goal_y - tmp_goal_y));
                if (cost < inaccessible_cost_ && dist < shortest_dist ) {
                    shortest_dist = dist;
                    valid_goal[0] = tmp_goal_x;
                    valid_goal[1] = tmp_goal_y;
                    goal_valid = true;
                }
                tmp_goal_x += 1;
            }
            tmp_goal_y += 1;
        }
    }
    if (!goal_valid) {
        path.clear();
        return false;
    } else {
        int start_index, goal_index;
        start_index = GetIndex(start_x, start_y);
        goal_index = GetIndex(valid_goal[0], valid_goal[1]);

        if(start_index == goal_index){
            path.clear();
            path.push_back(start);
            path.push_back(goal);
        } else {
            if (SearchPath(start_index, goal_index, path)) {
                path.back().pose.orientation = goal.pose.orientation;
                path.back().pose.position.z = goal.pose.position.z;                
            }
        }

    }
    double t2 = std::clock();
    double time_search = (t2 - t1)/CLOCKS_PER_SEC*1000;
    dbg(time_search);
    return true;
}

bool AStarPlanner::SearchPath(const int &start_index, const int &goal_index,
                             std::vector<geometry_msgs::PoseStamped> &path) {
    start_point = start_index;
    g_score_.clear();
    f_score_.clear();
    parent_.clear();
    state_.clear();
    
    g_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    f_score_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    parent_.resize(gridmap_height_ * gridmap_width_, std::numeric_limits<int>::max());
    state_.resize(gridmap_height_ * gridmap_width_, SearchState::NOT_HANDLED);

    auto compare = [&](const int &index1, const int &index2) {
        return f_score_.at(index1) > f_score_.at(index2);
    };
    std::priority_queue<int, std::vector<int>, decltype(compare)> openlist(compare);
    g_score_.at(start_index) = 0;
    openlist.push(start_index);

    std::vector<int> neighbors_index;
    int current_index, move_cost, h_score, count = 0;

    while (!openlist.empty()) {
        current_index = openlist.top();
        openlist.pop();
        state_.at(current_index) = SearchState::CLOSED;

        if (current_index == goal_index) {
            // ROS_INFO("Search takes %d cycle counts", count);
            break;
        }

        GetNineNeighbors(current_index, neighbors_index);

        for (auto neighbor_index : neighbors_index) {

            if (neighbor_index < 0 ||
                neighbor_index >= gridmap_height_ * gridmap_width_) {
                continue;
            }

            if (cost_[neighbor_index] >= inaccessible_cost_ ||
                state_.at(neighbor_index) == SearchState::CLOSED) {
                continue;
            }

            GetMoveCost(current_index, neighbor_index, move_cost);

            if (g_score_.at(neighbor_index) > g_score_.at(current_index) + move_cost) {

                g_score_.at(neighbor_index) = g_score_.at(current_index) + move_cost;
                parent_.at(neighbor_index) = current_index;

                if (state_.at(neighbor_index) == SearchState::NOT_HANDLED) {
                    GetEstimateDistance(neighbor_index, goal_index, h_score);
                    f_score_.at(neighbor_index) = g_score_.at(neighbor_index) + h_score;
                    openlist.push(neighbor_index);
                    state_.at(neighbor_index) = SearchState::OPEN;
                }
            }
        }
        count++;
    }

    if (current_index != goal_index) {
        return false;
    }

    int iter_index = current_index, iter_x, iter_y;

    geometry_msgs::PoseStamped iter_pos;
    iter_pos.pose.orientation.w = 1;
    iter_pos.header.frame_id = "map";
    path.clear();
    Index2Cells(iter_index, iter_x, iter_y);
    Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
    path.push_back(iter_pos);

    while (iter_index != start_index) {
        iter_index = parent_.at(iter_index);
        Index2Cells(iter_index, iter_x, iter_y);
        Map2World(iter_x, iter_y, iter_pos.pose.position.x, iter_pos.pose.position.y);
        path.push_back(iter_pos);
    }

    std::reverse(path.begin(),path.end());
    return true;

}

int AStarPlanner::GetMoveCost(const int &current_index, const int &neighbor_index, int &move_cost) {
    move_cost = 0;
    if (abs(neighbor_index - current_index) == 1 ||
        abs(neighbor_index - current_index) == gridmap_width_) {
        move_cost = 10;
    } else if (abs(neighbor_index - current_index) == (gridmap_width_ + 1) ||
                abs(neighbor_index - current_index) == (gridmap_width_ - 1)) {
        move_cost = 14;
    } else {
        return false;
    }
    // if (current_index != start_point) {
    //     int index1 = parent_.at(current_index);
    //     int index2 = current_index;
    //     int index3 = neighbor_index;
    //     int p1x, p1y, p2x, p2y, p3x, p3y;
    //     Index2Cells(index1, p1x, p1y);
    //     Index2Cells(index2, p2x, p2y);
    //     Index2Cells(index3, p3x, p3y);
    //     if ((p2x - p1x == p3x - p2x && p2y - p1y == p3y - p2y) || 
    //         (p2x == p1x && p3x == p2x) ||
    //         (p2y == p1y && p3y == p2y)) {
    //         ;
    //     } else {
    //         move_cost += 10;
    //     }

    // }
    return true;
}

void AStarPlanner::GetEstimateDistance(const int &index1, const int &index2, int &estimate_distance) const {
    auto d1 = abs(static_cast<int>(index1 / gridmap_width_ - index2 / gridmap_width_));//y
    auto d2 = abs(static_cast<int>(index1 % gridmap_width_ - index2 % gridmap_width_));//x
    // estimate_distance = 10 * std::sqrt(d1*d1 + d2*d2);
    estimate_distance = 10 * (std::min(d1, d2) + std::abs(d1 - d2));
}

void AStarPlanner::GetNineNeighbors(const int &current_index, std::vector<int> &neighbors_index) const {
    neighbors_index.clear();
    if(current_index - gridmap_width_ >= 0){
        neighbors_index.push_back(current_index - gridmap_width_);       //up
    }
    if(current_index - gridmap_width_ - 1 >= 0 && (current_index - gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
        neighbors_index.push_back(current_index - gridmap_width_ - 1); //left_up
    }
    if(current_index - 1 >= 0 && (current_index - 1 + 1) % gridmap_width_!= 0){
        neighbors_index.push_back(current_index - 1);        //left
    }
    if(current_index + gridmap_width_ - 1 < gridmap_width_* gridmap_height_
        && (current_index + gridmap_width_ - 1 + 1) % gridmap_width_!= 0){
        neighbors_index.push_back(current_index + gridmap_width_ - 1); //left_down
    }
    if(current_index + gridmap_width_ < gridmap_width_* gridmap_height_){
        neighbors_index.push_back(current_index + gridmap_width_);     //down
    }
    if(current_index + gridmap_width_ + 1 < gridmap_width_* gridmap_height_
        && (current_index + gridmap_width_ + 1 ) % gridmap_width_!= 0){
        neighbors_index.push_back(current_index + gridmap_width_ + 1); //right_down
    }
    if(current_index  + 1 < gridmap_width_* gridmap_height_
        && (current_index  + 1 ) % gridmap_width_!= 0) {
        neighbors_index.push_back(current_index + 1);                   //right
    }
    if(current_index - gridmap_width_ + 1 >= 0
        && (current_index - gridmap_width_ + 1 ) % gridmap_width_!= 0) {
        neighbors_index.push_back(current_index - gridmap_width_ + 1); //right_up
    }
}
