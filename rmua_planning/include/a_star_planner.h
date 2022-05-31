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
#pragma once
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "dbg.h"


class AStarPlanner {

public:
  AStarPlanner(const nav_msgs::OccupancyGrid& map);
  virtual ~AStarPlanner();
  /**
   * @brief Main Plan function(override the base-class function)
   * @param start Start pose input
   * @param goal Goal pose input
   * @param path Global plan path output
   * @return ErrorInfo which is OK if succeed
   */
  bool Plan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
            std::vector<geometry_msgs::PoseStamped> &path);
  bool World2Map(const double x, const double y, int& x_, int& y_);
  int GetCost(const int x, const int y);
  void UpdateMap(const nav_msgs::OccupancyGrid& map);
private:
 bool CheckBound(const int x, const int y);
 bool Map2World(const int x, const int y, double& x_, double& y_);
 int GetIndex(const int x, const int y);
 void Index2Cells(const int index, int& x, int & y);
  /**
   * @brief State enumerate for the cell.
   */
  enum SearchState {
    NOT_HANDLED, /**< The cell is not handled.*/
    OPEN, /**< The cell is in open priority queue.*/
    CLOSED /**< The cell is in close queue.*/
  };
  /**
   * @brief Plan based on 1D Costmap list. Input the index in the costmap and get the plan path.
   * @param start_index start pose index in the 1D costmap list
   * @param goal_index goal pose index in the 1D costmap list
   * @param path plan path output
   * @return ErrorInfo which is OK if succeed
   */
  bool SearchPath(const int &start_index, const int &goal_index, std::vector<geometry_msgs::PoseStamped> &path);
  /**
   * @brief Calculate the cost for the diagonal or parallel movement.
   * @param current_index Index of the current cell as input
   * @param neighbor_index Index of the neighbor cell as input
   * @param move_cost Movement cost as output
   * @return ErrorInfo which is OK if succeed
   */
  int GetMoveCost(const int &current_index, const int &neighbor_index, int &move_cost);
  /**
   * @brief Calculate the Manhattan distance between two cell index used as the heuristic function of A star algorithm.
   * @param index1 Index of the first cell as input
   * @param index2 Index of the second cell as input
   * @param manhattan_distance The Manhattan distance as output
   */
  void GetEstimateDistance(const int &index1,
                            const int &index2,
                            int &estimate_distance) const;
  /**
   * @brief Get the index of nine neighbor cell from the current cell
   * @param current_index Index of the current cell as input
   * @param neighbors_index Index of the neighbor cells as out
   */
  void GetNineNeighbors(const int &current_index,
                        std::vector<int> &neighbors_index) const;



  nav_msgs::OccupancyGrid map_;
  double res_;
  int start_point;
  
  int inaccessible_cost_;
  int goal_search_tolerance_;
  int gridmap_height_;
  int gridmap_width_;
  int8_t *cost_;
  std::vector<int> f_score_;
  std::vector<int> g_score_;
  std::vector<int> parent_;
  std::vector<AStarPlanner::SearchState> state_;
};