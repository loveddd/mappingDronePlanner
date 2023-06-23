/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_
#define POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_

#include <memory>
#include <optional>


// #include <polygon_coverage_msgs/PolygonService.h>
// #include <polygon_coverage_msgs/PolygonWithHolesStamped.h>
// #include <ros/ros.h>
// #include <std_srvs/Empty.h>
// #include <geometry_msgs/PointStamped.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <polygon_coverage_msgs/PlannerService.h>
#include "../build/proto/grid.grpc.pb.h"
#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_planners/cost_functions/path_cost_functions.h>
#include <polygon_coverage_planners/sensor_models/sensor_model_base.h>





namespace polygon_coverage_planning {

// A basic ros wrapper for planner in a 2D polynomial environment.
class PolygonPlannerBase {
 public:
  // Constructor
//   PolygonPlannerBase(const ros::NodeHandle& nh,
//                      const ros::NodeHandle& nh_private);
  PolygonPlannerBase(const hecto::mission::common::GridOptions* request);
  bool planPath(const Point_2& start,const Point_2& goal);
  // Visualization
  std::vector<Point_2> publishVisualization();
  
 protected:
  // Call to the actual planner.
  virtual bool solvePlanner(const Point_2& start, const Point_2& goal) = 0;
  // Reset the planner when a new polygon is set.
  virtual bool resetPlanner() = 0;



  // The solution waypoints for a given start and goal.
  std::vector<Point_2> solution_;

  // Parameters
  std::optional<PolygonWithHoles> polygon_;
  double wall_distance_;
  std::pair<PathCostFunction, CostFunctionType> path_cost_function_;
  std::optional<double> altitude_;
  bool latch_topics_;
//   std::string global_frame_id_;
//   bool publish_plan_on_planning_complete_;
//   bool publish_visualization_on_planning_complete_;
  std::optional<double> v_max_;
  std::optional<double> a_max_;
  bool set_start_goal_from_rviz_;
  bool set_polygon_from_rviz_;
  std::optional<Point_2> start_;
  std::optional<Point_2> goal_;

 private:
  // Solve the planning problem. Stores status planning_complete_ and publishes
  // trajectory and visualization if enabled.
  void solve(const Point_2& start, const Point_2& goal);

  // Initial interactions with ROS
  void getParametersFromClient(const hecto::mission::common::GridOptions* request);




  // Publishing the plan
  bool publishTrajectoryPoints();
  // Publishers and Services

  // Planner status
  bool planning_complete_;

};
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_
