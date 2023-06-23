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

#include "polygon_planner_base.h"


#include <functional>


#include <polygon_coverage_planners/cost_functions/path_cost_functions.h>

// #include "os_interface.h"
// #include <polygon_coverage_msgs/msg_from_xml_rpc.h>
// #include <geometry_msgs/PoseArray.h>
// #include <visualization_msgs/MarkerArray.h>

namespace polygon_coverage_planning {

PolygonPlannerBase::PolygonPlannerBase(const hecto::mission::common::GridOptions* request)
      : wall_distance_(0.0),
      path_cost_function_(
          {std::bind(&computeEuclideanPathCost, std::placeholders::_1),
           CostFunctionType::kTime}),
      latch_topics_(true),
      // global_frame_id_("world"),
      // publish_plan_on_planning_complete_(false),
      // publish_visualization_on_planning_complete_(true),
      set_start_goal_from_rviz_(false),
      set_polygon_from_rviz_(true),
      planning_complete_(false) {
  // Initial interactions
      getParametersFromClient(request);

}


void PolygonPlannerBase::getParametersFromClient(const hecto::mission::common::GridOptions* request) {
  // Load the polygon from polygon message from parameter server.
  Polygon_2 temp_poly;
  if(request->polygon().empty()){
    std::cout << "Empty polygon" << "\n";
  }
  for (const auto p : request->polygon()) {
    temp_poly.push_back(Point_2(p.latitudedeg(), p.longitudedeg()));
  } 

  PolygonWithHoles temp_pwh(temp_poly);
  polygon_ = std::make_optional(temp_pwh);
  // std::cout << polygon_ << "\n";
  altitude_ = std::make_optional(3);
  // altitude_ = std::make_optional(request->height());

  // Getting control params from the server
  // if (request->wall_distance()) {
  //   ROS_WARN_STREAM("No wall distance specified. Using default value of: "
  //                   << wall_distance_);
  // } else
  //   ROS_INFO_STREAM("Wall distance: " << wall_distance_ << " m");

  // Cost function
  double temp_v_max = 3; 
  if (true) {
    v_max_ = std::make_optional(temp_v_max);
  }
  double temp_a_max = 1; 
  if (true) {
    a_max_ = std::make_optional(temp_a_max);
  }

  // Cost function type.
  int cost_function_type_int = static_cast<int>(path_cost_function_.second);
  // if (!nh_private_.getParam("cost_function_type", cost_function_type_int)) {
  //   ROS_WARN_STREAM("No cost_function_type specified. Using default value of: "
  //                   << path_cost_function_.second << "("
  //                   << getCostFunctionTypeName(path_cost_function_.second)
  //                   << ").");
  // }
  if (!checkCostFunctionTypeValid(cost_function_type_int)) {
    ROS_WARN_STREAM("cost_function_type not valid. Resetting to default: "
                    << path_cost_function_.second << "("
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << ").");
    cost_function_type_int = static_cast<int>(path_cost_function_.second);
  }
  path_cost_function_.second =
      static_cast<CostFunctionType>(cost_function_type_int);

  ROS_INFO_STREAM(
      "Cost function: " << getCostFunctionTypeName(path_cost_function_.second));
  if (path_cost_function_.second == CostFunctionType::kTime) {
    if (!v_max_.has_value() || !a_max_.has_value()) {
      ROS_WARN_COND(!v_max_.has_value(), "Velocity 'v_max' not set.");
      ROS_WARN_COND(!a_max_.has_value(), "Acceleration 'a_max' not set.");
      ROS_WARN("Falling back to distance cost function.");
      path_cost_function_.second = CostFunctionType::kDistance;
    } else {
      ROS_INFO_STREAM("v_max: " << v_max_.value()
                                << ", a_max: " << a_max_.value());
    }
  }

  switch (path_cost_function_.second) {
    case CostFunctionType::kDistance: {
      path_cost_function_.first =
          std::bind(&computeEuclideanPathCost, std::placeholders::_1);
      break;
    }
    case CostFunctionType::kTime: {
      path_cost_function_.first =
          std::bind(&computeVelocityRampPathCost, std::placeholders::_1,
                    v_max_.value(), a_max_.value());
      break;
    }
    case CostFunctionType::kWaypoints: {
      path_cost_function_.first =
          std::bind(&computeWaypointsPathCost, std::placeholders::_1);
      break;
    }
    default: {
      ROS_WARN_STREAM("Cost function type: "
                      << getCostFunctionTypeName(path_cost_function_.second)
                      << "not implemented. Using euclidean distance.");
      break;
    }
  }
}

void PolygonPlannerBase::solve(const Point_2& start, const Point_2& goal) {
  ROS_INFO_STREAM("Start solving.");
  if ((planning_complete_ = solvePlanner(start, goal))) {
    ROS_INFO_STREAM("Finished plan."
                    << std::endl
                    << "Optimization Criterion: "
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << std::endl
                    << "Number of waypoints: " << solution_.size() << std::endl
                    << "Start point: " << start << std::endl
                    << "Goal point: " << goal << std::endl
                    << "Altitude: " << altitude_.value() << " [m]" << std::endl
                    << "Path length: " << computeEuclideanPathCost(solution_)
                    << " [m]");
    if (v_max_.has_value() && a_max_.has_value())
      ROS_INFO_STREAM("Path time: "
                      << computeVelocityRampPathCost(solution_, v_max_.value(),
                                                     a_max_.value())
                      << " [s]");

    // // Publishing the plan if requested
    // if (true) {
    //   publishTrajectoryPoints();
    // }
    // // Publishing the visualization if requested
    // if (true) {
    //   publishVisualization();
    // }
  } else {
    ROS_ERROR_STREAM("Failed calculating plan.");
  }
}

std::vector<Point_2> PolygonPlannerBase::publishVisualization() {

  // The planned path:
  const double kPathLineSize = 0.2;
  const double kPathPointSize = 0.2;

  // The original polygon:
  const double kPolygonLineSize = 0.4;

  return solution_;
}

bool PolygonPlannerBase::publishTrajectoryPoints() {
  if (!planning_complete_) {
    ROS_WARN("Cannot send trajectory messages because plan has not been made.");
    return false;
  }
  ROS_INFO_STREAM("Sending trajectory messages");

  // Convert path to pose array.
  if (!altitude_.has_value()) {
    ROS_WARN_STREAM("Cannot send trajectory because altitude not set.");
    return false;
  }

  // publish Trajectory

  // Success
  return true;
}

bool PolygonPlannerBase::planPath(const Point_2& start,const Point_2& goal) {
  planning_complete_ = false;
  if (!polygon_.has_value()) {
    ROS_WARN("Polygon not set. Cannot plan path.");
    return true;
  }

  solve(start, goal);  // Calculate optimal path.
  
  if (altitude_.has_value()) {
    // msgMultiDofJointTrajectoryFromPath(solution_, altitude_.value(),
    //                                    &response.sampled_plan);
  } else {
    ROS_WARN("Cannot plan path. Altitude not set.");
  }
  return true;
}

// Reset the planner when a new polygon is set.
bool PolygonPlannerBase::resetPlanner() {
  ROS_ERROR_STREAM("resetPlanner is not implemented.");
  return false;
}


}  // namespace polygon_coverage_planning
