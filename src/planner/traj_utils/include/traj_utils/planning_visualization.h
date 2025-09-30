#ifndef PLANNING_VISUALIZATION_H_
#define PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::vector;
namespace ego_planner {
class PlanningVisualization : public rclcpp::Node {
private:
  // ROS2 发布器类型变更
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_point_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_list_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_list_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      optimal_list_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr a_star_list_pub;

public:
  PlanningVisualization(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PlanningVisualization() = default;

  typedef std::shared_ptr<PlanningVisualization> Ptr;

  void displayMarkerList(
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
      const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color,
      int id, bool show_sphere = true);

  void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                const vector<Eigen::Vector3d> &list,
                                double scale, Eigen::Vector4d color, int id);

  void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                 const vector<Eigen::Vector3d> &list,
                                 double scale, Eigen::Vector4d color, int id);

  void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color,
                        const double scale, int id);

  void displayGlobalPathList(vector<Eigen::Vector3d> global_pts,
                             const double scale, int id);
  void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale,
                           int id);
  void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs,
                                const double scale);
  void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
  void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths,
                        int id);

  void displayArrowList(
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub,
      const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color,
      int id);

  // void
  // displayIntermediateState(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr&
  // intermediate_pub,
  //                              ego_planner::BsplineOptimizer::Ptr optimizer,
  //                              double sleep_time, const int start_iteration);
  // void
  // displayNewArrow(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr&
  // guide_vector_pub,
  //                     ego_planner::BsplineOptimizer::Ptr optimizer);
};
} // namespace ego_planner
#endif