#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/ColorOcTree.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <mutex>

#include "octomap_listener/graph.hh"
#include "octomap_listener/vertex.hh"

static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

// forward declaration of Graph
class Graph;
class Point;

class OctomapListener
{

public:
  
  OctomapListener(ros::NodeHandle&);
  void octomap_callback(const octomap_msgs::OctomapConstPtr&);
  void goal_callback(const geometry_msgs::PoseStamped::ConstPtr&);
  void odom_callback(const nav_msgs::Odometry::ConstPtr&);
  double getGraphVertexRes() const;
  void start_exploration();
  ~OctomapListener();

private:
  void search_front();
  void investigate() const;
  void start_navigation();
  void is_neighborhood_clear(Point&);
  bool is_front_clear(const geometry_msgs::PoseStamped&);
  bool is_distance_large(int);
  geometry_msgs::PoseStamped get_new_goal(const geometry_msgs::PoseStamped&);
  void adjust_direction_yaw(double, double);
  double calculate_radians_forTurn(double, double);
  double get_direction_yaw(double, double);
  double get_yaw_angle(double, double);
  double get_distance(const Point& _p1, const Point& _p2) const;

  ros::Time last_octomap_update_time_;
  ros::Time last_odom_update_time_;
  double odom_update_period_;
  double octomap_update_period_;
  nav_msgs::Odometry odom_;
  ros::NodeHandle nh_;
  ros::Publisher pub_cmdvel_;
  ros::Publisher pub_path_;
  ros::Publisher pub_marker_;
  std::vector<geometry_msgs::PoseStamped> goal_stack_;

  std::string action_client_name_;

  Graph* graph_;


  octomap::OcTree* octomap_;
  std::mutex octomap_mutex_;
  bool neighborhood_notclear_;
  bool check_neighborhood_;
  bool continue_exploration_;
  bool save_adj_list_;
  bool waiting_fresh_octomap_;          // When path planning is failed to find path, quickly enables the octomap to be updated
  double threshold_distance_;
  double threshold_height_;
  double ground_height_;
  double graph_vertex_res_;
  double yaw_;
  double RADIANS_TO_TURN_THRESHOLD;
  double YAW_ROTATION_SPEED;
  double DISTANCE_THRESHOLD;
  double bias_for_bbx = 0.2;
  double front_dist_for_bbx = 1;
};