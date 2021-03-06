#include "octomap_listener/octomap_listener_v1.hh"

// For publishing Path to Rviz
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>

#include "octomap_listener/graph.hh"
#include "octomap_listener/vertex.hh"
#include "octomap_listener/point.hh"

#include <boost/thread.hpp>

#include <thread>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <istream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <time.h>
#include <stack>
#include <chrono>

/* For file platform specific file separator */
#if defined(_WIN16) | defined(_WIN32) | defined(_WIN64)
#define FILE_SEPERATOR "\\"
#else
#define FILE_SEPERATOR "/"
#endif

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef std::shared_ptr<Vertex> VertexPtr;

OctomapListener::OctomapListener(ros::NodeHandle& _nh) : goal_stack_(), graph_(nullptr),
  octomap_(nullptr)
 {
  last_octomap_update_time_ = ros::Time::now();
  last_odom_update_time_ = ros::Time::now();
  ros::NodeHandle private_nh("~");
  continue_exploration_ = false;

  waiting_fresh_octomap_ = true;
  double octomap_update_freq, odom_update_freq;
  int diamond_size;
  private_nh.param("octomap_update_freq", octomap_update_freq, 10.0);
  private_nh.param("odometry_update_freq", odom_update_freq, 5.0);
  std::string twist_topic_name, move_base_goal_name;
  private_nh.param("twist_topic_name", twist_topic_name, std::string("aaa"));
  private_nh.param("move_base_goal_name", move_base_goal_name, std::string("/move_base_simple1/goal"));
  private_nh.param("radians_to_turn_threshold", RADIANS_TO_TURN_THRESHOLD, 0.2);
  private_nh.param("yaw_rotation_speed", YAW_ROTATION_SPEED, 0.5);
  private_nh.param("distance_threshold", DISTANCE_THRESHOLD, 0.5);
  private_nh.param("vertex_resolution", graph_vertex_res_, 1.);
  private_nh.param("ground_height", ground_height_, 0.4);
  private_nh.param("threshold_height", threshold_height_, 0.05);
  private_nh.param("threshold_distance", threshold_distance_, 0.4);
  private_nh.param("save_adj_list", save_adj_list_, false);
  private_nh.param("diamond_size", diamond_size, 4);
  private_nh.param("action_client_name", action_client_name_, std::string("/diff_car/move_base"));
  octomap_update_period_ = 1. / octomap_update_freq;
  odom_update_period_ = 1. / odom_update_freq;
  nh_ = _nh;
  pub_cmdvel_ = nh_.advertise<geometry_msgs::Twist>(twist_topic_name, 500);
  pub_path_ = nh_.advertise<nav_msgs::Path>("octomap_path", 1);
  pub_marker_ = nh_.advertise<visualization_msgs::Marker>("start_finish", 1);
  ROS_INFO("Advertised <geometry_msgs::Twist> on %s \n", twist_topic_name.c_str());
  ROS_INFO("Octomap Update Period = %.4f\nOdometry Update Period = %.4f", octomap_update_period_, odom_update_period_);
   graph_ = new Graph(graph_vertex_res_, ground_height_, threshold_height_, diamond_size);
   this->neighborhood_notclear_ = false;
}

OctomapListener::~OctomapListener() {}

void OctomapListener::octomap_callback(const octomap_msgs::OctomapConstPtr& msg) {

  // ROS_INFO("RECEIVED %d bytes\n", (int)msg->data.size());

  ros::Time current_time = ros::Time::now();
  double seconds_since_last_update = current_time.toSec() - last_octomap_update_time_.toSec();

  using Seconds = std::chrono::seconds;
  if ( waiting_fresh_octomap_ || seconds_since_last_update >= octomap_update_period_  /*&&
        lock.try_lock_for(Seconds(1)) )*/ ) {

    ROS_WARN("Inside octomap_cb");
   // if (graph_ != nullptr)
   //   delete graph_;
   // graph_ = new Graph(graph_vertex_res_, ground_height_, threshold_height_);
    /*std::ofstream fout1;
    std::ofstream fout2;
    srand(time(NULL));
    int random = rand();
    std::string file_seperator = FILE_SEPERATOR;
    std::string file_base_name = std::string("/home/azad/ros_workspace/octomap_ws/data/") + "info_";
    std::string file_name1 = file_base_name + "1_" + std::to_string(random);
    std::string file_name2 = file_base_name + "2_" + std::to_string(random);*/
    // fout1.open(file_name1);
    // fout2.open(file_name2);

    // octomap::OcTree* octomap = NULL;
    {
      std::lock_guard<std::mutex> lock(this->octomap_mutex_);
    if (octomap_ != nullptr) {
      delete octomap_;
      octomap_ = nullptr;
    }
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
      octomap_ = dynamic_cast<octomap::OcTree*>(tree);
      // ROS_INFO("Casted\n");
    }

    if (!octomap_) {
      ROS_ERROR("Failed to create octree structure\n");
      return;
    }

    }

   /* std::stringstream ss1("Info: ", std::ios_base::app | std::ios_base::out);
    std::stringstream ss2("Info: " , std::ios_base::app | std::ios_base::out);
    double minX, minY, minZ, maxX, maxY, maxZ;
    octomap->getMetricMin(minX, minY, minZ);
    octomap->getMetricMax(maxX, maxY, maxZ);
    // ROS_INFO("MIN x: %lf, y: %lf, z: %lf\t MAX: x: %lf, y: %lf, z: %lf\n",
      // minX, minY, minZ, maxX, maxY, maxZ);
    ss1 << "MIN x: " << minX << ", y: " << minY << ", z: " << minZ << "\n MAX: x: "
      << maxX << ", y: " << maxY << ", z: " << maxZ << std::endl;  
    octomap::point3d minPt = octomap::point3d(minX, minY, minZ);
    ss2 << "MIN x: " << minX << ", y: " << minY << ", z: " << minZ << "\n MAX: x: "
      << maxX << ", y: " << maxY << ", z: " << maxZ << std::endl;  

    unsigned int tree_depth = octomap->getTreeDepth();
    unsigned int octree_depth_ = max_octree_depth_;
    // ROS_INFO("tree_depth: %d, max_octree_depth: %d\n", tree_depth, octree_depth_);
    ss1 << "tree_depth" << tree_depth << ", max_octree_depth: " << octree_depth_ << std::endl;
    ss2 << "tree_depth" << tree_depth << ", max_octree_depth: " << octree_depth_ << std::endl;

    octomap::OcTreeKey paddedMinKey = octomap->coordToKey(minPt);
    
    unsigned int width, height;
    double res;

    res = octomap->getNodeSize(octree_depth_);
    width = (maxX - minX) / res + 1;
    height = (maxY - minY) / res + 1;
    unsigned int ds_shift = tree_depth - octree_depth_; 
    float org_x = minX - (res / (float)(1<<ds_shift)) + res;
    float org_y = minY - (res / (float)(1<<ds_shift));*/

    /*std::cout << "ds_shift: " << ds_shift << ", res: " << res <<
    ", width: " << width << ", height: " << height << ", org_x: " <<
    org_x << ", org_y: " << org_y << std::endl;
    */
    
    // ROS_INFO("treeDepth: %d\n", treeDepth);  

  /*  double node_size = octomap->getNodeSize(treeDepth);
    std::string tree_type = octomap->getTreeType();
    ROS_INFO("Node size: %lf, Tree type: %s", node_size, tree_type.c_str());*/
    // ROS_INFO("numOfLeafNodes: %d\n", (int)octomap->getNumLeafNodes());  
    unsigned int counter = 0;
    /*for (octomap::OcTree::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it) {
      bool occupied = octomap->isNodeOccupied(*it);
      int intSize = 1 << (octree_depth_ - it.getDepth());
      // ROS_INFO("%u. Occup: %d, intSize: %d", counter++, occupied, intSize);
    }*/
    double resolution = 0.05;

   /* for (octomap::OcTree::tree_iterator it = octomap->begin_tree(), end = octomap->end_tree();
        it != end; it++) {

      ss1 << counter++ << ") Coord: " << it.getCoordinate() << std::endl;
      ss1 << "\t Depth: " << it.getDepth() << " - " << "Size: " << it.getSize() <<
        " - Value: " << it->getValue() << " - Occupancy: " << it->getOccupancy() << std::endl;        
    }*/
/*    Vertex vx_outer_tmp(odom_.pose.pose.position.x, 
      odom_.pose.pose.position.y);
    if (odom_.pose.pose.position.z < 0.39)
      vx_outer_tmp.set_height(0.4);
    else
      vx_outer_tmp.set_height(odom_.pose.pose.position.z - 0.03);*/
    ROS_INFO("x: %.4f, y: %.4f, z: %.4f", odom_.pose.pose.position.x, 
      odom_.pose.pose.position.y, odom_.pose.pose.position.z);
    // graph_->addToNearestVertex(vx_outer_tmp, true);
    clock_t start = clock();
    for (octomap::OcTree::leaf_iterator it = octomap_->begin_leafs(), 
      end = octomap_->end_leafs(); it != end && ros::ok(); it++) {
     
      octomath::Vector3 pos_tmp = it.getCoordinate();
      Vertex vx_tmp(pos_tmp.x(), pos_tmp.y());
      // if the node is occupited assign its height, otherwise every vertex height would be the highest.
      bool is_occupied = false;
      if (octomap_->isNodeOccupied(*it)) {
        vx_tmp.set_height(pos_tmp.z());
        is_occupied = true;
      }
      else
        vx_tmp.set_height(ground_height_);
      graph_->addToNearestVertex(vx_tmp, is_occupied);
      counter++;
      /*
      if (octomap->isNodeOccupied(*it))
        ss2 << "\tNode is occupied!";
      else
        ss2 << "\tNode is not occupied!";
      ss2 << "\t Depth: " << it.getDepth() << " - " << "Size: " << it.getSize() <<
        " - Value: " << it->getValue() << " - Occupancy: " << it->getOccupancy() << std::endl;    
      */    
    }

    ROS_INFO("Removing vertices in and near holes");
    graph_->removeVerticesInHole();
    ROS_INFO("Removed");
    ROS_INFO("Constructing edges");
    try {
      graph_->constructEdges();
    } catch (...) {
      ROS_ERROR("Exception occured during edge construction...");
    }
    double duration = (double) (clock() - start) / CLOCKS_PER_SEC;
    ROS_INFO("# of cells: %d", counter);
    ROS_INFO("Done updating in %.5f secs", duration);
    ROS_INFO("Size of ADJ_map: %d", (int)graph_->adjacency_map_.size());
    if (save_adj_list_) {
      ROS_INFO("Saving adjacency_map_");
      graph_->saveAdjList();
      ROS_INFO("Saved");
    }
    continue_exploration_ = true;
    /*for (auto it = graph_->adjacency_map_.begin(), end = graph_->adjacency_map_.end();
          it != end; it++)
    {
      ROS_INFO("Size of outgoings: %d", (int)it->second.get()->outgoing_links_.size());
    }*/
    //graph_.printAdjList();
    //graph_.saveAdjList();
    /*const Vertex* result_vx = graph_.findPath(Vertex(5,3), Vertex(10,3));
    while (result_vx != nullptr) {
      std::cout << result_vx << " <- ";
      result_vx = result_vx->predecessor_vertex_;
    }
    std::cout << std::endl;
    ROS_INFO("Done\n");*/
    /*for (double ix = minX, end_ix = maxX; ix < end_ix; ix+=resolution) {
      for (double iy = minY, end_iy = maxY; iy < end_iy; iy+=resolution) {
        for (double iz = minZ, end_iz = maxZ; iz < end_iz; iz+=resolution) {
          octomap::OcTreeNode* cur_node = octomap->search(ix, iy, iz);
          if ( cur_node != NULL) {
            // std::cout << "Node at (" << ix << ", " << iy << ", " << iz << ") exists\n";
            ss << counter << ") Node at (" << ix << ", " << iy << ", " << iz << ") exists\n";
            // std::cout << "Value: " << cur_node->getValue();
            ss << "\tValue: " << cur_node->getValue() << std::endl;
            ss << "\tOccupancy: " << cur_node->getOccupancy() << std::endl;
          } else {
            // std::cout << "Node at (" << ix << ", " << iy << ", " << iz << ") does not exist\n";
            ss << "Node at (" << ix << ", " << iy << ", " << iz << ") does not exist\n";
          }

        }
      }
    }
    */
    /*for (octomap::OcTree::leaf_iterator it = octomap->begin_leafs(),
      end = octomap->end_leafs(); it != end; it++) {
      // if (octomap->isNodeOccupied(*it)) {
      ss << counter++ << ")\nNode center: " << it.getCoordinate() << std::endl;
      ss << "Node size: " << it.getSize() << std::endl;
      ss << "Node value: " << it->getValue() << std::endl;
      std::cout << counter++ << ")\nNode center: " << it.getCoordinate() << std::endl;
      std::cout << "Node size: " << it.getSize() << std::endl;
      std::cout << "Node value: " << it->getValue() << std::endl;
      std::cout << "occupancy: " << it->getOccupancy() << std::endl;
      // }

    }*/

    // fout1 << ss1.str();
    // fout2 << ss2.str();
    // delete octomap;
    waiting_fresh_octomap_ = false;
    last_octomap_update_time_ = ros::Time::now();
  }else {
    std::lock_guard<std::mutex> lock(this->octomap_mutex_);
    // ROS_WARN("Casting octomap..");
    if (this->octomap_ != nullptr)
      delete this->octomap_;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
      octomap_ = dynamic_cast<octomap::OcTree*>(tree);
      // ROS_INFO("Casted\n");
    }

    if (!octomap_) {
      ROS_ERROR("Failed to create octree structure\n");
      return;
    }
  }
}

void OctomapListener::is_neighborhood_clear(Point& _point) {
  while (check_neighborhood_ && ros::ok() ) {
    std::lock_guard<std::mutex> lock(this->octomap_mutex_);
    double cos_theta, sin_theta;
    double res = 0.05;
    double area_range = 0.4;
    double ground_height = graph_->getGroundHeight();
    double minX, minY, minZ, maxX, maxY, maxZ, odom_x, odom_y;
    odom_x = odom_.pose.pose.position.x;
    odom_y = odom_.pose.pose.position.y;
    octomap_->getMetricMin(minX, minY, minZ);
    octomap_->getMetricMax(maxX, maxY, maxZ);
    Point perp_vect(cos(3.14 - yaw_)/4, sin(3.14 - yaw_)/4);
    
    cos_theta = cos(yaw_);
    sin_theta = sin(yaw_);
    if (cos_theta > 0) {
      minX = odom_x + cos_theta - 0.1;
      maxX = odom_x + cos_theta + 0.1;
    } else {
      minX = odom_x + cos_theta - 0.1;
      maxX = odom_x + cos_theta + 0.1;
    }

    if (sin_theta > 0) {
      minY = odom_y  + sin_theta - 0.1;
      maxY = odom_y + sin_theta + 0.1;
    } else {
      minY = odom_y + sin_theta - 0.1;
      maxY = odom_y + sin_theta + 0.1;
    }

    /*minX = odom_x - 0.1; maxX = odom_x + 0.1;
    minY = odom_y - 0.1; maxY = odom_y + 0.1;*/


     // ROS_WARN("yaw: %.5f, x: %.5f, y: %.5f, minX: %.5f, minY:  %.5f, maxX: %.5f, maxY: %.5f", 
     //   yaw_, odom_x, odom_y, minX, minY, maxX, maxY);
   /* minX = odom_.pose.pose.position.x;// - area_range;
    maxX = odom_.pose.pose.position.x + area_range;
    minY = odom_.pose.pose.position.y;// - area_range;
    maxY = odom_.pose.pose.position.y + area_range;*/
    visualization_msgs::Marker neighborhood_markers;
    neighborhood_markers.header.frame_id = "diff_car/odom";
    neighborhood_markers.header.stamp = ros::Time::now();
    neighborhood_markers.ns = "neighborhood";
    neighborhood_markers.action = visualization_msgs::Marker::ADD;
    neighborhood_markers.pose.orientation.w = 1.0;
    neighborhood_markers.id = 0;
    neighborhood_markers.type = visualization_msgs::Marker::POINTS;
    neighborhood_markers.scale.x = 0.1;
    neighborhood_markers.scale.y = 0.1;

    neighborhood_markers.color.r = 1.0;
    neighborhood_markers.color.a = 1.0;
    for (double ix = minX, end_ix = maxX; ix < end_ix; ix+=res) {
      for (double iy = minY, end_iy = maxY; iy < end_iy; iy+=res) {
        /*double transformed_ix = cos_theta * (ix - odom_x) - sin_theta * (iy - odom_y);
        double transformed_iy = sin_theta * (ix - odom_x) + cos_theta * (iy - odom_y);
        transformed_ix += odom_x;
        transformed_iy += odom_y;*/
        // ROS_INFO("Yaw: %.4f\n Transformed from ix: %.4f, iy: %.4f to t_ix: %.4f,:"
          // "t_iy: %.4f", yaw_, ix, iy, transformed_ix, transformed_iy);
        geometry_msgs::Point n_point;
        n_point.x = ix; n_point.y = iy; n_point.z = odom_.pose.pose.position.z + 0.2;
        neighborhood_markers.points.push_back(n_point);
        for (double iz = minZ, end_iz = maxZ; iz < end_iz; iz+=res) {
          octomap::OcTreeNode* cur_node = octomap_->search(ix, iy, iz);
          if (cur_node != NULL && octomap_->isNodeOccupied(*cur_node)) {
            if ( (ground_height_ - iz) > 0.045) {
              ROS_ERROR("Near hole!!!");
              _point.x = ix;
              _point.y = iy;
              pub_marker_.publish(neighborhood_markers);
              this->neighborhood_notclear_ = true;
              return;
            }
            if ( fabs(odom_.pose.pose.position.z - iz) > 0.5) {
              ROS_ERROR("Near hill!!! z: %.5f, iz: %.5f", odom_.pose.pose.position.z, iz);
              _point.x = ix;
              _point.y = iy;
              pub_marker_.publish(neighborhood_markers);
              this->neighborhood_notclear_ = true;
              return;
            }
          }
        }
      }
    }
    // ROS_WARN("Clear area");
    pub_marker_.publish(neighborhood_markers);
    this->neighborhood_notclear_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  // ROS_ERROR("done checking");
}

void OctomapListener::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg) {
  
  start_exploration();

  /*std::cout << "Goal received\n";
  std::cout << "Frame_id: " << goal_msg->header.frame_id << "\nPose\n  Position:\n    ";
  std::cout << "x: " << goal_msg->pose.position.x << ", y: " << goal_msg->pose.position.y <<
  ", z: " << goal_msg->pose.position.z << std::endl;;
  std::cout << "  Quaternion:\n    " << "x: " << goal_msg->pose.orientation.x << ", y: " <<
  goal_msg->pose.orientation.y << ", z: " << goal_msg->pose.orientation.z << ", w: " <<
  goal_msg->pose.orientation.w << std::endl << std::endl;    
  double goal_pos_x = goal_msg->pose.position.x;
  double goal_pos_y = goal_msg->pose.position.y;
  // double theta = calculate_radians_forTurn(goal_pos_x, goal_pos_y);

  //lock.lock();
  VertexPtr result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                           Vertex(goal_pos_x,goal_pos_y));
  if (result_vx == nullptr) {
    adjust_direction_yaw(goal_pos_x, goal_pos_y);
    ros::Duration(2.0).sleep();
    waiting_fresh_octomap_ = true;
    ROS_WARN("Waiting for octomap to be updated");
    //lock.unlock();
    ros::Rate wait_octomap_rate(0.5);
    while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
   // lock.lock();
  }
  result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                 Vertex(goal_pos_x,goal_pos_y));
  //lock.unlock();
  std::stack<VertexPtr> path_to_goal;
  std::vector<VertexPtr> vect_path_to_goal;
  ROS_INFO("Creating stack of goals..");
  while (result_vx != nullptr) {
    path_to_goal.push(result_vx);
    vect_path_to_goal.push_back(result_vx);
    result_vx = result_vx->predecessor_vertex_;
  }

  visualization_msgs::Marker start_marker;
  start_marker.header.frame_id = "diff_car/odom";
  start_marker.header.stamp = ros::Time::now();
  start_marker.ns = "start_finish";
  start_marker.action = visualization_msgs::Marker::ADD;
  start_marker.pose.orientation.w = 1.0;
  start_marker.id = 0;
  start_marker.type = visualization_msgs::Marker::POINTS;
  start_marker.scale.x = 0.5;
  start_marker.scale.y = 0.5;

  start_marker.color.g = 1.0;
  start_marker.color.a = 1.0;
  geometry_msgs::Point p1, p2;
  p1.x = odom_.pose.pose.position.x;
  p1.y = odom_.pose.pose.position.y;
  p1.z = odom_.pose.pose.position.z + 0.2;
  p2.x = goal_pos_x;
  p2.y = goal_pos_y;
  p2.z = odom_.pose.pose.position.z + 0.2;
  start_marker.points.push_back(p1);
  start_marker.points.push_back(p2);
  pub_marker_.publish(start_marker);

  ROS_INFO("Publishing path..");
  nav_msgs::Path nav_msgs_path_to_goal;
  nav_msgs_path_to_goal.header.frame_id = std::string("diff_car/odom");
  nav_msgs_path_to_goal.header.stamp = ros::Time::now();
  int sequence = 1;
  // nav_msgs_path_to_goal.header.seq = sequence++;
  for (std::vector<VertexPtr>::const_iterator it = vect_path_to_goal.begin(), end= vect_path_to_goal.end();
          it != end ; it++) {
    VertexPtr vxptr_tmp = *it;
    geometry_msgs::PoseStamped goal_tmp;
    goal_tmp.header.stamp = ros::Time::now();
    goal_tmp.pose.position.x = vxptr_tmp.get()->get_center_point().x;
    goal_tmp.pose.position.y = vxptr_tmp.get()->get_center_point().y;
    goal_tmp.pose.position.z = graph_->getGroundHeight();
    goal_tmp.pose.orientation.x = 0.;
    goal_tmp.pose.orientation.y = 0.;
    goal_tmp.pose.orientation.z = 0.;
    goal_tmp.pose.orientation.w = 1.;
    nav_msgs_path_to_goal.poses.push_back(goal_tmp);
  }
  pub_path_.publish(nav_msgs_path_to_goal);
  adjust_direction_yaw(goal_pos_x, goal_pos_y);
  ROS_INFO("Done");
  geometry_msgs::Twist twist_msg_tmp;
  int counter = 0;
  int hole_counter = 0;
  while (!path_to_goal.empty() && ros::ok())
  {
    VertexPtr current_goal_vx = path_to_goal.top();
    path_to_goal.pop();
    ros::Rate pub_rate(5);
    std::cout << "Moving to goal: " << current_goal_vx.get() << std::endl;
    double distance = get_distance(Point(odom_.pose.pose.position.x, odom_.pose.pose.position.y),
                                          current_goal_vx.get()->get_center_point());
    
    Point nonclear_point;
    this->check_neighborhood_ = true;
    std::thread neighborhood_thread(&OctomapListener::is_neighborhood_clear,
                                      this, std::ref(nonclear_point));
    if (distance > DISTANCE_THRESHOLD)
      counter++;
    while (distance > DISTANCE_THRESHOLD && ros::ok() && !this->neighborhood_notclear_) 
    {
      double goal_x, goal_y;
      goal_x = current_goal_vx.get()->get_center_point().x;
      goal_y = current_goal_vx.get()->get_center_point().y;
      Point vect_org_target;
      vect_org_target.x = goal_x - odom_.pose.pose.position.x;
      vect_org_target.y = goal_y - odom_.pose.pose.position.y;
      double linear_x, angular_z;
      linear_x = sqrt(vect_org_target.x * vect_org_target.x +
                                      vect_org_target.y * vect_org_target.y);
      angular_z = get_yaw_angle(goal_x, goal_y);
      if (fabs(angular_z) > 0.7)
        adjust_direction_yaw(goal_x, goal_y);
      linear_x = linear_x >= 0.3 ? 0.3 : linear_x;
      angular_z = angular_z >= 1. ? 1 : angular_z; 
      angular_z = angular_z <= -1. ? -1 : angular_z;
      if (fabs(angular_z) - 0.4 >= 1e-2)
        linear_x = 0.25 * linear_x;
      twist_msg_tmp.linear.x = linear_x;
      twist_msg_tmp.angular.z = angular_z;
      ROS_INFO("Publishing: x: %.4f, z: %.4f", twist_msg_tmp.linear.x, twist_msg_tmp.angular.z);
      pub_cmdvel_.publish(twist_msg_tmp);
      pub_rate.sleep();
      double new_distance = get_distance(Point(odom_.pose.pose.position.x, odom_.pose.pose.position.y),
                                          current_goal_vx.get()->get_center_point());
      if ( new_distance > distance) {
        counter = 3;
        break;
      }
      distance = new_distance;
    }

    twist_msg_tmp.linear.x = 0; twist_msg_tmp.angular.z = 0;
    pub_cmdvel_.publish(twist_msg_tmp);
    this->check_neighborhood_ = false;
    neighborhood_thread.join();
    if (this->neighborhood_notclear_) {
      this->neighborhood_notclear_ = false;
      ROS_ERROR("after near hole");
      hole_counter++;
      waiting_fresh_octomap_ = true;
      twist_msg_tmp.linear.x = -0.2;
      twist_msg_tmp.angular.z = 0.;
      pub_cmdvel_.publish(twist_msg_tmp);
      ros::Duration(1.75).sleep();
      twist_msg_tmp.linear.x = 0.;
      pub_cmdvel_.publish(twist_msg_tmp);
      ros::Rate wait_octomap_rate(0.5);
      while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
      Vertex nonclear_vx(nonclear_point);
      graph_->removeVertex(nonclear_vx);
      // ROS_WARN("Not reached");
      counter = 0;
      result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                 Vertex(goal_pos_x,goal_pos_y));
      //lock.unlock();
      if (result_vx == nullptr)
        result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                   Vertex(goal_pos_x,goal_pos_y));
      if (result_vx == nullptr) {
        waiting_fresh_octomap_ = true;
        ROS_WARN("Waiting for octomap to be updated");
        //lock.unlock();
        ros::Rate wait_octomap_rate(0.5);
        while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
       // lock.lock();
      }
      result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                   Vertex(goal_pos_x,goal_pos_y));
      path_to_goal = std::stack<VertexPtr>();
      while (result_vx != nullptr) {
        path_to_goal.push(result_vx);
        result_vx = result_vx->predecessor_vertex_;
      }
      //nav_msgs_path_to_goal = nav_msgs::Path();
      nav_msgs_path_to_goal.header.frame_id = std::string("diff_car/odom");
      nav_msgs_path_to_goal.header.stamp = ros::Time::now();
      // nav_msgs_path_to_goal.header.seq = sequence++;
      nav_msgs_path_to_goal.poses = std::vector<geometry_msgs::PoseStamped>();
      for (std::vector<VertexPtr>::const_iterator it = vect_path_to_goal.begin(), end= vect_path_to_goal.end();
              it != end ; it++) {
        VertexPtr vxptr_tmp = *it;
        geometry_msgs::PoseStamped goal_tmp;
        goal_tmp.header.stamp = ros::Time::now();
        goal_tmp.pose.position.x = vxptr_tmp.get()->get_center_point().x;
        goal_tmp.pose.position.y = vxptr_tmp.get()->get_center_point().y;
        goal_tmp.pose.position.z = graph_->getGroundHeight();
        goal_tmp.pose.orientation.x = 0.;
        goal_tmp.pose.orientation.y = 0.;
        goal_tmp.pose.orientation.z = 0.;
        goal_tmp.pose.orientation.w = 1.;
        nav_msgs_path_to_goal.poses.push_back(goal_tmp);
      }
      pub_path_.publish(nav_msgs_path_to_goal);
    } 

    if (counter == 7) {
      ROS_WARN("Recreating");
      //lock.lock();
      counter = 0;
      result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                 Vertex(goal_pos_x,goal_pos_y));
      //lock.unlock();
      if (result_vx == nullptr)
        result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                   Vertex(goal_pos_x,goal_pos_y));
      if (result_vx == nullptr) {
        waiting_fresh_octomap_ = true;
        ROS_WARN("Waiting for octomap to be updated");
        //lock.unlock();
        ros::Rate wait_octomap_rate(0.5);
        while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
       // lock.lock();
      }
      result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                   Vertex(goal_pos_x,goal_pos_y));
      path_to_goal = std::stack<VertexPtr>();
      while (result_vx != nullptr) {
        path_to_goal.push(result_vx);
        result_vx = result_vx->predecessor_vertex_;
      }
      //nav_msgs_path_to_goal = nav_msgs::Path();
      nav_msgs_path_to_goal.header.frame_id = std::string("diff_car/odom");
      nav_msgs_path_to_goal.header.stamp = ros::Time::now();
      // nav_msgs_path_to_goal.header.seq = sequence++;
      nav_msgs_path_to_goal.poses = std::vector<geometry_msgs::PoseStamped>();
      for (std::vector<VertexPtr>::const_iterator it = vect_path_to_goal.begin(), end= vect_path_to_goal.end();
              it != end ; it++) {
        VertexPtr vxptr_tmp = *it;
        geometry_msgs::PoseStamped goal_tmp;
        goal_tmp.header.stamp = ros::Time::now();
        goal_tmp.pose.position.x = vxptr_tmp.get()->get_center_point().x;
        goal_tmp.pose.position.y = vxptr_tmp.get()->get_center_point().y;
        goal_tmp.pose.position.z = graph_->getGroundHeight();
        goal_tmp.pose.orientation.x = 0.;
        goal_tmp.pose.orientation.y = 0.;
        goal_tmp.pose.orientation.z = 0.;
        goal_tmp.pose.orientation.w = 1.;
        nav_msgs_path_to_goal.poses.push_back(goal_tmp);
      }
      pub_path_.publish(nav_msgs_path_to_goal);
    }
  }

  ROS_WARN("Goal reached!");*/
  // lock.unlock();
/*
  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac(action_client_name_, true);

  // wait for the activation server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.orientation.x = goal_msg->pose.orientation.x;
  goal.target_pose.pose.orientation.y = goal_msg->pose.orientation.y;
  goal.target_pose.pose.orientation.z = goal_msg->pose.orientation.z;
  goal.target_pose.pose.orientation.w = goal_msg->pose.orientation.w;
  bool ok_to_go = true;
  while (!path_to_goal.empty() && ok_to_go && ros::ok()) {
    VertexPtr current_goal_vx = path_to_goal.top();
    path_to_goal.pop();
    goal.target_pose.pose.position.x = current_goal_vx.get()->get_center_point().x;
    goal.target_pose.pose.position.y = current_goal_vx.get()->get_center_point().y;
    adjust_direction_yaw(current_goal_vx.get()->get_center_point().x, current_goal_vx.get()->get_center_point().y);
    std::cout << "Sending goal = " << current_goal_vx.get() << std::endl;
    ac.sendGoal(goal);
    ac.waitForResult();
    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_ERROR("Failed to reach destination.");
      ok_to_go = false;
    }
  }*/

}

void OctomapListener::odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
  // std::cout << "Odom received\n";
  ros::Time current_time = ros::Time::now();
  double seconds_since_last_update = current_time.toSec() - last_odom_update_time_.toSec();
  if ( seconds_since_last_update >= odom_update_period_) {
    odom_ = *odom_msg;
    // std::cout << "Frame_id: " << odom_msg->header.frame_id << std::endl;
    double x = odom_msg->pose.pose.orientation.x;
    double y = odom_msg->pose.pose.orientation.y;
    double z = odom_msg->pose.pose.orientation.z;
    double w = odom_msg->pose.pose.orientation.w;
    /*std::cout << "Position: x = " << odom_msg->pose.pose.position.x << ", y = " <<
    odom_msg->pose.pose.position.y << ", z = " << odom_msg->pose.pose.position.z << std::endl;
    std::cout << "Orientation: x: " << odom_msg->pose.pose.orientation.x << ", y: " << 
    odom_msg->pose.pose.orientation.y << ", z: " << odom_msg->pose.pose.orientation.z
     << ", w: " << odom_msg->pose.pose.orientation.w << std::endl;*/ 
     tf::Quaternion qt = tf::Quaternion(x, y, z, w);
     tf::Matrix3x3 m(qt);
     double roll, pitch, yaw;
     m.getRPY(roll, pitch, yaw);
     yaw_ = yaw;
     // std::cout << "yaw: " << yaw_ << std::endl;
     // std::cout << "roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
     last_odom_update_time_ = ros::Time::now();
  }
}

double OctomapListener::getGraphVertexRes() const {
  return this->graph_vertex_res_;
}

void OctomapListener::search_front() {
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0.; 
  twist_msg.angular.z = -0.3;
  pub_cmdvel_.publish(twist_msg);
  ros::Duration(1.5).sleep();
  twist_msg.angular.z = -twist_msg.angular.z;
  pub_cmdvel_.publish(twist_msg);
  ros::Duration(3).sleep();
  twist_msg.angular.z = 0.;
  pub_cmdvel_.publish(twist_msg);
}

void OctomapListener::investigate() const {
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0.;
  twist_msg.angular.z = 0.4;
  pub_cmdvel_.publish(twist_msg);
  ros::Duration(6).sleep();
  twist_msg.angular.z = 0.;
  pub_cmdvel_.publish(twist_msg);
}

void OctomapListener::start_exploration() {
  ros::Rate wait_rate(0.5);
  while (ros::ok() && !continue_exploration_) wait_rate.sleep();
  search_front();
  ros::Rate explore_rate(1);
  while (ros::ok()) {
    bool empty = false;
    Vertex goal_vx = graph_->getNearestDiamond(Vertex(odom_.pose.pose.position.x, odom_.pose.pose.position.y), empty);
    if (empty) {
      ROS_WARN("Empty diamond..Waiting for octomap to be updated");
      investigate();
      waiting_fresh_octomap_ = true;
      //lock.unlock();
      ros::Rate wait_octomap_rate(0.5);
      while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
      continue;
    }
    // double theta = calculate_radians_forTurn(goal_pos_x, goal_pos_y);

    adjust_direction_yaw(goal_vx.get_center_point().x, goal_vx.get_center_point().y);
    //lock.lock();
    VertexPtr result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                             goal_vx);
    if (result_vx == nullptr) {
      waiting_fresh_octomap_ = true;
      ROS_WARN("Waiting for octomap to be updated");
      //lock.unlock();
      ros::Rate wait_octomap_rate(0.5);
      while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
     // lock.lock();
    }
    result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                   goal_vx);
    //lock.unlock();
    std::stack<VertexPtr> path_to_goal;
    std::vector<VertexPtr> vect_path_to_goal;
    ROS_INFO("Creating stack of goals..");
    while (result_vx != nullptr) {
      path_to_goal.push(result_vx);
      vect_path_to_goal.push_back(result_vx);
      result_vx = result_vx->predecessor_vertex_;
    }

    visualization_msgs::Marker start_marker;
    start_marker.header.frame_id = "diff_car/odom";
    start_marker.header.stamp = ros::Time::now();
    start_marker.ns = "start_finish";
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.pose.orientation.w = 1.0;
    start_marker.id = 0;
    start_marker.type = visualization_msgs::Marker::POINTS;
    start_marker.scale.x = 0.5;
    start_marker.scale.y = 0.5;

    start_marker.color.g = 1.0;
    start_marker.color.a = 1.0;
    geometry_msgs::Point p1, p2;
    p1.x = odom_.pose.pose.position.x;
    p1.y = odom_.pose.pose.position.y;
    p1.z = odom_.pose.pose.position.z + 0.2;
    p2.x = goal_vx.get_center_point().x;
    p2.y = goal_vx.get_center_point().y;
    p2.z = odom_.pose.pose.position.z + 0.2;
    start_marker.points.push_back(p1);
    start_marker.points.push_back(p2);
    pub_marker_.publish(start_marker);

    ROS_INFO("Publishing path..");
    nav_msgs::Path nav_msgs_path_to_goal;
    nav_msgs_path_to_goal.header.frame_id = std::string("diff_car/odom");
    nav_msgs_path_to_goal.header.stamp = ros::Time::now();
    int sequence = 1;
    // nav_msgs_path_to_goal.header.seq = sequence++;
    for (std::vector<VertexPtr>::const_iterator it = vect_path_to_goal.begin(), end= vect_path_to_goal.end();
            it != end ; it++) 
    {
      VertexPtr vxptr_tmp = *it;
      geometry_msgs::PoseStamped goal_tmp;
      goal_tmp.header.stamp = ros::Time::now();
      goal_tmp.pose.position.x = vxptr_tmp.get()->get_center_point().x;
      goal_tmp.pose.position.y = vxptr_tmp.get()->get_center_point().y;
      goal_tmp.pose.position.z = graph_->getGroundHeight();
      goal_tmp.pose.orientation.x = 0.;
      goal_tmp.pose.orientation.y = 0.;
      goal_tmp.pose.orientation.z = 0.;
      goal_tmp.pose.orientation.w = 1.;
      nav_msgs_path_to_goal.poses.push_back(goal_tmp);
    }
    pub_path_.publish(nav_msgs_path_to_goal);

    adjust_direction_yaw(goal_vx.get_center_point().x, goal_vx.get_center_point().y);
    ROS_INFO("Done");
    geometry_msgs::Twist twist_msg_tmp;
    int counter = 0;
    int hole_counter = 0;
    while (!path_to_goal.empty() && ros::ok())
    {
      VertexPtr current_goal_vx = path_to_goal.top();
      path_to_goal.pop();
      ros::Rate pub_rate(5);
      std::cout << "Moving to goal: " << current_goal_vx.get() << std::endl;
      double distance = get_distance(Point(odom_.pose.pose.position.x, odom_.pose.pose.position.y),
                                            current_goal_vx.get()->get_center_point());
      /*
      if (current_goal_vx.get()->cell_size_ < 50) {
        waiting_fresh_octomap_ = true;
        ROS_WARN("Unknown vertex.. Waiting for fresh octomap..");
        ros::Rate wait_octomap_rate(0.5);
        while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
        result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                     Vertex(goal_pos_x,goal_pos_y));
        path_to_goal = std::stack<VertexPtr>();
        while (result_vx != nullptr) {
          path_to_goal.push(result_vx);
          result_vx = result_vx->predecessor_vertex_;
        }
      }*/
      geometry_msgs::Twist twist_msg_tmp;
      Point nonclear_point;
      this->check_neighborhood_ = true;
      std::thread neighborhood_thread(&OctomapListener::is_neighborhood_clear,
                                        this, std::ref(nonclear_point));
      if (distance > DISTANCE_THRESHOLD)
        counter++;
      while (distance > DISTANCE_THRESHOLD && ros::ok() && !this->neighborhood_notclear_) 
      {
        double goal_x, goal_y;
        goal_x = current_goal_vx.get()->get_center_point().x;
        goal_y = current_goal_vx.get()->get_center_point().y;
        Point vect_org_target;
        vect_org_target.x = goal_x - odom_.pose.pose.position.x;
        vect_org_target.y = goal_y - odom_.pose.pose.position.y;
        double linear_x, angular_z;
        linear_x = sqrt(vect_org_target.x * vect_org_target.x +
                                        vect_org_target.y * vect_org_target.y);
        angular_z = get_yaw_angle(goal_x, goal_y);
        if (fabs(angular_z) > 0.7)
          adjust_direction_yaw(goal_x, goal_y);
        linear_x = linear_x >= 0.3 ? 0.3 : linear_x;
        angular_z = angular_z >= 1. ? 1 : angular_z; 
        angular_z = angular_z <= -1. ? -1 : angular_z;
        if (fabs(angular_z) - 0.4 >= 1e-2)
          linear_x = 0.25 * linear_x;
        twist_msg_tmp.linear.x = linear_x;
        twist_msg_tmp.angular.z = angular_z;
        ROS_INFO("Publishing: x: %.4f, z: %.4f", twist_msg_tmp.linear.x, twist_msg_tmp.angular.z);
        pub_cmdvel_.publish(twist_msg_tmp);
        pub_rate.sleep();
        double new_distance = get_distance(Point(odom_.pose.pose.position.x, odom_.pose.pose.position.y),
                                            current_goal_vx.get()->get_center_point());
        if ( new_distance > distance) {
          counter = 3;
          break;
        }
        distance = new_distance;
      }

      twist_msg_tmp.linear.x = 0; twist_msg_tmp.angular.z = 0;
      pub_cmdvel_.publish(twist_msg_tmp);
      this->check_neighborhood_ = false;
      neighborhood_thread.join();
      if (this->neighborhood_notclear_) {
        this->neighborhood_notclear_ = false;
        ROS_ERROR("after near hole");
        hole_counter++;
        waiting_fresh_octomap_ = true;
        twist_msg_tmp.linear.x = -0.2;
        twist_msg_tmp.angular.z = 0.;
        pub_cmdvel_.publish(twist_msg_tmp);
        ros::Duration(1.25).sleep();
        twist_msg_tmp.linear.x = 0.;
        pub_cmdvel_.publish(twist_msg_tmp);
        ros::Rate wait_octomap_rate(0.5);
        while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
        Vertex nonclear_vx(nonclear_point);
        graph_->removeVertex(nonclear_vx);
        // ROS_WARN("Not reached");
        counter = 0;
        result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                   goal_vx);
        //lock.unlock();
        if (result_vx == nullptr)
          result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                     goal_vx);
        if (result_vx == nullptr) {
          waiting_fresh_octomap_ = true;
          ROS_WARN("Waiting for octomap to be updated");
          //lock.unlock();
          ros::Rate wait_octomap_rate(0.5);
          while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
         // lock.lock();
        }
        result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                     goal_vx);
        path_to_goal = std::stack<VertexPtr>();
        while (result_vx != nullptr) {
          path_to_goal.push(result_vx);
          result_vx = result_vx->predecessor_vertex_;
        }
        //nav_msgs_path_to_goal = nav_msgs::Path();
        nav_msgs_path_to_goal.header.frame_id = std::string("diff_car/odom");
        nav_msgs_path_to_goal.header.stamp = ros::Time::now();
        // nav_msgs_path_to_goal.header.seq = sequence++;
        nav_msgs_path_to_goal.poses = std::vector<geometry_msgs::PoseStamped>();
        for (std::vector<VertexPtr>::const_iterator it = vect_path_to_goal.begin(), end= vect_path_to_goal.end();
                it != end ; it++) {
          VertexPtr vxptr_tmp = *it;
          geometry_msgs::PoseStamped goal_tmp;
          goal_tmp.header.stamp = ros::Time::now();
          goal_tmp.pose.position.x = vxptr_tmp.get()->get_center_point().x;
          goal_tmp.pose.position.y = vxptr_tmp.get()->get_center_point().y;
          goal_tmp.pose.position.z = graph_->getGroundHeight();
          goal_tmp.pose.orientation.x = 0.;
          goal_tmp.pose.orientation.y = 0.;
          goal_tmp.pose.orientation.z = 0.;
          goal_tmp.pose.orientation.w = 1.;
          nav_msgs_path_to_goal.poses.push_back(goal_tmp);
        }
        pub_path_.publish(nav_msgs_path_to_goal);
      } 
      /*else if (hole_counter >= 5) {
        twist_msg_tmp.linear.x = -0.2;
        twist_msg_tmp.angular.z = 0.;
        pub_cmdvel_.publish(twist_msg_tmp);
        ros::Duration(1.5).sleep();
        twist_msg_tmp.linear.x = 0.;
        pub_cmdvel_.publish(twist_msg_tmp);
        ROS_WARN("Not reached");
        return;
      }*/

      if (counter == 7) {
        ROS_WARN("Recreating");
        //lock.lock();
        counter = 0;
        result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                   goal_vx);
        //lock.unlock();
        if (result_vx == nullptr)
          result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                     goal_vx);
        if (result_vx == nullptr) {
          waiting_fresh_octomap_ = true;
          ROS_WARN("Waiting for octomap to be updated");
          //lock.unlock();
          ros::Rate wait_octomap_rate(0.5);
          while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
         // lock.lock();
        }
        result_vx = graph_->findPath(Vertex(odom_.pose.pose.position.x,odom_.pose.pose.position.y),
                                     goal_vx);
        path_to_goal = std::stack<VertexPtr>();
        while (result_vx != nullptr) {
          path_to_goal.push(result_vx);
          result_vx = result_vx->predecessor_vertex_;
        }
        //nav_msgs_path_to_goal = nav_msgs::Path();
        nav_msgs_path_to_goal.header.frame_id = std::string("diff_car/odom");
        nav_msgs_path_to_goal.header.stamp = ros::Time::now();
        // nav_msgs_path_to_goal.header.seq = sequence++;
        nav_msgs_path_to_goal.poses = std::vector<geometry_msgs::PoseStamped>();
        for (std::vector<VertexPtr>::const_iterator it = vect_path_to_goal.begin(), end= vect_path_to_goal.end();
                it != end ; it++) {
          VertexPtr vxptr_tmp = *it;
          geometry_msgs::PoseStamped goal_tmp;
          goal_tmp.header.stamp = ros::Time::now();
          goal_tmp.pose.position.x = vxptr_tmp.get()->get_center_point().x;
          goal_tmp.pose.position.y = vxptr_tmp.get()->get_center_point().y;
          goal_tmp.pose.position.z = graph_->getGroundHeight();
          goal_tmp.pose.orientation.x = 0.;
          goal_tmp.pose.orientation.y = 0.;
          goal_tmp.pose.orientation.z = 0.;
          goal_tmp.pose.orientation.w = 1.;
          nav_msgs_path_to_goal.poses.push_back(goal_tmp);
        }
        pub_path_.publish(nav_msgs_path_to_goal);
      }
    }

  ROS_WARN("Goal reached!");
  }
}

void OctomapListener::start_navigation() {
  int goal_stack_index = 0;
  bool reached_destination = false;
  bool unable_to_reach = false;
  // while (!reached_destination && !unable_to_reach) {
    while (is_distance_large(goal_stack_index)) {
      geometry_msgs::PoseStamped new_goal = get_new_goal(goal_stack_[goal_stack_index]);
      std::cout << "Distance is large..\n";
      goal_stack_.push_back(new_goal);
      std::cout << "New goal: (x: " << new_goal.pose.position.x << ", y: " << new_goal.pose.position.y << ")\n";
      goal_stack_index++;
    }
  // }

  std::cout << "Goal stack contains..\n";
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = goal_stack_.end() - 1, 
    end = goal_stack_.begin(); it >= end; it--) {
    std::cout << "x: " << it->pose.position.x << ", y: " << it->pose.position.y << std::endl;
  }
}

bool OctomapListener::is_front_clear(const geometry_msgs::PoseStamped& _current_goal) {
  double pos_x = odom_.pose.pose.position.x;
  double pos_y = odom_.pose.pose.position.y;
  double bbx_left_x = -sin(yaw_) * bias_for_bbx + pos_x;
  double bbx_left_y = cos(yaw_) * bias_for_bbx + pos_y;
  double bbx_right_x = sin(yaw_) * bias_for_bbx + pos_x;
  double bbx_right_y = -cos(yaw_) * bias_for_bbx + pos_y;
  double bbx_end_left_x, bbx_end_right_x, bbx_end_left_y, bbx_end_right_y;
  bbx_end_left_x = _current_goal.pose.position.x + (bbx_left_x - pos_x);
  bbx_end_left_y = _current_goal.pose.position.y + (bbx_left_y - pos_y);
  bbx_end_right_x = _current_goal.pose.position.x  + (bbx_right_x - pos_x);
  bbx_end_right_y = _current_goal.pose.position.y + (bbx_right_y - pos_y);
  ROS_INFO("Found BBX: left_x: %.4lf, left_y: %.4lf, right_x: %.4lf, right_y: %.4lf,"
    "end_left_x: %.4lf, end_left_y: %.4lf, end_right_x: %.4lf, end_right_y: %.4lf\n", 
    bbx_left_x, bbx_left_y, bbx_right_x, bbx_right_y, bbx_end_left_x, bbx_end_left_y,
    bbx_end_right_x, bbx_end_right_y);
  return true;
}

double OctomapListener::get_distance(const Point& _p1, const Point& _p2) const {
  double x_d, y_d;
  x_d = _p2.x - _p1.x;
  y_d = _p2.y - _p1.y;
  double dist = sqrt(x_d*x_d + y_d*y_d);
  ROS_INFO("Distance: %.5f", dist);
  return dist;
  //return sqrt(x_d*x_d + y_d*y_d);
}

bool OctomapListener::is_distance_large(int _stack_index) {
  double x_tmp = goal_stack_[_stack_index].pose.position.x - odom_.pose.pose.position.x;
  double y_tmp = goal_stack_[_stack_index].pose.position.y - odom_.pose.pose.position.y;
  double distance = sqrt(x_tmp*x_tmp + y_tmp*y_tmp);
  return distance > DISTANCE_THRESHOLD;
}

geometry_msgs::PoseStamped OctomapListener::get_new_goal(const geometry_msgs::PoseStamped& _last_goal) {
  geometry_msgs::PoseStamped* new_goal = new geometry_msgs::PoseStamped();
  new_goal->pose.position.x = (_last_goal.pose.position.x + odom_.pose.pose.position.x) / 2;
  new_goal->pose.position.y = (_last_goal.pose.position.y + odom_.pose.pose.position.y) / 2;
  new_goal->header.frame_id = "goal";
  new_goal->pose.orientation.x = new_goal->pose.orientation.y = new_goal->pose.orientation.z = 0;
  new_goal->pose.orientation.w = 1;
  return *new_goal;
}

/* Converts the direction of robot towards the target until RADIANS_TO_TURN_THRESHOLD is satisfied */
void OctomapListener::adjust_direction_yaw(double goal_pos_x, double goal_pos_y) {
  
  geometry_msgs::Twist twist_msg_tmp;

  if (this->neighborhood_notclear_) {
    this->neighborhood_notclear_ = false;
    ROS_ERROR("after near hole");
    waiting_fresh_octomap_ = true;
    twist_msg_tmp.linear.x = -0.2;
    twist_msg_tmp.angular.z = 0.;
    pub_cmdvel_.publish(twist_msg_tmp);
    ros::Duration(1.25).sleep();
    twist_msg_tmp.linear.x = 0.;
    pub_cmdvel_.publish(twist_msg_tmp);
    ros::Rate wait_octomap_rate(0.5);
    while (waiting_fresh_octomap_ && ros::ok()) wait_octomap_rate.sleep();
  }
  if (get_distance(Point(odom_.pose.pose.position.x, odom_.pose.pose.position.y), 
                    Point(goal_pos_x, goal_pos_y)) < threshold_distance_)
  {
    twist_msg_tmp.linear.x =  -0.15;
    twist_msg_tmp.angular.z = 0;
    pub_cmdvel_.publish(twist_msg_tmp);
    ros::Duration(0.5).sleep();
    twist_msg_tmp.linear.x =  0.0;
    twist_msg_tmp.angular.z = 0.0;
    pub_cmdvel_.publish(twist_msg_tmp);
    adjust_direction_yaw(goal_pos_x, goal_pos_y);
  } 
  else  {
    twist_msg_tmp.linear.x = 0.0;
    ros::Rate loop_rate(10);
    while (RADIANS_TO_TURN_THRESHOLD < calculate_radians_forTurn(goal_pos_x, goal_pos_y) && ros::ok()) {
      double rotation_tmp = get_direction_yaw(goal_pos_x, goal_pos_y);
      twist_msg_tmp.angular.z = rotation_tmp * YAW_ROTATION_SPEED;
      // std::cout << "Publishing angular: " << twist_msg_tmp.angular.z << std::endl;
      pub_cmdvel_.publish(twist_msg_tmp);
      loop_rate.sleep();
    }
    twist_msg_tmp.angular.z = 0.0;
    pub_cmdvel_.publish(twist_msg_tmp);
    ROS_INFO("\nDone adjusting robot's direction!\n");
  }
}

/* Calculates the angle between the direction which the robot is currently heading to and the target direction (in radians) */
double OctomapListener::calculate_radians_forTurn(double _x, double _y) {
  double cur_pos_x = odom_.pose.pose.position.x;
  double cur_pos_y = odom_.pose.pose.position.y;
  // Target direction with respect to current position of robot = target - current_pose
  double tx_wrt_poseOrg = _x - cur_pos_x;
  double ty_wrt_poseOrg = _y - cur_pos_y;
  double dir_x = cos(yaw_);
  double dir_y = sin(yaw_);
  // std::cout << "dir_x: " << dir_x << ", dir_y: " << dir_y << std::endl;
  // Cosine formula to find degree between direction the robot is heading to and target direction
  double numerator = tx_wrt_poseOrg * dir_x + ty_wrt_poseOrg * dir_y;
  double denominator = sqrt(tx_wrt_poseOrg * tx_wrt_poseOrg + ty_wrt_poseOrg * ty_wrt_poseOrg) * sqrt(dir_x * dir_x + dir_y * dir_y);
  double radians_to_turn = acos(numerator / denominator);
  // std::cout << "Radian to turn: " << radians_to_turn << std::endl;
  return radians_to_turn; 
}

/* returns whether the robot should turn left(1) or right(-1) to adjust its direction towards target */
double OctomapListener::get_direction_yaw(double _x, double _y) {
  double cur_pos_x = odom_.pose.pose.position.x;
  double cur_pos_y = odom_.pose.pose.position.y;
  double dir_point_x = cur_pos_x + cos(yaw_);
  double dir_point_y = cur_pos_y + sin(yaw_);

  double vector_org_to_dir_x = dir_point_x - cur_pos_x;
  double vector_org_to_dir_y = dir_point_y - cur_pos_y;
  double vector_org_to_target_x = _x - cur_pos_x;
  double vector_org_to_target_y = _y - cur_pos_y;
  /*  exploits vector product between the vectors (v1 = from current_position to direction v2 = from current_position to target)
     to figure out whether the target lies on the left or right of the robot */
  double result = vector_org_to_dir_x * vector_org_to_target_y - vector_org_to_dir_y * vector_org_to_target_x;
   // std::cout << "Result for rotation: " << result << std::endl;
  if ( result < 0)
    return -1.0;
  else 
    return 1.0;
}

double OctomapListener::get_yaw_angle(double _x, double _y) {
  double cur_pos_x, cur_pos_y, dir_point_x, dir_point_y;
  cur_pos_x = odom_.pose.pose.position.x;
  cur_pos_y = odom_.pose.pose.position.y;
  dir_point_x = cur_pos_x + cos(yaw_);
  dir_point_y = cur_pos_y + sin(yaw_);
  double vector_org_to_dir_x, vector_org_to_dir_y, vector_org_to_target_x, vector_org_to_target_y;
  vector_org_to_dir_x = dir_point_x - cur_pos_x;
  vector_org_to_dir_y = dir_point_y - cur_pos_y;
  vector_org_to_target_x = _x - cur_pos_x;
  vector_org_to_target_y = _y - cur_pos_y;
  return vector_org_to_dir_x * vector_org_to_target_y - vector_org_to_dir_y * vector_org_to_target_x;
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "octomap_listener");
  ros::NodeHandle nh;

  OctomapListener octomap_listener(nh);
  ros::NodeHandle priv_nh("~");
  std::string goal_topic_name, odom_topic_name, octomap_topic_name;
  priv_nh.param("goal_topic_name", goal_topic_name, std::string("/move_base_simple/goal"));
  priv_nh.param("octomap_topic_name", octomap_topic_name, std::string("/octomap_binary"));
  priv_nh.param("odom_topic_name", odom_topic_name, std::string("/diff_car/odom"));
  ros::Subscriber subOctomap = nh.subscribe<octomap_msgs::Octomap>(octomap_topic_name, 250, 
    &OctomapListener::octomap_callback, &octomap_listener);
  ros::Subscriber subGoal_ = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic_name, 250,
    &OctomapListener::goal_callback, &octomap_listener);
  ros::Subscriber subOdom_ = nh.subscribe<nav_msgs::Odometry>(odom_topic_name, 200, 
    &OctomapListener::odom_callback, &octomap_listener);
/*
  ros::Subscriber sub_ = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 250, 
      boost::bind(&OctomapListener::octomap_callback, octomap_listener, _1));

  ros::Subscriber subGoal_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 250,
    boost::bind(&OctomapListener::goal_callback, octomap_listener, _1) );
  ros::Subscriber subOdom_ = nh.subscribe<nav_msgs::Odometry>("/diff_car/odom", 200, 
    boost::bind(&OctomapListener::odom_callback, octomap_listener, _1));
*/
  /*ros::MultiThreadedSpinner spinner(5);
  spinner.spin();*/
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}