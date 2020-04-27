#include <ceres/ceres.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Dense>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "ceres_pose_graph_3d.hpp"

#define PUB_SURROUND_PTS 1
#define PCD_SAVE_RAW 1
#define PUB_DEBUG_INFO 0
#define IF_PUBLISH_SURFACE_AND_CORNER_PTS 0

int g_if_undistore = 0;

int if_motion_deblur = 0;
double history_add_t_step = 0.00;
double history_add_angle_step = 0.00;

nav_msgs::Path m_laser_after_mapped_path, m_laser_after_loopclosure_path;

Ceres_pose_graph_3d::MapOfPoses pose3d_map, pose3d_map_ori;
Ceres_pose_graph_3d::VectorOfPose pose3d_vec;
Ceres_pose_graph_3d::VectorOfConstraints constrain_vec;

ros::Publisher m_pub_laser_aft_loopclosure_path;

static Ceres_pose_graph_3d::Constraint3d add_constrain_of_loop(
    int s_idx, int t_idx, Eigen::Quaterniond q_a, Eigen::Vector3d t_a,
    Eigen::Quaterniond q_b, Eigen::Vector3d t_b, Eigen::Quaterniond icp_q,
    Eigen::Vector3d icp_t, int if_verbose = 1) {
  Ceres_pose_graph_3d::Constraint3d pose_constrain;
  auto q_res = q_b.inverse() * icp_q.inverse() * q_a;
  auto t_res = q_b.inverse() * (icp_q.inverse() * (t_a - icp_t) - t_b);

  if (if_verbose == 0) {
    std::cout << "=== Add_constrain_of_loop ====" << std::endl;
    std::cout << q_a.coeffs().transpose() << std::endl;
    std::cout << q_b.coeffs().transpose() << std::endl;
    std::cout << icp_q.coeffs().transpose() << std::endl;
    std::cout << t_a.transpose() << std::endl;
    std::cout << t_b.transpose() << std::endl;
    std::cout << icp_t.transpose() << std::endl;
    std::cout << "Result: " << std::endl;
    std::cout << q_res.coeffs().transpose() << std::endl;
    std::cout << t_res.transpose() << std::endl;
  }
  pose_constrain.id_begin = s_idx;
  pose_constrain.id_end = t_idx;
  pose_constrain.t_be.p = t_res;
  pose_constrain.t_be.q = q_res;

  return pose_constrain;
}

void loop_closure_pub_optimzed_path(
    const Ceres_pose_graph_3d::MapOfPoses &pose3d_aft_loopclosure) {
  nav_msgs::Odometry odom;
  m_laser_after_loopclosure_path.header.stamp = ros::Time::now();
  m_laser_after_loopclosure_path.header.frame_id = "camera_init";
  for (auto it = pose3d_aft_loopclosure.begin();
       it != pose3d_aft_loopclosure.end(); it++) {
    geometry_msgs::PoseStamped pose_stamp;
    Ceres_pose_graph_3d::Pose3d pose_3d = it->second;
    pose_stamp.pose.orientation.x = pose_3d.q.x();
    pose_stamp.pose.orientation.y = pose_3d.q.y();
    pose_stamp.pose.orientation.z = pose_3d.q.z();
    pose_stamp.pose.orientation.w = pose_3d.q.w();

    pose_stamp.pose.position.x = pose_3d.p(0);
    pose_stamp.pose.position.y = pose_3d.p(1);
    pose_stamp.pose.position.z = pose_3d.p(2);

    pose_stamp.header.frame_id = "camera_init";

    m_laser_after_loopclosure_path.poses.push_back(pose_stamp);
  }

  m_pub_laser_aft_loopclosure_path.publish(m_laser_after_loopclosure_path);
}

void loop_closure_optimize(
    const Ceres_pose_graph_3d::VectorOfPose &pose3d_vec,
    const Ceres_pose_graph_3d::MapOfPoses &pose3d_map,
    Ceres_pose_graph_3d::MapOfPoses &pose3d_map_ori,
    const Ceres_pose_graph_3d::VectorOfConstraints &constrain_vec,
    const size_t &his,  // number of previous frame of the loop
    const Eigen::Quaterniond &icp_q, const Eigen::Vector3d &icp_t) {
  printf("I believe this is true loop.\r\n");
  auto Q_a = pose3d_vec[his].q;
  auto Q_b = pose3d_vec[pose3d_vec.size() - 1].q;
  auto T_a = pose3d_vec[his].p;
  auto T_b = pose3d_vec[pose3d_vec.size() - 1].p;
  auto ICP_q = icp_q;  // Quaternion
  auto ICP_t = icp_t;  // Position

  ICP_t = (ICP_q.inverse() * (-ICP_t));
  ICP_q = ICP_q.inverse();

  std::cout << "ICP_q = " << ICP_q.coeffs().transpose() << std::endl;
  std::cout << "ICP_t = " << ICP_t.transpose() << std::endl;
  for (int i = 0; i < 10; i++) {
    std::cout << "-------------------------------------" << std::endl;
    std::cout << ICP_q.coeffs().transpose() << std::endl;
    std::cout << ICP_t.transpose() << std::endl;
  }
  Ceres_pose_graph_3d::VectorOfConstraints constrain_vec_temp;
  constrain_vec_temp = constrain_vec;
  constrain_vec_temp.push_back(add_constrain_of_loop(
      pose3d_vec.size() - 1, his, Q_a, T_a, Q_b, T_b, ICP_q, ICP_t));
  std::string path_name =
      "/home/parallels/catkin_ws/src/A-LOAM/loop_closure_res";
  std::string g2o_filename = std::string(path_name).append("/loop.g2o");
  pose3d_map_ori = pose3d_map;
  auto temp_pose_3d_map = pose3d_map;
  // Scene_alignment<float>::save_edge_and_vertex_to_g2o(
  // g2o_filename.c_str(), temp_pose_3d_map, constrain_vec_temp);
  Ceres_pose_graph_3d::pose_graph_optimization(temp_pose_3d_map,
                                               constrain_vec_temp);
  Ceres_pose_graph_3d::OutputPoses(
      std::string(path_name).append("/poses_ori.txt"), pose3d_map_ori);
  Ceres_pose_graph_3d::OutputPoses(
      std::string(path_name).append("/poses_opm.txt"), temp_pose_3d_map);
  // m_scene_align.dump_file_name(std::string(path_name).append("/file_name.txt"),
  //                              map_file_name);

  loop_closure_pub_optimzed_path(temp_pose_3d_map);

  // for (int pc_idx = (int)map_id_pc.size() - 1; pc_idx >= 0; pc_idx -= 2) {
  //   screen_out << "*** Refine pointcloud, curren idx = " << pc_idx << " ***"
  //              << endl;
  //   auto refined_pt = map_rfn.refine_pointcloud(map_id_pc, pose3d_map_ori,
  //                                               temp_pose_3d_map, pc_idx, 0);
  //   pcl::toROSMsg(refined_pt, ros_laser_cloud_surround);
  //   ros_laser_cloud_surround.header.stamp = ros::Time::now();
  //   ros_laser_cloud_surround.header.frame_id = "camera_init";
  //   m_pub_pc_aft_loop.publish(ros_laser_cloud_surround);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // }
  // map_rfn.refine_mapping( path_name, 0 );
  // if (0) {
  //   map_rfn.refine_mapping(map_id_pc, pose3d_map_ori, temp_pose_3d_map, 1);
  //   pcl::toROSMsg(map_rfn.m_pts_aft_refind, ros_laser_cloud_surround);
  //   ros_laser_cloud_surround.header.stamp = ros::Time::now();
  //   ros_laser_cloud_surround.header.frame_id = "camera_init";
  //   m_pub_pc_aft_loop.publish(ros_laser_cloud_surround);
  // }
}

void data_collection(const nav_msgs::Path::ConstPtr &aft_map_path) {
  Eigen::Quaterniond q_curr;
  Eigen::Vector3d t_curr;

  q_curr.x() = aft_map_path->poses.back().pose.orientation.x;
  q_curr.y() = aft_map_path->poses.back().pose.orientation.y;
  q_curr.z() = aft_map_path->poses.back().pose.orientation.z;
  q_curr.w() = aft_map_path->poses.back().pose.orientation.w;
  t_curr.x() = aft_map_path->poses.back().pose.position.x;
  t_curr.y() = aft_map_path->poses.back().pose.position.y;
  t_curr.z() = aft_map_path->poses.back().pose.position.z;

  pose3d_vec.push_back(Ceres_pose_graph_3d::Pose3d(q_curr, t_curr));
  pose3d_map.insert(std::make_pair(
      pose3d_map.size(), Ceres_pose_graph_3d::Pose3d(q_curr, t_curr)));

  if (pose3d_vec.size() >= 2) {
    Ceres_pose_graph_3d::Constraint3d temp_csn;
    Eigen::Vector3d relative_T = pose3d_vec[pose3d_vec.size() - 2].q.inverse() *
                                 (t_curr - pose3d_vec[pose3d_vec.size() - 2].p);
    Eigen::Quaterniond relative_Q =
        pose3d_vec[pose3d_vec.size() - 2].q.inverse() * q_curr;

    temp_csn = Ceres_pose_graph_3d::Constraint3d(
        pose3d_vec.size() - 2, pose3d_vec.size() - 1, relative_Q, relative_T);
    constrain_vec.push_back(temp_csn);
  }
}

void loop_closure_handler(
    const nav_msgs::Odometry::ConstPtr &loop_closure_result) {
  // TODO: DEFINE icp_q, icp_t
  Eigen::Quaterniond icp_q;
  Eigen::Vector3d icp_t;

  icp_q.x() = loop_closure_result->pose.pose.orientation.x;
  icp_q.y() = loop_closure_result->pose.pose.orientation.y;
  icp_q.z() = loop_closure_result->pose.pose.orientation.z;
  icp_q.w() = loop_closure_result->pose.pose.orientation.w;
  icp_t.x() = loop_closure_result->pose.pose.position.x;
  icp_t.y() = loop_closure_result->pose.pose.position.y;
  icp_t.z() = loop_closure_result->pose.pose.position.z;

  int child_index = stoi(loop_closure_result->child_frame_id);

  loop_closure_optimize(pose3d_vec, pose3d_map, pose3d_map_ori, constrain_vec,
                        child_index,  // number of previous frame of the loop
                        icp_q, icp_t);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "loop_detection");
  ros::NodeHandle nh;

  ros::Subscriber subAftMapPath =
      nh.subscribe<nav_msgs::Path>("/aft_mapped_path", 100, data_collection);

  ros::Subscriber subLoopClosureDetect = nh.subscribe<nav_msgs::Odometry>(
      "/loop_closure_pose_idx", 100, loop_closure_handler);

  m_pub_laser_aft_loopclosure_path =
      nh.advertise<nav_msgs::Path>("/aft_loopclosure_path", 100);

  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}