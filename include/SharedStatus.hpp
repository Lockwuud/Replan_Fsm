/**
 * @FilePath     : /src/Replan_Fsm/include/SharedStatus.hpp
 * @Description  :  
 * @Author       : hejia 2736463842@qq.com
 * @Version      : 0.0.1
 * @LastEditors  : hejia 2736463842@qq.com
 * @LastEditTime : 2025-02-20 10:18:53
 * @Copyright    : G AUTOMOBILE RESEARCH INSTITUTE CO.,LTD Copyright (c) 2025.
**/
#ifndef SHARED_STATUS_HPP
#define SHARED_STATUS_HPP

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ego_planner/DataDisp.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#define sqrt2 1.41421356237 

enum Status {
    CALL,
    WAIT,
    REGENERATE,
    EMERGENCY,
    PUBLISHED,
    RECALL
};

inline Status status;

enum Diraction {
    UPLEFT = 1,
    UPRIGHT = 2,
    DOWNLEFT = 3,
    DOWNRIGHT = 4,
    LEFT = 5,
    RIGHT = 6,
    DOWN = 7
};

inline Diraction diraction;

inline float kp1, kp2;
inline float ed_left, ed_right, ed_ceil, ed_floor;
inline float delay;

inline Eigen::Vector3f pose;

#endif