/**
 * @FilePath     : /src/Replan_Fsm/src/regenerate_target_node.cpp
 * @Description  :  
 * @Author       : hejia 2736463842@qq.com
 * @Version      : 0.0.1
 * @LastEditors  : hejia 2736463842@qq.com
 * @LastEditTime : 2025-02-18 08:49:22
 * @Copyright    : G AUTOMOBILE RESEARCH INSTITUTE CO.,LTD Copyright (c) 2025.
**/

#include <iostream>
#include <ros/ros.h>
#include <ego_planner/DataDisp.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub;
bool rePlan = false;
float kp, ed_left, ed_right, ed_ceil, ed_floor;
float delay;
Eigen::Vector3f pose;
geometry_msgs::PoseStamped goal_latest;

void cloud_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (!rePlan) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    Eigen::Vector3f total_force(0.0, 0.0, 0.0);

    for (const auto& point : cloud.points) {
        Eigen::Vector3f point_vec(point.x, point.y, point.z);
        point_vec -= pose;
        float distance = point_vec.norm();
        if (distance == 0) continue; // Avoid division by zero

        Eigen::Vector3f repulsive_force = point_vec / (distance * distance);
        total_force += repulsive_force;
    }

    total_force += pose;
    total_force *= kp;
    
    total_force.y() < ed_right ? ed_right : total_force.y();
    total_force.y() > ed_left ? ed_left : total_force.y();
    total_force.x() > ed_ceil ? ed_ceil : total_force.x();
    total_force.x() < ed_floor ? ed_floor : total_force.x();

    ROS_INFO_STREAM("Total repulsive force: " << total_force.transpose());
    
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = total_force.x();
    goal.pose.position.y = total_force.y();
    goal.pose.position.z = -1;
    pub.publish(goal);
    rePlan = false;
}

void fsm_cbk(const ego_planner::DataDisp::ConstPtr &msg)
{
    if (msg->a > 0) {
        rePlan = true;
        pose(0) = msg->b;
        pose(1) = msg->c;
        pose(2) = 0;
        ROS_INFO("Emergency Flag Received, Regenerating target!!!")
    }
}

void goal_cbk(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(msg->pose.position.z == 0){
        goal_latest = *msg;
    }
    else if(msg->pose.position.z == -1){
        goal_latest.pose.position.x = msg->pose.position.x;
        goal_latest.pose.position.y = msg->pose.position.y;
        goal_latest.pose.position.z = 0;
        sleep(delay);
        pub.publish(goal_latest);
    }
}

int main(int argc, const *argv[])
{
    ros::init(argc, argv, "regenerate_target_node");
    ros::NodeHandle nh("~");

    ros::param::get("kp", kp);
    ros::param::get("ed_left", ed_left);
    ros::param::get("ed_right", ed_right);
    ros::param::get("ed_ceil", ed_ceil);
    ros::param::get("ed_floor", ed_floor);
    ros::param::get("delay", delay);

    ros::Subscriber sub_cloud = nh.subscribe("/cloud_withoutedge", 1, cloud_cbk);
    ros::Subscriber sub_fsm = nh.subscribe("/planning/data_display", 1, fsm_cbk);
    // ros::Subscriber sub_goal = nh.subscribe("/move_base_simple/goal", 1, goal_cbk);
    pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::spin();
    return 0;
}   

