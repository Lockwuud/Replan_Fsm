/**
 * @FilePath     : /src/Replan_Fsm/src/regenerate_target_node.cpp
 * @Description  :  
 * @Author       : hejia 2736463842@qq.com
 * @Version      : 0.0.1
 * @LastEditors  : hejia 2736463842@qq.com
 * @LastEditTime : 2025-02-20 19:37:02
 * @Copyright    : G AUTOMOBILE RESEARCH INSTITUTE CO.,LTD Copyright (c) 2025.
**/

#include "CAN.hpp"
#include "SharedStatus.hpp"

ros::Publisher pub;
usbCANFD can;
geometry_msgs::PoseStamped goal;

void cloud_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    Eigen::Vector3f total_force(0.0, 0.0, 0.0);

    switch(status){
        case REGENERATE:
            for (const auto& point : cloud.points) {
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                point_vec -= pose;
                float distance = point_vec.norm();
                if (distance == 0) continue; // Avoid division by zero
        
                Eigen::Vector3f repulsive_force = point_vec / (distance * distance);
                total_force += repulsive_force;
            }
        
            total_force += pose;
            total_force *= kp1;
            
            total_force.y() < ed_right ? ed_right : total_force.y();
            total_force.y() > ed_left ? ed_left : total_force.y();
            total_force.x() > ed_ceil ? ed_ceil : total_force.x();
            total_force.x() < ed_floor ? ed_floor : total_force.x();
        
            std::cout << "Total repulsive force: " << total_force.transpose() << std::endl;
            
            goal.pose.position.x = total_force.x();
            goal.pose.position.y = total_force.y();
            goal.pose.position.z = -1;
            pub.publish(goal);
            status = PUBLISHED;
            break;
            
        case RECALL:
            Eigen::Vector3f preset_force(0.0, 0.0, 0.0);
            switch(diraction){
                case UPLEFT:
                    preset_force.x() = sqrt1_2 * kp2;
                    preset_force.y() = sqrt1_2 * kp2;
                    break;
                case UPRIGHT:
                    preset_force.x() = sqrt1_2 * kp2;
                    preset_force.y() = -sqrt1_2 * kp2;
                    break;
                case DOWNLEFT:
                    preset_force.x() = -sqrt1_2 * kp2;
                    preset_force.y() = sqrt1_2 * kp2;
                    break;
                case DOWNRIGHT:
                    preset_force.x() = -sqrt1_2 * kp2;
                    preset_force.y() = -sqrt1_2 * kp2;
                    break;
                case LEFT:
                    preset_force.x() = 0;
                    preset_force.y() = kp2;
                    break;
                case RIGHT:
                    preset_force.x() = 0;
                    preset_force.y() = -kp2;
                    break;
                case DOWN:
                    preset_force.x() = -kp2;
                    preset_force.y() = 0;
                    break;
                default:
                    break;
            }
            
            // 半径搜索
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
            kdtree.setInputCloud(cloud_ptr);
            pcl::PointXYZ searchPoint;
            searchPoint.x = pose.x();
            searchPoint.y = pose.y();
            searchPoint.z = 0;

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            // if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            // {
            //     for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            //     {
            //         Eigen::Vector3f point_vec((*cloud)[ pointIdxRadiusSearch[i] ].x, (*cloud)[ pointIdxRadiusSearch[i] ].y, (*cloud)[ pointIdxRadiusSearch[i] ].z);
            //         point_vec -= pose;
            //         float distance = point_vec.norm();
            //         if (distance == 0) continue;
            //         Eigen::Vector3f repulsive_force = point_vec / (distance * distance);
            //         total_force += repulsive_force;
            //     }
            // }
        
            if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                for (const auto& point : cloud.points) {
                    Eigen::Vector3f point_vec(point.x, point.y, point.z);
                    point_vec -= pose;
                    float distance = point_vec.norm();
                    if (distance == 0) continue; // Avoid division by zero
            
                    Eigen::Vector3f repulsive_force = point_vec / (distance * distance);
                    total_force += repulsive_force;
                }
            }

            total_force += preset_force;
            total_force += pose;
            total_force *= kp2;
             
            total_force.y() < ed_right ? ed_right : total_force.y();
            total_force.y() > ed_left ? ed_left : total_force.y();
            total_force.x() > ed_ceil ? ed_ceil : total_force.x();
            total_force.x() < ed_floor ? ed_floor : total_force.x();
        
            std::cout << "Total repulsive force: " << total_force.transpose() << std::endl;
            
            goal.pose.position.x = total_force.x();
            goal.pose.position.y = total_force.y();
            goal.pose.position.z = -1;
            pub.publish(goal);
            status = PUBLISHED;
            break;

    }
}

void fsm_cbk(const ego_planner::DataDisp::ConstPtr &msg)
{
    if (msg->a > 0) {
        status = EMERGENCY;
        pose(0) = msg->b;
        pose(1) = msg->c;
        pose(2) = 0;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "regenerate_target_node");
    ros::NodeHandle nh("~");

    ros::param::get("kp1", kp1);
    ros::param::get("kp2", kp2);
    ros::param::get("ed_left", ed_left);
    ros::param::get("ed_right", ed_right);
    ros::param::get("ed_ceil", ed_ceil);
    ros::param::get("ed_floor", ed_floor);
    ros::param::get("radius", radius);

    ros::Subscriber sub_cloud = nh.subscribe("/cloud_withoutedge", 1, cloud_cbk);
    ros::Subscriber sub_fsm = nh.subscribe("/planning/data_display", 1, fsm_cbk);
    pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    /* 接收线程 */
    std::thread THREAD_RECEIVER(&usbCANFD::receiveCanMessages, &can);

    /* 状态机线程 */
    std::thread THREAD_FSM([&](){
        while(ros::ok()){
            // 状态机
            switch (status)
            {
                case CALL:
                    status = RECALL;
                    ROS_INFO("Received call, waiting for recalling...");
                    break;
                case RECALL:
                    ROS_INFO("Recalling target...");
                    break;
                case EMERGENCY:
                    status = REGENERATE;
                    ROS_INFO("Received emergency, waiting for regenerating...");
                    break;   
                case PUBLISHED:
                    status = WAIT;
                    ROS_INFO("Target(%f, %f) published...",goal.pose.position.x, goal.pose.position.y);
                    break;
                case WAIT:
                    ROS_INFO("Waiting for new call or emergency...");
                    break;
                case REGENERATE:
                    ROS_INFO("Regenerating target...");
                    break;
                default:
                    break;
            }
            // 接收线程维护
            if (!can.receiverRunning)
            {
                std::cerr << "Receiver thread not running, restarting..." << std::endl;
                THREAD_RECEIVER.detach();
                can.receiverRunning = true;                                        // 重置标志
                THREAD_RECEIVER = std::thread(&usbCANFD::receiveCanMessages, &can); // 重启线程
            }
        }
    });

    ros::spin();

    THREAD_RECEIVER.join();
    THREAD_FSM.join();

    return 0;
}   

