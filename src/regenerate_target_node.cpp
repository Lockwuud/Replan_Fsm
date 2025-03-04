/**
 * @FilePath     : /src/Replan_Fsm/src/regenerate_target_node.cpp
 * @Description  :  
 * @Author       : hejia 2736463842@qq.com
 * @Version      : 0.0.1
 * @LastEditors  : hejia 2736463842@qq.com
 * @LastEditTime : 2025-03-04 21:00:43
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
    Eigen::Vector3f offset_force(0.0, 0.0, 0.0);
    Eigen::Vector3f bias_force(0.0, 0.0, 0.0);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    kdtree.setInputCloud(cloud_ptr);
    pcl::PointXYZ searchPoint;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<float> data;

    switch(status){
        case REGENERATE:
            for (const auto& point : cloud.points) {
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                point_vec -= pose;
                offset_force += point_vec / point_vec.norm() * point_vec.norm();
            }
            offset_force /= offset_force.norm();
            total_force += offset_force * kp1;
            total_force += pose;

            searchPoint.x = total_force.x();
            searchPoint.y = total_force.y();
            searchPoint.z = 0;
            
            while (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                ROS_INFO("Have Obstacle! Repush!");
                for (const auto& point : cloud.points) {
                    Eigen::Vector3f point_vec(point.x, point.y, point.z);
                    point_vec -= total_force;
                    bias_force += point_vec / point_vec.norm() * point_vec.norm();
                }
                total_force += (bias_force / bias_force.norm()) * kp1;

                searchPoint.x = total_force.x();
                searchPoint.y = total_force.y();
                searchPoint.z = 0;

                pointIdxRadiusSearch.clear();
                pointIdxRadiusSearch.shrink_to_fit();
                pointRadiusSquaredDistance.clear();
                pointRadiusSquaredDistance.shrink_to_fit();
            }

            total_force.y() = total_force.y() < ed_right ? ed_right : total_force.y();
            total_force.y() = total_force.y() > ed_left ? ed_left : total_force.y();
            total_force.x() = total_force.x() > ed_ceil ? ed_ceil : total_force.x();
            total_force.x() = total_force.x() < ed_floor ? ed_floor : total_force.x();
        
            std::cout << "Total repulsive force: " << total_force.transpose() << std::endl;
            
            data = {total_force.x(), total_force.y()};
            can.sendData(0x105, data, 8);

            goal.pose.position.x = total_force.x();
            goal.pose.position.y = total_force.y();
            goal.pose.position.z = 0;
            pub.publish(goal);
            status = PUBLISHED;
            break;
            
        case RECALL:        // make a feint repulsive force
            struct timeval sTime, eTime;
            gettimeofday(&sTime, NULL);

            Eigen::Vector3f preset_force(0.0, 0.0, 0.0);
            switch(diraction){
                case UPLEFT:
                    preset_force.x() = sqrt1_2 * kp2;
                    preset_force.y() = sqrt1_2 * kp2;
                    std::cout << "Diraction: UPLEFT" << std::endl;
                    break;
                case UPRIGHT:
                    preset_force.x() = sqrt1_2 * kp2;
                    preset_force.y() = -sqrt1_2 * kp2;
                    std::cout << "Diraction: UPRIGHT" << std::endl;
                    break;
                case DOWNLEFT:
                    preset_force.x() = -sqrt1_2 * kp2;
                    preset_force.y() = sqrt1_2 * kp2;
                    std::cout << "Diraction: DOWNLEFT" << std::endl;
                    break;
                case DOWNRIGHT:
                    preset_force.x() = -sqrt1_2 * kp2;
                    preset_force.y() = -sqrt1_2 * kp2;
                    std::cout << "Diraction: DOWNRIGHT" << std::endl;
                    break;
                case LEFT:
                    preset_force.x() = 0;
                    preset_force.y() = kp2;
                    std::cout << "Diraction: LEFT" << std::endl;
                    break;
                case RIGHT:
                    preset_force.x() = 0;
                    preset_force.y() = -kp2;
                    std::cout << "Diraction: RIGHT" << std::endl;
                    break;
                case DOWN:
                    preset_force.x() = -kp2;
                    preset_force.y() = 0;
                    std::cout << "Diraction: DOWN" << std::endl;
                    break;
                default:
                    break;
            }
            
            total_force += preset_force;
            total_force += pose;

            // 半径搜索
            searchPoint.x = total_force.x();
            searchPoint.y = total_force.y();
            searchPoint.z = 0;

            while (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                ROS_INFO("Have Obstacle! Repush!");
                for (const auto& point : cloud.points) {
                    Eigen::Vector3f point_vec(point.x, point.y, point.z);
                    point_vec -= total_force;
                    bias_force += point_vec / point_vec.norm() * point_vec.norm();
                }
                total_force += (bias_force / bias_force.norm()) * kp3;

                searchPoint.x = total_force.x();
                searchPoint.y = total_force.y();
                searchPoint.z = 0;

                pointIdxRadiusSearch.clear();
                pointIdxRadiusSearch.shrink_to_fit();
                pointRadiusSquaredDistance.clear();
                pointRadiusSquaredDistance.shrink_to_fit();
            }
            
            gettimeofday(&eTime, NULL);
            long exeTime = (eTime.tv_sec-sTime.tv_sec)+(eTime.tv_usec-sTime.tv_usec)*1e-6;
            Eigen::Vector3f speedFeedForward(0,0,0);
            speedFeedForward = velocity * (float(exeTime) + delay);
            total_force += speedFeedForward;

            total_force.y() = total_force.y() < ed_right ? ed_right : total_force.y();
            total_force.y() = total_force.y() > ed_left ? ed_left : total_force.y();
            total_force.x() = total_force.x() > ed_ceil ? ed_ceil : total_force.x();
            total_force.x() = total_force.x() < ed_floor ? ed_floor : total_force.x();
        
            std::cout << "Pose: " << pose.transpose() << std::endl;
            std::cout << "Velocity: " << velocity.transpose() << std::endl;
            std::cout << "Speed Feed Forward: " << speedFeedForward.transpose() << std::endl;
            std::cout << "Motion force: " << total_force.transpose() << std::endl;
            
            data = {total_force.x(), total_force.y()};
            can.sendData(0x105, data, 8);

            goal.pose.position.x = total_force.x();
            goal.pose.position.y = total_force.y();
            goal.pose.position.z = 0.0;
            pub.publish(goal);
            status = PUBLISHED;
            break;
    }
}

void fsm_cbk(const traj_utils::DataDisp::ConstPtr &msg)
{
    if (msg->a > 0) {
        status = EMERGENCY;
        pose(0) = float(msg->b);
        pose(1) = float(msg->c);
        pose(2) = 0;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "regenerate_target_node");
    ros::NodeHandle nh("~");

    ros::param::get("kp1", kp1);
    ros::param::get("kp2", kp2);
    ros::param::get("kp3", kp3);
    ros::param::get("ed_left", ed_left);
    ros::param::get("ed_right", ed_right);
    ros::param::get("ed_ceil", ed_ceil);
    ros::param::get("ed_floor", ed_floor);
    ros::param::get("radius", radius);
    ros::param::get("delay", delay);

    status = WAIT;

    ros::Subscriber sub_cloud = nh.subscribe("/cloud_withoutedge", 1, cloud_cbk);
    ros::Subscriber sub_fsm = nh.subscribe("/drone_1_planning/data_display", 1, fsm_cbk);
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
                    // ROS_INFO("Received call, waiting for recalling...");
                    break;
                case RECALL:
                    // ROS_INFO("Recalling target...");
                    break;
                case EMERGENCY:
                    status = REGENERATE;
                    // ROS_INFO("Received emergency, waiting for regenerating...");
                    break;   
                case PUBLISHED:
                    status = WAIT;
                    ROS_INFO("Target(%f, %f) published...",goal.pose.position.x, goal.pose.position.y);
                    break;
                case WAIT:
                    // ROS_INFO("Waiting for new call or emergency...");
                    break;
                case REGENERATE:
                    // ROS_INFO("Regenerating target...");
                    break;
                default:
                    break;
                usleep(10000);
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

