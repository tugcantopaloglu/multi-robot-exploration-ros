#ifndef MULTI_ROBOT_EXPLORER_H
#define MULTI_ROBOT_EXPLORER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MultiRobotExplorer
{
public:
    MultiRobotExplorer(ros::NodeHandle &nh);
    void explore();

private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void assignGoals();
    std::vector<geometry_msgs::Point> findFrontiers();
    std::vector<geometry_msgs::Point> clusterFrontiers(const std::vector<geometry_msgs::Point> &frontiers, double distance_threshold);
    geometry_msgs::Point getRobotPosition(int robot_index);
    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result, int robot_index);

    ros::NodeHandle nh_;
    ros::Subscriber map_subscriber_;

    int num_robots_;
    std::vector<MoveBaseClient *> action_clients_;
    std::vector<bool> robot_is_busy_;
    std::vector<std::string> robot_names_;
    std::vector<geometry_msgs::Point> assigned_frontiers_;
    std::vector<geometry_msgs::Point> blacklisted_frontiers_;

    nav_msgs::OccupancyGrid current_map_;
    bool map_received_ = false;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

#endif
