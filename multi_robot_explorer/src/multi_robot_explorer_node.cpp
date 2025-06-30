#include "multi_robot_explorer/multi_robot_explorer.h"

MultiRobotExplorer::MultiRobotExplorer(ros::NodeHandle &nh) : nh_(nh), tf_listener_(tf_buffer_)
{
    nh_.param("num_robots", num_robots_, 4);

    robot_names_.resize(num_robots_);
    for (int i = 0; i < num_robots_; ++i)
    {
        robot_names_[i] = "tb3_" + std::to_string(i);
    }

    map_subscriber_ = nh_.subscribe("/map", 1, &MultiRobotExplorer::mapCallback, this);

    robot_is_busy_.resize(num_robots_, false);
    action_clients_.resize(num_robots_);
    assigned_frontiers_.resize(num_robots_);

    for (int i = 0; i < num_robots_; ++i)
    {
        std::string action_server_name = "/" + robot_names_[i] + "/move_base";
        action_clients_[i] = new MoveBaseClient(action_server_name, true);

        ROS_INFO("Robot %d icin action sunucusu bekleniyor: %s", i, action_server_name.c_str());
        if (!action_clients_[i]->waitForServer(ros::Duration(5.0)))
        {
            ROS_ERROR("Robot %d icin action sunucusu bulunamadi!", i);
        }
        else
        {
            ROS_INFO("Robot %d icin action sunucusu bulundu.", i);
        }
    }
}

void MultiRobotExplorer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    current_map_ = *msg;
    map_received_ = true;
    ROS_INFO_ONCE("Ilk harita alindi. Kesif basliyor...");
}

geometry_msgs::Point MultiRobotExplorer::getRobotPosition(int robot_index)
{
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Point position;
    try
    {
        // Robotun 'base_footprint' frame'inin 'map' frame'ine göre transformunu al
        transformStamped = tf_buffer_.lookupTransform("map", robot_names_[robot_index] + "/base_footprint", ros::Time(0));
        position.x = transformStamped.transform.translation.x;
        position.y = transformStamped.transform.translation.y;
        position.z = 0; // 2D ortamdayız
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Robot %d icin transform alinamadi: %s", robot_index, ex.what());
        position.x = std::numeric_limits<double>::infinity();
        position.y = std::numeric_limits<double>::infinity();
    }
    return position;
}

std::vector<geometry_msgs::Point> MultiRobotExplorer::findFrontiers()
{
    std::vector<geometry_msgs::Point> frontiers;
    if (!map_received_)
        return frontiers;

    const auto &map_data = current_map_.data;
    const auto &map_info = current_map_.info;
    const unsigned int width = map_info.width;
    const unsigned int height = map_info.height;

    // haritadaki her hücreyi dolaş
    for (unsigned int y = 1; y < height - 1; ++y)
    {
        for (unsigned int x = 1; x < width - 1; ++x)
        {
            unsigned int i = y * width + x;

            if (map_data[i] != 0)
                continue;

            bool has_unknown_neighbor = false;
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if (dx == 0 && dy == 0)
                        continue;
                    unsigned int neighbor_i = (y + dy) * width + (x + dx);
                    if (map_data[neighbor_i] == -1)
                    {
                        has_unknown_neighbor = true;
                        break;
                    }
                }
                if (has_unknown_neighbor)
                    break;
            }

            if (has_unknown_neighbor)
            {
                geometry_msgs::Point p;
                p.x = map_info.origin.position.x + (x + 0.5) * map_info.resolution;
                p.y = map_info.origin.position.y + (y + 0.5) * map_info.resolution;
                p.z = 0;
                frontiers.push_back(p);
            }
        }
    }
    return frontiers;
}

std::vector<geometry_msgs::Point> MultiRobotExplorer::clusterFrontiers(const std::vector<geometry_msgs::Point> &frontiers, double distance_threshold)
{
    std::vector<geometry_msgs::Point> centroids;
    if (frontiers.empty())
        return centroids;

    std::vector<bool> visited(frontiers.size(), false);

    for (size_t i = 0; i < frontiers.size(); ++i)
    {
        if (visited[i])
            continue;

        std::vector<geometry_msgs::Point> current_cluster;
        std::vector<size_t> queue;

        queue.push_back(i);
        visited[i] = true;

        size_t head = 0;
        while (head < queue.size())
        {
            size_t current_idx = queue[head++];
            current_cluster.push_back(frontiers[current_idx]);

            for (size_t j = 0; j < frontiers.size(); ++j)
            {
                if (visited[j])
                    continue;

                double dist = std::hypot(frontiers[current_idx].x - frontiers[j].x, frontiers[current_idx].y - frontiers[j].y);
                if (dist < distance_threshold)
                {
                    visited[j] = true;
                    queue.push_back(j);
                }
            }
        }

        geometry_msgs::Point centroid;
        for (const auto &p : current_cluster)
        {
            centroid.x += p.x;
            centroid.y += p.y;
        }
        centroid.x /= current_cluster.size();
        centroid.y /= current_cluster.size();
        centroids.push_back(centroid);
    }

    return centroids;
}

void MultiRobotExplorer::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result, int robot_index)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Robot %d hedefine ulasti.", robot_index);
    }
    else
    {
        ROS_WARN("Robot %d hedefine ulasamadi. Durum: %s", robot_index, state.toString().c_str());
        ROS_INFO("Hedef [x:%.2f, y:%.2f] kara listeye eklendi.", assigned_frontiers_[robot_index].x, assigned_frontiers_[robot_index].y);
        blacklisted_frontiers_.push_back(assigned_frontiers_[robot_index]);
    }
    robot_is_busy_[robot_index] = false;
}

void MultiRobotExplorer::assignGoals()
{
    if (!map_received_)
    {
        ROS_INFO_THROTTLE(5, "Harita henuz alinmadi, hedef atama erteleniyor.");
        return;
    }

    auto all_frontiers = findFrontiers();
    if (all_frontiers.empty())
    {
        ROS_INFO_THROTTLE(10, "Kesfedilecek yeni sinir bulunamadi. Kesif tamamlanmis olabilir.");
        // robotlar meşgul değilse ve yeni sınır yoksa keşfi sonlandırıyoruz
        bool all_idle = true;
        for (const auto &busy : robot_is_busy_)
            if (busy)
                all_idle = false;
        if (all_idle)
        {
            ROS_INFO_NAMED("explorer", "Tum robotlar bosta ve yeni sinir yok. Kesif tamamlandi!");
            ros::shutdown();
        }
        return;
    }
    auto frontier_centroids = clusterFrontiers(all_frontiers, 1.0);

    for (int i = 0; i < num_robots_; ++i)
    {
        if (!robot_is_busy_[i])
        {
            geometry_msgs::Point robot_pos = getRobotPosition(i);
            if (!std::isfinite(robot_pos.x))
                continue;

            double min_dist = std::numeric_limits<double>::max();
            geometry_msgs::Point best_frontier;
            bool frontier_found = false;

            for (const auto &frontier : frontier_centroids)
            {
                // blacklist ile takılmaları atlamaya çalıştım. bazen etrafında dönerek kalıyorlar.
                // kara listeye alınmış sınır noktalarına yakın olanları atla.
                bool is_blacklisted = false;
                for (const auto &blacklisted_pt : blacklisted_frontiers_)
                {
                    double dist_to_blacklisted = std::hypot(frontier.x - blacklisted_pt.x, frontier.y - blacklisted_pt.y);
                    if (dist_to_blacklisted < 1.0)
                    {
                        is_blacklisted = true;
                        break;
                    }
                }
                if (is_blacklisted)
                    continue;

                bool is_assigned = false;
                for (int j = 0; j < num_robots_; ++j)
                {
                    if (robot_is_busy_[j])
                    {
                        double dist_to_assigned = std::hypot(frontier.x - assigned_frontiers_[j].x, frontier.y - assigned_frontiers_[j].y);
                        if (dist_to_assigned < 0.5)
                        {
                            is_assigned = true;
                            break;
                        }
                    }
                }
                if (is_assigned)
                    continue;

                double dist = std::hypot(robot_pos.x - frontier.x, robot_pos.y - frontier.y);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    best_frontier = frontier;
                    frontier_found = true;
                }
            }

            if (frontier_found)
            {
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position = best_frontier;
                goal.target_pose.pose.orientation.w = 1.0;

                action_clients_[i]->sendGoal(goal, boost::bind(&MultiRobotExplorer::goalDoneCallback, this, _1, _2, i));

                robot_is_busy_[i] = true;
                assigned_frontiers_[i] = best_frontier;
                ROS_INFO("Robot %d icin yeni hedef atandi: [x: %.2f, y: %.2f]", i, best_frontier.x, best_frontier.y);
            }
        }
    }
}

void MultiRobotExplorer::explore()
{
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        assignGoals();
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_robot_explorer_node");
    ros::NodeHandle nh("~");

    MultiRobotExplorer explorer(nh);
    explorer.explore();

    return 0;
}
