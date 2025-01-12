#ifndef OCCUPANCY_GRID_MAPPER_NODE_HPP
#define OCCUPANCY_GRID_MAPPER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include <vector>
#include <cmath>

// Define constants
const double UNKNOWN = -1;
const double OCCUPIED = 100;
const double FREE = 0;
const double LASER_MAX_RANGE = 10.0; // Maximum range of the laser in meters
const double GRID_RESOLUTION = 0.1;  // Resolution of the occupancy grid in meters per cell

class OccupancyGridMapper : public rclcpp::Node
{
public:
    OccupancyGridMapper();

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void initializeMap();
    void updateMap(const sensor_msgs::msg::LaserScan::SharedPtr &scan);
    bool getRobotPose(geometry_msgs::msg::PoseStamped &pose);
    void updateOccupancyGrid(const sensor_msgs::msg::LaserScan::SharedPtr &scan, double robot_x, double robot_y, nav_msgs::msg::OccupancyGrid &map);
    void transformLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan, const std::string &target_frame, std::vector<geometry_msgs::msg::PointStamped> &transformed_points);
    void markObstacle(double x, double y, nav_msgs::msg::OccupancyGrid &map);
    void markFreeSpace(double robot_x, double robot_y, double obstacle_x, double obstacle_y, nav_msgs::msg::OccupancyGrid &map);
    void publishLocalMap();
    void publishFullMap();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr full_map_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::msg::OccupancyGrid local_map_;
    nav_msgs::msg::OccupancyGrid full_map_;

    const std::string ODOM_FRAME = "diff_drive/odom";
    const std::string BASE_LINK_FRAME = "diff_drive";

    const int map_width_ = 120;
    const int map_height_ = 200;
    const double map_origin_x_ = 0;
    const double map_origin_y_ = -10;
    const int full_map_width_ = 500;
    const int full_map_height_ = 500;
    const double full_map_origin_x_ = -25;
    const double full_map_origin_y_ = -25;
};

#endif // OCCUPANCY_GRID_MAPPER_NODE_HPP
