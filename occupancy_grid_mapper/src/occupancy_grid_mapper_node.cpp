#include "occupancy_grid_mapper/bresenham_line.hpp"
#include "occupancy_grid_mapper/occupancy_grid_mapper_node.hpp"

OccupancyGridMapper::OccupancyGridMapper()
    : Node("obstacle_map_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
{
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "diff_drive/scan", 10, std::bind(&OccupancyGridMapper::laserScanCallback, this, std::placeholders::_1));

  local_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 10);
  full_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("full_map", 10);

  initializeMap();
}

void OccupancyGridMapper::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  updateMap(msg);

  publishLocalMap();
  publishFullMap();
}

void OccupancyGridMapper::initializeMap()
{
  local_map_.header.frame_id = BASE_LINK_FRAME;
  local_map_.info.resolution = GRID_RESOLUTION;
  local_map_.info.width = map_width_;
  local_map_.info.height = map_height_;
  local_map_.info.origin.position.x = map_origin_x_;
  local_map_.info.origin.position.y = map_origin_y_;
  local_map_.data.assign(map_width_ * map_height_, UNKNOWN);

  full_map_.header.frame_id = ODOM_FRAME;
  full_map_.info.resolution = GRID_RESOLUTION;
  full_map_.info.width = full_map_width_;
  full_map_.info.height = full_map_height_;
  full_map_.info.origin.position.x = full_map_origin_x_;
  full_map_.info.origin.position.y = full_map_origin_y_;
  full_map_.data.assign(full_map_width_ * full_map_height_, UNKNOWN);
}

void OccupancyGridMapper::updateMap(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
{
  // Retrieve robot pose in order to transform the point cloud
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose))
  {
    RCLCPP_WARN(this->get_logger(), "Could not get robot pose from TF.");
    return;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  // Reset and update local map
  local_map_.data.assign(map_width_ * map_height_, UNKNOWN);
  updateOccupancyGrid(scan, robot_x, robot_y, local_map_);

  // Update full map (if needed)
  updateOccupancyGrid(scan, robot_x, robot_y, full_map_);
}

bool OccupancyGridMapper::getRobotPose(geometry_msgs::msg::PoseStamped &pose)
{
  try
  {
    auto transform = tf_buffer_.lookupTransform(
        ODOM_FRAME, BASE_LINK_FRAME, tf2::TimePointZero);

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    return true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
    return false;
  }
}

void OccupancyGridMapper::updateOccupancyGrid(const sensor_msgs::msg::LaserScan::SharedPtr &scan, double robot_x, double robot_y, nav_msgs::msg::OccupancyGrid &map)
{
  std::vector<geometry_msgs::msg::PointStamped> transformed_points;
  transformLaserScan(scan, map.header.frame_id, transformed_points);

  for (const auto &point : transformed_points)
  {
    const double x = point.point.x;
    const double y = point.point.y;
    const double robot_x_in_map = map.header.frame_id == BASE_LINK_FRAME ? 0 : robot_x;
    const double robot_y_in_map = map.header.frame_id == BASE_LINK_FRAME ? 0 : robot_y;

    const double dist = std::hypot(x, y);
    if (dist >= LASER_MAX_RANGE)
    {
      markFreeSpace(robot_x_in_map, robot_y_in_map, x, y, map);
    }
    else
    {
      markObstacle(x, y, map);
      markFreeSpace(robot_x_in_map, robot_y_in_map, x, y, map);
    }
  }
}

void OccupancyGridMapper::transformLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr &scan, const std::string &target_frame, std::vector<geometry_msgs::msg::PointStamped> &transformed_points)
{
  transformed_points.clear();

  geometry_msgs::msg::TransformStamped transformStamped;
  try
  {
    transformStamped = tf_buffer_.lookupTransform(target_frame, scan->header.frame_id, tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not transform laser scan: %s", ex.what());
    return;
  }

  // Transform each point in the laser scan
  double angle = scan->angle_min;
  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    double range = scan->ranges[i];
    if (range > scan->range_max || range < scan->range_min)
    {
      range = LASER_MAX_RANGE;
    }

    // Calculate the point in the laser frame
    geometry_msgs::msg::PointStamped point_in, point_out;
    point_in.header = scan->header;
    point_in.point.x = range * std::cos(angle);
    point_in.point.y = range * std::sin(angle);
    point_in.point.z = 0.0;

    // Transform the point to the target frame
    tf2::doTransform(point_in, point_out, transformStamped);
    transformed_points.push_back(point_out);

    angle += scan->angle_increment;
  }
}

void OccupancyGridMapper::markObstacle(double x, double y, nav_msgs::msg::OccupancyGrid &map)
{
  const int map_x = std::round((x - map.info.origin.position.x) / map.info.resolution);
  const int map_y = std::round((y - map.info.origin.position.y) / map.info.resolution);

  if (map_x >= 0 && map_x < static_cast<int>(map.info.width) && map_y >= 0 && map_y < static_cast<int>(map.info.height))
  {
    map.data[map_y * map.info.width + map_x] = OCCUPIED;
  }
}

void OccupancyGridMapper::markFreeSpace(double robot_x, double robot_y, double obstacle_x, double obstacle_y, nav_msgs::msg::OccupancyGrid &map)
{
  const int x0 = std::round((robot_x - map.info.origin.position.x) / map.info.resolution);
  const int y0 = std::round((robot_y - map.info.origin.position.y) / map.info.resolution);
  const int x1 = std::round((obstacle_x - map.info.origin.position.x) / map.info.resolution);
  const int y1 = std::round((obstacle_y - map.info.origin.position.y) / map.info.resolution);

  const GridPoint start{x0, y0};
  const GridPoint end{x1, y1};
  const GridLine lidar_line{start, end};
  bresenhamLine(map, lidar_line);
}

void OccupancyGridMapper::publishLocalMap()
{
  local_map_.header.stamp = this->now();
  local_map_pub_->publish(local_map_);
}

void OccupancyGridMapper::publishFullMap()
{
  full_map_.header.stamp = this->now();
  full_map_pub_->publish(full_map_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridMapper>());
  rclcpp::shutdown();
  return 0;
}
