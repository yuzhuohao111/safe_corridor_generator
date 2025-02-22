#include "safe_corridor_generator/safe_corridor_generator.hpp"

namespace safe_corridor_generator
{

SafeCorridorGenerator::SafeCorridorGenerator(const rclcpp::NodeOptions &options)
    : Node("safe_corridor_generator", options)
{
    // 声明参数
    declare_parameter("vehicle_length", 0.4);
    declare_parameter("vehicle_width", 0.3);
    declare_parameter("sampling_distance", 0.8);
    declare_parameter("increment", 0.15);
    declare_parameter("occupied_threshold", 50);

    // 获取参数
    vehicle_length_ = get_parameter("vehicle_length").as_double();
    vehicle_width_ = get_parameter("vehicle_width").as_double();
    sampling_distance_ = get_parameter("sampling_distance").as_double();
    increment_ = get_parameter("increment").as_double();
    occupied_threshold_ = get_parameter("occupied_threshold").as_int();

    // 创建订阅者和发布者
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "plan", 10, std::bind(&SafeCorridorGenerator::pathCallback, this, std::placeholders::_1));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        std::bind(&SafeCorridorGenerator::mapCallback, this, std::placeholders::_1));
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).best_effort(),
        std::bind(&SafeCorridorGenerator::laserCallback, this, std::placeholders::_1));

    corridor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("corridor_markers", 10);
    sample_points_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("sample_points_markers", 10);

    // 初始化TF2缓冲器和监听器
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void SafeCorridorGenerator::pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg)
{
    if (!map_msg_)
    {
        RCLCPP_WARN(get_logger(), "Map not received yet.");
        return;
    }

    path_msg_ = path_msg;
    std::vector<geometry_msgs::msg::Point> sampled_points = samplePath(path_msg_->poses);

    corridor_.clear();
    for (const auto &point : sampled_points)
    {
        Rectangle rect = generateRectangle(point, vehicle_length_, vehicle_width_);
        expandRectangle(rect, map_msg_, increment_, dynamic_obstacles_);
        corridor_.push_back(rect);
    }

    publishCorridorMarkers();
}

void SafeCorridorGenerator::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_msg_ = map_msg;
}

void SafeCorridorGenerator::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    std::vector<geometry_msgs::msg::Point> dynamic_obstacles;
    tf2::Transform laser_to_map;
    try
    {
        auto transform = tf_buffer_->lookupTransform(
            map_msg_->header.frame_id, scan_msg->header.frame_id, scan_msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
        laser_to_map = tf2::Transform(
            tf2::Quaternion(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
            tf2::Vector3(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(get_logger(), "Failed to transform laser scan to map frame: %s", ex.what());
        return;
    }

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        double range = scan_msg->ranges[i];
        if (range < scan_msg->range_min || range > scan_msg->range_max)
            continue;

        double x = range * cos(angle);
        double y = range * sin(angle);

        geometry_msgs::msg::Point point;
        point.x = laser_to_map.getOrigin().x() + x * cos(laser_to_map.getRotation().getAngle()) - y * sin(laser_to_map.getRotation().getAngle());
        point.y = laser_to_map.getOrigin().y() + x * sin(laser_to_map.getRotation().getAngle()) + y * cos(laser_to_map.getRotation().getAngle());
        point.z = 0.0;

        // 过滤无效点
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isinf(point.x) && !std::isinf(point.y))
        {
            dynamic_obstacles.push_back(point);
        }
    }

    // 更新动态障碍物
    {
        std::lock_guard<std::mutex> lock(dynamic_obstacles_mutex_);
        dynamic_obstacles_ = dynamic_obstacles;
    }

    // 立即检查并更新走廊
    if (checkCorridorCollision())
    {
        RCLCPP_INFO(get_logger(), "Dynamic obstacle detected. Regenerating corridor.");
        regenerateCorridor();
    }
}

bool SafeCorridorGenerator::checkCollision(const Rectangle &rect, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    int start_grid_x, start_grid_y;
    int end_grid_x, end_grid_y;
    if (!worldToGrid(rect.min_x, rect.min_y, map_msg, start_grid_x, start_grid_y))
        return false;
    if (!worldToGrid(rect.max_x, rect.max_y, map_msg, end_grid_x, end_grid_y))
        return false;

    for (int x = start_grid_x; x <= end_grid_x; ++x)
    {
        for (int y = start_grid_y; y <= end_grid_y; ++y)
        {
            if (isOccupied(x, y, map_msg))
            {
                RCLCPP_WARN(get_logger(), "Collision detected at grid (x=%d, y=%d).", x, y);
                return true;
            }
        }
    }
    return false;
}

bool SafeCorridorGenerator::checkCollisionWithObstacles(const Rectangle &rect, const std::vector<geometry_msgs::msg::Point> &obstacles)
{
    for (const auto &point : obstacles)
    {
        if (point.x >= rect.min_x && point.x <= rect.max_x &&
            point.y >= rect.min_y && point.y <= rect.max_y)
        {
            RCLCPP_WARN(get_logger(), "Collision detected with dynamic obstacle at (%.2f, %.2f).", point.x, point.y);
            return true;
        }
    }
    return false;
}

void SafeCorridorGenerator::expandRectangle(Rectangle &rect, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, double increment, const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles)
{
    bool expanded = true;
    int max_iterations = 10;
    int iteration = 0;
    while (expanded && iteration < max_iterations)
    {
        expanded = false;
        std::vector<RectangleSide> expansion_order = {RectangleSide::LEFT, RectangleSide::REAR, RectangleSide::RIGHT, RectangleSide::FRONT};

        for (auto side : expansion_order)
        {
            Rectangle backup_rect = rect;
            expandSide(rect, map_msg, increment, side);
            if (rect != backup_rect)
            {
                if (checkCollision(rect, map_msg) || checkCollisionWithObstacles(rect, dynamic_obstacles))
                {
                    rect = backup_rect;
                }
                else
                {
                    expanded = true;
                }
            }
        }
        iteration++;
    }

}

void SafeCorridorGenerator::expandSide(Rectangle &rect, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, double increment, RectangleSide side)
{
    Rectangle backup_rect = rect;
    switch (side)
    {
    case RectangleSide::LEFT:
        rect.min_x -= increment;
        break;
    case RectangleSide::REAR:
        rect.min_y -= increment;
        break;
    case RectangleSide::RIGHT:
        rect.max_x += increment;
        break;
    case RectangleSide::FRONT:
        rect.max_y += increment;
        break;
    }
    updateVertices(rect);

    // 检查扩展后的矩形是否与障碍物发生碰撞
    if (checkCollision(rect, map_msg) || checkCollisionWithObstacles(rect, dynamic_obstacles_))
    {
        rect = backup_rect; // 恢复到扩展前的状态
        increment *= 0.5;   // 缩小扩展步长
    }
}

void SafeCorridorGenerator::updateVertices(Rectangle &rect)
{
    rect.vertices.clear();

    // 显式创建 geometry_msgs::msg::Point 对象
    geometry_msgs::msg::Point p1;
    p1.x = rect.min_x;
    p1.y = rect.min_y;
    p1.z = 0.0;
    rect.vertices.push_back(p1);

    geometry_msgs::msg::Point p2;
    p2.x = rect.max_x;
    p2.y = rect.min_y;
    p2.z = 0.0;
    rect.vertices.push_back(p2);

    geometry_msgs::msg::Point p3;
    p3.x = rect.max_x;
    p3.y = rect.max_y;
    p3.z = 0.0;
    rect.vertices.push_back(p3);

    geometry_msgs::msg::Point p4;
    p4.x = rect.min_x;
    p4.y = rect.max_y;
    p4.z = 0.0;
    rect.vertices.push_back(p4);
}

bool SafeCorridorGenerator::worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, int &grid_x, int &grid_y)
{
    grid_x = static_cast<int>((x - map_msg->info.origin.position.x) / map_msg->info.resolution);
    grid_y = static_cast<int>((y - map_msg->info.origin.position.y) / map_msg->info.resolution);
    return (grid_x >= 0 && grid_x < map_msg->info.width && grid_y >= 0 && grid_y < map_msg->info.height);
}

bool SafeCorridorGenerator::isOccupied(int grid_x, int grid_y, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    int index = grid_y * map_msg->info.width + grid_x;
    return (index >= 0 && index < map_msg->data.size() && map_msg->data[index] >= occupied_threshold_);
}

std::vector<geometry_msgs::msg::Point> SafeCorridorGenerator::samplePath(const std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
    std::vector<geometry_msgs::msg::Point> sampled_points;
    double accumulated_distance = 0.0;
    geometry_msgs::msg::Point prev_point = poses[0].pose.position;
    sampled_points.push_back(prev_point);

    for (size_t i = 1; i < poses.size(); ++i)
    {
        geometry_msgs::msg::Point current_point = poses[i].pose.position;
        double dx = current_point.x - prev_point.x;
        double dy = current_point.y - prev_point.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        accumulated_distance += distance;

        while (accumulated_distance >= sampling_distance_)
        {
            double ratio = sampling_distance_ / distance;
            geometry_msgs::msg::Point sample_point;
            sample_point.x = prev_point.x + dx * ratio;
            sample_point.y = prev_point.y + dy * ratio;
            sampled_points.push_back(sample_point);
            accumulated_distance -= sampling_distance_;
            prev_point = sample_point;
            dx = current_point.x - prev_point.x;
            dy = current_point.y - prev_point.y;
            distance = std::sqrt(dx * dx + dy * dy);
        }
        prev_point = current_point;
    }

    // 确保路径末尾的点被采样
    if (sampled_points.back() != poses.back().pose.position)
    {
        sampled_points.push_back(poses.back().pose.position);
    }

    return sampled_points;
}

Rectangle SafeCorridorGenerator::generateRectangle(const geometry_msgs::msg::Point &center, double length, double width)
{
    Rectangle rect;
    rect.center = center;
    rect.min_x = center.x - width / 2.0;
    rect.max_x = center.x + width / 2.0;
    rect.min_y = center.y - length / 2.0;
    rect.max_y = center.y + length / 2.0;
    updateVertices(rect);
    return rect;
}

bool SafeCorridorGenerator::checkCorridorCollision()
{
    std::lock_guard<std::mutex> lock(dynamic_obstacles_mutex_);
    for (const auto &rect : corridor_)
    {
        if (checkCollisionWithObstacles(rect, dynamic_obstacles_))
        {
            return true;
        }
    }
    return false;
}

void SafeCorridorGenerator::regenerateCorridor()
{
    if (!path_msg_ || !map_msg_)
    {
        RCLCPP_WARN(get_logger(), "Path or map not received yet.");
        return;
    }

    std::vector<geometry_msgs::msg::Point> sampled_points = samplePath(path_msg_->poses);
    corridor_.clear();

    for (const auto &point : sampled_points)
    {
        Rectangle rect = generateRectangle(point, vehicle_length_, vehicle_width_);
        expandRectangle(rect, map_msg_, increment_, dynamic_obstacles_);
        corridor_.push_back(rect);
    }

    publishCorridorMarkers();
}

void SafeCorridorGenerator::publishCorridorMarkers()
{
    visualization_msgs::msg::MarkerArray corridor_markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    corridor_markers.markers.push_back(clear_marker);

    for (size_t i = 0; i < corridor_.size(); ++i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_msg_->header.frame_id;
        marker.header.stamp = now();
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        for (size_t j = 0; j < corridor_[i].vertices.size(); ++j)
        {
            geometry_msgs::msg::Point p1 = corridor_[i].vertices[j];
            geometry_msgs::msg::Point p2 = corridor_[i].vertices[(j + 1) % corridor_[i].vertices.size()];
            marker.points.push_back(p1);
            marker.points.push_back(p2);
        }

        corridor_markers.markers.push_back(marker);
    }

    corridor_pub_->publish(corridor_markers);
}

} // namespace safe_corridor_generator

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(safe_corridor_generator::SafeCorridorGenerator)