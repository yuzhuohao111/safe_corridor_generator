#ifndef SAFE_CORRIDOR_GENERATOR_HPP
#define SAFE_CORRIDOR_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <cmath>
#include <tf2_ros/buffer.h>

namespace safe_corridor_generator
{

struct Rectangle
{
    double min_x, max_x;
    double min_y, max_y;
    geometry_msgs::msg::Point center;
    std::vector<geometry_msgs::msg::Point> vertices;

    bool operator!=(const Rectangle &other) const
    {
        return min_x != other.min_x || max_x != other.max_x ||
               min_y != other.min_y || max_y != other.max_y;
    }
};

enum class RectangleSide
{
    LEFT,
    RIGHT,
    FRONT,
    REAR
};

class SafeCorridorGenerator : public rclcpp::Node
{
public:
    SafeCorridorGenerator(const rclcpp::NodeOptions &options);

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    bool checkCollision(const Rectangle &rect, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    bool checkCollisionWithObstacles(const Rectangle &rect, const std::vector<geometry_msgs::msg::Point> &obstacles);
    void expandRectangle(Rectangle &rect, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, double increment, const std::vector<geometry_msgs::msg::Point> &dynamic_obstacles);
    void expandSide(Rectangle &rect, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, double increment, RectangleSide side);
    void updateVertices(Rectangle &rect);
    bool worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, int &grid_x, int &grid_y);
    bool isOccupied(int grid_x, int grid_y, const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
    std::vector<geometry_msgs::msg::Point> samplePath(const std::vector<geometry_msgs::msg::PoseStamped> &poses);
    Rectangle generateRectangle(const geometry_msgs::msg::Point &center, double length, double width);
    bool checkCorridorCollision();
    void regenerateCorridor();
    void publishCorridorMarkers();

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sample_points_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex map_mutex_;
    std::mutex dynamic_obstacles_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    nav_msgs::msg::Path::SharedPtr path_msg_;
    std::vector<Rectangle> corridor_;
    std::vector<geometry_msgs::msg::Point> dynamic_obstacles_;

    double vehicle_length_;
    double vehicle_width_;
    double sampling_distance_;
    double increment_;
    int occupied_threshold_;
};

} // namespace safe_corridor_generator

#endif // SAFE_CORRIDOR_GENERATOR_HPP