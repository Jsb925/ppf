#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std;

rclcpp::Node::SharedPtr node;
nav_msgs::msg::Odometry odom;

void scan_convert_cbk(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // std::cout << "Received!!!!!" << std::endl;
    float origin_x = odom.pose.pose.position.x;
    float origin_y = odom.pose.pose.position.y;
    float origin_z = odom.pose.pose.position.z;

    float rot_x = odom.pose.pose.orientation.x;
    float rot_y = odom.pose.pose.orientation.y;
    float rot_z = odom.pose.pose.orientation.z;
    float rot_w = odom.pose.pose.orientation.w;

    const float angle_min = msg->angle_min;
    const float angle_increment = msg->angle_increment;
    const int num_pts = msg->ranges.size();

    const int occ_grid_size = 100;
    const float resolution = 0.05;

    nav_msgs::msg::OccupancyGrid occ_grid;
    geometry_msgs::msg::Pose occ_grid_pose;
    rclcpp::Clock clock;

    std::cout << origin_x << " " << origin_y << " " << origin_z << endl;
    occ_grid_pose.position.x = -occ_grid_size * resolution / 2 + origin_x;
    occ_grid_pose.position.y = -occ_grid_size * resolution / 2 + origin_y;
    occ_grid_pose.position.z = origin_z;
    occ_grid_pose.orientation.x = rot_x;
    occ_grid_pose.orientation.y = rot_y;
    occ_grid_pose.orientation.z = rot_z;
    occ_grid_pose.orientation.w = rot_w;

    occ_grid.data.resize(occ_grid_size * occ_grid_size, 0); // Initialize occupancy grid data

    occ_grid.info.resolution = resolution;
    occ_grid.info.origin = occ_grid_pose;
    occ_grid.info.height = occ_grid_size;
    occ_grid.info.width = occ_grid_size;
    occ_grid.header.stamp = clock.now(); // Get current time from the node's clock
    occ_grid.header.frame_id = "ego_racecar/laser";    // Set the appropriate frame ID

    for (int i = 0; i < num_pts; i++)
    {
        float dist = msg->ranges[i];
        float angle = msg->angle_min + i * angle_increment;
        float x = dist * std::cos(angle);
        float y = dist * std::sin(angle);

        if (abs(x) > occ_grid_size * resolution / 2 | abs(y) > occ_grid_size * resolution / 2)
            continue;

        int grid_x = ((x / resolution) + (occ_grid_size / 2));
        int grid_y = ((y / resolution) + (occ_grid_size / 2));

        occ_grid.data[(grid_y) * (occ_grid_size) + grid_x] = 100; // Mark as occupied
    }

    static auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/occ_grid", 10);
    pub->publish(occ_grid);
}

void odom_cbk(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom = *msg;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("Scan_to_occ");
    auto scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, scan_convert_cbk);
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, odom_cbk);

    rclcpp::spin(node);
    return 0;
}
