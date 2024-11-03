#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h> // For quaternion handling
#include <vector>
#include <iostream>
#include <cmath>
#include "ppf_n.cpp" // Include your path planning algorithm header

#define M_PI 3.14159265358979323846 /* pi */

class PPFNode : public rclcpp::Node
{
public:
    PPFNode() : Node("ppf_node"), distance(0), roll(0), pitch(0), yaw(0)
    {
        // Create subscriptions
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map2", 1, std::bind(&PPFNode::occupancyGridCallback, this, std::placeholders::_1));

        goal_marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "/target_arrow", 1, std::bind(&PPFNode::targetCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path_check", 1);
    }

private:
    void printMap(const std::vector<std::vector<int>> &map)
    {
        for (const auto &row : map)
        {
            for (int cell : row)
            {
                if (cell == -1)
                {
                    std::cout<<" ";
                }
                else if (cell == 100)
                {
                    std::cout<<"1"; 
                }
                else
                {
                    std::cout<<cell;
                }
            }
            std::cout << std::endl;
        }
    }
    double distance = 0, roll = 0, pitch = 0, yaw = 0,heading=0;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr goal_marker_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void rotate90(std::vector<std::vector<int>> &mat)
    {
        int n = mat.size();
        std::vector<std::vector<int>> res(n, std::vector<int>(n));

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                res[j][n - i - 1] = mat[i][j];
            }
        }

        mat = res;
    }

    double yaw_handler(double yaw)
    {
        double out = yaw;
        if (yaw >= 0.4636 && yaw <= M_PI)
        {
            yaw = 0.4636;
        }
        else if (yaw > M_PI && yaw <= 2 * M_PI - 0.4636)
        {
            yaw = 2 * M_PI - 0.4636;
        }
        return out;
    }

    void targetCallback(const visualization_msgs::msg::Marker::SharedPtr marker)
    {

        double qx = marker->pose.orientation.x;
        double qy = marker->pose.orientation.y;
        double qz = marker->pose.orientation.z;
        double qw = marker->pose.orientation.w;

        yaw = qx;
        heading=qy;
        distance= marker->scale.x;
        RCLCPP_INFO(this->get_logger(), "Distance: %f, pitch: %f", distance, yaw);
    }

    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        int width = msg->info.width;
        int height = msg->info.height;
        float resolution = msg->info.resolution;
        const std::vector<int8_t> &data = msg->data;
        
        RCLCPP_INFO(this->get_logger(), "Received Occupancy Grid: Width: %d, Height: %d, Resolution: %.2f", width, height, resolution);

        std::vector<std::vector<int>> matrix(height, std::vector<int>(width, 0));

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = x + y * width;
                int8_t cell_value = data[index];
                matrix[height - x - 1][width - y - 1] = cell_value;
            }
        }

        int sx = 49;
        int sy = 24;
        double ly = -std::sin(yaw) * std::abs(distance) * 10 + 24;
        double lx = 49 - std::cos(yaw) * std::abs(distance) * 10;

        // Limiters
        lx = std::max(0.0, std::min(lx, 49.0));
        ly = std::max(0.0, std::min(ly, 49.0));

        int gx = static_cast<int>(lx);
        int gy = static_cast<int>(ly);

        // Main path planning function
        int pFlag=0;
        std::vector<std::vector<int>> path = predictivePotentialFieldPlanner(matrix, Point(sx, sy), Point(gx, gy), 500, resolution, 1,pFlag);

        std::cout<<"Path Size: "<<path.size()<<std::endl;
        //TEST
        // path={{40,20},{30,10},{20,0}};
        RCLCPP_INFO(this->get_logger(), "Path Calculated");

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "lidar";

        // Visualization
        matrix[sx][sy] = 9;
        matrix[int(lx)][int(ly)] = 8;

        for(auto p: path)
        {
            matrix[p[0]][p[1]] = 6;
        }
        printMap(matrix);
        std::cout<<"map printed"<<std::endl;
        geometry_msgs::msg::PoseStamped pose;
        for (size_t i = 0; i < path.size(); ++i)
        {
            pose.pose.position.x = -(path[i][0] - sx) * resolution;
            pose.pose.position.y = -(path[i][1] - sy) * resolution;
            pose.pose.position.z = 0;
            path_msg.poses.push_back(pose);
            std::cout << "Path: (" << path[i][0]  << ", " << path[i][1] << std::endl;
        }
        if(pFlag==0){
            path_pub_->publish(path_msg);
        }
        RCLCPP_INFO(this->get_logger(), "Path Published");

        double maxDist = std::abs(50 * std::cos(yaw));
        RCLCPP_INFO(this->get_logger(), "Max Distance: %f", maxDist);
        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PPFNode>());
    rclcpp::shutdown();
    return 0;
}
