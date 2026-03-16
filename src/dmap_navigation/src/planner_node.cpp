#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "dmap_navigation/planner.h"

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("dmap_planner") {
        _map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

        _goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

        _path_pub = create_publisher<nav_msgs::msg::Path>("/plan", 10);

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!_dmap) {
            _dmap = std::make_unique<dmap_navigation::DMap>(msg->info.width, msg->info.height, msg->info.resolution);
        }
        _dmap->compute(msg->data);
        _planner = std::make_unique<dmap_navigation::Planner>(_dmap.get());
        RCLCPP_INFO(this->get_logger(), "Planner: DMap aggiornata e pronta.");
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!_planner) {
            RCLCPP_WARN(this->get_logger(), "Mappa non ancora ricevuta!");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            //dove si trova il robot ORA rispetto alla mappa
            transform = _tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Impossibile trovare posa robot: %s", ex.what());
            return;
        }

        Eigen::Vector2f start(transform.transform.translation.x, transform.transform.translation.y);
        Eigen::Vector2f goal(msg->pose.position.x, msg->pose.position.y);

        auto points = _planner->plan(start, goal);

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (const auto& p : points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            path_msg.poses.push_back(pose);
        }
        _path_pub->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Percorso calcolato e pubblicato! Nodi: %zu", points.size());
    }

    std::unique_ptr<dmap_navigation::DMap> _dmap;
    std::unique_ptr<dmap_navigation::Planner> _planner;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
    
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}