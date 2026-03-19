#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "dmap_navigation/planner.h"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("dmap_planner"), _map_origin(0.0f, 0.0f) {
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        _map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

        _goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

        _path_pub = create_publisher<nav_msgs::msg::Path>("/plan", 10);

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        _cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("/robot_1/cmd_vel", 10);

        _marker_pub = create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);
        
        _timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlannerNode::controlLoop, this));
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!_dmap) {
            _dmap = std::make_unique<dmap_navigation::DMap>(msg->info.width, msg->info.height, msg->info.resolution);
        }
        
        _map_origin.x() = msg->info.origin.position.x;
        _map_origin.y() = msg->info.origin.position.y;

        _dmap->compute(msg->data);
        _planner = std::make_unique<dmap_navigation::Planner>(_dmap.get());
        //RCLCPP_INFO(this->get_logger(), "Planner: DMap aggiornata. Origine: (%.2f, %.2f)", _map_origin.x(), _map_origin.y());
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!_planner) {
            RCLCPP_WARN(this->get_logger(), "Mappa non ancora ricevuta!");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = _tf_buffer->lookupTransform("map", "robot_1", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Impossibile trovare posa robot: %s", ex.what());
            return;
        }

        Eigen::Vector2f start_world(transform.transform.translation.x, transform.transform.translation.y);
        Eigen::Vector2f goal_world(msg->pose.position.x, msg->pose.position.y);

        // Sposto le coordinate nel sistema "interno" della mappa
        Eigen::Vector2f start_map = start_world - _map_origin;
        Eigen::Vector2f goal_map = goal_world - _map_origin;

        auto points = _planner->plan(start_map, goal_map);

        _current_path.clear();
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (const auto& p : points) {
            Eigen::Vector2f p_world = p + _map_origin;
            _current_path.push_back(p_world);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = p_world.x();
            pose.pose.position.y = p_world.y();
            path_msg.poses.push_back(pose);
        }

        _path_pub->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Percorso calcolato! Nodi: %zu", _current_path.size());
    }

    void controlLoop() {
        // MARKER 
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "robot_1";
        marker.ns = "robot_base";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Posizione (Centro di tf)
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.05; 
        marker.pose.orientation.w = 1.0;
        
        // Dimensioni
        marker.scale.x = 0.6; 
        marker.scale.y = 0.6;
        marker.scale.z = 0.02;
        
        // Colore
        marker.color.r = 0.0f;
        marker.color.g = 0.5f;
        marker.color.b = 1.0f; 
        marker.color.a = 0.5f;
        
        _marker_pub->publish(marker);
        // FINE CERCHIO

        if (_current_path.empty()) return;

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = _tf_buffer->lookupTransform("map", "robot_1", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            return; 
        }

        float rx = transform.transform.translation.x;
        float ry = transform.transform.translation.y;
        float qz = transform.transform.rotation.z;
        float qw = transform.transform.rotation.w;
        float ryaw = 2.0 * atan2(qz, qw);

        Eigen::Vector2f robot_pos(rx, ry);
        Eigen::Vector2f target = _current_path.front();

        if ((target - robot_pos).norm() < 0.2) {
            _current_path.erase(_current_path.begin());
            if (_current_path.empty()) {
                geometry_msgs::msg::Twist stop_cmd;
                _cmd_vel_pub->publish(stop_cmd);
                RCLCPP_INFO(this->get_logger(), "Destinazione raggiunta!");
                return;
            }
            target = _current_path.front();
        }

        float angle_to_target = atan2(target.y() - robot_pos.y(), target.x() - robot_pos.x());
        float angle_error = angle_to_target - ryaw;
        
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = 1.5 * angle_error; 
        
        if (std::abs(angle_error) < 0.5) {
            cmd.linear.x = 0.3; 
        } else {
            cmd.linear.x = 0.0; 
        }

        _cmd_vel_pub->publish(cmd);
    }

    std::unique_ptr<dmap_navigation::DMap> _dmap;
    std::unique_ptr<dmap_navigation::Planner> _planner;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _path_pub;
    
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
    std::vector<Eigen::Vector2f> _current_path;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker_pub;
    
    // variabile per memorizzare origine
    Eigen::Vector2f _map_origin;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}