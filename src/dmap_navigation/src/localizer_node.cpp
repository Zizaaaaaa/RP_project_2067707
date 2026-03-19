#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "dmap_navigation/localizer.h"

class LocalizerNode : public rclcpp::Node {
public:
    LocalizerNode() : Node("dmap_localizer"), _localizer(nullptr), _map_origin(0.0f, 0.0f) {
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        _map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&LocalizerNode::mapCallback, this, std::placeholders::_1));

        _scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_1/scan", 10, std::bind(&LocalizerNode::scanCallback, this, std::placeholders::_1));

        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _current_pose = Eigen::Isometry2f::Identity();

        _initial_pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&LocalizerNode::initialPoseCallback, this, std::placeholders::_1));
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        _map_origin.x() = msg->info.origin.position.x;
        _map_origin.y() = msg->info.origin.position.y;

        RCLCPP_INFO(this->get_logger(), "Ricevuta mappa. Origine: (%.2f, %.2f)", _map_origin.x(), _map_origin.y());
        
        _dmap = std::make_unique<dmap_navigation::DMap>(msg->info.width, msg->info.height, msg->info.resolution);
        _dmap->compute(msg->data);
        _localizer = std::make_unique<dmap_navigation::Localizer>(_dmap.get());
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!_localizer) return;
        
        // Convert laser points
        std::vector<Eigen::Vector2f> points;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max) continue;
            float angle = msg->angle_min + i * msg->angle_increment;
            points.push_back(Eigen::Vector2f(r * cos(angle), r * sin(angle)));
        }

        // Temporarily move the robot to the "internal" coordinates of the map
        _current_pose.translation() -= _map_origin;
        
        // Gauss-Newton
        _current_pose = _localizer->localize(points, _current_pose);
        
        // bring the robot back to the real world
        _current_pose.translation() += _map_origin;

        // publish the transform and spawn the robot in rviz
        publishDirectTransform(msg->header.stamp);
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        float x = msg->pose.pose.position.x;
        float y = msg->pose.pose.position.y;
        float qz = msg->pose.pose.orientation.z;
        float qw = msg->pose.pose.orientation.w;
        float yaw = 2.0 * atan2(qz, qw);
        
        _current_pose.translation() = Eigen::Vector2f(x, y);
        _current_pose.linear() = Eigen::Rotation2Df(yaw).toRotationMatrix();
        
        RCLCPP_INFO(this->get_logger(), "Calcio d'inizio ricevuto! Posa: x=%.2f, y=%.2f", x, y);
    }

    void publishDirectTransform(const rclcpp::Time& stamp) {
        // publish the tf map
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "robot_1";
        
        t.transform.translation.x = _current_pose.translation().x();
        t.transform.translation.y = _current_pose.translation().y();
        t.transform.translation.z = 0.0;
        
        Eigen::Rotation2Df rot(_current_pose.linear());
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = sin(rot.angle() / 2.0);
        t.transform.rotation.w = cos(rot.angle() / 2.0);

        _tf_broadcaster->sendTransform(t);
    }

    std::unique_ptr<dmap_navigation::DMap> _dmap;
    std::unique_ptr<dmap_navigation::Localizer> _localizer;
    Eigen::Isometry2f _current_pose;
    Eigen::Vector2f _map_origin;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _initial_pose_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}