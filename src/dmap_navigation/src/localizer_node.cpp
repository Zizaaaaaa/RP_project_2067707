#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "dmap_navigation/localizer.h"

class LocalizerNode : public rclcpp::Node {
public:
    LocalizerNode() : Node("dmap_localizer"), _localizer(nullptr) {
        //subscriber mappa
        _map_sub = create_subscription<nav_msgs::msg::Occupancy_Grid>(
            "/map", 10, std::bind(&LocalizerNode::mapCallback, this, std::placeholders::_1));
        //subscriber laser
        _scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LocalizerNode::scanCallback, this, std::placeholders::_1));

        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _current_pose = Eigen::Isometry2f::Identity();
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Ricevuta mappa, calcolo DMap...");
        _dmap = std::make_unique<dmap_navigation::DMap>(msg->info.width, msg->info.height, msg->info.resolution);
        _dmap->compute(msg->data);
        _localizer = std::make_unique<dmap_navigation::Localizer>(_dmap.get());
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!_localizer) return;
        // Conversione da LaserScan in punti Eigen
        std::vector<Eigen::Vector2f> points;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (r < msg->range_min || r > msg->range_max) continue;
            float angle = msg->angle_min + i * msg->angle_increment;
            points.push_back(Eigen::Vector2f(r * cos(angle), r * sin(angle)));
        }

        //Localizzazione (odometria in real life)
        _current_pose = _localizer->localize(points, _current_pose);
        // Infine pubblicazione TF map -> base_link
        publishTransform(msg->header.stamp);
    }

    void publishTransform(const rclcpp::Time& stamp) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = _current_pose.translation().x();
        t.transform.translation.y = _current_pose.translation().y();
        
        Eigen::Rotation2Df rot(_current_pose.linear());
        t.transform.rotation.z = sin(rot.angle() / 2.0);
        t.transform.rotation.w = cos(rot.angle() / 2.0);

        _tf_broadcaster->sendTransform(t);
    }
    std::unique_ptr<dmap_navigation::DMap> _dmap;
    std::unique_ptr<dmap_navigation::Localizer> _localizer;
    Eigen::Isometry2f _current_pose;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}