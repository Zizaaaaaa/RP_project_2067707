#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "dmap_navigation/localizer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

class LocalizerNode : public rclcpp::Node {
public:
    LocalizerNode() : Node("dmap_localizer"), _localizer(nullptr) {
        //subscriber mappa
        _map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&LocalizerNode::mapCallback, this, std::placeholders::_1));
        //subscriber laser
        _scan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LocalizerNode::scanCallback, this, std::placeholders::_1));

        _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
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
        updateMapToOdomTransform(msg->header.stamp);
    }

    void updateMapToOdomTransform(const rclcpp::Time& stamp) {
        try {
            // Cerchiamo la trasformata odom -> base_link attuale
            geometry_msgs::msg::TransformStamped odom_to_base = 
                _tf_buffer->lookupTransform("odom", "base_link", tf2::TimePointZero);

            // Convertiamo in Isometry2f (manuale per semplicità)
            float tx = odom_to_base.transform.translation.x;
            float ty = odom_to_base.transform.translation.y;
            float qz = odom_to_base.transform.rotation.z;
            float qw = odom_to_base.transform.rotation.w;
            float angle = 2.0 * atan2(qz, qw);

            Eigen::Isometry2f T_odom_base = Eigen::Isometry2f::Identity();
            T_odom_base.translation() = Eigen::Vector2f(tx, ty);
            T_odom_base.linear() = Eigen::Rotation2Df(angle).toRotationMatrix();

            // T_map_odom = T_map_base * T_odom_base.inverse()
            Eigen::Isometry2f T_map_odom = _current_pose * T_odom_base.inverse();

            // Pubblichiamo T_map_odom
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stamp;
            t.header.frame_id = "map";
            t.child_frame_id = "odom";
            t.transform.translation.x = T_map_odom.translation().x();
            t.transform.translation.y = T_map_odom.translation().y();
            
            Eigen::Rotation2Df rot(T_map_odom.linear());
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = sin(rot.angle() / 2.0);
            t.transform.rotation.w = cos(rot.angle() / 2.0);

            _tf_broadcaster->sendTransform(t);

        } catch (tf2::TransformException &ex) {
            // È normale che fallisca nei primi secondi se l'odometria non è ancora partita
            return; 
        }
    }

    std::unique_ptr<dmap_navigation::DMap> _dmap;
    std::unique_ptr<dmap_navigation::Localizer> _localizer;
    Eigen::Isometry2f _current_pose;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}