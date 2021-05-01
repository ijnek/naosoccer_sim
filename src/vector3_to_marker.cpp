#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class Vector3ToMarker : public rclcpp::Node
{
public:
    Vector3ToMarker() : Node("Vector3ToMarker")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/ball_marker", 10);
        subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/vision/ball", 1,
            [this](geometry_msgs::msg::Vector3Stamped::SharedPtr vector) {
                publisher_->publish(convert(*vector));
            });
    }

private:
    visualization_msgs::msg::Marker convert(geometry_msgs::msg::Vector3Stamped &vector)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = vector.header;
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point p_start;
        marker.points.push_back(p_start);
        geometry_msgs::msg::Point p_end;
        p_end.x = vector.vector.x;
        p_end.y = vector.vector.y;
        p_end.z = vector.vector.z;
        marker.points.push_back(p_end);

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.04;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr subscriber_;
};

int
main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Vector3ToMarker>());
    rclcpp::shutdown();
    return 0;
}