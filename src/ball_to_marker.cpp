#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define VISUALISATION_BALL_DIAMETER 0.1 // m

class PointToMarker : public rclcpp::Node
{
public:
    PointToMarker() : Node("PointToMarker")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_marker", 10);
        subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "vision/ball", 1,
            [this](geometry_msgs::msg::PointStamped::SharedPtr point) {
                publisher_->publish(convert(*point));
            });
    }

private:
    visualization_msgs::msg::Marker convert(geometry_msgs::msg::PointStamped &point)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = point.header;
        marker.ns = "";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.point.x;
        marker.pose.position.y = point.point.y;
        marker.pose.position.z = point.point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = VISUALISATION_BALL_DIAMETER;
        marker.scale.y = VISUALISATION_BALL_DIAMETER;
        marker.scale.z = VISUALISATION_BALL_DIAMETER;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
};

int
main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointToMarker>());
    rclcpp::shutdown();
    return 0;
}