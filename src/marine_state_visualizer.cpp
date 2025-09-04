#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"


class MarineStateVisualizer : public rclcpp::Node {
public:
    MarineStateVisualizer() : rclcpp::Node("marine_state_visualizer"),
        position_{0.0, 0.0}, velocity_{0.0, 0.0}, last_time_(this->now()) {
        sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "marine_state", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 2) {
                    // Integrate acceleration to velocity and position
                    rclcpp::Time now = this->now();
                    double dt = (now - last_time_).seconds();
                    last_time_ = now;
                    double ax = msg->data[0];
                    double ay = msg->data[1];
                    velocity_[0] += ax * dt;
                    velocity_[1] += ay * dt;
                    position_[0] += velocity_[0] * dt;
                    position_[1] += velocity_[1] * dt;

                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = now;
                    marker.ns = "marine_state";
                    marker.id = 0;
                    marker.type = visualization_msgs::msg::Marker::SPHERE;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.pose.position.x = position_[0];
                    marker.pose.position.y = position_[1];
                    marker.pose.position.z = 0.0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.2;
                    marker.scale.y = 0.2;
                    marker.scale.z = 0.2;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    pub_->publish(marker);
                }
            }
        );
        pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marine_state_marker", 10);
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    std::array<double, 2> position_;
    std::array<double, 2> velocity_;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarineStateVisualizer>());
    rclcpp::shutdown();
    return 0;
}
