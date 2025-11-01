#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

class SquareMover : public rclcpp::Node
{
public:
    SquareMover() : Node("square_mover"), state_(0)
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&SquareMover::pose_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&SquareMover::control_loop, this));

        start_x_ = 5.5;
        start_y_ = 5.5;
        start_theta_ = 0.0;
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        pose_ = *msg;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist msg;

        switch (state_)
        {
        case 0: // mulai jalan lurus
            start_x_ = pose_.x;
            start_y_ = pose_.y;
            state_ = 1;
            break;

        case 1: // jalan lurus
        {
            msg.linear.x = 1.5;
            msg.angular.z = 0.0;

            double dx = pose_.x - start_x_;
            double dy = pose_.y - start_y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist >= 2.0)
            {
                start_theta_ = pose_.theta;
                state_ = 2;
            }
            break;
        }

        case 2: // belok 90 derajat
        {
            msg.linear.x = 0.0;
            msg.angular.z = 1.2;

            double delta = pose_.theta - start_theta_;
            if (delta < 0)
                delta += 2 * M_PI;

            if (delta >= M_PI / 2.0)
            {
                state_ = 0; // ulang siklus: lurus lagi
            }
            break;
        }
        }

        pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose pose_;
    int state_;
    double start_x_, start_y_, start_theta_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareMover>());
    rclcpp::shutdown();
    return 0;
}
