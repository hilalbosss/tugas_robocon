#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <cmath>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class TurtleFlower : public rclcpp::Node
{
public:
    TurtleFlower() : Node("turtle_flower")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        std::this_thread::sleep_for(2s);
        draw_flower();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;

    void stop()
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
    }

    void set_pen(bool setpen)
    {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request-> r = 255;
        request-> g = 255;
        request-> b = 255;
        request-> width = 2;
        request-> off = setpen? 0: 1;

        pen_client_->async_send_request(request);
        std::this_thread::sleep_for(100ms);
    }

    void curve(double linear_speed, double angular_speed, double duration)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear_speed;
        msg.angular.z = angular_speed;

        auto start = this->now();
        rclcpp::Rate rate(100);
        while ((this->now() - start).seconds() < duration)
        {
            publisher_->publish(msg);
            rate.sleep();
        }
        stop();
    }

    void rotate(double angular_speed_deg, double angle_deg, bool clockwise)
    {
        geometry_msgs::msg::Twist msg;
        double angular_speed = angular_speed_deg * M_PI / 180.0;
        msg.angular.z = clockwise ? -angular_speed : angular_speed;

        auto start = this->now();
        rclcpp::Rate rate(100);
        while ((this->now() - start).seconds() < angle_deg / angular_speed_deg)
        {
            publisher_->publish(msg);
            rate.sleep();
        }
        stop();
    }

    void draw_petal()
    {
        curve(0.8, 1.0, 2.0);
        curve(0.8, -1.0, 2.0);
    }

    void draw_petals()
    {
        int petals = 6; 
        for (int i = 0; i < petals; i++)
        {
            RCLCPP_INFO(this->get_logger(), "Menggambar kelopak ke-%d", i + 1);
            draw_petal();
            rotate(30, 60, false); 
        }
    }

    void draw_flower()
    {
        draw_petals();

        set_pen(false);

        rotate(30, 107, false);
        rclcpp::Rate rate(100);
        auto start = this->now();
    
        start= this->now();
        while ((this->now() - start).seconds() < 2.65)
        {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 1.0;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
            rate.sleep();
        }

        set_pen(true);

        stop();
        start= this->now();
        while ((this->now() - start).seconds() < 6.0)
        {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.5;
            msg.angular.z = 1.5;
            publisher_->publish(msg);
            rate.sleep();
        }

        stop();
        set_pen(false);

        start= this->now();
        while ((this->now() - start).seconds() < 3.0)
        {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 1.0;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleFlower>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
