#include "pingpong_tester/core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.h>

namespace GunControllerTesters
{
GunControllerTester::GunControllerTester() : rclcpp_lifecycle::LifecycleNode("pingpong_tester_node"), time_(0), offset_(OFFSET_DEFAULT), switch_period_(SWITCH_PERIOD_DEFAULT)
{
    RCLCPP_INFO(get_logger(), "Ping Pong Tester Node constructed.");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerTester::on_configure(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on configure");

    this->declare_parameter<float>("offset", OFFSET_DEFAULT);
    this->declare_parameter<float>("switch_period", SWITCH_PERIOD_DEFAULT);
    this->offset_ = this->get_parameter("offset").as_double();
    this->switch_period_ = this->get_parameter("switch_period").as_double();
    RCLCPP_INFO(get_logger(), "Parameters: {offset: %f, switch_period: %f", this->offset_, this->switch_period_);

    this->velocity_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pico/gun/velocity", 10);
    this->pose_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/pico/pose", 10);
    this->loading_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/pico/loading/deg", 10);
    this->position_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/pico/stepper/position/raw", 10);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerTester::on_activate(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on activate");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_PERIOD_MS), std::bind(&GunControllerTester::timer_callback, this));
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerTester::on_deactivate(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on deactivate");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerTester::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on cleanup");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GunControllerTester::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "on shutdown");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GunControllerTester::set_velocity(float p, float d)
{
    RCLCPP_INFO(get_logger(), "Publish velocity: %f, %f", p, d);

    std_msgs::msg::Float32MultiArray msg;
    msg.data = {p, d};
    this->velocity_publisher_->publish(msg);
}

void GunControllerTester::set_pose(float r, float p, float y)
{
    RCLCPP_INFO(get_logger(), "Publish pose: %f, %f, %f", r, p, y);

    geometry_msgs::msg::Vector3 msg;
    msg.x = r;
    msg.p = p;
    msg.y = y;
    this->pose_publisher_->publish(msg);
}

void GunControllerTester::set_loading(bool tf)
{
    deg = tf ? 10 : 130;

    RCLCPP_INFO(get_logger(), "Publish loading: %f", deg);

    std_msgs::msg::Float32 msg;
    msg.data = deg;
    this->loading_publisher_->publish(msg);
}

void GunControllerTester::set_position(float p)
{
    RCLCPP_INFO(get_logger(), "Publish position: %f", p);

    std_msgs::msg::Float32 msg;
    msg.data = p;
    this->position_publisher_->publish(msg);
}

void GunControllerTester::behavior_pattern()
{
    if(S2MS(0 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(1 * this->switch_period_ + this->offset_)) set_velocity(100.0f, -180.0f);
    else if(S2MS(1 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(2 * this->switch_period_ + this->offset_)) set_velocity(100.0f, 0.0f);
    else if(S2MS(2 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(3 * this->switch_period_ + this->offset_)) set_velocity(100.0f, 180.0f);
    else if(S2MS(3 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(4 * this->switch_period_ + this->offset_)) set_velocity(0.0f, -180.0f);
    else if(S2MS(4 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(5 * this->switch_period_ + this->offset_)) set_velocity(0.0f, 0.0f);
    else if(S2MS(5 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(6 * this->switch_period_ + this->offset_)) set_velocity(0.0f, 180.0f);
    else if(S2MS(6 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(7 * this->switch_period_ + this->offset_)) set_velocity(-100.0f, -180.0f);
    else if(S2MS(7 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(8 * this->switch_period_ + this->offset_)) set_velocity(-100.0f, 0.0f);
    else if(S2MS(8 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(9 * this->switch_period_ + this->offset_)) set_velocity(-100.0f, 180.0f);
    else set_velocity(0, 0);
}

void GunControllerTester::behavior_hard()
{
    if(S2MS(0 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(1 * this->switch_period_ + this->offset_)) set_velocity(100.0f, -180.0f);
    else if(S2MS(1 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(2 * this->switch_period_ + this->offset_)) set_velocity(-100.0f, 180.0f);
    else if(S2MS(2 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(3 * this->switch_period_ + this->offset_)) set_velocity(100.0f, -180.0f);
    else if(S2MS(3 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(4 * this->switch_period_ + this->offset_)) set_velocity(-100.0f, 180.0f);
    else if(S2MS(4 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(5 * this->switch_period_ + this->offset_)) set_velocity(100.0f, -180.0f);
    else if(S2MS(5 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(6 * this->switch_period_ + this->offset_)) set_velocity(-100.0f, 180.0f);
    else if(S2MS(6 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(7 * this->switch_period_ + this->offset_)) set_velocity(100.0f, -180.0f);
    else if(S2MS(7 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(8 * this->switch_period_ + this->offset_)) set_velocity(-100.0f, 180.0f);
    else if(S2MS(8 * this->switch_period_ + this->offset_) <= this->time_ && this->time_ < S2MS(9 * this->switch_period_ + this->offset_)) set_velocity(100.0f, -180.0f);
    else set_velocity(0, 0);
}

void GunControllerTester::timer_callback()
{
    this->time_ = this->time_ + TIMER_PERIOD_MS;
    // this->behavior_pattern();
    // this->behavior_hard();
}
}


