#ifndef __CORE_HPP__
#define __CORE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#define DT 2
#define TIMER_PERIOD_MS 1

#define OFFSET_DEFAULT 2
#define SWITCH_PERIOD_DEFAULT 1

#define S2MS(second) ((second) * 1000)

namespace GunControllerTesters
{
class GunControllerTester : public rclcpp_lifecycle::LifecycleNode
{
    public:
        GunControllerTester();
    protected:
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    private:
        void timer_callback();
        void set_velocity(float, float);
        void set_pose(float, float, float);
        void set_loading(bool);
        void set_position(float);
        void behavior_pattern();
        void behavior_hard();

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pose_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr loading_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        unsigned int time_;
        float offset_;
        float switch_period_;
};
}

#endif

