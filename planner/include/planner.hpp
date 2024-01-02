// WARNING: non-standard pragma
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <obstacles_msgs/msg/obstacle_array_msg.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <latch>
#include <vector>
#include <execution>
#include <ranges>
#include "offsetting.hpp"
#include "coordinating.hpp"
#include "clipper.h"
#include "multi_dubins.hpp"

// since the visilibity library has some warnings that we can't really handle
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "visilibity.hpp"
#pragma GCC diagnostic pop

#define n_data 6

class Planner : public rclcpp::Node{
    private:
        // enviroment for collision checks
        Clipper2Lib::PathsD min_env_;
        // Visilibity enviroment
        VisiLibity::Environment env_;

        // this can be modular but for now there are 3 robots
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub0_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub2_;

        // auxiliary variables to handle concurrency
        std::latch data_{n_data};
        rclcpp::CallbackGroup::SharedPtr cb_group_;

        // subscribers needed to gather necessary data
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub0_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub1_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub2_;
        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacles_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr borders_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr gate_sub_;

        // msg to save necessary data
        geometry_msgs::msg::Pose pos0_;
        geometry_msgs::msg::Pose pos1_;
        geometry_msgs::msg::Pose pos2_;
        obstacles_msgs::msg::ObstacleArrayMsg obstacles_msg_;
        geometry_msgs::msg::Polygon borders_msg_;
        geometry_msgs::msg::PoseArray gate_msg_;

        rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr client0_;
        rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr client1_;
        rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr client2_;
        
        // callbacks
        void obstacles_callback(obstacles_msgs::msg::ObstacleArrayMsg);
        void borders_callback(geometry_msgs::msg::Polygon);
        void gate_callback(geometry_msgs::msg::PoseArray);
        void sub0_callback(geometry_msgs::msg::PoseWithCovarianceStamped);
        void sub1_callback(geometry_msgs::msg::PoseWithCovarianceStamped);
        void sub2_callback(geometry_msgs::msg::PoseWithCovarianceStamped);

    protected:
        dubins::d_curve get_safe_curve(VisiLibity::Point, VisiLibity::Point, VisiLibity::Point, VisiLibity::Point&, double);
        dubins::d_curve sample_curve(VisiLibity::Point, double, VisiLibity::Point, double, double);
        nav_msgs::msg::Path get_path_msg(const Clipper2Lib::PathD&);
        
    public:
        // robot constraints
        // minimum radius for curves
        static inline constexpr double min_r = .5;
        // by assuming the diameter of the circle is 0.8 (instead of 0.5 * sqrt(2)) we are already adding some safety to our paths
        static inline constexpr double hrobot_sz = .4;
        // static inline constexpr double velocity = x; // not needed?

        Planner();
        void plan();
        multi_dubins::path_t dubins_path(const VisiLibity::Polyline&, double, double, double);
        // void test();
};
