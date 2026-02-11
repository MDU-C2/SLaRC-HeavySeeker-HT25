#pragma once

#include <rclcpp/rclcpp.hpp>

template<typename T> struct Status {
    
    public:
    T last_msg;
    rclcpp::Time time_recived;
};