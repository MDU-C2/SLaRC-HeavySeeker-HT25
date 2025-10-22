
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "hs_msgs/msg/operation_modes.hpp"


class HealthCheckNode : public rclcpp::Node {

    public:
    HealthCheckNode()
     : Node("health_check"), h_emergency_stop(true) {

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // set initial allowed modes before sensor data can be read
        allowed_modes.local_telop = false;
        allowed_modes.remote_telop = false;
        allowed_modes.autonomy_drive = false;

        sub_emergency_stop = this->create_subscription<std_msgs::msg::Bool>("emergency_stop", qos_profile, std::bind(&HealthCheckNode::callback_emergency_stop, this, std::placeholders::_1));

        pub_allowed_operation_modes = this->create_publisher<hs_msgs::msg::OperationModes>("allowed_operation_modes", 10);

        using namespace std::chrono_literals;
        timer_health_update = this->create_timer(500ms, std::bind(&HealthCheckNode::callback_timer_health_update, this));

        RCLCPP_INFO(this->get_logger(), "Health Checks Is Running");
    }

    void callback_timer_health_update() {

        RCLCPP_INFO(this->get_logger(), "Timer Callback");
        
        publishAllowedModes();
        return;
    }

    void callback_emergency_stop(std_msgs::msg::Bool emergency_stop) {

        // do nothing as nothing has changed
        if (h_emergency_stop == emergency_stop.data)
            return;

        h_emergency_stop = emergency_stop.data;

        if(h_emergency_stop) {
            allowed_modes.local_telop = false;
            allowed_modes.remote_telop = false;
            allowed_modes.autonomy_drive = false;
        }
        else checkHealth();
        
        timer_health_update.reset();
        publishAllowedModes();
    }

    private:

    rclcpp::Publisher<hs_msgs::msg::OperationModes>::SharedPtr pub_allowed_operation_modes;
    rclcpp::TimerBase::SharedPtr timer_health_update;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emergency_stop;

    bool h_emergency_stop;
    hs_msgs::msg::OperationModes allowed_modes;
    

    // function indicates allowed operation modes based on non critical parameters
    void checkHealth() {

        allowed_modes.local_telop = true;
        allowed_modes.remote_telop = true;
        allowed_modes.autonomy_drive = true;

        return;
    }

    inline void publishAllowedModes() {
        this->pub_allowed_operation_modes->publish(allowed_modes);
    }

};

int main (int argv, char* argc[]) {

    rclcpp::init(argv, argc);
    rclcpp::spin(std::make_shared<HealthCheckNode>());
    rclcpp::shutdown();

}