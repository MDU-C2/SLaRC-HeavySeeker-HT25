
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "hs_msgs/msg/operation_modes.hpp"

/** Class HealthCheckNode
 * 
 * This node checks the system health and informs the system about allowed operation modes, this includes
 * an heartbeat functionality. The heartbeat is a message sent every 500ms that includes information about 
 * the allowed operation modes.
 * 
 * Subscribers:
 *  - /emergency_stop - std_msgs/msg/Bool - state of emergency stop, pressed=True
 * 
 * Publishers:
 *  - /allowed_operation_modes - hs_msgs/msg/operation_modes - information about allowed operation modes
 */
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

        sub_emergency_stop = this->create_subscription<std_msgs::msg::Bool>("/emergency_stop", qos_profile, std::bind(&HealthCheckNode::callback_emergency_stop, this, std::placeholders::_1));

        pub_allowed_operation_modes = this->create_publisher<hs_msgs::msg::OperationModes>("/allowed_operation_modes", 10);

        using namespace std::chrono_literals;
        timer_health_update = this->create_timer(500ms, std::bind(&HealthCheckNode::callback_timer_health_update, this));

        RCLCPP_INFO(this->get_logger(), "Health Checks Is Running");
    }

    private:
    // timer callback that sends hear beats to the robot node
    void callback_timer_health_update() {

        checkHealth();
        
        publishAllowedModes();
        return;
    }

    // callback for emergency button
    void callback_emergency_stop(std_msgs::msg::Bool emergency_stop) {

        // do nothing as nothing has changed
        if (h_emergency_stop == emergency_stop.data)
            return;

        h_emergency_stop = emergency_stop.data;

        if(h_emergency_stop) {
            RCLCPP_INFO(this->get_logger(), "Emergency Button Pressed");
            allowed_modes.local_telop = false;
            allowed_modes.remote_telop = false;
            allowed_modes.autonomy_drive = false;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Emergency Button Relesed");
            checkHealth();
        }

        publishAllowedModes();
    }

    rclcpp::Publisher<hs_msgs::msg::OperationModes>::SharedPtr pub_allowed_operation_modes;
    rclcpp::TimerBase::SharedPtr timer_health_update;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emergency_stop;

    bool h_emergency_stop;
    hs_msgs::msg::OperationModes allowed_modes;
    

    // general health check, allows operation modes based on system status
    void checkHealth() {
        
        // dont update status if emergency button is pressed
        if(this->h_emergency_stop)
            return;
        
        this->allowed_modes.local_telop = true;
        this->allowed_modes.remote_telop = true;
        this->allowed_modes.autonomy_drive = true;

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