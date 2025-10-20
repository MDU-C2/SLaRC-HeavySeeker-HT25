
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"



class HealthCheckNode : public rclcpp::Node {

    public:
    HealthCheckNode()
     : Node("health_check"), h_emergency_stop(true){

        using std::placeholders::_1;

        allow_auto.data = false;
        allow_telop.data = false;

        
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        sub_emergency_stop = this->create_subscription<std_msgs::msg::Bool>("emergency_stop", qos_profile, std::bind(&HealthCheckNode::callback_emergency_stop, this, _1));

        pub_health_allow_telop = this->create_publisher<std_msgs::msg::Bool>("health_allow_telop", 5);
        pub_health_allow_auto = this->create_publisher<std_msgs::msg::Bool>("health_allow_auto", 5);
    }

    void callback_emergency_stop(std_msgs::msg::Bool emergency_stop) {

        RCLCPP_INFO(this->get_logger(), "Stop: %d", (int) emergency_stop.data);

        h_emergency_stop = emergency_stop.data;

        checkHealth();
        publishAllowedModes();
    }

    private:

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_health_allow_telop;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_health_allow_auto;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emergency_stop;

    bool h_emergency_stop;

    std_msgs::msg::Bool allow_telop;
    std_msgs::msg::Bool allow_auto;

    

    // function indicates allowed operation modes based on critical parameters
    void checkHealth() {

        // maybe turn this around to start as false
        allow_telop.data = true;
        allow_auto.data = true;

        if(h_emergency_stop) {
            allow_telop.data = false;
            allow_auto.data = false;
            return;
        }
    
        return;
    }

    void publishAllowedModes() {
        this->pub_health_allow_telop->publish(allow_telop);
        this->pub_health_allow_auto->publish(allow_auto); 
    }


};




int main (int argv, char* argc[]) {

    rclcpp::init(argv, argc);
    rclcpp::spin(std::make_shared<HealthCheckNode>());
    rclcpp::shutdown();
    
}