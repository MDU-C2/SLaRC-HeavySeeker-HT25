
#include <chrono>
#include <iostream>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "s_msgs/msg/operation_modes.hpp"
#include "s_robot/status_info.hpp"

/** Class HealthCheckNode
 * 
 * This node checks the system health and informs the system about allowed operation modes, this includes
 * an heartbeat functionality. The heartbeat is a message sent every 500ms that includes information about 
 * the allowed operation modes.
 * 
 * Subscribers:
 *  - /emergency_stop - std_msgs/msg/Bool - state of emergency stop, pressed=True
 * 
 *  - /battery_info - s_msgs/msg/BatterInfo - info about current battery status
 *  - /motor_info - s_msgs/msg/MotorInfo - info about current motor status
 * 
 *  sensors
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
        allowed_modes.local_control = false;
        allowed_modes.remote_control = false;
        allowed_modes.autonomy_drive = false;

        sub_emergency_stop = this->create_subscription<std_msgs::msg::Bool>("/emergency_stop", qos_profile, std::bind(&HealthCheckNode::callback_emergency_stop, this, std::placeholders::_1));

        pub_allowed_operation_modes = this->create_publisher<s_msgs::msg::OperationModes>("/allowed_operation_modes", 10);
        
        using namespace std::chrono_literals;
        timer_health_update = this->create_timer(500ms, std::bind(&HealthCheckNode::callback_timer_health_update, this));

        RCLCPP_INFO(this->get_logger(), "Health Checks Is Running");

        // declare ros params
        this->declare_parameter<std::string>("platform.motor", "/motors_ok");
        this->declare_parameter<std::string>("platform.battery", "/battery_ok");
        this->declare_parameter<std::vector<std::string>>("manual_controllers.topics", std::vector<std::string>{});
        this->declare_parameter<std::string>("requirements.telop.fpv_camera", "");
        this->declare_parameter<std::string>("requirements.autonomy.lidar", "");
        this->declare_parameter<std::string>("requirements.autonomy.camera1", "");
        this->declare_parameter<std::string>("requirements.autonomy.camera2", "");

        telop_req_topics[0] = this->get_parameter("requirements.telop.fpv_camera").as_string();

        autonomy_req_topics[0] = this->get_parameter("requirements.autonomy.lidar").as_string();
        autonomy_req_topics[1] = this->get_parameter("requirements.autonomy.camera1").as_string();
        autonomy_req_topics[2] = this->get_parameter("requirements.autonomy.camera2").as_string();

        
        // Platform status subscribers
        sub_platform[0] = this->create_subscription<std_msgs::msg::Bool>(this->get_parameter("platform.motor").as_string(), 10, std::bind(&HealthCheckNode::callback_sub_platform_motor, this, std::placeholders::_1));
        sub_platform[1] = this->create_subscription<std_msgs::msg::Bool>(this->get_parameter("platform.battery").as_string(), 10, std::bind(&HealthCheckNode::callback_sub_platform_battery, this, std::placeholders::_1));

        // manual controllers subscribers
        controller_topics = this->get_parameter("manual_controllers.topics").as_string_array();
        for (const auto &topic : controller_topics) {
            sub_controllers.push_back(this->create_subscription<sensor_msgs::msg::Joy>(topic, 10, std::bind(&HealthCheckNode::callback_sub_manual_controllers, this, std::placeholders::_1)));
        }

        // sensor subscribtions

        // publishers
        pub_saftey_stop = this->create_publisher<std_msgs::msg::Bool>("/saftey_stop", 10);
    }

    private:
    std::string telop_req_topics[1];
    std::string autonomy_req_topics[3];
    std::vector<std::string> controller_topics;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_platform[2];
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr> sub_controllers;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_telop_requirements[1];
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_requirements[3];

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_saftey_stop;

    Status<std_msgs::msg::Bool> status_battery;
    Status<std_msgs::msg::Bool> status_motor;
    std::array<std::reference_wrapper<Status<std_msgs::msg::Bool>>, 2> status_platform = {status_motor, status_battery};

    Status<sensor_msgs::msg::Joy> status_controller;


    std_msgs::msg::Bool saftey_stop;
    rclcpp::Time current_time;

    void callback_sub_platform_motor(std_msgs::msg::Bool motorReady){
        this->status_motor.last_msg = motorReady;
        this->status_motor.time_recived = this->get_clock()->now();

        // Stop system if motors not ready
        if(this->status_motor.last_msg.data == false) {
            saftey_stop.data = true;
            pub_saftey_stop->publish(this->saftey_stop);
        }
    }

    void callback_sub_platform_battery(std_msgs::msg::Bool batteryReady){
        this->status_battery.last_msg = batteryReady;
        this->status_battery.time_recived = this->get_clock()->now();

        // Stop system if battery not ready
        if(this->status_motor.last_msg.data == false) {
            saftey_stop.data = true;
            pub_saftey_stop->publish(this->saftey_stop);
        }
    }

    void platform_saftey_check() {

        for (const Status<std_msgs::msg::Bool> &status : status_platform) { 
            // check platform message age
            if (this->get_clock()->now().nanoseconds() - status.time_recived.nanoseconds() > 600*1000000) {
                saftey_stop.data = true;
                pub_saftey_stop->publish(this->saftey_stop);
                return;
            }

            // check platform status
            if (status.last_msg.data == false) {
                saftey_stop.data = true;
                pub_saftey_stop->publish(this->saftey_stop);
                return;
            }
        }

        // all topics are up to date and delivers system ok
        saftey_stop.data = false;
        pub_saftey_stop->publish(saftey_stop);
    
    }

    void callback_sub_manual_controllers(sensor_msgs::msg::Joy joy) {
        this->status_controller.last_msg = joy;
        this->status_controller.time_recived = this->get_clock()->now();
    }

    // All things below is old ------------------------------------------------------
    private:
    // timer callback that sends hear beats to the robot node
    void callback_timer_health_update() {

        platform_saftey_check();

        //topics = this->get_topic_names_and_types();
        //std::cout << topics.begin()->first << std::endl;

        // add function for finding sensors and to update curent sensor list
        

        //checkHealth();
        
        //publishAllowedModes();
        //publishCurrentMode();
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
            allowed_modes.local_control = false;
            allowed_modes.remote_control = false;
            allowed_modes.autonomy_drive = false;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Emergency Button Relesed");
            checkHealth();
        }

        publishAllowedModes();
    }

    rclcpp::Publisher<s_msgs::msg::OperationModes>::SharedPtr pub_allowed_operation_modes;
    rclcpp::TimerBase::SharedPtr timer_health_update;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emergency_stop;

    bool h_emergency_stop;
    s_msgs::msg::OperationModes allowed_modes;

    std::map<std::string, std::vector<std::string>> topics;
    

    // general health check, allows operation modes based on system status
    void checkHealth() {
        
        // check if platform is ok to run
        //for(std::string topic: platform_topics) {

        //}

        // check if the registred sensors are delivering data



        // dont update status if emergency button is pressed
        if(this->h_emergency_stop)
            return;
        
        this->allowed_modes.local_control = true;
        this->allowed_modes.remote_control = true;
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