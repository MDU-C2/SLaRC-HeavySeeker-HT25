
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "hs_msgs/msg/operation_modes.hpp"


class RobotNode : public rclcpp::Node {

    public:
    RobotNode()
     : Node("robot_node") {
        using std::placeholders::_1;

        sub_allowed_operation_modes = this->create_subscription<hs_msgs::msg::OperationModes>("allowed_operation_modes", 10, std::bind(&RobotNode::callback_operation_modes, this, _1));

        sub_remote_telop_cmd_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>("telop_cmd_vel", 10,  std::bind(&RobotNode::callback_remote_telop_cmd, this, _1));
        sub_auto_cmd_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>("auto_cmd_vel", 10,  std::bind(&RobotNode::callback_auto_cmd, this, _1));
        
        pub_cmd_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

        restartTimerHeartBeat();
        restartTimerSoftStop();
    }

    private:

    void callback_timer_heart_beat() {

        RCLCPP_WARN(this->get_logger(), "Missing heart beat, NO operation allowed");

        allow_local_telop = false;
        allow_remote_telop = false;
        allow_auto = false;

        geometry_msgs::msg::TwistStamped cmd_vel;
        publishCmdVel(cmd_vel);

        stopTimer(timer_heart_beat);
    }

    void callback_timer_soft_stop() {

        if(abs(this->time_last_cmd_vel.nanoseconds() - this->now().nanoseconds()) < 50*pow(10, 6))
            return;

        RCLCPP_WARN(this->get_logger(), "Missing cmd vel, setting velocity to 0");
        
        geometry_msgs::msg::TwistStamped cmd_vel;
        publishCmdVel(cmd_vel);
        
        stopTimer(this->timer_soft_stop);
    }

    void callback_operation_modes(hs_msgs::msg::OperationModes allowed_modes) {

        if(timer_heart_beat == nullptr)
            RCLCPP_INFO(this->get_logger(), "Heart Beat is back, operation allowed");

        restartTimerHeartBeat();
        allow_local_telop = allowed_modes.local_telop;
        allow_remote_telop = allowed_modes.remote_telop;
        allow_auto = allowed_modes.autonomy_drive;
    }


    void callback_remote_telop_cmd(geometry_msgs::msg::TwistStamped cmd_vel) {
        // deactivate auto maode
        this->m_mode_auto = false;
        
        if(!allow_remote_telop)
            return;
        

        restartTimerSoftStop();
        publishCmdVel(cmd_vel);
    }

    void callback_auto_cmd(geometry_msgs::msg::TwistStamped cmd_vel) {
        
        if(!this->m_mode_auto)
            return;
        
        if(!this->allow_auto)
            return;

        restartTimerSoftStop();
        publishCmdVel(cmd_vel);
    }

    private:
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_remote_telop_cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_auto_cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_op_mode;
    
    rclcpp::Subscription<hs_msgs::msg::OperationModes>::SharedPtr sub_allowed_operation_modes;

    rclcpp::TimerBase::SharedPtr timer_soft_stop;
    rclcpp::TimerBase::SharedPtr timer_heart_beat;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel;

    bool m_mode_auto = false;
    bool local_telop_active = false;

    bool allow_local_telop;
    bool allow_remote_telop;
    bool allow_auto;

    rclcpp::Time time_last_cmd_vel;

    inline void publishCmdVel(geometry_msgs::msg::TwistStamped cmd_vel) {
        this->time_last_cmd_vel = cmd_vel.header.stamp;
        this->pub_cmd_vel->publish(cmd_vel);
    }

    inline void stopTimer(rclcpp::TimerBase::SharedPtr& timer) {
        timer.reset();
    }

    inline void restartTimerSoftStop() {
        using namespace std::chrono_literals;
        this->timer_soft_stop.reset();
        this->timer_soft_stop = this->create_timer(100ms, std::bind(&RobotNode::callback_timer_soft_stop, this));
    }

    inline void restartTimerHeartBeat() {
        using namespace std::chrono_literals;
        this->timer_heart_beat.reset();
        this->timer_heart_beat = this->create_timer(600ms, std::bind(&RobotNode::callback_timer_heart_beat, this));
    }
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}