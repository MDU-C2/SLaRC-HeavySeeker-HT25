
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "hs_msgs/msg/operation_modes.hpp"


/** Class RobotNode
 * 
 * This node control subscribes to control inputs for manual and autonomous drive and based on allowed and selected modes 
 * publish the correct control input to /cmd_vel
 * 
 * Subscribers:
 *  - /allowed_operation_modes - hs_msgs/msg/OperationModes - information about allowed operation modes, this is the heartbeat
 *  - /telop_cmd_vel - geometry_msgs/msg/TwistStamped - cmd_vel for manual control
 *  - /auto_cmd_vel - geometry_msgs/msg/TwistStamped - cmd_vel for autonomous control
 *  - /activate_autonomous_drive - std_msgs/msg/Bool - signal for robot node to activate autonomous drive
 * 
 * Publishers:
 *  - /cmd_vel - geometry_msgs/msg/TwistStamped - cmd_vel
 *  - /autonomous_drive_active - std_msgs/msg/Bool - shows the current status for autonomous drive
 * 
 */
class RobotNode : public rclcpp::Node {

    public:
    RobotNode()
     : Node("robot_node") {
        using std::placeholders::_1;

        sub_allowed_operation_modes = this->create_subscription<hs_msgs::msg::OperationModes>("/allowed_operation_modes", 10, std::bind(&RobotNode::callback_operation_modes, this, _1));

        sub_remote_telop_cmd_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>("/telop_cmd_vel", 10,  std::bind(&RobotNode::callback_remote_telop_cmd, this, _1));
        sub_auto_cmd_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>("/auto_cmd_vel", 10,  std::bind(&RobotNode::callback_auto_cmd, this, _1));
        sub_activate_autonomous_drive = this->create_subscription<std_msgs::msg::Bool>("/activate_autonomous_drive", 10, std::bind(&RobotNode::callback_activate_autonomous_drive, this, _1));
        
        pub_cmd_vel = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        pub_autonomous_drive_status = this->create_publisher<std_msgs::msg::Bool>("/autonomous_drive_active", 10);

        // timer to check if hearbeat is missing
        restartTimerHeartBeat();
        // timer to stop driving if cmd_vel is missing or old
        restartTimerSoftStop();

        // timer for showing current status
        using namespace std::chrono_literals;
        this->timer_autonomous_drive_status = this->create_timer(500ms, std::bind(&RobotNode::callback_autonomous_drive_status, this));
    }

    private:

    void callback_timer_heart_beat() {

        RCLCPP_WARN(this->get_logger(), "Missing heart beat, NO operation allowed");

        this->allow_local_control = false;
        this->allow_remote_control = false;
        this->allow_auto = false;

        sendZeroVel();
        stopTimer(this->timer_heart_beat);
    }

    void callback_timer_soft_stop() {

        // if cmd_vel is older then 100ms, cmd_vel is considered missing
        if(abs(this->time_last_cmd_vel.nanoseconds() - this->now().nanoseconds()) < 100*pow(10, 6))
            return;

        RCLCPP_WARN(this->get_logger(), "Missing cmd vel, setting velocity to 0");
        
        sendZeroVel();
        stopTimer(this->timer_soft_stop);
    }

    void callback_autonomous_drive_status() {
        std_msgs::msg::Bool status;
        status.data = this->autonomous_drive;

        this->pub_autonomous_drive_status->publish(status);

    }

    void callback_operation_modes(hs_msgs::msg::OperationModes allowed_modes) {

        if(timer_heart_beat == nullptr)
            RCLCPP_INFO(this->get_logger(), "Heart Beat is back, operation allowed");

        restartTimerHeartBeat();
        this->allow_local_control = allowed_modes.local_control;
        this->allow_remote_control = allowed_modes.remote_control;
        this->allow_auto = allowed_modes.autonomy_drive;

        if (!this->allow_auto) {
            if (this->autonomous_drive)
                RCLCPP_INFO(this->get_logger(), "Autonomous drive is deactivated");
                
            this->autonomous_drive = false;
        }
    }

    void callback_activate_autonomous_drive(std_msgs::msg::Bool activate_autonomous_drive) {
        this->autonomous_drive = activate_autonomous_drive.data;

        if (this->autonomous_drive)
            RCLCPP_INFO(this->get_logger(), "Autonomous drive is activated");
        else
            RCLCPP_INFO(this->get_logger(), "Autonomous drive is deactivated");
    }

    void callback_remote_telop_cmd(geometry_msgs::msg::TwistStamped cmd_vel) {
        // deactivate auto mode
        this->autonomous_drive = false;
        
        // check if manual drive is allowed
        if(!this->allow_remote_control)
            return;
        

        restartTimerSoftStop();
        publishCmdVel(cmd_vel);
    }

    void callback_auto_cmd(geometry_msgs::msg::TwistStamped cmd_vel) {
        
        // check if mode auto is True
        if(!this->autonomous_drive)
            return;
        
        // check if autonomous drive is allowed
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_activate_autonomous_drive;

    rclcpp::TimerBase::SharedPtr timer_soft_stop;
    rclcpp::TimerBase::SharedPtr timer_heart_beat;
    rclcpp::TimerBase::SharedPtr timer_autonomous_drive_status;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_autonomous_drive_status;

    bool autonomous_drive = false;
    bool local_control_active = false; // currently not used

    bool allow_local_control;
    bool allow_remote_control;
    bool allow_auto;

    rclcpp::Time time_last_cmd_vel;

    void publishCmdVel(geometry_msgs::msg::TwistStamped& cmd_vel) {
        this->time_last_cmd_vel = cmd_vel.header.stamp;
        
        if(abs(this->time_last_cmd_vel.nanoseconds() - this->now().nanoseconds()) > 100*pow(10, 6)) {
            RCLCPP_WARN(this->get_logger(), "Old cmd_vel, ignoring");
            return;
        }

        this->pub_cmd_vel->publish(cmd_vel);
    }

    inline void sendZeroVel() {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        publishCmdVel(cmd_vel);
    }

    inline void stopTimer(rclcpp::TimerBase::SharedPtr& timer) {
        timer.reset();
    }

    inline void restartTimerSoftStop() {
        using namespace std::chrono_literals;
        stopTimer(this->timer_soft_stop);
        this->timer_soft_stop = this->create_timer(100ms, std::bind(&RobotNode::callback_timer_soft_stop, this));
    }

    inline void restartTimerHeartBeat() {
        using namespace std::chrono_literals;
        stopTimer(this->timer_heart_beat);
        this->timer_heart_beat = this->create_timer(600ms, std::bind(&RobotNode::callback_timer_heart_beat, this));
    }
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}