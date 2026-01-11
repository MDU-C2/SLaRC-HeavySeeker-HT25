
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "s_robot/can_bus.hpp"

/** Class TestNode
 * 
 */
class TestNode : public rclcpp::Node {

    public:
    TestNode()
     : Node("test_node"), battery_bus(MOTOR_ADAPTER_ID) {


        using namespace std::chrono_literals;
        timer = this->create_timer(100ms, std::bind(&TestNode::callback_timer, this));

        RCLCPP_INFO(this->get_logger(), "Test Node Is Running");
    
            
       
    }

    private:
    // timer callback that sends hear beats to the robot node
    void callback_timer() {


        if (battery_bus.recive_frame(frame) > 0) {
            RCLCPP_INFO(this->get_logger(), "DATA_BEGIN");
            std::cout << "ID: " << std::hex << frame.can_id
                << " Data: ";
            for (int i = 0; i < frame.can_dlc; i++)
                std::cout << std::hex << (int)frame.data[i] << " ";
            std::cout << std::endl;
            RCLCPP_INFO(this->get_logger(), "DATA_END");
        }
        else
            RCLCPP_INFO(this->get_logger(), "NO_DATA");


        return;
    }

    rclcpp::TimerBase::SharedPtr timer;
    CanBus battery_bus;
    struct can_frame frame {};

};

int main (int argv, char* argc[]) {

    rclcpp::init(argv, argc);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();

}