#pragma once

#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>


const std::string MOTOR_ADAPTER_ID = "000D000A5656430C20393039";
const std::string BATTERY_ADAPTER_ID = "8004001C5656430C20393039";
const std::string BITRATE_str = "1000000";

class CanBus {
    
    public:
    CanBus(std::string adapter_ID);
    CanBus(std::string interface_Name, bool use_ifacename);
    ~CanBus();

    void send_frame(const struct can_frame &frame);
    int recive_frame(struct can_frame &frame);

    private:
    void get_interface_from_ID(const std::string &adapter_ID);
    void bring_up_can();
    void setup_socket();
    
    std::string m_interface;
    
    int m_socket;
    struct ifreq m_ifr;
    struct sockaddr_can m_addr;
    

};



