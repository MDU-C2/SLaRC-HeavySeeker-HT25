
#include <iostream>
#include <string>
#include <cstring>
#include <fstream>
#include <array>
#include <memory>
#include <algorithm>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>


#include "can_bus.hpp"

void runCmd(std::string &buffer, const std::string &cmd) {
    std::array<char, 128> local_buffer;
    
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }

    while(fgets(local_buffer.data(), static_cast<int>(local_buffer.size()), pipe.get()) != nullptr) {
        buffer += local_buffer.data();
    }    
}

CanBus::CanBus(std::string adapter_ID) {
    
    get_interface_from_ID(adapter_ID);

    std::cout << "Interface to use: " << m_interface << std::endl;
    bring_up_can();
    setup_socket();
 
}

CanBus::CanBus(std::string interface_Name, bool use_ifacename) {
    m_interface = interface_Name;

    std::cout << "Interface to use: " << m_interface << std::endl;
    bring_up_can();
    setup_socket();
 

}

CanBus::~CanBus() {

    close(m_socket);

    //bring down interface
    std::string set_down = "sudo ip link set " + m_interface + " down";
    std::system(set_down.c_str());
    
}

void CanBus::send_frame(const struct can_frame &frame) {

    if (write(m_socket, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("write to can socket");
        return;
    }
}

int CanBus::recive_frame(struct can_frame &frame) {

    return read(m_socket, &frame, sizeof(frame));

}

void CanBus::get_interface_from_ID(const std::string &adapter_ID){
        
    std::string interfaces;
    std::string interface_to_check;
    std::string buffer;
    std::string cmd;

    // check for available interfaces
    cmd = "ls /sys/class/net/ | grep can";
    runCmd(interfaces, cmd);

    size_t num_interfaces = std::count(interfaces.begin(), interfaces.end(), '\n');
    
    for (int i = 0; i < num_interfaces; i++) {
        interface_to_check = interfaces.substr(0, interfaces.find('\n'));
        interfaces.erase(0, interfaces.find('\n')+1);

        cmd = "udevadm info /sys/class/net/" + interface_to_check + " | grep ID_SERIAL_SHORT";
        buffer = "";
        runCmd(buffer, cmd);

        buffer.erase(0, buffer.find('=')+1);
        buffer.erase(buffer.find('\n'));
        if (buffer.compare(adapter_ID) == 0) {
            m_interface = interface_to_check;
            return;
        }
    }

    // error for not finding the adapter
    std::cout << "Could Not Find ID" << m_interface << std::endl;
    


}

void CanBus::bring_up_can() {
    std::string set_down = "sudo ip link set " + m_interface + " down";
    std::string settings = "sudo ip link set " + m_interface + " type can bitrate " + BITRATE_str;
    std::string set_up = "sudo ip link set " + m_interface + " up";

    std::system(set_down.c_str());
    std::system(settings.c_str());
    std::system(set_up.c_str());

    //is slepp required
    usleep(250000);

}

void CanBus::setup_socket() {

    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socket < 0) {
        perror("socket");
        return;
    }

    std::strcpy(m_ifr.ifr_name, m_interface.c_str());
    ioctl(m_socket, SIOCGIFINDEX, &m_ifr);

    memset(&m_addr, 0, sizeof(m_addr));
    m_addr.can_family = AF_CAN;
    m_addr.can_ifindex = m_ifr.ifr_ifindex;

    if (bind(m_socket, (struct sockaddr *)&m_addr, sizeof(m_addr)) < 0) {
        perror("Bind");
        return;
    }

}
