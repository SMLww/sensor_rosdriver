#include "mpmgs201_ros/mpmgs201_driver.h"
#include <cstring>
#include <ros/console.h>

Mpmgs201_Driver::Mpmgs201_Driver(ros::NodeHandle& nh){
    nh.param("port", port_, std::string("/dev/ttyUSB0"));
    nh.param("baudrate", baudrate_, 115200);
    nh.param("device_address", dev_addr_, 6);
    nh.param("read_frequency", freq_, 100.0); // 100 sometimes require each second

    mag_offset_pub = nh.advertise<geometry_msgs::PoseStamped>("/mag_offset", 10);
}

Mpmgs201_Driver::~Mpmgs201_Driver() {
    if (serial_.isOpen()) {
        serial_.close();
    }
}

bool Mpmgs201_Driver::init() {
    try {
        serial_.setPort(port_);
        serial_.setBaudrate(baudrate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
        serial_.setTimeout(timeout);
        serial_.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Magnetic navigation sensor port open failed: " << e.what());
        return false;
    }
    return serial_.isOpen();
}


void Mpmgs201_Driver::mainLoop() {
    ros::Rate rate(freq_);
    while (ros::ok() && serial_.isOpen()) {
        ros::spinOnce();
        if (sendCommand(read_cmd,read_cmd.size())) {
            std::string response_str = serial_.read(11);
            std::vector<uint8_t> response(response_str.begin(), response_str.end());
            parseResponse(response);
        }
        rate.sleep();
    }
}

bool Mpmgs201_Driver::sendCommand(const std::vector<uint8_t> command, const std::vector<uint8_t>::size_type size)
{
    return serial_.write(command) == size;
}

void Mpmgs201_Driver::parseResponse(const std::vector<uint8_t>& response){
    uint8_t field_num = response[3];

    if (field_num != 1){
        ROS_WARN("The number of sections is error, pleace check!!!");
        return;
    }

    geometry_msgs::PoseStamped mag_offset;

    mag_offset.header.stamp = ros::Time::now();

    mag_offset.pose.position.y = static_cast<double>(response[4] / 1000.0);

    mag_offset_pub.publish(mag_offset);
    ROS_INFO("The Y axis offset = %lf", mag_offset.pose.position.y);
}
