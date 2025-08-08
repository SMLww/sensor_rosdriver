#include "mpmgs201_ros/mpmgs201_driver.h"
#include <cstring>
#include <ros/console.h>

Mpmgs201_Driver::Mpmgs201_Driver(ros::NodeHandle& nh){
    nh.param("port", port_, std::string("/dev/ttyUSB0"));
    nh.param("baudrate", baudrate_, 115200);
    nh.param("device_address", dev_addr_, 6);
    nh.param("read_frequency", freq_, 200.0); // 需要高于115200对应的100hz固定发送频率

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
    while (ros::ok() && serial_.available()) {
        /* communication protocol RS-485*/
        // if (sendCommand(read_cmd,read_cmd.size())) {
        //     std::string response_str = serial_.read(11);
        //     std::vector<uint8_t> response(response_str.begin(), response_str.end());
        //     parseResponse(response);
        // }

        /* communication protocol RS-232*/
        size_t n = serial_.read(&buffer[buffer_len], sizeof(buffer) - buffer_len);
        buffer_len += n;

        for (size_t i = 0; i < buffer_len; ++i) {
            if (buffer[i] == 0x4D && (buffer_len - i) >= 7) { // minimum frame length：1（head）+ 1 + 1 + 1 + 1 + 2 + 1=8 bytes
                uint8_t* frame = &buffer[i];
                uint8_t field_num = frame[1];
                uint8_t checksum = frame[7]; //the eighth byte is check sum

                if (calculateChecksum(&frame[1], 6) == checksum && frame[1] == 0x01) {
                    geometry_msgs::PoseStamped mag_offset;

                    mag_offset.header.stamp = ros::Time::now();

                    mag_offset.pose.position.y = static_cast<double>(static_cast<int8_t>(frame[2]) / 1000.0);

                    mag_offset_pub.publish(mag_offset);
                    ROS_INFO("The Y axis offset = %lf", mag_offset.pose.position.y);
                }
                // reset buffer_len, buffer and i;
                buffer_len = 0;
                memset(buffer, 0, sizeof(buffer));
                i = 0; // recheck the buffer
                break; // end a data reading
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * @brief calculate and check sun - RS-232
 */
uint8_t Mpmgs201_Driver::calculateChecksum(const uint8_t* data, size_t len) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < len; ++i) {
        checksum += data[i];
    }
    return static_cast<uint8_t>(checksum);
}

/**
 * @brief Towards serial send operation command - RS-484
 */
bool Mpmgs201_Driver::sendCommand(const std::vector<uint8_t> command, const std::vector<uint8_t>::size_type size)
{
    return serial_.write(command) == size;
}

/**
 * @brief Received and verified the Magnetic navigation sensor check result
 * @param field_num the available maximum number of magnetic stripe segements
 * @note current driver code only need a segement bar magnetic
 * @note Communication protocol RS-484
 */
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
