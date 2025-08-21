#ifndef MPMGS201_DRIVER_H
#define MPMGS201_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <glog/logging.h>
#include <geometry_msgs/PoseStamped.h>
#include <memsplus_sensor_ros/MagneticOffset.h>

class Mpmgs201_Driver
{
public:
    Mpmgs201_Driver(ros::NodeHandle& nh);
    ~Mpmgs201_Driver();

    bool init();        // intial sensor port -- 初始化接口
    void mainLoop();    // main loop function -- 主循环

    std::string magnetic_log_dir_;

private:

    bool sendCommand(const std::vector<uint8_t> command, const std::vector<uint8_t>::size_type size);
    void parseResponse(const std::vector<uint8_t>& response);
    uint8_t calculateChecksum(const uint8_t* data, size_t len);

    serial::Serial serial_;
    ros::Publisher mag_offset_pub;

    std::string port_;
    int baudrate_;
    int dev_addr_;
    double freq_;

    bool idAddCount;
    int count;
    int wait_count;
    const int WAIT_THRESHOLD = 10;

    uint8_t buffer[1024];
    size_t buffer_len = 0;

    // The data read command -- 数据读取指令
    const std::vector<uint8_t> read_cmd = {0x04, 0x03, 0x00, 0x04, 0x00, 0x03, 0x44, 0x5F};
};

#endif // MPMGS201_DRIVER_H