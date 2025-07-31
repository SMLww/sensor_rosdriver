#ifndef MPRID1356_DRIVER_H
#define MPRID1356_DRIVER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <memsplus_sensor_ros/RFIDTag.h>

class Mprid1356_Driver
{
private:
    /* data */
public:
    Mprid1356_Driver(ros::NodeHandle& nh);
    ~Mprid1356_Driver();

    bool init();        // intial sensor port -- 初始化接口
    void mainLoop();    // main loop function -- 主循环

private:
    void processTagPosition(const std::vector<uint8_t>& response);
    void processMappingMode();

    bool sendCommand(const std::vector<uint8_t> command, const std::vector<uint8_t>::size_type size);
    void printDataContent(const std::vector<uint8_t> data, const std::string prefix, size_t start = 0);
    bool parseResponse(const std::vector<uint8_t>& data);

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void offsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void build_write_command(uint16_t tag_id, const std::vector<uint8_t>& x_bytes, const std::vector<uint8_t>& y_bytes);
    std::vector<uint8_t> coord_to_bytes(double coord, double precision = 1000.0);
    double bytes_to_coord(const std::vector<uint8_t>& bytes, size_t start = 0, double precision = 1000.0);
    void resetWriteCmdbyte();

    uint16_t crc16(const uint8_t* data, size_t len);

    serial::Serial serial_;
    ros::Publisher tag_pose_pub;

    std::string port_;
    int baudrate_;
    int dev_addr_;
    double freq_;
    int available_bytes_;
    
    bool is_mapping_mode_;
    mutable std::mutex pose_mutex_;
    ros::Subscriber pose_sub_;
    double pose_x_;
    double pose_y_;

    mutable std::mutex mag_mutex_;
    ros::Subscriber offset_sub_;
    double offset_y_;

    int intial_tag_id_;
    double tag_distance_thr_;
    uint16_t next_tag_id_;
    bool tag_status_;
    int min_signal_thr_;
    uint8_t set_signal_level_;
    uint8_t current_signal_level_;
    
    std::vector<double> id_help_pose = {0.0, 0.0};
    

    // communication command -- 通信指令
    // The data read command -- 数据读取指令
    const std::vector<uint8_t> read_cmd_4byte = {0x06, 0x03, 0x00, 0x25, 0x00, 0x03, 0x15, 0xB7};

    const std::vector<uint8_t> read_cmd_8byte = {0x06, 0x03, 0x00, 0x25, 0x00, 0x05, 0x95, 0xB5};

    // clearing history data command of Rfid sensor -- 清除Rfid读写器存储数据指令
    const std::vector<uint8_t> clear_cmd = {0x06, 0x06, 0x00, 0x00, 0x01, 0x06, 0x09, 0xEF};

    // The custom sign write command -- 标签数据写入指令
    // 采用4个字节：第8 - 11 个字节可自由定义，第12 - 13 个字节需要通过CRC_16计算获得
    // 采用8个字节：第8 - 15 个字节可自由定义，第16 - 17 个字节需要通过CRC_16计算获得
    std::vector<uint8_t> write_cmd_4byte = {0x06, 0x10, 0x00, 0x17, 0x00, 0x02, 0x04};

    std::vector<uint8_t> write_cmd_8byte = {0x06, 0x10, 0x00, 0x17, 0x00, 0x04, 0x08};

    // Get write statue command -- 获取写入状态指令
    const std::vector<uint8_t> write_statue_cmd = {0x06, 0x03, 0x00, 0x20, 0x00, 0x01, 0x84, 0x77};

    // command return data -- 写入后返回数据
    const std::vector<uint8_t> send_cmd_4byte_return = {0x06, 0x10, 0x00, 0x17, 0x00, 0x02};
    const std::vector<uint8_t> send_cmd_8byte_return = {0x06, 0x10, 0x00, 0x17, 0x00, 0x04};

    // reading wirte status respon -- 读取写入状态返回，[4] - 00 : 成功， 02: 未检测到标签
    const std::vector<uint8_t> read_write_status_cmd = {0x06, 0x03, 0x00, 0x20, 0x00, 0x01, 0x84, 77};
};

#endif // MPRID1356_DRIVER_H