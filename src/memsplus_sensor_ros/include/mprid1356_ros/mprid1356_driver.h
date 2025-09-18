#ifndef MPRID1356_DRIVER_H
#define MPRID1356_DRIVER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

#include <mutex>
#include <vector>
#include <string>
#include <serial/serial.h>

#include <glog/logging.h>

class Mprid1356_Driver
{
public:
    Mprid1356_Driver(ros::NodeHandle& nh);
    ~Mprid1356_Driver();

    bool init();        // intial sensor port
    void mainLoop();    // main loop function
    
    std::string rfid_log_dir_;

private:
    void initializeParameters(ros::NodeHandle& nh);
    bool parseResponse(const std::vector<uint8_t>& data);

    void processTagPosition(const std::vector<uint8_t>& response);
    void processMappingMode();

    void resetWriteCmdbyte();
    bool sendCommand(const std::vector<uint8_t> command, const std::vector<uint8_t>::size_type size);
    void build_write_command(uint16_t tag_id, const std::vector<uint8_t>& x_bytes, const std::vector<uint8_t>& y_bytes);
    std::vector<uint8_t> coord_to_bytes(double coord, double precision = 1000.0);
    double bytes_to_coord(const std::vector<uint8_t>& bytes, size_t start = 0, double precision = 1000.0);

    std::string getMapID(const std::string& path = "");
    bool initTagRecordFile(const std::string& file_path);
    bool isTagExists(const std::string& filename, const uint16_t& tag_id);
    void writeTagJson(const std::string& filename, const uint16_t& id, const double& x, const double& y);
    void printDataContent(const std::vector<uint8_t> data, const std::string prefix, size_t start = 0);

    void modeCallback(const std_msgs::Bool::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void offsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    uint16_t crc16(const uint8_t* data, size_t len);

    serial::Serial serial_;
    std::string port_;
    int baudrate_;
    int dev_addr_;
    double freq_;
    int available_bytes_;

    ros::Publisher tag_pose_pub_;
    ros::Subscriber offset_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber mode_sub_;

    std::mutex pose_mutex_;
    std::mutex mag_mutex_;
    std::mutex file_mutex_;
    std::mutex mode_mutex_;
    std::mutex cmd_mutex_;
    std::mutex serial_mutex_;

    bool is_mapping_mode_;
    bool is_addition_mode_;
    double pose_x_;
    double pose_y_;
    double offset_y_;
    
    bool tag_status_;
    int intial_tag_id_;
    uint16_t write_tag_id_;
    uint16_t current_tag_id_;
    double tag_distance_thr_;
    
    int watch_fd_;                  // wathch descriptor for inotify
    int inotify_fd_;                // file descriptor for inotify
    std::string current_map_id_; 
    std::string tag_recorde_file_;
    
    int min_signal_thr_;
    uint8_t set_signal_level_;
    uint8_t max_signal_level_;
    uint8_t current_signal_level_;

    bool isTriggeredOnce;
    double old_x, old_y;
    std::vector<double> id_help_pose = {0.0, 0.0};

    int wait_count;
    const int WAIT_THRESHOLD = 30;


    // communication command

    // The data read command 
    const std::vector<uint8_t> read_cmd_4byte = {0x06, 0x03, 0x00, 0x25, 0x00, 0x03, 0x15, 0xB7};
    const std::vector<uint8_t> read_cmd_8byte = {0x06, 0x03, 0x00, 0x25, 0x00, 0x05, 0x95, 0xB5};

    // clearing history data command of Rfid sensor
    const std::vector<uint8_t> clear_cmd = {0x06, 0x06, 0x00, 0x00, 0x01, 0x06, 0x09, 0xEF};

    // The custom sign write command
    // 采用4个字节：第8 - 11 个字节可自由定义，第12 - 13 个字节需要通过CRC_16计算获得
    // 采用8个字节：第8 - 15 个字节可自由定义，第16 - 17 个字节需要通过CRC_16计算获得
    std::vector<uint8_t> write_cmd_4byte = {0x06, 0x10, 0x00, 0x17, 0x00, 0x02, 0x04};
    std::vector<uint8_t> write_cmd_8byte = {0x06, 0x10, 0x00, 0x17, 0x00, 0x04, 0x08};

    // Get write statue command
    const std::vector<uint8_t> write_statue_cmd = {0x06, 0x03, 0x00, 0x20, 0x00, 0x01, 0x84, 0x77};

    // command return data
    // const std::vector<uint8_t> send_cmd_4byte_return = {0x06, 0x10, 0x00, 0x17, 0x00, 0x02};
    // const std::vector<uint8_t> send_cmd_8byte_return = {0x06, 0x10, 0x00, 0x17, 0x00, 0x04};

    // reading wirte status respon -- 读取写入状态返回，[4] - 00 : 成功， 02: 未检测到标签
    // const std::vector<uint8_t> read_write_status_cmd = {0x06, 0x03, 0x00, 0x20, 0x00, 0x01, 0x84, 77};
};

#endif // MPRID1356_DRIVER_H