#include"mprid1356_ros/mprid1356_driver.h"


#include <fstream>
#include <unistd.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/inotify.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

Mprid1356_Driver::Mprid1356_Driver(ros::NodeHandle& nh) {
    nh.param("port", port_, std::string("/dev/ttyRFID"));
    nh.param("baudrate", baudrate_, 115200);
    nh.param("device_address", dev_addr_, 6);
    nh.param("read_frequency", freq_, 100.0);               // 100 sometimes require each second
    nh.param("available_bytes", available_bytes_, 8);
    nh.param("is_mapping_mode", is_mapping_mode_, false);   // default false -> location
    nh.param("is_addition_mode", is_addition_mode_, false);   // default false -> regular mode
    nh.param("tag_distance_thr", tag_distance_thr_, 1.0);   // tag id help distance threshold
    nh.param("intial_tag_id", intial_tag_id_, 1);
    nh.param("min_signal_thr", min_signal_thr_, 7);         // recommand setting 1 as min_signal threshold
    nh.param("rfid_log_dir",rfid_log_dir_, std::string("/home/sunqian/history/Rfid/"));
    nh.param("tag_recorde_file", tag_recorde_file_, std::string("/home/sunqian/A14.json"));

    // Initialize all parameters and setup publishers/subscribers
    initializeParameters(nh);
}

Mprid1356_Driver::~Mprid1356_Driver() {
    // remove inotify watch
    if (watch_fd_ != -1) {
        inotify_rm_watch(inotify_fd_, watch_fd_);
    }
    // close inotify fd
    if (inotify_fd_ != -1) {
        close(inotify_fd_);
    }

    if (serial_.isOpen()) {
        try {
            serial_.close();
            LOG(INFO) << "Rfid port closed on destructor";
        } catch (serial::IOException& e) {
            LOG(WARNING) << "Error closing serial port on destructor: " << e.what();
        }
    }
}

bool Mprid1356_Driver::init() {
    // get tag record file name, through /.ros/map id.txt
    std::string map_id = getMapID();
    current_map_id_ = map_id;
    tag_recorde_file_ = map_id.empty() ? tag_recorde_file_ : map_id + ".json";

    // check tag record file
    if (!initTagRecordFile(tag_recorde_file_)) {
        LOG(ERROR) << "Failed to initialize tag record file";
        return false;
    }

    // init serial port;include baudrate, port, timeout
    try{
        serial_.setPort(port_);
        serial_.setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        serial_.setTimeout(to);
        serial_.open();
    } catch (serial::IOException& e) {
        LOG(ERROR) << "Rfid sensor port open failed: " << e.what();
        return false;
    }
    return serial_.isOpen();
}

void Mprid1356_Driver::mainLoop() {
    ros::Rate rate(freq_); // defualt 100 Hz
    const int EVENT_SIZE = sizeof(struct inotify_event);
    const int BUF_LEN = 1024 * (EVENT_SIZE + 16);
    char buffer[BUF_LEN];

    while (ros::ok()) {
        // check map id change
        ssize_t length = read(inotify_fd_, buffer, BUF_LEN);
        if (length > 0) {
            int i = 0;
            while (i < length) {
                struct inotify_event *event = (struct inotify_event *) &buffer[i];
                if (event->mask & IN_MODIFY) {
                    std::string new_map_id = getMapID();
                    if (new_map_id != current_map_id_) {
                        LOG(INFO) << "Map ID changed from " << current_map_id_ << " to " << new_map_id;
                        current_map_id_ = new_map_id;
                        tag_recorde_file_ = new_map_id.empty() ? tag_recorde_file_ : new_map_id + ".json";

                        if (!initTagRecordFile(tag_recorde_file_)) {
                            LOG(ERROR) << "Failed to initialize tag record file";
                            continue;
                        }
                    }
                }
                i += EVENT_SIZE + event->len; 
            }
        }

        // check serial port status；if closed, try to reinit
        if (!serial_.isOpen()) {
            LOG(WARNING) << "Rfid port closed, trying to reinit...";
            if (!init()) {
                ros::Duration(1).sleep(); // sleep for 1 second
                continue;
            }
        }

        size_t expected_len = 15;
        if (available_bytes_ == 8) {
            if(sendCommand(read_cmd_8byte,read_cmd_8byte.size())) {
                std::string response_str = serial_.read(expected_len);
                
                // Explicit conversion
                std::vector<uint8_t> response(response_str.begin(), response_str.end());
                //printDataContent(response, "The reading command return data:");
                
                if(parseResponse(response) && is_mapping_mode_ != true) {
                    processTagPosition(response);
                }
                else if(is_mapping_mode_ && tag_status_) {
                    processMappingMode();
                }
            }
        }
        else if(available_bytes_ == 4) {
            // if future need can be added here
        }
        else {
            LOG(ERROR) << "The Rfid does not a" << available_bytes_ << "bytes mode. Please correct the available_bytes parameter";
        }
            
        ros::spinOnce();
        rate.sleep();
    }
}

void Mprid1356_Driver::initializeParameters(ros::NodeHandle& nh) {
    // Initialize signal levels
    current_signal_level_ = 0x00;
    max_signal_level_ = 0x00;

    // Initialize pose and offset values
    pose_x_ = 0.0; 
    pose_y_ = 0.0; 
    offset_y_ = 0.0; 

    // Initialize trigger and position tracking
    isTriggeredOnce = true;
    old_x = 0.0;
    old_y = 0.0;
    wait_count = 0;

    // Initialize ID help pose
    id_help_pose.resize(2, 0.0);

    // Initialize start tag ID with validation
    if (intial_tag_id_ < 0 || intial_tag_id_ > UINT16_MAX) {
        LOG(WARNING) << "Invalid tag ID value: " << intial_tag_id_ << ". Using default 0x0001.";
        write_tag_id_ = 0x0001;
    } else {
        write_tag_id_ = static_cast<uint16_t>(intial_tag_id_);
    }

    // Initialize min tag signal threshold with validation
    if (min_signal_thr_ < 0 || min_signal_thr_ > 7) {
        LOG(WARNING) << "Invalid min signal threshold value: " << min_signal_thr_ << ". Using default 0x07.";
        set_signal_level_ = 0x07;
    } else {
        set_signal_level_ = static_cast<uint8_t>(min_signal_thr_);
    }

    // Output running mode to screen
    if (is_mapping_mode_) {
        LOG(INFO) << "Mapping mode";
    } else {
        LOG(INFO) << "Location mode";
    }
    
    // Initialize inotify for map_id.txt file
    inotify_fd_ = inotify_init1(IN_NONBLOCK);
    if (inotify_fd_ == -1) {
        LOG(ERROR) << "Failed to initialize inotify";
        return;
    }
    // Initialize watch for map_id.txt file
    std::string map_id_path = std::string(std::getenv("HOME")) + "/.ros/map_id.txt";
    watch_fd_ = inotify_add_watch(inotify_fd_, map_id_path.c_str(), IN_MODIFY);
    if (watch_fd_ == -1) {
        LOG(ERROR) << "Failed to add watch for map_id.txt file";
        close(inotify_fd_);
        return;
    }

    // Initialize Publishers
    tag_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/tag_pose", 20);

    // Initialize Subscribers
    mode_sub_ = nh.subscribe("/toggle_mapping_mode", 10, &Mprid1356_Driver::modeCallback, this);
    pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 10, &Mprid1356_Driver::poseCallback, this);
    offset_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mag_offset", 20, &Mprid1356_Driver::offsetCallback, this);

}

/**
 * @brief Ensuring Rfid is reading tag status
 * @param size the serial return data size
 * @note The "^" operation signal is Exclusive-OR gate -- 异或门
 */
bool Mprid1356_Driver::parseResponse(const std::vector<uint8_t>& data) {
    // Non-reading state (data length != 15 bytes , or status byte is 0x00)
    int size = data.size();
    if (!((size != 15) ^ (size != 11)) || data[3] == 0x00) {
        //LOG(INFO) << "Waitting for tag trigger...";
        if (++wait_count >= WAIT_THRESHOLD) {
            wait_count = 0; 
            tag_status_ = false;
            max_signal_level_ = 0x00;
            isTriggeredOnce = true;
            current_tag_id_ = 0x0000; // reset tag id
        }
        return false;
    }

    // Reading state (data length == 15 bytes , or status byte is 0x01)
    wait_count = 0;
    tag_status_ = true;                         // tag is in reading state
    current_tag_id_ = data[5] << 8 | data[6];   // current tag id
    current_signal_level_ = data[4];            // current signal level
    // LOG(INFO) << "cureent signal level" << static_cast<int>(current_signal_level_);

    if (is_mapping_mode_) return false;

    return true;
}

/**
 * @brief The location mode setting main process, only support 8 byte for now
 */
void Mprid1356_Driver::processTagPosition(const std::vector<uint8_t>& response) {
    tag_status_ = false; // intial status flag bits

    geometry_msgs::PoseStamped tag_position;
    uint16_t crc_received = (response[13] << 8) | response[14];
    uint16_t crc_calculated = crc16(response.data(), 13);
    
    if (crc_received != crc_calculated) {
        LOG(ERROR) << "Verification failed";
        return;
    }
    // LOG(INFO) << "Verification success";
    
    if (current_signal_level_ >= max_signal_level_) {
        max_signal_level_ = current_signal_level_;
        old_x = bytes_to_coord(response, 7);    // 7, 8, 9 -- x
        old_y = bytes_to_coord(response, 10);   // 10, 11, 12 -- y
    }
    else if (isTriggeredOnce && current_signal_level_ < max_signal_level_) {
        // check the current tag id is in the record file
        if (isTagExists(tag_recorde_file_,current_tag_id_)){
            // in theory, it is 10 mm motion error
            tag_position.header.stamp = ros::Time::now();
            tag_position.pose.position.x = old_x;
            tag_position.pose.position.y = old_y;
            tag_position.pose.position.z = static_cast<double>(current_tag_id_);
            tag_pose_pub_.publish(tag_position);
            LOG(INFO) << "Tag id = " << current_tag_id_ << ", Position: x = "
                    << tag_position.pose.position.x << ", y = " << tag_position.pose.position.y
                    << "current signal level = " << current_signal_level_;
        } else {
            LOG(ERROR) << "Tag id = " << current_tag_id_ << " is not in the record file" << tag_recorde_file_;
        }

        isTriggeredOnce = false;
    }
}

/**
 * @brief The mapping mode setting main process, only support 8 byte for now
 */
void Mprid1356_Driver::processMappingMode() {
    tag_status_ = false; // intial status flag bits

    if (current_signal_level_ >= set_signal_level_ && current_signal_level_ >= max_signal_level_) {
        LOG(INFO) << "current_signal_level = " << static_cast<int>(current_signal_level_)
                << ", max_signal_level_ = " << static_cast<int>(max_signal_level_);
        max_signal_level_ = current_signal_level_;

        std::lock_guard<std::mutex> lock(pose_mutex_);
        double current_pose_x = pose_x_;
        double current_pose_y = pose_y_;

        std::lock_guard<std::mutex> mag_lock(mag_mutex_);
        double current_offset_y = offset_y_;

        auto x_bytes = coord_to_bytes(current_pose_x);
        auto y_bytes = coord_to_bytes(current_pose_y + current_offset_y);

        build_write_command(write_tag_id_, x_bytes, y_bytes);
        //printDataContent(write_cmd_8byte, "Send writting command data:");
        sendCommand(write_cmd_8byte, write_cmd_8byte.size()); // writting custom 8 bytes data -- 写入8个字节的数据

        std::vector<unsigned char> response;
        const int READ_LEN = 8;
        std::string response_str_ = serial_.read(READ_LEN);
        
        // check the length of the read data is as expected
        if (response_str_.size() != READ_LEN) {
            LOG(ERROR) << "Read " << response_str_.size() << " bytes (expected " << READ_LEN << ")";
            return;
        }
        response = std::vector<unsigned char>(response_str_.begin(), response_str_.end());
        //printDataContent(response, "Reveived writting command data:");

        if (response.size() == 8 && response[5] == 0x04) {
            LOG(INFO) << "Tag id = " << write_tag_id_ << " writing success, x = " << current_pose_x 
                    << " y = " << current_pose_y << ", y direction offset = " << current_offset_y;
            writeTagJson(tag_recorde_file_, write_tag_id_, current_pose_x, current_pose_y + current_offset_y); 

            if (std::hypot(id_help_pose[0] - current_pose_x, id_help_pose[1] - current_pose_y) >= tag_distance_thr_) {
                if (id_help_pose[0] != 0.0 && id_help_pose[1] != 0.0 && is_addition_mode_) {
                    write_tag_id_ += 3;
                } else {
                    write_tag_id_ += 1;
                }
                id_help_pose[0] = current_pose_x;
                id_help_pose[1] = current_pose_y;
            }
            offset_y_ = 0.0;
        } else {
            LOG(ERROR) << "Writing failed";
            return;
        }
    }
    else{
        if (current_signal_level_ < max_signal_level_) {
            LOG(INFO) << "Tag Id " << write_tag_id_ << " has complete position writting";
        }
        LOG(WARNING) << "Current tag signal level = " << static_cast<int>(current_signal_level_)
                << ", doese not meet writting requirements min signal threshold = " << static_cast<int>(set_signal_level_);
    }
}

/**
 * @brief After every writting command is completed, needing it to intial the command
 */
void Mprid1356_Driver::resetWriteCmdbyte() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    if(available_bytes_ == 8) {
        write_cmd_8byte = {0x06, 0x10, 0x00, 0x17, 0x00, 0x04, 0x08};
    } 
    else if (available_bytes_ == 4) {
        write_cmd_4byte =  {0x06, 0x10, 0x00, 0x17, 0x00, 0x02, 0x04};
    }
    else {
        LOG(ERROR) << "The Rfid does not support " << available_bytes_ << " bytes mode. Please correct the available_bytes parameter";
    }
}

/**
 * @brief Towards serial send operation command
 */
bool Mprid1356_Driver::sendCommand(const std::vector<uint8_t> command, const std::vector<uint8_t>::size_type size) {
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    return serial_.write(command) == size;
}

/**
 * @brief The function constructe 8 bytes writting command only support for now
 */
void Mprid1356_Driver::build_write_command(uint16_t tag_id, const std::vector<uint8_t>& x_bytes, const std::vector<uint8_t>& y_bytes) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    resetWriteCmdbyte();
    if (write_cmd_8byte.size() != 7) {
        LOG(FATAL) << "write_cmd_8byte size incorrect: " << write_cmd_8byte.size();
        return;
    }
    if (write_cmd_8byte.size() == 7 && write_cmd_8byte[6] == 0x08) {
        // 标签编号 - 2字节高位在前
        write_cmd_8byte.push_back((static_cast<uint8_t>((tag_id >> 8) & 0xFF)));
        write_cmd_8byte.push_back(static_cast<uint8_t>(tag_id & 0xFF));

        // X坐标
        write_cmd_8byte.insert(write_cmd_8byte.end(),x_bytes.begin(), x_bytes.end());

        // Y坐标
        write_cmd_8byte.insert(write_cmd_8byte.end(),y_bytes.begin(), y_bytes.end());

        // 计算CRC16校验位
        uint16_t crc = crc16(write_cmd_8byte.data(), write_cmd_8byte.size());

        write_cmd_8byte.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
        write_cmd_8byte.push_back(static_cast<uint8_t>(crc & 0xFF));
    }
    else{
        LOG(ERROR) << "The write_cmd_8byte size error!!!";
    }
}

/**
 * @brief The double type coord message transform to Hex byte informans
 */
std::vector<uint8_t> Mprid1356_Driver::coord_to_bytes(double coord, double precision) {
    // 1. double to int ,the precision is 0.001 - keep three decimal places
    int32_t val = static_cast<int32_t>(coord * precision);

    // 2. offset operation to confirm plus-mins(±)
    uint32_t offset_val = val + 0x800000;

    // 3. reconstruction 3 byte to save
    return {
        static_cast<uint8_t>((offset_val >> 16) & 0xFF),
        static_cast<uint8_t>((offset_val >> 8) & 0xFF),
        static_cast<uint8_t>(offset_val & 0xFF)
    };
}

/**
 * @brief The vector type Hex byte message transform to the double type corrd information
 * @param start set the frist opeation position include after 2, three in total
 */
double Mprid1356_Driver::bytes_to_coord(const std::vector<uint8_t>& bytes, size_t start, double precision) {
    // 1. combine 3 byte to constructe a uint32_t data type
    uint32_t offset_val = (static_cast<uint32_t>(bytes[start]) << 16)       // the first byte, highest
                        | (static_cast<uint32_t>(bytes[start + 1]) << 8)    // the second byte, middle
                        | static_cast<uint32_t>(bytes[start + 2]);          // the third byte, lowest
    
    // 2. minus offset to restore negative number
    int32_t val = static_cast<int32_t>(offset_val) - 0x800000;
    
    // 3. precision operation to get the double type coord information
    if (precision == 0.0) {
        LOG(ERROR) << "Precision cannot be zero";
        return 0.0;
    }
    return static_cast<float>(val) / precision;
}

std::string Mprid1356_Driver::getMapID(const std::string& path) {
    std::string default_path = std::getenv("HOME") ? std::getenv("HOME") : "";
    default_path += "/.ros/map_id.txt";
    
    std::string file_path = path.empty() ? default_path : path;
    
    LOG(INFO) << "map id path = " << file_path;
    
    std::ifstream infile(file_path);
    if (!infile.is_open()) {
        LOG(ERROR) << "Failed to open map_id file: " << file_path;
        return "";
    }

    std::map<std::string, std::string> map_scope;
    std::string str_line, map_id;
    
    try {
        while (getline(infile, str_line)) {
            if (str_line.empty()) continue;
            
            size_t position = str_line.find(": ");
            if (position == std::string::npos) {
                LOG(WARNING) << "Invalid line format in map_id file: " << str_line;
                continue;
            }
            
            map_scope.insert({
                str_line.substr(0, position),
                str_line.substr(position + 2)
            });
        }
        
        if (map_scope.find("map_id") != map_scope.end()) {
            map_id = map_scope.at("map_id");
            LOG(INFO) << "read map_id: " << map_id;
        } else {
            LOG(ERROR) << "map_id not found in configuration file";
            return "";
        }
    } catch (const std::exception& e) {
        LOG(ERROR) << "Error parsing map_id file: " << e.what();
        return "";
    }
    
    return map_id;
}

/**
 * @brief The function to check the tag record file is exist or not
 * @note If the file does not exist, it will be created and initialized with an empty array
 * @note If the file exists but is not readable or writable, it will be set to readable and writable
 * @return true if the file is exist and ready to use, otherwise false
 */
bool Mprid1356_Driver::initTagRecordFile(const std::string& file_name){
    std::string ros_dir = std::getenv("HOME") ? std::getenv("HOME") : "";
    if (ros_dir.empty()) {
        LOG(ERROR) << "Cannot get HOME directory";
        return false;
    }

    ros_dir += "/.ros/tag_jsons/";
    std::string dir_path = ros_dir;
    if (access(dir_path.c_str(), F_OK) != 0) {
        LOG(INFO) << "Creating directory: " << dir_path;
        if (mkdir(dir_path.c_str(), 0777) != 0) {
            LOG(ERROR) << "Failed to create directory: " << dir_path;
            return false;
        }
    }

    std::string path = file_name.empty() ? path : ros_dir + file_name ;
    if (access(path.c_str(), F_OK) != 0) {
        LOG(ERROR) << "Tag record file does not exist: " << path;
        std::ofstream ofs(path, std::ios::out | std::ios::trunc);
        if (ofs.is_open()) {
            ofs << "[]";
            ofs.close();
            chmod(path.c_str(), 0666);
        } else {
            LOG(ERROR) << "Failed to open tag record file: " << path;
            return false;
        }
    } else {
        if ((access(path.c_str(), R_OK | W_OK) != 0)) {
            LOG(ERROR) << "Tag record file is not readable/writable: " << path;
            if (chmod(path.c_str(), 0666) != 0) {
                LOG(ERROR) << "Failed to set tag record file permissions: " << path;   
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief Check if a tag ID exists in the tag record file
 * @param filename The path to the JSON tag record file
 * @param tag_id The tag ID to check for existence
 * @return true if the tag ID exists in the file, false otherwise
 */
bool Mprid1356_Driver::isTagExists(const std::string& filename, const uint16_t& tag_id) {
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        LOG(WARNING) << "Failed to open " << filename << " for reading";
        return false;
    }
    
    try {
        json json_data;
        ifs >> json_data;
        ifs.close();
        
        // ensoure JSON data is array type
        if (!json_data.is_array()) {
            LOG(WARNING) << "Invalid JSON format in " << filename << ", expected array";
            return false;
        }
        
        // check current tag id is exist or not
        for (const auto& tag : json_data) {
            if (tag.contains("id") && tag["id"] == tag_id) {
                return true;
            }
        }
    } catch (const json::parse_error& e) {
        LOG(WARNING) << "Failed to parse JSON in " << filename << ": " << e.what();
        ifs.close();
        return false;
    }
    
    return false;
}

/**
 * @brief The function to write the tag information to json file
 * @param filename the json file name
 * @note The json file is a array type, each tag information is a json object
 */
void Mprid1356_Driver::writeTagJson(const std::string& filename, const uint16_t& id, const double& x, const double& y) {
    std::lock_guard<std::mutex> lock(file_mutex_);

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_time;
    localtime_r(&now_c, &local_time);
    char time_str[20];
    std::strftime(time_str, sizeof(time_str), "%Y/%m/%d %H:%M:%S", &local_time);
    std::string current_time = time_str;

    json json_data;
    std::ifstream ifs(filename);
    if (ifs.is_open()) {
        try {
            ifs >> json_data;
            if (!json_data.is_array()) {
                json_data = json::array();
                LOG(WARNING) << "Invalid JSON format in " << filename << ", resetting to empty array";
            }
        } catch (const json::parse_error& e) {
            json_data = json::array();
            LOG(WARNING) << "Failed to parse JSON in " << filename << ": " << e.what() << ", resetting to empty array";
        }
        ifs.close();
    } else {
        json_data = json::array();
        LOG(WARNING) << "Failed to open " << filename << " for reading, initializing new array";
    }
    
    bool tag_exists = false;
    for (auto& tag : json_data) {
        if (tag.contains("id") && tag["id"] == id) {
            tag["x"] = x;
            tag["y"] = y;
            tag["last_updated"] = current_time;

            tag_exists = true;
            LOG(INFO) << "Updated tag " << id << " in JSON file";
            break;
        }
    } 

    if (!tag_exists) {
        json new_tag;
        new_tag["id"] = id;
        new_tag["x"] = x;
        new_tag["y"] = y;
        new_tag["first_added"] = current_time;
        new_tag["last_updated"] = current_time;
        json_data.push_back(new_tag);
        LOG(INFO) << "Added new tag " << id << " to JSON file";
    }

    std::ofstream ofs(filename, std::ios::out | std::ios::trunc);
    if (ofs.is_open()) {
        ofs << std::setw(4) << json_data << std::endl;
        ofs.close();
    } else {
        LOG(ERROR) << "Failed to open " << filename << " for writing";
    }
}

/**
 * @brief Printing data content
 * @param start set the frist position，default = 0
 */
void Mprid1356_Driver::printDataContent(const std::vector<uint8_t> data, const std::string prefix, size_t start) {
    std::stringstream ss;
    ss << prefix << ": ";
    for (size_t i = start; i < data.size(); ++i) {
        ss << "0x" << std::hex << std::setw(2) << std::setfill('0') 
           << static_cast<int>(data[i]) << std::dec;
    }
    ROS_INFO("%s", ss.str().c_str());
}

/**
 * @brief The /pose subscriber Callback main to operat position information
 */
void Mprid1356_Driver::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;
    // ROS_INFO("x =  %lf, y = %lf", msg->pose.position.x, msg->pose.position.y);
}

/**
 * @brief The /mag_offset subscriber Callback main to correct Y axis offset value
 */
void Mprid1356_Driver::offsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mag_mutex_);
    offset_y_ = msg->pose.position.y;
    // ROS_INFO("Y axis offset =  %lf", offset_y_);
}

/**
 * @brief The /mapping_mode subscriber Callback main to change the mapping mode
 */
void Mprid1356_Driver::modeCallback(const std_msgs::Bool::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    if (is_mapping_mode_ != msg->data) {
        ROS_INFO("Mapping mode changed via topic: %s", 
                 is_mapping_mode_ ? "true" : "false");
    } else {
        ROS_INFO("Mapping mode is already %s", 
                 is_mapping_mode_ ? "true" : "false");
    }
    if (is_mapping_mode_) {
        LOG(INFO) << "Mapping mode";
    } else {
        LOG(INFO) << "Location mode";
    }
}

/**
 * @brief Through seach table method, calculation or verfication the writting or reading data
 */
uint16_t Mprid1356_Driver::crc16(const uint8_t* data, size_t len) {
    static const unsigned char aucCRCHi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40
    };

    static const unsigned char aucCRCLo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
        0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
        0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
        0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
        0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
        0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
        0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
        0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
        0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
        0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
        0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
        0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
        0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
        0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
        0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
        0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
        0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
        0x41, 0x81, 0x80, 0x40
    };

    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    uint8_t iIndex; 

    while (len--) {
        iIndex = ucCRCHi ^ *data++;             // compute index
        ucCRCHi = ucCRCLo ^ aucCRCHi[iIndex];   // update CRC Low bit
        ucCRCLo = aucCRCLo[iIndex];             // update CRC High bit
    }

    // ROS_INFO("CRC intial value: 0x%02X%02X", 
    //          static_cast<unsigned int>(ucCRCHi), 
    //          static_cast<unsigned int>(ucCRCLo));
    
    return (ucCRCHi << 8) | ucCRCLo;  // two 8 bit combined 16 bit CRC value
}
