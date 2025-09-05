#include"mprid1356_ros/mprid1356_driver.h"
#include<ros/ros.h>

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "mprid1356_node");
    ros::NodeHandle nh("~");
    Mprid1356_Driver driver(nh);
    google::SetLogDestination(google::GLOG_INFO,(driver.rfid_log_dir_ + "rfid_info").c_str());
    google::SetLogDestination(google::GLOG_FATAL,(driver.rfid_log_dir_ + "rfid_fatal").c_str());
    google::SetLogDestination(google::GLOG_WARNING,(driver.rfid_log_dir_ + "rfid_warning").c_str());
    google::SetLogDestination(google::GLOG_ERROR,(driver.rfid_log_dir_ + "rfid_error").c_str());
    if (!driver.init()){
        LOG(ERROR) << "The driver failed to initialize";
        google::ShutdownGoogleLogging();
        return -1;
    }
    
    LOG(INFO) << "The MPRID1356-S RFID read-write device has launched successfully";
    driver.mainLoop();
    google::ShutdownGoogleLogging();
    return 0;
}