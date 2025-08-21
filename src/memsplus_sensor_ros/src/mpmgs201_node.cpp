#include"mpmgs201_ros/mpmgs201_driver.h"
#include<ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "mpmgs201_node");
    ros::NodeHandle nh("~");
    
    Mpmgs201_Driver driver(nh);
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
    google::SetLogDestination(google::GLOG_INFO,(driver.magnetic_log_dir_ + "mag_info").c_str());
    google::SetLogDestination(google::GLOG_FATAL,(driver.magnetic_log_dir_ + "mag_fatal").c_str());
    google::SetLogDestination(google::GLOG_WARNING,(driver.magnetic_log_dir_ + "mag_warning").c_str());
    google::SetLogDestination(google::GLOG_ERROR,(driver.magnetic_log_dir_ + "mag_error").c_str());
    if (!driver.init()){
        LOG(ERROR) << "The magnetic navigation driver failed to initialize";
        ROS_ERROR("The magnetic navigation driver failed to initialize");
        return -1;
    }
    
    LOG(INFO) << "The MPMGS201-F01 device has launched successfully";
    ROS_INFO("The MPMGS201-F01 device has launched successfully");
    driver.mainLoop();
    return 0;
}