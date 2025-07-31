#include"mprid1356_ros/mprid1356_driver.h"
#include<ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "mprid1356_node");
    ros::NodeHandle nh("~");
    
    Mprid1356_Driver driver(nh);
    if (!driver.init()){
        ROS_ERROR("The driver failed to initialize");
        return -1;
    }
    
    ROS_INFO("The MPRID1356-S RFID read-write device has launched successfully");
    driver.mainLoop();
    return 0;
}