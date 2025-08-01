#include"mpmgs201_ros/mpmgs201_driver.h"
#include<ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "mpmgs201_node");
    ros::NodeHandle nh("~");
    
    Mpmgs201_Driver driver(nh);
    if (!driver.init()){
        ROS_ERROR("The magnetic navigation driver failed to initialize");
        return -1;
    }
    
    ROS_INFO("The MPMGS201-F01 device has launched successfully");
    driver.mainLoop();
    return 0;
}