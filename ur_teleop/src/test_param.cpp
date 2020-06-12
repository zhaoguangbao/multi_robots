#include <ros/ros.h>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_param");
    double parking_x;
    double parking_y;
    std::string str="";
    ros::NodeHandle nh_private("~");
    nh_private.param("parking_x",parking_x,0.0);
    nh_private.param("parking_y",parking_y,0.0);
    nh_private.getParam("str",str);
    ROS_INFO("parking_x: %.3f", parking_x);
    ROS_INFO("parking_y: %.3f", parking_y);
    ROS_INFO("str: %s", str.c_str());
    getchar();
    return 0;
}
