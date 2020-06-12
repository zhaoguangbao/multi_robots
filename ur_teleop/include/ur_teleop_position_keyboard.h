#ifndef UR_TELEOP_POSITION_KEYBOARD_H
#define UR_TELEOP_POSITION_KEYBOARD_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <termio.h>
#include <signal.h>
#include <sys/poll.h>
#include <vector>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>

#define KEYCODE_A     0X61
#define KEYCODE_D     0X64
#define KEYCODE_S     0X73
#define KEYCODE_W     0X77
#define KEYCODE_Q     0X71
#define KEYCODE_E     0X65

class ArmTeleopPositionKeyboard
{
public:
    ArmTeleopPositionKeyboard();
    ~ArmTeleopPositionKeyboard();
    void spinTeleopArm();
    void stopTeleopArm();
private:
    ros::NodeHandle                       xm_nh_;
    std_msgs::Float64                     arm_pos_;
    std::map<std::string, ros::Publisher> arm_pos_pub_;
    std::map<std::string, size_t>            arm_index_;
    double                                arm_pos_step_;
    std::vector<std::string>              arm_name_;
    std::string                           ns_;
};

#endif // XM_ARM_TELEOP_POSITION_KEYBOARD_H
