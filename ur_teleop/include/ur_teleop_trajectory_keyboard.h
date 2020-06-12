#ifndef XM_ARM_TELEOP_TRAJECTORY_KEYBOARD
#define XM_ARM_TELEOP_TRAJECTORY_KEYBOARD

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <termio.h>
#include <signal.h>
#include <sys/poll.h>
#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#define KEYCODE_A     0X61
#define KEYCODE_D     0X64
#define KEYCODE_S     0X73
#define KEYCODE_W     0X77
#define KEYCODE_Q     0X71
#define KEYCODE_E     0X65

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajectoryClient;

class ArmTeleopTrajectoryKeyboard
{
public:
    ArmTeleopTrajectoryKeyboard();
    ~ArmTeleopTrajectoryKeyboard();
    void spinTeleopArm();
    void stopTeleopArm();
private:
    ros::NodeHandle                         xm_nh_;
    std::map<std::string, size_t>              arm_index_;
    double                                  arm_pos_step_;
    std::vector<std::string>                arm_name_;
    control_msgs::FollowJointTrajectoryGoal arm_goal_;
    boost::shared_ptr<TrajectoryClient>     trajectory_client_;
    std::string                             teleop_type_;
};

#endif // XM_ARM_TELEOP_TRAJECTORY_KEYBOARD
