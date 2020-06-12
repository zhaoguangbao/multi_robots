#include <ur_teleop_trajectory_keyboard.h>

int            g_kfd = 0;
struct termios g_cooked;
struct termios g_raw;

ArmTeleopTrajectoryKeyboard::ArmTeleopTrajectoryKeyboard()
{
    arm_name_.push_back("shoulder_pan_joint");
    arm_name_.push_back("shoulder_lift_joint");
    arm_name_.push_back("elbow_joint");
    arm_name_.push_back("wrist_1_joint");
    arm_name_.push_back("wrist_2_joint");
    arm_name_.push_back("wrist_3_joint");

    arm_goal_.trajectory.joint_names.push_back("shoulder_pan_joint");
    arm_goal_.trajectory.joint_names.push_back("shoulder_lift_joint");
    arm_goal_.trajectory.joint_names.push_back("elbow_joint");
    arm_goal_.trajectory.joint_names.push_back("wrist_1_joint");
    arm_goal_.trajectory.joint_names.push_back("wrist_2_joint");
    arm_goal_.trajectory.joint_names.push_back("wrist_3_joint");

    arm_goal_.trajectory.points.resize(1);
    arm_goal_.trajectory.points[0].positions.resize(arm_name_.size());
    arm_goal_.trajectory.points[0].velocities.resize(arm_name_.size());
    arm_goal_.trajectory.points[0].accelerations.resize(arm_name_.size());

    for (size_t i = 0; i < arm_name_.size(); i++)
    {
        arm_index_[arm_name_[i]] = i;
        arm_goal_.trajectory.points[0].positions[i]     = 0.0;
        arm_goal_.trajectory.points[0].velocities[i]    = 0.0;
        arm_goal_.trajectory.points[0].accelerations[i] = 0.0;
    }

    ros::NodeHandle n_private("~");
    n_private.param("arm_pose_step", arm_pos_step_, 0.1);


    trajectory_client_ =  boost::make_shared<TrajectoryClient>(
        "arm_controller/follow_joint_trajectory", true);

    while (!trajectory_client_->waitForServer(ros::Duration(5)))
        ROS_INFO_STREAM("Waiting for the joint_trajectory_action server");
}

ArmTeleopTrajectoryKeyboard::~ArmTeleopTrajectoryKeyboard()
{
    xm_nh_.shutdown();
}

void ArmTeleopTrajectoryKeyboard::spinTeleopArm()
{
    char   keyboard_cmd;
    double arm_position[6];
    bool   flag = false;
    memset(arm_position, 0, sizeof(arm_position));

    tcgetattr(g_kfd, &g_cooked);
    memcpy(&g_raw, &g_cooked, sizeof(struct termios));
    g_raw.c_lflag &=~ (ICANON | ECHO);
    g_raw.c_cc[VEOL] = 1;
    g_raw.c_cc[VEOF] = 2;
    tcsetattr(g_kfd, TCSANOW, &g_raw);

    puts("--------------------------------------------------");
    puts("         Teleop Trajectory Arm By Keyboard        ");
    puts("--------------------------------------------------");
    puts("q                       w                        e");
    puts("a                                                d");
    puts("                        s                         ");
    puts("--------------------------------------------------");
    puts("q: wrist_1_joint w: wrist_2_joint e: wrist_3_joint");
    puts("a:pan-joint                           d:lift-joint");
    puts("                  s: elbow-joint                  ");
    puts("--------------------------------------------------");
    puts("                PRESS CTRL-C TO QUIT              ");

    struct pollfd ufd;
    ufd.fd = g_kfd;
    ufd.events = POLLIN;
    arm_pos_step_ = 0.1;

    while(true)
    {
        boost::this_thread::interruption_point();
        int num;

        if((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("Function poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(g_kfd, &keyboard_cmd, 1) < 0)
            {
                perror("Function read():");
                return;
            }
        }
        else
        {
            if(flag == true)
                continue;
        }

        switch(keyboard_cmd)
        {
           case KEYCODE_A:
                arm_position[arm_index_["shoulder_pan_joint"]] += arm_pos_step_;
                flag = true;
                break;
            case KEYCODE_D:
                arm_position[arm_index_["shoulder_lift_joint"]] += arm_pos_step_;
                flag = true;
                break;
            case KEYCODE_S:
                arm_position[arm_index_["elbow_joint"]] += arm_pos_step_;
                flag = true;
                break;
            case KEYCODE_Q:
                arm_position[arm_index_["wrist_1_joint"]] += arm_pos_step_;
                flag = true;
                break;
            case KEYCODE_W:
                arm_position[arm_index_["wrist_2_joint"]] += arm_pos_step_;
                flag = true;
                break;
            case KEYCODE_E:
                arm_position[arm_index_["wrist_3_joint"]] += arm_pos_step_;
                flag = true;
                break;
            default:
                flag = false;
        }

        arm_goal_.trajectory.points[0].positions[arm_index_["shoulder_pan_joint"]] = arm_position[arm_index_["shoulder_pan_joint"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["shoulder_lift_joint"]] = arm_position[arm_index_["shoulder_lift_joint"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["elbow_joint"]] = arm_position[arm_index_["elbow_joint"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["wrist_1_joint"]] = arm_position[arm_index_["wrist_1_joint"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["wrist_2_joint"]] = arm_position[arm_index_["wrist_2_joint"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["wrist_3_joint"]] = arm_position[arm_index_["wrist_3_joint"]];
        arm_goal_.trajectory.points[0].time_from_start = ros::Duration(5);
        arm_goal_.goal_time_tolerance = ros::Duration(0);
        trajectory_client_->sendGoal(arm_goal_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ur_teleop_trajectory_keyboard",
        ros::init_options::NoSigintHandler);

    ArmTeleopTrajectoryKeyboard teleop_trajectory_keyboard;

    boost::thread make_thread = boost::thread(boost::bind(
        &ArmTeleopTrajectoryKeyboard::spinTeleopArm,
        &teleop_trajectory_keyboard));

    ros::spin();

    make_thread.interrupt();
    make_thread.join();
    tcsetattr(g_kfd, TCSANOW, &g_cooked);

    return 0;
}
