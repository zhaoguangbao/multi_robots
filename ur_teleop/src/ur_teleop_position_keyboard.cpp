#include <ur_teleop_position_keyboard.h>

int            g_kfd = 0;
struct termios g_cooked;
struct termios g_raw;

ArmTeleopPositionKeyboard::ArmTeleopPositionKeyboard()
{
    arm_name_.push_back("shoulder_pan_joint");
    arm_name_.push_back("shoulder_lift_joint");
    arm_name_.push_back("elbow_joint");
    arm_name_.push_back("wrist_1_joint");
    arm_name_.push_back("wrist_2_joint");
    arm_name_.push_back("wrist_3_joint");

    ns_="";
    ros::NodeHandle n_private("~");
    n_private.param("arm_pose_step", arm_pos_step_, 0.1);
    n_private.getParam("robot_ns",ns_);

    for (size_t i = 0; i < arm_name_.size(); i++)
    {
        arm_index_[arm_name_[i]] = i;
        if(ns_!="")
        {
            arm_pos_pub_[arm_name_[i]] = xm_nh_.advertise<std_msgs::Float64>(
                ns_+"/"+arm_name_[i] + "/command", 1000);
        }else{
            arm_pos_pub_[arm_name_[i]] = xm_nh_.advertise<std_msgs::Float64>(
                arm_name_[i] + "/command", 1000);
        }
    }

}

ArmTeleopPositionKeyboard::~ArmTeleopPositionKeyboard()
{
    xm_nh_.shutdown();
}

void ArmTeleopPositionKeyboard::spinTeleopArm()
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

    puts(ns_.c_str());
    puts("--------------------------------------------------");
    puts("          Teleop Position Arm By KeyBoard         ");
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
        }else{
            if(flag==true)//pub once when press a key
                continue;
        }

        switch(keyboard_cmd)
        {
            case KEYCODE_A:
                arm_position[arm_index_["shoulder_pan_joint"]] += arm_pos_step_;
                arm_pos_.data = arm_position[arm_index_["shoulder_pan_joint"]];
                arm_pos_pub_["shoulder_pan_joint"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_D:
                arm_position[arm_index_["shoulder_lift_joint"]] += arm_pos_step_;
                arm_pos_.data = arm_position[arm_index_["shoulder_lift_joint"]];
                arm_pos_pub_["shoulder_lift_joint"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_S:
                arm_position[arm_index_["elbow_joint"]] += arm_pos_step_;
                arm_pos_.data = arm_position[arm_index_["elbow_joint"]];
                arm_pos_pub_["elbow_joint"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_Q:
                arm_position[arm_index_["wrist_1_joint"]] += arm_pos_step_;
                arm_pos_.data = arm_position[arm_index_["wrist_1_joint"]];
                arm_pos_pub_["wrist_1_joint"].publish(arm_pos_);
                flag = true;
                break;
            case KEYCODE_W:
                arm_position[arm_index_["wrist_2_joint"]] += arm_pos_step_;
                arm_pos_.data = arm_position[arm_index_["wrist_2_joint"]];
                arm_pos_pub_["wrist_2_joint"].publish(
                    arm_pos_);
                flag = true;
                break;
            case KEYCODE_E:
                arm_position[arm_index_["wrist_3_joint"]] += arm_pos_step_;
                arm_pos_.data = arm_position[arm_index_["wrist_3_joint"]];
                arm_pos_pub_["wrist_3_joint"].publish(arm_pos_);
                flag = true;
                break;
            default:
                flag = false;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ur_teleop_position_keyboard",
        ros::init_options::NoSigintHandler);

    ArmTeleopPositionKeyboard teleop_position_keyboard;

    boost::thread make_thread = boost::thread(boost::bind(
        &ArmTeleopPositionKeyboard::spinTeleopArm, &teleop_position_keyboard));

    ros::spin();

    make_thread.interrupt();
    make_thread.join();
    tcsetattr(g_kfd, TCSANOW, &g_cooked);

    return 0;
}
