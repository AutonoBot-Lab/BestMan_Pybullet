#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

/* 带有shift键 */
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

class AgvKeyboardTeleopNode
{
private:
    double walk_vel_;
//    double run_vel_;
    double yaw_rate_;
//    double yaw_rate_run_;

    geometry_msgs::Twist agv_move_twist;
    ros::NodeHandle n_;
    ros::Publisher pub_agv_move_twist;

public:
    AgvKeyboardTeleopNode()
    {
        pub_agv_move_twist = n_.advertise<geometry_msgs::Twist>("/agv_control/agv_move_twist", 1);

        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel_, 0.2);
//        n_private.param("run_vel", run_vel_, 1.0);
        n_private.param("yaw_rate", yaw_rate_, 1.0);
//        n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
    }

    ~AgvKeyboardTeleopNode() { }
    void keyboardLoop();

    void stopRobot()
    {
        agv_move_twist.linear.x = 0.0;
        agv_move_twist.angular.z = 0.0;
        pub_agv_move_twist.publish(agv_move_twist);

//        ROS_INFO("linear x:%.3lf  angular:%.3lf", agv_move_twist.linear.x, agv_move_twist.angular.z);
    }
};

AgvKeyboardTeleopNode* tbk;

/**
 * 文件描述符
 * 内核（kernel）利用文件描述符（file descriptor）来访问文件。文件描述符是非负整数。
 * 标准输入（standard input）的文件描述符是 0，标准输出（standard output）是 1，标准错误（standard error）是 2。
 */
int kfd = 0;

/**
 *  === struct termios ===
 *  tcflag_t c_iflag;  输入模式
 *　tcflag_t c_oflag;  输出模式
 *　tcflag_t c_cflag;  控制模式
 *  tcflag_t c_lflag;  本地模式
 *  cc_t c_cc[NCCS];   控制字符
 */
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    AgvKeyboardTeleopNode tbk;

    /* 创建一个新的线程 */
    boost::thread t = boost::thread(boost::bind(&AgvKeyboardTeleopNode::keyboardLoop, &tbk));

    ros::spin();

    t.interrupt();
    t.join();
    tbk.stopRobot();

    /* 设置终端参数 */
    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void AgvKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    int speed = 0;
    int turn = 0;

    /**
	 * 从终端中获取按键
	 * int tcgetattr(int fd, struct termios *termios_p);
	 */
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));

    /**
	 * c_lflag : 本地模式标志，控制终端编辑功能
	 * ICANON: 使用标准输入模式
	 * ECHO: 显示输入字符
	 */
    raw.c_lflag &=~ (ICANON | ECHO);

    /**
     * c_cc[NCCS]：控制字符，用于保存终端驱动程序中的特殊字符，如输入结束符等
     * VEOL: 附加的End-of-file字符
     * VEOF: End-of-file字符
     * */
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    /* *
     * struct pollfd {
　　       int fd;        文件描述符
　       　short events;  等待的事件
　　       short revents; 实际发生了的事件
　       　};
    */
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    for(;;)
    {
        boost::this_thread::interruption_point();

        /* get the next event from the keyboard */
        int num;

        /**
         * poll:把当前的文件指针挂到设备内部定义的等待队列中。
         * unsigned int (*poll)(struct file * fp, struct poll_table_struct * table)
         */
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            /**
             * perror( ) 用来将上一个函数发生错误的原因输出到标准设备(stderr)。
             * 参数s所指的字符串会先打印出,后面再加上错误原因字符串。
             * 此错误原因依照全局变量errno 的值来决定要输出的字符串。
             * */
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            /* 每按下一次动一下 */
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
                ROS_INFO("stopRobot");
            }

            continue;
        }

        switch(c)
        {
            case KEYCODE_W:
                max_tv = walk_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
                ROS_INFO("KEYCODE_W Press");
                break;
            case KEYCODE_S:
                max_tv = walk_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
                break;
            case KEYCODE_A:
                max_rv = yaw_rate_;
                speed = 0;
                turn = 1;
                dirty = true;
                break;
            case KEYCODE_D:
                max_rv = yaw_rate_;
                speed = 0;
                turn = -1;
                dirty = true;
                break;

//            case KEYCODE_W_CAP:
//                max_tv = run_vel_;
//                speed = 1;
//                turn = 0;
//                dirty = true;
//                break;
//            case KEYCODE_S_CAP:
//                max_tv = run_vel_;
//                speed = -1;
//                turn = 0;
//                dirty = true;
//                break;
//            case KEYCODE_A_CAP:
//                max_rv = yaw_rate_run_;
//                speed = 0;
//                turn = 1;
//                dirty = true;
//                break;
//            case KEYCODE_D_CAP:
//                max_rv = yaw_rate_run_;
//                speed = 0;
//                turn = -1;
//                dirty = true;
//                break;

            default:
                max_tv = walk_vel_;
                max_rv = yaw_rate_;
                speed = 0;
                turn = 0;
                dirty = false;
        }

        agv_move_twist.linear.x = speed * max_tv;
        agv_move_twist.angular.z = turn * max_rv;
        pub_agv_move_twist.publish(agv_move_twist);
//        ROS_INFO("linear x:%.3lf  angular:%.3lf", agv_move_twist.linear.x, agv_move_twist.angular.z);
    }
}