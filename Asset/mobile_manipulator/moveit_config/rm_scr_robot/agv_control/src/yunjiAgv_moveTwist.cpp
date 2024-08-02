//
// Created by ubuntu on 21-10-25.
//

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <geometry_msgs/Twist.h>

using namespace std;


int sock_cli = -1;
//定义sockaddr_in
struct sockaddr_in servaddr;


void agvCtr_callback(const geometry_msgs::Twist::ConstPtr &msg) {


    string strControldata =
            "/api/joy_control?angular_velocity=" + to_string(msg->angular.z) + "&linear_velocity=" + to_string(msg->linear.x);

    cout << " send msg " << strControldata << endl;

    if (send(sock_cli, strControldata.c_str(), strControldata.length(), 0) < 0) {
        printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
//                return 0;
        close(sock_cli);
        sock_cli = socket(AF_INET, SOCK_STREAM, 0);
        connect(sock_cli, (struct sockaddr *) &servaddr, sizeof(servaddr));
    }

}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "yunjiAgv_moveTwist");
    // 创建节点句柄
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

    string ip;
    int port;

    nh.param<string>("ser_ip", ip, "192.168.10.10");
    nh.param<int>("ser_port", port, 31001);

    ros::Subscriber sub_agvCtr = nh.subscribe("/agv_control/agv_move_twist", 10, agvCtr_callback);

    //定义 sockfd
    sock_cli = socket(AF_INET, SOCK_STREAM, 0);


    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);  //服务器端口
    servaddr.sin_addr.s_addr = inet_addr(ip.c_str());  //服务器ip， inet_addr用于IPv4的IP转换(十进制转换为二进制)
    //连接服务器， 成功返回0， 错误返回 -1
    while (connect(sock_cli, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0) {
        cout << " connect " << ip << "failed! Attempt to reconnect..." << endl;
        ros::WallDuration(2.0).sleep(); //等待2秒
    }

    cout << " connect " << ip << "succeed!" << endl;

    ros::waitForShutdown();
    close(sock_cli);

    return 0;
}