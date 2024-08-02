#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <rm_msgs/Servo_Move.h>
#include <rm_msgs/Servo_GetAngle.h>

serial::Serial ros_ser;
#define	sBUFFERSIZE	10//send buffer size 串口发送缓存长度
unsigned char s_buffer[sBUFFERSIZE];//发送缓存

//舵机转动控制回调函数
void callback_servoMove(const rm_msgs::Servo_Move& msg){
    //  cmd_to_serial(msg);
    //  cmd_servo_move(msg.servo_id,msg.angle);
    memset(s_buffer,0,sizeof(s_buffer));
     s_buffer[0]=0x55;
     s_buffer[1]=0x55;
     s_buffer[2]=0x08;
     s_buffer[3]=0x03;
     s_buffer[4]=0x01;
     s_buffer[5]=0xe8;
     s_buffer[6]=0x03;
     s_buffer[7]=msg.servo_id;  //舵机ID
     s_buffer[8]=msg.angle; //角度位置低八位
     s_buffer[9]=msg.angle >> 8;    //角度位置高八位
    //  data_to_serial(s_buffer, 10);
    ros_ser.write(s_buffer, sBUFFERSIZE);
    ROS_INFO_STREAM("Control Servo Move");
 }

 //读取舵机角度位置值回调函数
void callback_servoGetAngle(const rm_msgs::Servo_GetAngle& msg){
    //  cmd_to_serial(msg);
    //  cmd_servo_move(msg.servo_id,msg.angle);
    memset(s_buffer,0,sizeof(s_buffer));
     s_buffer[0]=0x55;
     s_buffer[1]=0x55;
     s_buffer[2]=0x04;
     s_buffer[3]=0x15;
     s_buffer[4]=0x01;
     s_buffer[5]=msg.servo_id;  //舵机ID
    //  data_to_serial(s_buffer, 10);
    ros_ser.write(s_buffer, 6);
    ROS_INFO_STREAM("Get Servo Angle");
 }

int main (int argc, char** argv){
     ros::init(argc, argv, "my_serial_node");
     ros::NodeHandle n;
     //订阅控制舵机转动主题
     ros::Subscriber sub_servo_move = n.subscribe("/servo_control/move", 1000, callback_servoMove);
     //订阅获取舵机角度位置的主题
     ros::Subscriber sub_servo_getAngle = n.subscribe("/servo_control/get_angle", 1000, callback_servoGetAngle);
     //发布主题:舵机状态
    //  ros::Publisher pub_servo_state = n.advertise<std_msgs::String>("/servo_state", 1000);
     try
     {
         ros_ser.setPort("/dev/ttyUSB0");
         ros_ser.setBaudrate(9600);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser.setTimeout(to);
         ros_ser.open();
     }
     catch (serial::IOException& e)
     {
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
     }
     if(ros_ser.isOpen()){
         ROS_INFO_STREAM("Serial Port opened");
     }else{
         return -1;
     }
     ros::Rate loop_rate(10);
     while(ros::ok()){
         ros::spinOnce();

        //获取缓冲区内的字节数
        size_t n = ros_ser.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = ros_ser.read(buffer, n);
            ROS_INFO_STREAM("Reading from serial port");
            for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
        }
        //  if(ros_ser.available()){
        //      ROS_INFO_STREAM("Reading from serial port");
        //      std_msgs::String serial_data;
        //      //获取串口数据
        //      serial_data.data = ros_ser.read(ros_ser.available());
        //      ROS_INFO_STREAM("Read: " << serial_data.data);
        //      //将串口数据发布到主题sensor
        //     //  pub_servo_state.publish(serial_data);
        //  }
         loop_rate.sleep();
     }
     ros_ser.close();
 }
