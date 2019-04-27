//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include "geometry_msgs/Twist.h"
#include "message/Teleop.h"
#include <iostream>
    using namespace serial;

struct Command_convbot
{
    float action;
    float fwd;
    float th;
    float x;
    float y;
    float z;
    float grp;
 };
 Command_convbot command_convbot;

//创建一个serial类
serial::Serial sp;

//收到订阅的消息后，会进入消息回调函数
void cmd_convbotCallback(const message::Teleop& cmd_convbot);


int main(int argc, char** argv)
{
    struct Command
    {
        uint8_t action;
        uint32_t x;
        uint32_t y;
        uint32_t z;
    };
    Command command;
    command.action=1;
    command.x=100;
    command.y=200;
    command.z=300;

    command_convbot.action=0;
    command_convbot.fwd=0;
    command_convbot.th=0;
    command_convbot.x=0;
    command_convbot.y=0;
    command_convbot.z=0;
    command_convbot.grp=0;

    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //设置串口数据位
    sp.setBytesize(eightbits);
    //设置串口奇偶校验位
    sp.setParity(parity_none);
    //设置串口停止位
    sp.setStopbits(stopbits_one);
    //设置是否流控制
    sp.setFlowcontrol(flowcontrol_none);
    //设置串口timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened successfully.");
    }
    else
    {
        return -1;
    }
    
    //创建一个Subcriber,订阅名为chatter的话题，注册回调函数chatterCallback
    ros::Subscriber sub = n.subscribe("cmd_convbot", 1000, cmd_convbotCallback);
   
    ros::Rate loop_rate(500);
   /*
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            
            for(int i=0; i<n; i++)
            {   
                //16进制的方式打印到屏幕
                std::cout  << std::hex <<(buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, n);
        }
        loop_rate.sleep();
    }
    */


  //  sp.write((uint8_t *)&command, sizeof(command));
   
     while(ros::ok())
    {
         ros::spinOnce();
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            
            for(int i=0; i<n; i++)
            {   
                //16进制的方式打印到屏幕
                std::cout  << std::hex <<(buffer[i] & 0xff) << " ";
               // printf("%d",buffer[i]&0xff);
            }
            std::cout << std::endl;
        }
         loop_rate.sleep();
        
    }
    /*
    //关闭串口
    sp.close();
    */
   // ros::spin();
    return 0;
}

//消息回调函数
void cmd_convbotCallback(const message::Teleop& cmd_convbot)
{
    //将收到的消息打印出来
    //ROS_INFO("I heard : [%s]",msg->x);
    unsigned long a;
    ROS_INFO("Received a /cmd_convbot message!");
	ROS_INFO("Message Components:[%f,%f,%f]",cmd_convbot.fwd,cmd_convbot.th,cmd_convbot.x);
	ROS_INFO("Message Components:[%f,%f,%f]",cmd_convbot.y,cmd_convbot.z,cmd_convbot.grp);
    command_convbot.fwd=cmd_convbot.fwd;
    command_convbot.th=cmd_convbot.th;
    command_convbot.x=cmd_convbot.x;
    command_convbot.y=cmd_convbot.y;
    command_convbot.z=cmd_convbot.z;
    command_convbot.grp=cmd_convbot.grp;
    ROS_INFO("Here shows the content of serial transport!");
	ROS_INFO("Message Components:[%f,%f,%f]",command_convbot.fwd,command_convbot.th,command_convbot.x);
	ROS_INFO("Message Components:[%f,%f,%f]",command_convbot.y,command_convbot.z,command_convbot.grp);
   
    a = sp.write((uint8_t *)&command_convbot, sizeof(command_convbot));
    ROS_INFO("length:[%d]",a);
}