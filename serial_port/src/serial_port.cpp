//serial_port.cpp

#include <ros/ros.h>
#include <serial/serial.h>
#include "geometry_msgs/Twist.h"
#include "message/Teleop.h"
#include <iostream>

    using namespace serial;

int keystate=0;
clock_t keytime,systime;
unsigned long datalength;
struct Command_convbot
{
    float header;
    float action;
    float fwd;
    float th;
    float x;
    float y;
    float z;
    float grp;
    float tail;
 };
 Command_convbot command_convbot;
 Command_convbot command_convbot_buffer;

//创建两个serial类
serial::Serial sp;

//收到订阅的消息后，会进入消息回调函数
void cmd_convbotCallback(const message::Teleop& cmd_convbot);


int main(int argc, char** argv)
{
    command_convbot.header=(float)(0xFC<<8|0x03);
    command_convbot.action=0;
    command_convbot.fwd=0;
    command_convbot.th=0;
    command_convbot.x=0;
    command_convbot.y=0;
    command_convbot.z=0;
    command_convbot.grp=0;
    command_convbot.tail=(float)(0x03<<8|0xFC);
    command_convbot_buffer.header=(float)(0xFC<<8|0x03);
    command_convbot_buffer.action=0;
    command_convbot_buffer.fwd=0;
    command_convbot_buffer.th=0;
    command_convbot_buffer.x=0;
    command_convbot_buffer.y=0;
    command_convbot_buffer.z=0;
    command_convbot_buffer.grp=0;
    command_convbot_buffer.tail=(float)(0x03<<8|0xFC);
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
    //创建一个Subcriber,订阅名为chatter的话题，注册回调函数chatterCallback
    ros::Subscriber sub = n.subscribe("cmd_convbot", 1000, cmd_convbotCallback);
    //打开串口ttyUSB0
    try
    {
        //打开串口
        sp.open();

    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("unable to open port ttyUSB0.");
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

    ros::Rate loop_rate(50);
   
     while(ros::ok())
    {
        ros::spinOnce();
        if(keystate==1)
        {
            ROS_INFO("Here shows the content of serial transport!");
	        ROS_INFO("Message Components:[%f,%f,%f]",command_convbot.fwd,command_convbot.th,command_convbot.x);
	        ROS_INFO("Message Components:[%f,%f,%f]",command_convbot.y,command_convbot.z,command_convbot.grp);
            datalength = sp.write((uint8_t *)&command_convbot, sizeof(command_convbot));
            ROS_INFO("The length of data transmited to stm32:[%d]",datalength);
        }
        if(keystate==0)
        {
            command_convbot_buffer.grp = command_convbot.grp;
            ROS_INFO("Here shows the content of serial transport!");
	        ROS_INFO("Message Components:[%f,%f,%f]",command_convbot_buffer.fwd,command_convbot_buffer.th,command_convbot_buffer.x);
	        ROS_INFO("Message Components:[%f,%f,%f]",command_convbot_buffer.y,command_convbot_buffer.z,command_convbot_buffer.grp);
            datalength = sp.write((uint8_t *)&command_convbot_buffer, sizeof(command_convbot_buffer));
            ROS_INFO("The length of data transmited to stm32:[%d]",datalength);
        }
        systime = clock();
        if(systime-keytime>40000)
        {
            keystate = 0;
        }
        //获取串口发来的数据
        /*
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            ROS_INFO("Here shows the content got from stm32:");
            for(int i=0; i<n; i++)
            {   
                //16进制的方式打印到屏幕
                std::cout  << std::hex <<(buffer[i] & 0xff) << " ";
               // printf("%d",buffer[i]&0xff);
            }
            std::cout << std::endl;
        }
        */

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
    
    keystate = 1;
    keytime = clock();
    //ROS_INFO("Received a /cmd_convbot message!");
	//ROS_INFO("Message Components:[%f,%f,%f]",cmd_convbot.fwd,cmd_convbot.th,cmd_convbot.x);
	//ROS_INFO("Message Components:[%f,%f,%f]",cmd_convbot.y,cmd_convbot.z,cmd_convbot.grp);
    command_convbot.fwd=cmd_convbot.fwd;
    command_convbot.th=cmd_convbot.th;
    command_convbot.x=cmd_convbot.x;
    command_convbot.y=cmd_convbot.y;
    command_convbot.z=cmd_convbot.z;
    command_convbot.grp=cmd_convbot.grp;
    
}

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
   