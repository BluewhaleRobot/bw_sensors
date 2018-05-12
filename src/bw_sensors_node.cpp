#include "bw_sensors/AsyncSerial.h"
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "bw_sensors/bw_sensors.h"

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to bw_sensors serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "bw_sensors_serial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/bwSensors");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    cout<<"port:"<<port<<" baud:"<<baud<<endl;



    try {
        CallbackAsyncSerial serial(port,baud);
        bw_sensors::StatusPublisher bwSensors_server(&serial);
        serial.setCallback(boost::bind(&bw_sensors::StatusPublisher::UpdateStatus,&bwSensors_server,_1,_2));
        boost::thread cmd2serialThread(&bw_sensors::StatusPublisher::run,&bwSensors_server);

        ros::Rate r(50);//发布周期为50hz
        while (ros::ok())
        {
            bwSensors_server.Refresh();//定时发布状态
            r.sleep();
        }
        quit:
        serial.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }
    ros::shutdown();
    return 0;
}
