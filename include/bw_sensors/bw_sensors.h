#ifndef BWSENSORS_H
#define BWSENSORS_H

#include "bw_sensors/AsyncSerial.h"

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"

#define PI 3.14159265

namespace bw_sensors
{
typedef struct {
  int status;//状态，0表示未初始化，1表示正常，-1表示error
  float power;//外部测量电压【9 13】v
  float quat[4];//4元数
  float IMU[9];//mpu9250 9轴数据
  float distance[4];//超声波距离，单位m
  unsigned int time_stamp_sensor;//时间戳
  float theta;//方位角，【0 360）°
}SENSOR_STATUS;

class StatusPublisher
{
public:
    StatusPublisher(CallbackAsyncSerial* cmd_serial);
    void Refresh();
    void UpdateStatus(const char *data, unsigned int len);
    void UpdateOdom(const nav_msgs::Odometry & current_odom);
    void imuCalibration(const std_msgs::Bool& calFlag);
    void run();
    SENSOR_STATUS car_status;

private:

    std_msgs::Float64 CarPower;// 小车电池信息
    sensor_msgs::Imu  CarIMU;
    sensor_msgs::Range CarSonar1;
    sensor_msgs::Range CarSonar2;
    sensor_msgs::Range CarSonar3;
    sensor_msgs::Range CarSonar4;

    geometry_msgs::Twist  CarTwist;

    ros::NodeHandle mNH;

    ros::Publisher mStatusFlagPub;
    ros::Publisher mPowerPub;
    ros::Publisher mIMUPub;
    ros::Publisher mSonar1Pub;
    ros::Publisher mSonar2Pub;
    ros::Publisher mSonar3Pub;
    ros::Publisher mSonar4Pub;
    ros::Publisher mStopPub;
    bool mbUpdated_;

    boost::mutex mMutex;

    float yaw_deltas[100];
    int yaw_index;
    float yaw_sum;
    float yaw_omega;
    bool yaw_ready;

    bool debug_flag;
    CallbackAsyncSerial* cmd_serial_;
};

} //bw_sensors

#endif // BWSENSORS_H
