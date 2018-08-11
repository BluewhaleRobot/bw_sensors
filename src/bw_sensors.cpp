#include "bw_sensors/AsyncSerial.h"
#include "bw_sensors/bw_sensors.h"
#define DISABLE 0
#define ENABLE 1

namespace bw_sensors
{
StatusPublisher::StatusPublisher(CallbackAsyncSerial* cmd_serial)
                                :cmd_serial_(cmd_serial)
{
  mbUpdated_=false;

  car_status.status = 0;
  car_status.power = 0.0;
  car_status.theta = 0.0;

  CarSonar1.header.frame_id = "sonar1";
  CarSonar1.radiation_type = 0;
  CarSonar1.field_of_view = 0.7;
  CarSonar1.min_range = 0.19;
  CarSonar1.max_range = 4.2;

  CarSonar2.header.frame_id = "sonar2";
  CarSonar2.radiation_type = 0;
  CarSonar2.field_of_view = 0.7;
  CarSonar2.min_range = 0.19;
  CarSonar2.max_range = 4.2;

  CarSonar3.header.frame_id = "sonar3";
  CarSonar3.radiation_type = 0;
  CarSonar3.field_of_view = 0.7;
  CarSonar3.min_range = 0.19;
  CarSonar3.max_range = 4.2;

  CarSonar4.header.frame_id = "sonar4";
  CarSonar4.radiation_type = 0;
  CarSonar4.field_of_view = 0.7;
  CarSonar4.min_range = 0.19;
  CarSonar4.max_range = 4.2;

  CarTwist.linear.x=0.0;
  CarTwist.linear.y=0.0;
  CarTwist.linear.z=0.0;
  CarTwist.angular.x=0.0;
  CarTwist.angular.y=0.0;
  CarTwist.angular.z=0.0;

  mSonar1Pub = mNH.advertise<sensor_msgs::Range>("bw_sensors/Sonar1", 1, true);
  mSonar2Pub = mNH.advertise<sensor_msgs::Range>("bw_sensors/Sonar2", 1, true);
  mSonar3Pub = mNH.advertise<sensor_msgs::Range>("bw_sensors/Sonar3", 1, true);
  mSonar4Pub = mNH.advertise<sensor_msgs::Range>("bw_sensors/Sonar4", 1, true);
  mIMUPub = mNH.advertise<sensor_msgs::Imu>("bw_sensors/IMU", 1, true);
  mPowerPub = mNH.advertise<std_msgs::Float64>("bw_sensors/Power", 1, true);
  mStatusFlagPub = mNH.advertise<std_msgs::Int32>("bw_sensors/StatusFlag",1,true);

  debug_flag=true;
  yaw_index=0;
  yaw_sum=0;
  yaw_omega=0;
  yaw_ready=false;
  for(int i=0;i<100;i++)
  {
    yaw_deltas[i]=0.0;
  }
}

void StatusPublisher::Refresh()
{
  static bool update_theta=false;
  //先处理imu
  {
    static float yaw_last=0.0;
    static int update_nums=0;
    boost::mutex::scoped_lock lock(mMutex);

    float angle;
    if(car_status.status==1 && yaw_ready )
    {
      update_theta=true;
    }
    else
    {
      update_theta=false;
    }

    if(mbUpdated_ && car_status.status==1)
    {
      //4元数转角度
      float pitch,roll,yaw;
      float q0,q1,q2,q3;
      q0 = car_status.quat[0];
      q1 = car_status.quat[1];
      q2 = car_status.quat[2];
      q3 = car_status.quat[3];
      pitch = asin(2*q1*q3 - 2*q0*q2)*57.3;
      roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
      yaw = atan2(2*(q1*q2 + q0*q3), q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
      if(!yaw_ready)
      {
        //先计算飘逸平均值初始值
        if(yaw_index==0) yaw_last = yaw;
        if(std::fabs(yaw - yaw_last)<0.01)
        {
          yaw_sum -= yaw_deltas[yaw_index];
          yaw_deltas[yaw_index] = yaw - yaw_last;
          yaw_sum += yaw_deltas[yaw_index];

          yaw_index++;
          if(yaw_index>99)
          {
            yaw_index = 0;
          }

          update_nums++;
          if(update_nums>300)
          {
            yaw_ready = true;
            update_theta=true;
            yaw_omega = yaw_sum/100.0;
            update_nums=25;
          }

        }
      }
     else
     {
       //更新飘逸速率
       if(std::fabs(CarTwist.linear.x) < 0.001 && std::fabs(CarTwist.angular.z) < 0.001 )
       {
         //
         if(update_nums<25)
         {
           update_nums++;
         }
         else
         {
           if(std::fabs(yaw - yaw_last)<0.01)
           {
             yaw_sum -= yaw_deltas[yaw_index];
             yaw_deltas[yaw_index] = yaw - yaw_last;
             yaw_sum += yaw_deltas[yaw_index];

             yaw_index++;
             if(yaw_index>99)
             {
               yaw_index = 0;
             }
             yaw_omega = yaw_sum/100.0;
           }
         }
         if(debug_flag)
         {
           if((yaw - yaw_last)<-179.999)
           {
             car_status.theta += 360 + (yaw - yaw_last) - yaw_omega;
           }
           else if((yaw - yaw_last)>179.999)
           {
             car_status.theta += -360 + (yaw - yaw_last) - yaw_omega;
           }
           else
           {
             car_status.theta += yaw - yaw_last - yaw_omega;
           }
         }
       }
       else
       {
         if(update_nums > 0) update_nums--;
         //将yaw转换成360度
         if((yaw - yaw_last)<-179.9999)
         {
           car_status.theta += 360 + (yaw - yaw_last) - yaw_omega;
         }
         else if((yaw - yaw_last)>179.999)
         {
           car_status.theta += -360 + (yaw - yaw_last) - yaw_omega;
         }
         else
         {
           car_status.theta += yaw - yaw_last - yaw_omega;
         }
       }
     }

      if( car_status.theta > 360) car_status.theta -= 360;
      if( car_status.theta < 0 ) car_status.theta += 360;
      yaw_last=yaw;
      //发布IMU topic
      ros::Time current_time = ros::Time::now();
      tf::Quaternion q;
      q.setRPY(roll/180.0*PI, -pitch/180.0*PI, car_status.theta/180.0*PI);
      CarIMU.header.stamp = current_time;
      CarIMU.header.frame_id = "bw_sensors_imu";
      CarIMU.orientation.x=q.x();
      CarIMU.orientation.y=q.y();
      CarIMU.orientation.z=q.z();
      CarIMU.orientation.w=q.w();

      CarIMU.angular_velocity.x=car_status.IMU[3]* PI /180.0f;
      CarIMU.angular_velocity.y=car_status.IMU[4]* PI /180.0f;
      CarIMU.angular_velocity.z=car_status.IMU[5]* PI /180.0f;

      CarIMU.linear_acceleration.x=car_status.IMU[0];
      CarIMU.linear_acceleration.y=car_status.IMU[1];
      CarIMU.linear_acceleration.z=car_status.IMU[2];

      mIMUPub.publish(CarIMU);
      mbUpdated_ = false;

      static unsigned int ii=0;
      ii++;
      if(ii%5==0)
      {
        //发布超声波topic
        if(car_status.distance[0]>0.1)
        {
          CarSonar1.header.stamp = current_time;
          CarSonar1.range = car_status.distance[0];
          mSonar1Pub.publish(CarSonar1);
        }

        if(car_status.distance[1]>0.1)
        {
          CarSonar2.header.stamp = current_time;
          CarSonar2.range = car_status.distance[1];
          mSonar2Pub.publish(CarSonar2);
        }

        if(car_status.distance[2]>0.1)
        {
          CarSonar3.header.stamp = current_time;
          CarSonar3.range = car_status.distance[2];
          mSonar3Pub.publish(CarSonar3);
        }

        if(car_status.distance[3]>0.1)
        {
          CarSonar4.header.stamp = current_time;
          CarSonar4.range = car_status.distance[3];
          mSonar4Pub.publish(CarSonar4);
        }
      }
    }
    //flag
    std_msgs::Int32 flag;
    if(car_status.status==1  && yaw_ready)
    {
      flag.data=1;
    }
    else
    {
      flag.data=0;
    }
    mStatusFlagPub.publish(flag);
    //电压
    CarPower.data = car_status.power;
    mPowerPub.publish(CarPower);
  }
}

void StatusPublisher::UpdateStatus(const char *data, unsigned int len)
{
    int i=0,j=0;
    int * receive_byte;
    static unsigned char last_str[2]={0x00,0x00};
    static unsigned char new_packed_ctr=DISABLE;//ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len=0;//包的理论长度
    static int new_packed_len=0;//包的实际长度
    static unsigned char cmd_string_buf[512];
    unsigned char current_str=0x00;
    const int cmd_string_max_size=512;
    receive_byte=(int *)&car_status;

    for(i=0;i<len;i++)
    {
        current_str=data[i];
        //判断是否有新包头
        if(last_str[0]==205&&last_str[1]==235&&current_str==215) //包头 205 235 215
        {
            //std::cout<<"runup1 "<<std::endl;
            new_packed_ctr=ENABLE;
            new_packed_ok_len=0;
            new_packed_len=new_packed_ok_len;
            last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
            last_str[1]=current_str;
            continue;
        }
        last_str[0]=last_str[1];//保存最后两个字符，用来确定包头
        last_str[1]=current_str;
        if (new_packed_ctr==ENABLE)
        {

            //获取包长度
            new_packed_ok_len=current_str;
            if(new_packed_ok_len>cmd_string_max_size) new_packed_ok_len=cmd_string_max_size; //包内容最大长度有限制
            new_packed_ctr=DISABLE;
            //std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        }
        else
        {
            //判断包当前大小
            if(new_packed_ok_len<=new_packed_len)
            {
                //std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
                //包长度已经大于等于理论长度，后续内容无效
                continue;
            }
            else
            {
                //获取包内容
                new_packed_len++;
                cmd_string_buf[new_packed_len-1]=current_str;
                if(new_packed_ok_len==new_packed_len&&new_packed_ok_len>0)
                {
                    //std::cout<<"runup4 "<<std::endl;
                    boost::mutex::scoped_lock lock(mMutex);
                    //当前包已经处理完成，开始处理
                    if(new_packed_ok_len==100)
                    {
                      for(j=0;j<20;j++)
                      {
                          memcpy(&receive_byte[j],&cmd_string_buf[5*j],4);
                      }
                      mbUpdated_=true;
                    }

                    if(mbUpdated_)
                    {
                      for(j=0;j<19;j++)
                      {
                          if(cmd_string_buf[5*j+4]!=32)
                          {
                            mbUpdated_=false;
                            break;
                          }
                      }
                    }
                    new_packed_ok_len=0;
                    new_packed_len=0;
                }
            }

        }

    }

    return;
}

void StatusPublisher::UpdateOdom(const nav_msgs::Odometry & current_odom)
{
  boost::mutex::scoped_lock lock(mMutex);
  CarTwist = current_odom.twist.twist;
}
void StatusPublisher::imuCalibration(const std_msgs::Bool& calFlag)
{
  if(calFlag.data)
  {
    //下发底层ｉｍｕ标定命令
    boost::mutex::scoped_lock lock(mMutex);
    char cmd_str[5]={(char)0xcd,(char)0xeb,(char)0xd7,(char)0x01,(char)0x43};
    if(NULL!=cmd_serial_)
    {
        cmd_serial_->write(cmd_str,5);
    }
  }
}
void StatusPublisher::run()
{
  ros::Subscriber sub1 = mNH.subscribe("bw_sensors/Odom", 1, &StatusPublisher::UpdateOdom, this);
  ros::Subscriber sub2 = mNH.subscribe("bw_sensors/IMU_cal", 1, &StatusPublisher::imuCalibration,this);
  ros::spin();
}

}
