#define USING_SERIAL_PORT_
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
using namespace std;
#include "robot_controller/sensormodule.h"
#include "robot_controller/mainrobotcontroller.h"

#include "robot_controller/cmdRobotControllerMsg.h"
#include "robot_controller/SAMJointPos12Msg.h"



SensorModule  *mySensor;
struct Sys_flag_struct{
    unsigned char getFilteredData_50Hz:1;
    unsigned char getRawData_50Hz:1;
    unsigned char getFilteredData_125Hz:1;
    unsigned char getRawData_125Hz:1;
    unsigned char getFilteredData_25Hz:1;
}sys_flag;


#define SENSOR_TURNOFF 0
#define SENSOR_SEND_FILTERED_25HZ 1
//#define SENSOR_SEND_FILTERED_125HZ 2
//#define SENSOR_SEND_RAW_50HZ 3
//#define SENSOR_SEND_RAW_125HZ 4



#define ZMP_POSITION_0_X 0
#define ZMP_POSITION_0_Y 0

#define ZMP_POSITION_1_X 71
#define ZMP_POSITION_1_Y 0

#define ZMP_POSITION_2_X 80.5
#define ZMP_POSITION_2_Y 200

#define ZMP_POSITION_3_X 7
#define ZMP_POSITION_3_Y 200
#define ZMP_OFSET_ORIGIN_X_RIGHT 40
#define ZMP_OFSET_ORIGIN_Y_RIGHT 70

#define ZMP_POSITION_4_X 0
#define ZMP_POSITION_4_Y 0

#define ZMP_POSITION_5_X 71
#define ZMP_POSITION_5_Y 0

#define ZMP_POSITION_6_X 64
#define ZMP_POSITION_6_Y 200

#define ZMP_POSITION_7_X -9.5
#define ZMP_POSITION_7_Y 200

#define ZMP_OFSET_ORIGIN_X_LEFT 30
#define ZMP_OFSET_ORIGIN_Y_LEFT 70

double angle_controller[24];
double angle_robot[24];
const int samPos12_hardware[25]={1620,1680,2050,2060,663,3325,1260,2910,2730,2173,1237,2370,
                                 2025,2050,2050,2050,2050,2050,3100,940,0,0,2170,1500,2050};//
const double degreeToPose12=1/0.083;

void sub_function(const robot_controller::cmdRobotControllerMsg::ConstPtr& msg){
    switch (msg->command){
    case SENSOR_SEND_FILTERED_25HZ:
        ROS_INFO("SENSOR_SEND_FILTERED_25HZ : %d", msg->command);
        sys_flag.getFilteredData_25Hz=1;
        //        sys_flag.getFilteredData_125Hz=0;
        //        sys_flag.getRawData_50Hz=0;
        //        sys_flag.getRawData_125Hz=0;
        break;
        //    case SENSOR_SEND_FILTERED_125HZ:
        //        ROS_INFO("SENSOR_SEND_FILTERED_125HZ: %d", msg->command);
        //        sys_flag.getFilteredData_50Hz=0;
        //        sys_flag.getFilteredData_125Hz=1;
        //        sys_flag.getRawData_50Hz=0;
        //        sys_flag.getRawData_125Hz=0;
        //        break;

        //    case SENSOR_SEND_RAW_50HZ:
        //        ROS_INFO("SENSOR_SEND_RAW_50HZ : %d", msg->command);
        //        sys_flag.getFilteredData_50Hz=0;
        //        sys_flag.getFilteredData_125Hz=0;
        //        sys_flag.getRawData_50Hz=1;
        //        sys_flag.getRawData_125Hz=0;
        //        break;
        //    case SENSOR_SEND_RAW_125HZ:
        //        ROS_INFO("SENSOR_SEND_RAW_125HZ: %d", msg->command);
        //        sys_flag.getFilteredData_50Hz=0;
        //        sys_flag.getFilteredData_125Hz=0;
        //        sys_flag.getRawData_50Hz=0;
        //        sys_flag.getRawData_125Hz=1;
        //        break;
    default:
        ROS_INFO("receive msg, turn off sensor : %d", msg->command);
        sys_flag.getFilteredData_25Hz=0;
        //        sys_flag.getFilteredData_125Hz=0;
        //        sys_flag.getRawData_50Hz=0;
        //        sys_flag.getRawData_125Hz=0;
        break;
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE_1000Hz_);
    mySensor =new SensorModule;

    //    ros::Publisher robot_controller_pub =  n.advertise<sensor_hub::dataSensorMsg>("robot_controller_pub",1000);
    //    ros::Subscriber robot_controller_sub=n.subscribe<robot_controller_hub::cmdSensorMsg>("robot_controller_sub",200,sub_function);
    //    sensor_hub::dataSensorMsg sensorMsg;
    ros::Publisher robot_controller_pub =  n.advertise<robot_controller::SAMJointPos12Msg>("sam_pos12_sub",1000);
    ros::Subscriber robot_controller_sub= n.subscribe<robot_controller::cmdRobotControllerMsg>("robot_controller_sub",1000,sub_function);
    robot_controller::SAMJointPos12Msg SAMPos12Msg;

    sys_flag.getFilteredData_25Hz=0;
 ROS_INFO("ready");
    if(mySensor->Serial!= -1)
    {
        while(ros::ok())
        {
            if(FlagTimer.Hz_25){
                FlagTimer.Hz_25=0;
                //=================
                if(sys_flag.getFilteredData_25Hz){
                    mySensor->getFilteredData();
                }
            }
            if(FlagTimer.Hz_50)
            {
                FlagTimer.Hz_50=0;
                //================
                //                ROS_INFO("%s", "get data");
            }
            if(FlagTimer.Hz_100)
            {
                FlagTimer.Hz_100=0;
            }
            if(FlagTimer.Hz_125)
            {
                FlagTimer.Hz_125=0;
                //===============
            }


            //==========================================
            mySensor->Recev_Data_hanlder();
            if(mySensor->flagDataReceived_readRaw){
                mySensor->flagDataReceived_readRaw=0;
                //===========================================
                //                unsigned char Trans_chr[1] ={ '1'};
                //                mySensor->Send_Serial_String(mySensor->Serial, Trans_chr, 1);
//                if(mySensor->sensorDataAvail[0])
//                {
//                    SAMPos12Msg.SAMMode[12]=1;
//                    SAMPos12Msg.SAMPos12[12]=mySensor->sensorData[0];
//                }
//                else{
//                    SAMPos12Msg.SAMMode[12]=0;
//                }

//                if(mySensor->sensorDataAvail[1])
//                {
//                    SAMPos12Msg.SAMMode[14]=1;
//                    SAMPos12Msg.SAMPos12[14]=mySensor->sensorData[1];;
//                }
//                else{
//                    SAMPos12Msg.SAMMode[14]=0;
//                }

                if(mySensor->sensorDataAvail[0]&&mySensor->sensorDataAvail[3]){
                    angle_controller[12]=-(mySensor->sensorData[0]-930)*300/1024*M_PI/180;
                    angle_controller[14]=(mySensor->sensorData[3]-238)*300/1024*M_PI/180;

                    if(angle_controller[12]<0)
                        angle_controller[12]=0;
                    else if (angle_controller[12]>=M_PI/2) {
                        angle_controller[12]=M_PI/2-0.001;
                    }

                    if(angle_controller[14]<0)
                        angle_controller[14]=0;
                    else if(angle_controller[14]>M_PI)
                    {
                        angle_controller[14]=M_PI;
                    }


//                    if(cos(angle_controller[14])<0){
//                        angle_robot[14]=M_PI-asin(sin(angle_controller[12])*cos(angle_controller[14]));
//                    }
//                    else{
//                        angle_robot[14]=asin(sin(angle_controller[12])*cos(angle_controller[14]));
//                    }

                     angle_robot[14]=asin(sin(angle_controller[12])*cos(angle_controller[14]));

                    if(cos(angle_robot[14])!=0){
//                        angle_robot[12]=acos(cos(angle_controller[12])*cos(angle_controller[14])/cos(angle_robot[14]));
                        angle_robot[12]=asin(sin(angle_controller[14])/cos(angle_robot[14]));
                    }
                    else{
                        angle_robot[12]=M_PI/2;
                    }

                    SAMPos12Msg.SAMMode[14]=1;
                    SAMPos12Msg.SAMMode[12]=1;

                    SAMPos12Msg.SAMPos12[14]=(unsigned int)((angle_robot[14]*180/M_PI-90)*degreeToPose12+(double)samPos12_hardware[14]);;
                    SAMPos12Msg.SAMPos12[12]=(unsigned int)((angle_robot[12]*180/M_PI)*degreeToPose12+(double)samPos12_hardware[12]);;



                }else{
                    SAMPos12Msg.SAMMode[14]=0;
                    SAMPos12Msg.SAMMode[12]=0;
                }

                if(mySensor->sensorDataAvail[1])
                {
                    angle_robot[16]=(mySensor->sensorData[1]-486)*300/1024*M_PI/180;
                     SAMPos12Msg.SAMMode[16]=1;
                      SAMPos12Msg.SAMPos12[16]=(unsigned int)((angle_robot[16]*180/M_PI)*degreeToPose12+(double)samPos12_hardware[16]);;
                }else
                    SAMPos12Msg.SAMMode[16]=0;

                if(mySensor->sensorDataAvail[2])
                {
                    angle_robot[18]=(mySensor->sensorData[2]-568)*300/1024*M_PI/180;
                     SAMPos12Msg.SAMMode[18]=1;
                      SAMPos12Msg.SAMPos12[18]=(unsigned int)((angle_robot[18]*180/M_PI)*degreeToPose12+(double)samPos12_hardware[18]);;
                }



                 cout<<mySensor->sensorData[0]<<" : "<< angle_controller[12]*180/M_PI<<" -- "<<mySensor->sensorData[3]<<" : "<<angle_controller[14]*180/M_PI<<
                                             " -- "<<  angle_robot[12]*180/M_PI<<" : "<< angle_robot[14]*180/M_PI<<endl;
                robot_controller_pub.publish(SAMPos12Msg);

                //                sensorMsg.zmp_P0=mySensor->sensorData[0];
                //                sensorMsg.zmp_P1=mySensor->sensorData[1];
                //                sensorMsg.zmp_P2=mySensor->sensorData[2];
                //                sensorMsg.zmp_P3=mySensor->sensorData[3];
                //                sensorMsg.zmp_P4=mySensor->sensorData[4];
                //                sensorMsg.zmp_P5=mySensor->sensorData[5];
                //                sensorMsg.zmp_P6=mySensor->sensorData[6];
                //                sensorMsg.zmp_P7=mySensor->sensorData[7];
                //                sensorMsg.body_roll=mySensor->sensorData[8]/100.0;
                //                sensorMsg.body_pitch=mySensor->sensorData[9]/100.0;
                //                sensorMsg.body_yaw=mySensor->sensorData[10]/100.0;
                //                sensorMsg.body_roll_rate=mySensor->sensorData[11]/100.0;
                //                sensorMsg.body_pitch_rate=mySensor->sensorData[12]/100.0;
                //                sensorMsg.body_yaw_rate=mySensor->sensorData[13]/100.0;

                //                sensorMsg.right_amplitude=sensorMsg.zmp_P0+sensorMsg.zmp_P1+sensorMsg.zmp_P2+sensorMsg.zmp_P3;
                //                if(sensorMsg.right_amplitude==0){
                //                    sensorMsg.right_x=0;
                //                    sensorMsg.right_y=0;
                //                }
                //                else{
                //                sensor_pub.publish(sensorMsg);

            }
            ros::spinOnce();
            loop_rate.sleep();
            Timer_handler();
        }
#ifdef USING_SERIAL_PORT_
    }else{
        cout << "SERIAL : " << mySensor->Serial << " Connection error." << endl;
    }
    cout << endl;
    cout << "SERIAL : " << mySensor->Serial << " Device close." << endl;
    close(mySensor->Serial);
    cout << "SERIAL : " << "uxa_serial node terminate." << endl;
#endif


    return 0;
}
