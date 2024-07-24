/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"
#include "sensor_msgs/Joy.h"

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

struct xbox_buttons{
 int buttonA;
 int buttonB;
 int buttonX;
 int buttonY;
 int buttonLB;
 int buttonRB;
 int buttonBack;
 int buttonStart;
 int buttonUnNammed;
 int buttonStickLeft;
 int buttonStickRight;
} xbox_buttons_t;

struct xbox_axis{
 float leftStickHorizontal;
 float leftStickVeritical;
 float leftLT;
 float rightStickHorizontal;
 float rightStickVertical;
 float rightRT;
 float axisUPDown;
 float axisRightLeft;
} xbox_axis_t;

xbox_axis xAxis;
xbox_buttons xButtons;

void chatterCallback(const sensor_msgs::Joy::ConstPtr& msg){
    memcpy((void*)&xAxis.leftStickHorizontal,(const void*)&msg->axes[0] , (sizeof(float)*8)); 
    memcpy((void*)&xButtons.buttonA, (const void*)&msg->buttons[0] , (sizeof(int)*11));

}

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    
    ros::Subscriber sub = n.subscribe("/joy", 1000, chatterCallback);

    //SetLevel(HIGHLEVEL);
    long motiontime = 0;
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);
    uint8_t xMode = 1; //0 seems to be invalid or not do anything; 1 is stationary, relatively safe to start
    while (ros::ok()){
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        
        std::cout << "Mode: " << (int) RecvHighROS.mode;
        std::cout << " Forward Speed: " << RecvHighROS.forwardSpeed;
        std::cout << " Side Speed: "  << RecvHighROS.sideSpeed ;
        std::cout << " Rotate Speed: "  << RecvHighROS.rotateSpeed << std::endl ;


        if(xButtons.buttonA == 1)
        {
            xMode = 0;
        }
        
        if(xButtons.buttonB == 1)
        {
            xMode = 1;
        }

        if(xButtons.buttonX == 1)
        {
            xMode = 2;
        }

        if(xButtons.buttonY == 1)
        {
            xMode = 3;
        }

        SendHighROS.mode = xMode;
        SendHighROS.forwardSpeed = xAxis.leftStickVeritical;
        SendHighROS.sideSpeed = xAxis.rightStickHorizontal;
        SendHighROS.rotateSpeed = xAxis.leftStickHorizontal;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = xAxis.rightStickVertical;
        //SendHighROS.bodyHeight = xAxis.rightStickVertical;  // Do not update the bodyheight all the time
        
        // std::cout << "Mode: " << (int) SendHighROS.mode;
        // std::cout << " Forward Speed: " << SendHighROS.forwardSpeed;
        // std::cout << " Side Speed: "  << SendHighROS.sideSpeed ;
        // std::cout << " Rotate Speed: "  << SendHighROS.rotateSpeed << std::endl ;
        
        SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "walk_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    #ifdef SDK3_1
        aliengo::Control control(aliengo::HIGHLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    #endif

    #ifdef SDK3_2
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        // UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    #endif
    
}
