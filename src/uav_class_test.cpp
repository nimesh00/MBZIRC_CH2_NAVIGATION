#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <vector>
#include "UAV.h"
#include "UAV.cpp"

#define no_of_uavs 1

int main(int argc, char* argv[]) {

    // float uav_initial_pos[3][3]={{0,0,0},{1,0,0},{-1,0,0}};

    ros::init(argc, argv, "uav_class_test");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    // UAV uav1(1, -10.0, -10.0, 0.0, nh);
    UAV uav[no_of_uavs];

    // UAV **uav;
    // uav = new UAV* [no_of_uavs];

    for (int i = 0; i < no_of_uavs; i++) {
        // uav[i].INIT_UAV(i, uav_initial_pos[i][0], uav_initial_pos[i][1], uav_initial_pos[i][2], nh);
        uav[i].INIT_UAV(i, -10, -10, 0, nh);
    }

    cout << "UAVs initialized successfully!!" << endl;

    for (int i = 0; i < no_of_uavs; i++) {
        while(ros::ok() && !uav[i].current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        for (int i = 0; i < no_of_uavs; i++) {
            if( uav[i].current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( uav[i].set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !uav[i].current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(1.0))){
                    if( uav[i].arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            

            // can use publish_velocity() or publish_position() methods to contrl the UAV
            uav[i].approach_target();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}