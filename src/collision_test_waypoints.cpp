/*
* @file new_waypoint_calc.cpp
* Converts global waypoints (Lat, Lon, Alt) to Local waypoints (x, y, z) and sends over the /updated_coordinates topic
*/

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/HomePosition.h>
#include <math.h>
#include <stdlib.h>
#include <string>

using namespace std;

#define no_of_uav 3


int main(int argc, char **argv) {

    ros::init(argc, argv, "collision_test_waypoints");
    ros::NodeHandle nh;

    array<ros::Publisher, no_of_uav> wp_pub;
    for (int i = 0; i < no_of_uav; i++) {
        string pub_topic = "/updated_coordinates" + to_string(i);
        wp_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(pub_topic, 10);
    }

    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped target_point[no_of_uav];
    
    target_point[0].pose.position.x = 0.0;
    target_point[0].pose.position.y = -5.0;
    target_point[0].pose.position.z = 2.0;

    target_point[1].pose.position.x = -5.0;
    target_point[1].pose.position.y = 5.0;
    target_point[1].pose.position.z = 2.0;

    target_point[2].pose.position.x = 5.0;
    target_point[2].pose.position.y = 5.0;
    target_point[2].pose.position.z = 2.0;

    while(ros::ok()) {

        for (int i = 0; i < no_of_uav; i++) {
            wp_pub[i].publish(target_point[i]);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}