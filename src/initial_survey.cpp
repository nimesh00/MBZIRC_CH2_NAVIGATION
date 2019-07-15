#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include<vector>
#include<iostream>
#include<cmath> // use of math.h is permitted but not advisable
#include <stdlib.h>

using namespace std;

#define no_of_uav 3

// For mbzirc_nav.sh
float uav_initial_pos[3][3]={{0,0,0},{1,0,0},{-1,0,0}};

//For multi_uav.sh
// float uav_initial_pos[3][3]={{0,5,0},{5,-5,0},{-5,-5,0}};

bool detected_pickup_zone = 0;
bool detected_drop_zone = 0;

geometry_msgs::PoseStamped current_coordinates[3];
void curr_coords_cb0(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates[0] = *msg;
    current_coordinates[0].pose.position.x += uav_initial_pos[0][0];
    current_coordinates[0].pose.position.y += uav_initial_pos[0][1];
}
void curr_coords_cb1(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates[1] = *msg;
    current_coordinates[1].pose.position.x += uav_initial_pos[1][0];
    current_coordinates[1].pose.position.y += uav_initial_pos[1][1];
}
void curr_coords_cb2(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates[2] = *msg;
    current_coordinates[2].pose.position.x += uav_initial_pos[2][0];
    current_coordinates[2].pose.position.y += uav_initial_pos[2][1];
}



float distance_between_cartesian_points(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2) {
    float delta_x = point1.pose.position.x - point2.pose.position.x;
    float delta_y = point1.pose.position.y - point2.pose.position.y;
    // float delta_z = point1.pose.position.z - point2.pose.position.z;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2));
}

geometry_msgs::PoseStamped get_quarternions(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2) {
    float delta_y = point2.pose.position.y - point1.pose.position.y;
    float delta_x = point2.pose.position.x - point1.pose.position.x;

    float angle = atan2(delta_y, delta_x);
    geometry_msgs::PoseStamped return_value;
    return_value.pose.orientation.w = cos(angle / 2);
    return_value.pose.orientation.z = sin(angle / 2);
    return_value.pose.orientation.y = 0;
    return_value.pose.orientation.x = 0;
    return return_value;
}

int brick_picked_up(int id, geometry_msgs::PoseStamped target) {
    float distance = distance_between_cartesian_points(current_coordinates[id], target);
    if (distance < 0.2) {
        return 1;
    }
    return 0;
}

int brick_placed(int id, geometry_msgs::PoseStamped target) {
    float distance = distance_between_cartesian_points(current_coordinates[id], target);
    if (distance < 0.2) {
        return 1;
    }
    return 0;
}

int current_picking_drone = 0;
int current_dropping_drone = -1;
int current_idle_drone = 1;

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "initial_survey");
    ros::NodeHandle nh;

    ros::Subscriber crnt_cords_s0 = nh.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, curr_coords_cb0);
    ros::Publisher local_pos_pub0 = nh.advertise<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10);
    ros::Subscriber crnt_cords_s1 = nh.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, curr_coords_cb1);
    ros::Publisher local_pos_pub1 = nh.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
    ros::Subscriber crnt_cords_s2 = nh.subscribe<geometry_msgs::PoseStamped>("uav2/mavros/local_position/pose", 10, curr_coords_cb2);
    ros::Publisher local_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>("uav2/mavros/setpoint_position/local", 10);

    array<ros::Publisher, 3> wp_for_node;
    for (size_t i = 0; i < wp_for_node.size(); i++) {
        stringstream topic_name;
        topic_name << "/updated_coordinates" << i;
        wp_for_node[i] = nh.advertise<geometry_msgs::PoseStamped>(topic_name.str(), 10);
    }


    ros::Rate loop_rate(100);
    ros::Time begin = ros::Time::now();
    // ros::Time a_lii(0.001);

    vector<array<float, 2>> survey_points[3];

    int x_max = 50;
    int y_max = 60;

    float FOV = 2;
    float angle = 360.0 / no_of_uav;

    int r = 1;
    float theta = 0.0;

    for (int i = 0; i < no_of_uav; i++) {
        for (r = 2; r < 50; r += FOV) {
            if (r % 4 != 0) {
                for (theta = i * 120; theta < 120 * (i + 1); theta += 30) {
                    float angle = theta * (M_PI / 180);
                    array<float, 2> coordinates;
                    coordinates[0] = r * cos(angle);
                    coordinates[1] = r * sin(angle);
                    if (coordinates[0] > x_max) {
                        coordinates[0] = x_max;
                    }
                    if (coordinates[1] > y_max) {
                        coordinates[1] = y_max;
                    }
                    survey_points[i].push_back(coordinates);
                }
            } else {
                for (theta = 120 + i * 120; theta > i * 120; theta -= 30) {
                    float angle = theta * (M_PI / 180);
                    array<float, 2> coordinates;
                    coordinates[0] = r * cos(angle);
                    coordinates[1] = r * sin(angle);
                    if (coordinates[0] > x_max) {
                        coordinates[0] = x_max;
                    }
                    if (coordinates[1] > y_max) {
                        coordinates[1] = y_max;
                    }
                    survey_points[i].push_back(coordinates);
                }
            }
        }
    }

    int wp_no[3] = {0, 0, 0};
    geometry_msgs::PoseStamped current_target[3];

    while(ros::ok()) {

        if (detected_pickup_zone == 1 && detected_drop_zone == 1) {
            break;
        }
        for (int i = 0; i < no_of_uav; i++) {
            current_target[i].pose.position.x = survey_points[i][wp_no[i]][0];
            current_target[i].pose.position.y = survey_points[i][wp_no[i]][1];
            current_target[i].pose.position.z = 5;

            float distance_to_next = distance_between_cartesian_points(current_coordinates[i], current_target[i]);
            if (distance_to_next < 0.5) {
                wp_no[i] += 1;
            }
            wp_for_node[i].publish(current_target[i]);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    

  return 0;
}