//C++ program for visual servoing Method 2
//Abhishek Kashyap, Narayani, Rahul
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
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

float uav_initial_pos[3][3]={{0,0,0},{1,0,0},{-1,0,0}};

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
    float delta_z = point1.pose.position.z - point2.pose.position.z;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
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
  ros::init(argc, argv, "juggle_mbzirc");
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

    float brick_length=1,brick_width=1;
    float brick_initial_posx= -15, brick_initial_posy= 9;
    int siz=7;
    int i=0, j=0, k=0;
    char sequence[siz]={'R','G','R','B','G','R','B'};
    char sequence1[siz] = {0, 1, 0, 2, 1, 0, 2};

    geometry_msgs::PoseStamped hand1, hand2, after_pick_zone;

    geometry_msgs::PoseStamped drone_single_waypoints[3];

    bool switch_drone = 0;

    int sequence_number = 0;

    float pickup_zones[4][2] = {{16, -4}, {16, -16}, {16, -10}, {16, -22}};

    int dropper_drone = 0;
    int pickup_drone = 0;

    int drone_wp_no[3] = {1, 1, 1};

    float distance_to_next = 0.0;

    ros::Time last_request = ros::Time::now();

    int drop_no = 0, pickup_no = 0;

    int ready_to_drop = -1;
    int ready_to_pick = 1;
    int ready_to_idle = 2;
    int brick_pickup_flag = 0;

    while (ros::ok())
    {
        brick_pickup_flag = 0;
        // cout << "current dropper: " << current_dropping_drone << endl;
        // cout << "current_pickup: " << current_picking_drone << endl;
        // cout << "current idle: " << current_idle_drone << endl;
        // cout << ready_to_drop << ready_to_pick << ready_to_idle << endl;
        cout << "Pickup no: " << pickup_no << endl;
        cout << "Drop no: " << drop_no << endl;
        hand1.pose.position.x = pickup_zones[sequence1[pickup_no]][0];
        hand1.pose.position.y = pickup_zones[sequence1[pickup_no]][1];
        hand1.pose.position.z = 5;

        hand2.pose.position.x = brick_initial_posx;
        hand2.pose.position.y = brick_initial_posy + drop_no * brick_length;
        hand2.pose.position.z = 5;

        if (hand2.pose.position.y >= 13) {
            hand2.pose.position.y = 13;
            hand2.pose.position.x = brick_initial_posx + (drop_no - 4) * brick_width;
        }

        after_pick_zone.pose.position.x = 5;
        after_pick_zone.pose.position.y = 5;
        after_pick_zone.pose.position.z = 5;

        // cout << "initial referencing." << endl;


        if (current_picking_drone != -1) {
            if(abs(current_coordinates[current_picking_drone].pose.position.x - hand1.pose.position.x) < 0.5 &&
                abs(current_coordinates[current_picking_drone].pose.position.y - hand1.pose.position.y) < 0.5) {
                hand1.pose.position.z = 0.5;
                cout << "fake picking!!";
            }
            
            if (brick_picked_up(current_picking_drone, hand1)) {
                brick_pickup_flag = 1;
                wp_for_node[current_picking_drone].publish(after_pick_zone);
                ready_to_drop = current_picking_drone;
                current_picking_drone = -1;
                pickup_no = (pickup_no + 1) % 7;
                hand1.pose.position.z = 5;
            } else {
                drone_single_waypoints[current_picking_drone].pose.position.x = hand1.pose.position.x;
                drone_single_waypoints[current_picking_drone].pose.position.y = hand1.pose.position.y;
                drone_single_waypoints[current_picking_drone].pose.position.z = hand1.pose.position.z;

                wp_for_node[current_picking_drone].publish(drone_single_waypoints[current_picking_drone]);
            }
        } else {
            if (ready_to_pick != -1) {
                current_picking_drone = ready_to_pick;
                ready_to_pick = -1;
            }
        }

        // cout << "picking part!!" << endl;

        if (current_dropping_drone != -1) {
            if(abs(current_coordinates[current_dropping_drone].pose.position.x - hand2.pose.position.x) < 0.5 &&
                abs(current_coordinates[current_dropping_drone].pose.position.y - hand2.pose.position.y) < 0.5) {
                hand2.pose.position.z = 0.5;
                cout << "fake dropping!!";
            }
            if (brick_placed(current_dropping_drone, hand2)) {
                ready_to_idle = current_dropping_drone;
                current_dropping_drone = -1;
                drop_no = (drop_no + 1) % 7;
            } else {
                drone_single_waypoints[current_dropping_drone].pose.position.x = hand2.pose.position.x;
                drone_single_waypoints[current_dropping_drone].pose.position.y = hand2.pose.position.y;
                drone_single_waypoints[current_dropping_drone].pose.position.z = hand2.pose.position.z;

                wp_for_node[current_dropping_drone].publish(drone_single_waypoints[current_dropping_drone]);
            }
        } else {
            if (ready_to_drop != -1) {
                current_dropping_drone = ready_to_drop;
                ready_to_drop = -1;
            }
        }

        // cout << "dropping part" << endl;

        if (current_idle_drone != -1) {
            if (brick_pickup_flag) {
                ready_to_pick = current_idle_drone;
                current_idle_drone = -1;
            } else {
                drone_single_waypoints[current_idle_drone].pose.position.x = -10;
                drone_single_waypoints[current_idle_drone].pose.position.y = -10;
                drone_single_waypoints[current_idle_drone].pose.position.z = 5;
                wp_for_node[current_idle_drone].publish(drone_single_waypoints[current_idle_drone]);
            }
        } else {
            if (ready_to_idle != -1) {
                current_idle_drone = ready_to_idle;
                ready_to_idle = -1;
            }
        }

        // cout << "idle part!!" << endl;        

        ros::spinOnce();
        loop_rate.sleep();
     
    }

  return 0;
}