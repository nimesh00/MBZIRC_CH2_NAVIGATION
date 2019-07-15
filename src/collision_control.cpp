#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

using namespace std;

#define no_of_uav 3

// float uav_initial_pos[3][3]={{0,5,0},{5,-5,0},{-5,-5,0}};
float uav_initial_pos[3][3]={{0,0,0},{1,0,0},{-1,0,0}};

geometry_msgs::Point C[no_of_uav], T[no_of_uav], velocity[no_of_uav];
void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, int id) {
    C[id].x = msg->pose.position.x + uav_initial_pos[id][0];
    C[id].y = msg->pose.position.y + uav_initial_pos[id][1];
    C[id].z = msg->pose.position.z + uav_initial_pos[id][2];
}

void target_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, int id) {
    T[id].x = msg->pose.position.x;
    T[id].y = msg->pose.position.y;
    T[id].z = msg->pose.position.z;
}

void velocity_cb(const nav_msgs::Odometry::ConstPtr& msg, int id) {
    velocity[id].x = msg->twist.twist.linear.x;
    velocity[id].y = msg->twist.twist.linear.y;
    velocity[id].z = msg->twist.twist.linear.z;
}

float distance_between_cartesian_points(geometry_msgs::Point point1, geometry_msgs::Point point2) {
    float delta_x = point1.x - point2.x;
    float delta_y = point1.y - point2.y;
    // float delta_z = point1.z - point2.z;
    // return sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2));
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "collision_control");
    ros::NodeHandle nh;
    ros::Rate rate(5.0);

    array<ros::Subscriber, no_of_uav> curr_coords_sub;
    array<ros::Subscriber, no_of_uav> target_coords_sub;
    array<ros::Subscriber, no_of_uav> velocity_sub;
    array<ros::Publisher, no_of_uav> corr_vel_pub;

    for (size_t i; i < no_of_uav; i++) {
        string target_coords_sub_topic = "/updated_coordinates" + to_string(i);
        string curr_coords_sub_topic = "/uav" + to_string(i) + "/mavros/local_position/pose";
        string velocity_sub_topic = "/uav" + to_string(i) + "/mavros/global_position/local";
        string corr_vel_pub_topic = "/uav" + to_string(i) + "/vel_correction_factor1";
        

        curr_coords_sub[i] = nh.subscribe<geometry_msgs::PoseStamped>(curr_coords_sub_topic, 10, boost::bind(&curr_coords_cb, _1, i));
        target_coords_sub[i] = nh.subscribe<geometry_msgs::PoseStamped>(target_coords_sub_topic, 10, boost::bind(&target_coords_cb, _1, i));
        velocity_sub[i] = nh.subscribe<nav_msgs::Odometry>(velocity_sub_topic, 10, boost::bind(&velocity_cb, _1, i));
        corr_vel_pub[i] = nh.advertise<std_msgs::Float64>(corr_vel_pub_topic, 10);
    }

    int i = 0, j = 0;
    geometry_msgs::Point O;
    float vel_correction_factor[3];
    std_msgs::Float64 vel_corr_factor_pub[3];

    while (ros::ok()) {
        for (i = 0; i < no_of_uav - 1; i++) {
            for(j = i + 1; j < no_of_uav; j++) {

                // cout << "for " << i << ": T: " << T[i] << "\tC: " << C[i] << endl;
                float m1 = (float)(T[i].y - C[i].y) / (T[i].x - C[i].x);
                float m2 = (float)(T[j].y - C[j].y) / (T[j].x - C[j].x);

                if (m1 == m2) {
                    vel_correction_factor[i] = 1;
                    vel_correction_factor[j] = 1;
                } else {
                    float c1 = (float)(T[i].x * C[i].y - T[i].y * C[i].x) / (T[i].x - C[i].x);
                    float c2 = (float)(T[j].x * C[j].y - T[j].y * C[j].x) / (T[j].x - C[j].x);
                    O.x = (float)(c2 - c1) / (m1 - m2);
                    O.y = (float)(c2 * m1 - c1 * m2) / (m1 - m2);

                    float OC1 = distance_between_cartesian_points(O, C[i]);
                    float OC2 = distance_between_cartesian_points(O, C[j]);
                    float TC1 = distance_between_cartesian_points(T[i], C[i]);
                    float TC2 = distance_between_cartesian_points(T[j], C[j]);
                    float OT1 = distance_between_cartesian_points(O, T[i]);
                    float OT2 = distance_between_cartesian_points(O, T[j]);

                    float error_1 = abs(OC1 + OT1 - TC1);
                    float error_2 = abs(OC2 + OT2 - TC2);

                    // cout << "error for " << i << " and " << j << ": " << error_1 << "\t" << error_2 << endl;

                    if ((error_1 > 0.1) || (error_2 > 0.1)) {
                        vel_correction_factor[i] = 1;
                        vel_correction_factor[j] = 1;
                        // cout << i << " and " << j << " approaching but won't collide!!" << endl;
                    } else {
                        float v1 = sqrt(pow(velocity[i].x, 2) + pow(velocity[i].y, 2));
                        float v2 = sqrt(pow(velocity[j].x, 2) + pow(velocity[j].y, 2));
                        // float td = 1.0 / (OC1 >= OC2) ? v2 : v1;
                        float td = 2.0;
                        float delay_time = 5.0;
                        float safety_factor = 5.0;
                        if (abs(((float)OC1 / v1) - ((float)OC2 / v2)) < td) {
                        // if (abs(OC1 - OC2) < td) {
                            cout << "Possible Collistion of " << i << " with " << j << endl;
                            if (OC1 >= OC2) {
                                // vel_correction_factor[i] = (float)(v1 * OC2) / (v2 * (OC1 - 1));
                                vel_correction_factor[i] = (float)(safety_factor * v1 * (OC2 + delay_time * v2)) / (v2 * OC1);
                                vel_correction_factor[j] = 1;
                                cout << "Slowing down " << i << " by " << vel_correction_factor[i] << endl;
                            } else {
                                // vel_correction_factor[j] = (float)(v2 * OC1) / (v1 * (OC2 - 1));
                                vel_correction_factor[i] = (float)(safety_factor * v2 * (OC1 + delay_time * v1)) / (v1 * OC2);
                                vel_correction_factor[i] = 1;
                                cout << "Slowing down " << j << " by " << vel_correction_factor[j] << endl;
                            }
                        } else {
                            // cout << i << " and " << j << " approaching each other but won't collide!!" << endl;
                            vel_correction_factor[i] = 1;
                            vel_correction_factor[j] = 1;
                        }
                    }

                }

                vel_corr_factor_pub[i].data = vel_correction_factor[i];
                corr_vel_pub[i].publish(vel_corr_factor_pub[i]);
                vel_corr_factor_pub[j].data = vel_correction_factor[j];
                corr_vel_pub[j].publish(vel_corr_factor_pub[j]);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}