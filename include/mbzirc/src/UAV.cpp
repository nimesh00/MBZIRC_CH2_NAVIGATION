#include "UAV.h"
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>

void UAV::new_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    last_coords_wrapper = updated_coords_wrapper;
    updated_coords_wrapper = *msg;

    got_new_coordinates = 1;
    if (!(last_coords_wrapper.pose.position.x == updated_coords_wrapper.pose.position.x &&
        last_coords_wrapper.pose.position.y == updated_coords_wrapper.pose.position.y &&
        last_coords_wrapper.pose.position.z == updated_coords_wrapper.pose.position.z)) {
            got_new_coordinates = 1;
        }

    last_coordinates.x = last_coords_wrapper.pose.position.x;
    last_coordinates.y = last_coords_wrapper.pose.position.y;
    last_coordinates.z = last_coords_wrapper.pose.position.z;

    updated_coordinates.x = updated_coords_wrapper.pose.position.x;
    updated_coordinates.y = updated_coords_wrapper.pose.position.y;
    updated_coordinates.z = updated_coords_wrapper.pose.position.z;
}

void UAV::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void UAV::curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates_wrapper = *msg;
    current_coordinates_wrapper.pose.position.x += x_initial;
    current_coordinates_wrapper.pose.position.y += y_initial;
    current_coordinates_wrapper.pose.position.z += z_initial;
    current_coordinates.x = current_coordinates_wrapper.pose.position.x;
    current_coordinates.y = current_coordinates_wrapper.pose.position.y;
    current_coordinates.z = current_coordinates_wrapper.pose.position.z;
}

void UAV::compass_cb(const std_msgs::Float64::ConstPtr& msg) {
    compass_current_reading = *msg;
    compass_current = compass_current_reading.data;
}

void UAV::publish_velocities(float velocity_x, float velocity_y, float velocity_z, float yaw_rate) {
    velocity_control_data.header.stamp = ros::Time::now();
    velocity_control_data.velocity.x = velocity_x;
    velocity_control_data.velocity.y = velocity_y;
    velocity_control_data.velocity.z = velocity_z;
    velocity_control_data.yaw_rate = yaw_rate;
    velocity_control_commands.publish(velocity_control_data);
}

void UAV::publish_points(float x, float y, float z, float angle=0.0) {
    position_control_data.pose.position.x = x;
    position_control_data.pose.position.y = y;
    position_control_data.pose.position.z = z;
    position_control_data.pose.orientation.w = cos(angle / 2);
    position_control_data.pose.orientation.z = sin(angle / 2);
    position_control_data.pose.orientation.y = 0;
    position_control_data.pose.orientation.x = 0;
    position_control_commands.publish(position_control_data);
}