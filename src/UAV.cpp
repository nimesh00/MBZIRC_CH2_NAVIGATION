#include "UAV.h"
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>

float distance_between_cartesian_points(geometry_msgs::Point point1, geometry_msgs::Point point2) {
    float delta_x = point1.x - point2.x;
    float delta_y = point1.y - point2.y;
    float delta_z = point1.z - point2.z;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
}

float get_angle(geometry_msgs::Point point1, geometry_msgs::Point point2) {
    float delta_y = point2.y - point1.y;
    float delta_x = point2.x - point1.x;

    return atan2(delta_y, delta_x);
}

float from360to180(float angle) {
    if (abs(angle) <= 180) {
        return angle;
    } else {
        return angle - 360;
    }
}

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

    target_coordinates.x = updated_coords_wrapper.pose.position.x;
    target_coordinates.y = updated_coords_wrapper.pose.position.y;
    target_coordinates.z = updated_coords_wrapper.pose.position.z;
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
    compass_current = (float)compass_current_reading.data;
}

void UAV::INIT_UAV(int id, float x_init, float y_init, float z_init, ros::NodeHandle nh) {
    // NO_OF_UAV++;
    ID = id;
    target_coordinates.x = x_initial = x_init;
    target_coordinates.y = y_initial = y_init;
    z_initial = z_init;
    target_coordinates.z = 2.0;
    got_new_coordinates = 0;
    MAX_VELOCITY = 10.0;
    SOFTEN_PROFILE = 1.2;
    compass_initial = 0.0;
    compass_current = 0.0;
    // stringstream uav_id;
    // uav_id << "/uav" << id;

    // topics for the diffrent publishers and subscribers
    string current_coords_sub_topic = "/uav" + to_string(ID) + "/mavros/local_position/pose";
    string target_coords_sub_topic = "/updated_coordinates" + to_string(ID);
    string state_sub_topic = "/uav" + to_string(ID) + "/mavros/state";
    string compass_sub_topic = "/uav" + to_string(ID) + "/mavros/global_position/compass_hdg";
    string velocity_control_commands_pub_topic = "/uav" + to_string(ID) + "/mavros/setpoint_raw/local";
    string position_control_commands_pub_topic = "uav" + to_string(ID) + "/mavros/setpoint_position/local";
    string armin_client_topic = "/uav" + to_string(ID) + "/mavros/cmd/arming";
    string set_mode_client_toipc = "/uav" + to_string(ID) + "/mavros/set_mode";

    // assigning toipcs to corresponding publisher and subscriber
    current_coords_sub = nh.subscribe<geometry_msgs::PoseStamped>(current_coords_sub_topic, 10, &UAV::curr_coords_cb, this);
    updated_coords_sub = nh.subscribe<geometry_msgs::PoseStamped>(target_coords_sub_topic, 10, &UAV::new_coords_cb, this);
    state_sub          = nh.subscribe<mavros_msgs::State>(state_sub_topic, 10, &UAV::state_cb, this);
    compass_sub        = nh.subscribe<std_msgs::Float64>(compass_sub_topic, 10, &UAV::compass_cb, this);
    velocity_control_commands = nh.advertise<mavros_msgs::PositionTarget>(velocity_control_commands_pub_topic, 10);
    position_control_commands = nh.advertise<geometry_msgs::PoseStamped>(position_control_commands_pub_topic, 10);
    arming_client      = nh.serviceClient<mavros_msgs::CommandBool>(armin_client_topic);
    set_mode_client    = nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_toipc);
    

    // initial setup of the UAV
    // initialize_UAV();


    // setting up control parameters
    compass_initial = compass_current;
    velocity_control_data.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;   //FRAME_LOCAL_NED=inertial frame        FRAME_BODY_NED = body frame
    velocity_control_data.header.frame_id = "world";
    velocity_control_data.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::FORCE |
                    mavros_msgs::PositionTarget::IGNORE_YAW;
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

void UAV::approach_target() {
    float yaw_rate = 0.0;
    float precision_distance = 0.5;
    // float softening_factor = 0.5;
    float velocity_factor = MAX_VELOCITY;
    float delta_x = target_coordinates.x - current_coordinates.x;
    float delta_y = target_coordinates.y - current_coordinates.y;
    float delta_z = target_coordinates.z - current_coordinates.z;

    // cout << "Current " << ID << ": X: " << current_coordinates.x << "\tY: " << current_coordinates.y << "\tZ: " << current_coordinates.z << endl;

    float distance_to_next = distance_between_cartesian_points(target_coordinates, current_coordinates);
    float angle = get_angle(current_coordinates, target_coordinates);
    float angle_degrees = angle * (180.0 / M_PI);
    float current_angle = compass_current - compass_initial;
    current_angle = from360to180(current_angle);
    // angle_degrees = from360to180(angle_degrees);
    float delta_angle = angle_degrees - current_angle;
    delta_angle = from360to180(delta_angle);
    delta_angle = delta_angle * (M_PI / 180);

    cout << "Angles: " << "T: " << angle_degrees << "\tC_i, C_c: " << current_angle << "\t D: " << from360to180(angle_degrees - current_angle) << endl;

    // cout << "Current Distance " << ID << "\t" << distance_to_next << endl;

    if (!((distance_to_next - abs(delta_z)) < 0.2)) {
        yaw_rate = -4 * tanhl(delta_angle / 5);
    } else {
        yaw_rate = 0;
    }
    if (distance_to_next < precision_distance) {
        SOFTEN_PROFILE = 5;
        // softening_factor = SOFTEN_PROFILE;
    } else {
        SOFTEN_PROFILE = 0.8;
        // softening_factor = 2;
    }

    float unit_vec_x = delta_x / distance_to_next;
    float unit_vec_y = delta_y / distance_to_next;
    float unit_vec_z = delta_z / distance_to_next;

    float velocity_x = MAX_VELOCITY * tanhl(unit_vec_x / (SOFTEN_PROFILE * MAX_VELOCITY));
    float velocity_y = MAX_VELOCITY * tanhl(unit_vec_y / (SOFTEN_PROFILE * MAX_VELOCITY));
    float velocity_z = MAX_VELOCITY * tanhl(unit_vec_z / (SOFTEN_PROFILE * MAX_VELOCITY));
    // cout << "Velocities for UAV" << ID << ": " << "X: " << velocity_x << "\tY: " << velocity_y << "\tZ:" << velocity_z << endl;
    
    // using velocity control function
    publish_velocities(velocity_x, velocity_y, velocity_z, yaw_rate);
}