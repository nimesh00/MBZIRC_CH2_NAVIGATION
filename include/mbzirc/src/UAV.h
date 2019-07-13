#ifndef __UAV_CLASS
#define __UAV_CLASS

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

using namespace std;

// typedef struct cartesian_coordinates {
//     float x;
//     float y;
//     float z;
// } cartesian_coordinates;

class UAV {
    static int NO_OF_UAV;
    public:
        // general variables for UAV
        int ID;
        float compass_initial;
        float compass_current;
        int got_new_coordinates;
        // mavros_msgs::SetMode offb_set_mode;
        // mavros_msgs::CommandBool arm_cmd;
        float x_initial, y_initial, z_initial;
        float velocity_x, velocity_y, velocity_z;
        
        geometry_msgs::Point current_coordinates, updated_coordinates, last_coordinates;
        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped updated_coords_wrapper, last_coords_wrapper, current_coordinates_wrapper;
        std_msgs::Float64 compass_current_reading;

        // Desired UAV control data to be sent
        mavros_msgs::PositionTarget velocity_control_data;
        geometry_msgs::PoseStamped position_control_data;

        // Publishers and Subscriber for coordinates, state, compass, control, arming, changing mode(in order)
        ros::Subscriber current_coords_sub;
        ros::Subscriber updated_coords_sub;
        ros::Subscriber state_sub;
        ros::Subscriber compass_sub;
        ros::Publisher  velocity_control_commands;
        ros::Publisher  position_control_commands;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;


    // callback functions for subscribers
    void new_coords_cb(const geometry_msgs::PoseStamped::ConstPtr&);
    void state_cb(const mavros_msgs::State::ConstPtr&);
    void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr&);
    void compass_cb(const std_msgs::Float64::ConstPtr&);

    // UAV control function declarations
    void publish_velocities(float, float, float, float);
    void publish_points(float, float, float, float);


    // constructor for UAV
    UAV(int id, float x_init, float y_init, float z_init, ros::NodeHandle nh) {
        NO_OF_UAV++;
        ID = NO_OF_UAV;
        got_new_coordinates = 0;
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
        position_control_commands = nh.advertise<mavros_msgs::PositionTarget>(position_control_commands_pub_topic, 10);
        arming_client      = nh.serviceClient<mavros_msgs::CommandBool>(armin_client_topic);
        set_mode_client    = nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_toipc);
        

        // initial setup of the UAV
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

    // destructor for UAV(just testing if we need this or not)
    ~UAV() {
        cout << "Called Destructor for UAV " << ID;
    }
};

#endif