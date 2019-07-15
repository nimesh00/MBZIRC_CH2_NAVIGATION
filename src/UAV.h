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
    public:
        // general variables for UAV
        int ID;
        float MAX_VELOCITY;
        float SOFTEN_PROFILE;
        float compass_initial;
        float compass_current;
        int got_new_coordinates;
        // mavros_msgs::SetMode offb_set_mode;
        // mavros_msgs::CommandBool arm_cmd;
        float x_initial, y_initial, z_initial;
        float velocity_x, velocity_y, velocity_z;
        float velocity_correction_factor;
        
        geometry_msgs::Point current_coordinates, target_coordinates, last_coordinates;
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
        ros::Subscriber velocity_correction_factor_sub;
        ros::Publisher  velocity_control_commands;
        ros::Publisher  position_control_commands;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;


    // callback functions for subscribers
    void new_coords_cb(const geometry_msgs::PoseStamped::ConstPtr&);
    void state_cb(const mavros_msgs::State::ConstPtr&);
    void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr&);
    void compass_cb(const std_msgs::Float64::ConstPtr&);
    void velocity_correction_factor_cb(const std_msgs::Float64::ConstPtr&);

    // UAV control function declarations
    void publish_velocities(float, float, float, float);
    void publish_points(float, float, float, float);
    void approach_target();

    // initialization of the UAV
    void INIT_UAV(int, float, float, float, ros::NodeHandle);


    // constructor for UAV
    UAV() {

    }
    // destructor for UAV(just testing if we need this or not) (We don't!!)
    // ~UAV() {
    //     cout << "Called Destructor for UAV " << ID;
    // }
};

#endif