//C++ program for visual servoing Method 2
//Abhishek Kashyap, Narayani, Rahul
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include<vector>
#include<iostream>
#include<cmath> // use of math.h is permitted but not advisable
#include <stdlib.h>

using namespace std;

geometry_msgs::PoseStamped current_coordinates[3];
void curr_coords_cb0(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates[0] = *msg;
}
void curr_coords_cb1(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates[1] = *msg;
}
void curr_coords_cb2(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates[2] = *msg;
}



mavros_msgs::State current_state[3];
void state_cb0(const mavros_msgs::State::ConstPtr& msg){
    current_state[0] = *msg;
}
void state_cb1(const mavros_msgs::State::ConstPtr& msg){
    current_state[1] = *msg;
}
void state_cb2(const mavros_msgs::State::ConstPtr& msg){
    current_state[2] = *msg;
}

float uav_initial_pos[3][3]={{0,0,5},{5,0,5},{-5,0,5}};

void seq_v2(char sequence[],vector < vector<float> > &ix,vector < vector<float> > &iy,vector < vector<float> > &iz,float brick_pos[],float brick_specs[], int siz, float uav_initial_pos[][3])
{
    int m=0,n=0,o=0;
    int i=0;
    float brick_initial_posx = brick_pos[0];
    float brick_initial_posy = brick_pos[1];
    float brick_length = brick_specs[0];
    float brick_width = brick_specs[0];
    int height1=5, height2 = 5, height3=5;
    float x,y,z;
    for(int l=0;l<siz;l++)
    {
        if(sequence[l]=='R')
        {
            x=5;
            y=-4;
            z=height1;
        }
        else if(sequence[l]=='G')
        {
            x=16;
            y=-16;
            z=height2;
        }
        else if(sequence[l]=='B')
        {
            x=16;
            y=-10;
            z=height3;
        }
        ix[i].push_back(x);
        ix[i].push_back(x);
        ix[i].push_back(brick_initial_posx);
        ix[i].push_back(brick_initial_posx);
        ix[i].push_back(uav_initial_pos[i][0]);

        iy[i].push_back(y);
        iy[i].push_back(y);
        iy[i].push_back(brick_initial_posy);
        iy[i].push_back(brick_initial_posy);
        iy[i].push_back(uav_initial_pos[i][1]);

        iz[i].push_back(z);
        iz[i].push_back(0.5);
        iz[i].push_back(5);
        iz[i].push_back(0.5);
        iz[i].push_back(uav_initial_pos[i][2]);

        i+=1;

        if(i>=3)
        {
            i=0;
        }
            brick_initial_posy=brick_length+brick_initial_posy;
        if(brick_initial_posy>13)
        {
            brick_initial_posy=13;
            brick_initial_posx=brick_initial_posx+brick_width;
        }
    }
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

void brick_pickup() {
    return;
}

void brick_release() {
    return;
}


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "ch2_complete_new");
    ros::NodeHandle nh;
    ros::Subscriber state_sub0 = nh.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, state_cb0);
    ros::Subscriber state_sub1 = nh.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, state_cb1);
    ros::Subscriber state_sub2 = nh.subscribe<mavros_msgs::State>("uav2/mavros/state", 10, state_cb2);

    ros::Subscriber crnt_cords_s0 = nh.subscribe<geometry_msgs::PoseStamped>("uav3/mavros/local_position/pose", 10, curr_coords_cb0);
    ros::Publisher local_pos_pub0 = nh.advertise<geometry_msgs::PoseStamped>("uav3/mavros/setpoint_position/local", 10);
    ros::Subscriber crnt_cords_s1 = nh.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, curr_coords_cb1);
    ros::Publisher local_pos_pub1 = nh.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
    ros::Subscriber crnt_cords_s2 = nh.subscribe<geometry_msgs::PoseStamped>("uav2/mavros/local_position/pose", 10, curr_coords_cb2);
    ros::Publisher local_pos_pub2 = nh.advertise<geometry_msgs::PoseStamped>("uav2/mavros/setpoint_position/local", 10);
    
    ros::ServiceClient arming_client0 = nh.serviceClient<mavros_msgs::CommandBool>("uav3/mavros/cmd/arming");
    ros::ServiceClient arming_client1 = nh.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
    ros::ServiceClient arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>("uav2/mavros/cmd/arming");

    ros::ServiceClient set_mode_client0 = nh.serviceClient<mavros_msgs::SetMode>("uav3/mavros/set_mode");
    ros::ServiceClient set_mode_client1 = nh.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");
    ros::ServiceClient set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>("uav2/mavros/set_mode");


    // std_srvs::Empty srv;
  // gripper_on.call(srv);


    ros::Rate loop_rate(100);
    ros::Time begin = ros::Time::now();
    // ros::Time a_lii(0.001);


    while(ros::ok() && !current_state[0].connected && !current_state[1].connected && !current_state[2].connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    float brick_length=1,brick_width=1;
    float brick_initial_posx= -15, brick_initial_posy= 9;
    int ptx=0;
    int siz=7;
    int fgg=1;
    int uav1_waypoint=0,uav2_waypoint=0,uav3_waypoint=0;
    int i=0, j=0, k=0;
    char sequence[siz]={'R','G','R','B','G','R','B'};

    vector < vector <float> > ix(3);
    vector < vector <float> > iy(3);
    vector < vector <float> > iz(3);


    float brick_pos[2]={brick_initial_posx,brick_initial_posy};
    float brick_specs[2]={brick_length,brick_width};

    // seq(sequence,ix1, iy1,iz1,ix2, iy2,iz2,ix3, iy3,iz3,brick_pos,brick_specs,siz);

    for(int i=0;i<3;i++)
    {
      ix[i].push_back(uav_initial_pos[i][0]);
      iy[i].push_back(uav_initial_pos[i][1]);
      iz[i].push_back(uav_initial_pos[i][2]);
    }
    
    seq_v2(sequence,ix,iy,iz,brick_pos,brick_specs,siz, uav_initial_pos);


  cout << "ix" << endl;
    for (int gb = 0; gb < ix.size(); gb++) { 
          int cc = ix[gb].size();
          for (int hn = 0; hn < cc; hn++) 
              cout << ix[gb][hn] << " "; 
          cout << endl; 
      }

  cout << "iy" << endl;
    for (int gb = 0; gb < iy.size(); gb++) { 
      int cc = iy[gb].size();
          for (int hn = 0; hn < cc; hn++) 
              cout << iy[gb][hn] << " "; 
          cout << endl; 
      }
  cout << "iz" << endl;
    for (int gb = 0; gb < iz.size(); gb++) { 
      int cc = iz[gb].size();
          for (int hn = 0; hn < cc; hn++) 
              cout << iz[gb][hn] << " "; 
          cout << endl; 
      }

    geometry_msgs::PoseStamped next_point;
    next_point.pose.position.x = 0;
    next_point.pose.position.y = 0;
    next_point.pose.position.z = 5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(next_point);
        local_pos_pub1.publish(next_point);
        local_pos_pub2.publish(next_point);
        ros::spinOnce();
        loop_rate.sleep();
    }

    bool switch_drone = 0;

    int sequence_number = 0;

    int current_active_drone = (sequence[sequence_number] == 'R') ? 0 : ((sequence[sequence_number] == 'G') ? 1 : 2);

    int drone_wp_no[3] = {1, 1, 1};

    float distance_to_next = 0.0;

    ros::Time last_request = ros::Time::now();

    cout << "Startig main loop!!" << endl;

    while (ros::ok())
    {
        if( (current_state[0].mode != "OFFBOARD" && current_state[1].mode != "OFFBOARD" && current_state[2].mode != "OFFBOARD") &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            
            int smcli0 = set_mode_client0.call(offb_set_mode);
            int smcli1 = set_mode_client1.call(offb_set_mode);
            int smcli2 = set_mode_client2.call(offb_set_mode);
            cout << smcli0 << smcli1 << smcli2 << endl;
            if( (smcli0 && smcli1 && smcli2) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( (!current_state[0].armed && !current_state[1].armed && !current_state[2].armed) &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( (arming_client0.call(arm_cmd) && arming_client1.call(arm_cmd) && arming_client2.call(arm_cmd)) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        next_point.pose.position.x = ix[current_active_drone][drone_wp_no[current_active_drone]];
        next_point.pose.position.y = iy[current_active_drone][drone_wp_no[current_active_drone]];
        next_point.pose.position.z = iz[current_active_drone][drone_wp_no[current_active_drone]];
        if ((next_point.pose.position.x == uav_initial_pos[current_active_drone][0]) &&
            (next_point.pose.position.y == uav_initial_pos[current_active_drone][1]) &&
            (next_point.pose.position.z == uav_initial_pos[current_active_drone][2])) {
            switch_drone = 1;
        }

        distance_to_next = distance_between_cartesian_points(current_coordinates[current_active_drone], next_point);

        float delta_z = next_point.pose.position.z - current_coordinates[current_active_drone].pose.position.z;

        geometry_msgs::PoseStamped quarternions = get_quarternions(current_coordinates[current_active_drone], next_point);

        if (!((distance_to_next - abs(delta_z)) < 0.5)) {
            next_point.pose.orientation.z = quarternions.pose.orientation.z;
            next_point.pose.orientation.w = quarternions.pose.orientation.w;
        }

        if (distance_to_next <= 0.5) {
            drone_wp_no[current_active_drone] += 1;
        }

        if (next_point.pose.position.x == 16 && next_point.pose.position.z == 0.5) {
            brick_pickup();
        }

        if (current_active_drone == 0) {
            local_pos_pub0.publish(next_point);
        }
        else if (current_active_drone == 1) {
            local_pos_pub1.publish(next_point);
        }
        else if (current_active_drone == 2) {
            local_pos_pub2.publish(next_point);
        }
        
        if (switch_drone == 1) {
            sequence_number++;
            int current_active_drone = (sequence[sequence_number] == 'R') ? 0 : ((sequence[sequence_number] == 'G') ? 1 : 2);
            switch_drone = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
     
    }

  return 0;



}