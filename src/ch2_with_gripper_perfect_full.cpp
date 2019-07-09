//C++ program for visual servoing Method 2
//Abhishek Kashyap, Narayani, Rahul
#include "ros/ros.h"
// #include "interface/test1.h"
// #include "interface/test2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include<vector>
#include<stdio.h>
#include<iostream>
#include<algorithm>
#include<vector>
#include<array>
#include<functional> 
#include<cmath> // use of math.h is permitted but not advisable
// #include<Eigen/Dense> // #include<Eigen/Eigenvalues>
// #include<Eigen/Geometry>
#include <list> 
#include <iterator> 
#include <stdlib.h>
using namespace std; 
// using namespace Eigen;

string res1, res2, res3;
float px[3];
float py[3];
float pz[3];
float vx[3];
float vy[3];
float vz[3];
float wx[3];
float wy[3];
float wz[3];
float qx;
float qy;
float qz;
float qw;
float linearacc_x;
float linearacc_y;
float linearacc_z;
int image_h=240;
int image_w=320;
int count1 =0;
int xmax=5;
int ymax=5;
int t_sec=0;
int t_nsec=0;
int i =1;
int front_boxx=0, down_boxx=0;
int front_boxy=0, down_boxy=0;
int front_boxarea=0, down_boxarea=0;
int centx,centy;
int gripper_state=0;

float uav_initial_pos[3][3]={{0,0,5},{-5,0,5},{-5,0,5}};

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

void handle_poses1(const nav_msgs::Odometry::ConstPtr& msg)
{
  px[0]= msg->pose.pose.position.x;
  py[0]= msg->pose.pose.position.y;
  pz[0]= msg->pose.pose.position.z;
}
void handle_poses2(const nav_msgs::Odometry::ConstPtr& msg)
{
  px[1]= msg->pose.pose.position.x-5;//uav initial pos
  py[1]= msg->pose.pose.position.y;
  pz[1]= msg->pose.pose.position.z;
}
void handle_poses3(const nav_msgs::Odometry::ConstPtr& msg)
{
  px[2]= msg->pose.pose.position.x;//uav iniitla pos
  py[2]= msg->pose.pose.position.y-5;
  pz[2]= msg->pose.pose.position.z;
}
void handle_vel1(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  vx[0]= msg->twist.linear.x;
  vy[0]= msg->twist.linear.y;
  vz[0]= msg->twist.linear.z;
  wx[0]= msg->twist.angular.x;
  wy[0]= msg->twist.angular.y;
  wz[0]= msg->twist.angular.z;
}
void handle_vel2(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  vx[1]= msg->twist.linear.x;
  vy[1]= msg->twist.linear.y;
  vz[1]= msg->twist.linear.z;
  wx[1]= msg->twist.angular.x;
  wy[1]= msg->twist.angular.y;
  wz[1]= msg->twist.angular.z;
}
void handle_vel3(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  vx[2]= msg->twist.linear.x;
  vy[2]= msg->twist.linear.y;
  vz[2]= msg->twist.linear.z;
  wx[2]= msg->twist.angular.x;
  wy[2]= msg->twist.angular.y;
  wz[2]= msg->twist.angular.z;
}

// void handle_orientation1(const sensor_msgs::Imu::ConstPtr& data)
// {
// //   // ROS_INFO("x:[%f] y:[%f] z:[%f]", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
//   qx= data->orientation.x;
//   qy= data->orientation.y;
//   qz= data->orientation.z;
//   qw= data->orientation.w;
// //   double angularvelocity_x= data->angular_velocity.x;
// //   double angularvelocity_y= data->angular_velocity.y;
// //   double angularvelocity_z= data->angular_velocity.z;
//   linearacc_x= data->linear_acceleration.x;
//   linearacc_y= data->linear_acceleration.y;
//   linearacc_z= data->linear_acceleration.z;
// }
// void simple_image_in_front(const interface::test1::ConstPtr& msg)
// {
//   front_boxx= msg->front_cam_boxx;
//   front_boxy= msg->front_cam_boxy;
//   front_boxarea = msg->front_cam_boxarea;
// }
// void simple_image_in_down(const interface::test2::ConstPtr& msg)
// {
//   down_boxx= msg->down_cam_boxx;
//   down_boxy= msg->down_cam_boxy;
//   down_boxarea = msg->down_cam_boxarea;
// }
// void grab(const std_msgs::Bool::ConstPtr& msg)
// {
//   if(msg->data==true)
//   {
//     gripper_state=1;
//   }
//   else
//   {
//     gripper_state=0;
//   }
// }


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "ch2_complete_new");
    ros::NodeHandle n;
    // ros::Subscriber simple_image1 = n.subscribe("/front_cam", 10, simple_image_in_front);
    // ros::Subscriber simple_image2 = n.subscribe("/down_cam", 10, simple_image_in_down);
    ros::Subscriber position1 = n.subscribe("/uav0/mavros/global_position/local", 10, handle_poses1);
    ros::Subscriber velocity1 = n.subscribe("/uav0/mavros/local_position/velocity", 10, handle_vel1);
    ros::Subscriber position2 = n.subscribe("/uav1/mavros/global_position/local", 10, handle_poses2);
    ros::Subscriber velocity2 = n.subscribe("/uav1/mavros/local_position/velocity", 10, handle_vel2);
    ros::Subscriber position3 = n.subscribe("/uav2/mavros/global_position/local", 10, handle_poses3);
    ros::Subscriber velocity3 = n.subscribe("/uav2/mavros/local_position/velocity", 10, handle_vel3);
    // ros::Subscriber gripper = n.subscribe("/vacuum_gripper/grasping", 10, grab);
    // ros::ServiceClient gripper_on = n.serviceClient<std_srvs::Empty>("/vacuum_gripper/on");
    // ros::ServiceClient gripper_off = n.serviceClient<std_srvs::Empty>("/vacuum_gripper/off");
    // ros::Subscriber orientation1 = n.subscribe("/uav1/mavros/imu/data", 1000, handle_orientation1);
    ros::Publisher desired_velocity1= n.advertise<geometry_msgs::Twist>("chatter1", 100);
    ros::Publisher desired_velocity2= n.advertise<geometry_msgs::Twist>("chatter2", 100);
    ros::Publisher desired_velocity3= n.advertise<geometry_msgs::Twist>("chatter3", 100);
    geometry_msgs::Twist msg1,msg2 , msg3;

    std_srvs::Empty srv;
  // gripper_on.call(srv);


    ros::Rate loop_rate(100);
    //Quad Quaternion parameters
    float desx[3];
    float desy[3];    
    float desz[3];
    float desroll[3];
    float despitch[3];
    float desyaw[3];
    float disst[3];
    int flagg=0, flagg1=0, flagg2=0;
    //Initialization of clock time      
    float c_time=0, next_time=0,gett_time=0,t_diff=0.01;
    ros::Time begin = ros::Time::now();
    ros::Time a_lii(0.001);
    float x_offset_prev = 0;
    float y_offset_prev = 0;
    float z_offset_prev = 0;
    float rng_offset_prev = 0;
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
    // for (int const &i: ix1)
    //  {
    //    ROS_INFO("ix1:[%d]", i);
    //     // std::cout << i << ' ';
    //  }
    //  for (int const &i: iy1)
    //  {
    //    ROS_INFO("iy1:[%d]", i);
    //     // std::cout << i << ' ';
    //  }
    // for (int const &i: iz1)
    //  {
    //    ROS_INFO("iz1:[%d]", i);
    //     // std::cout << i << ' ';
    //  }
    // for (int i=0;i<4*uav1_siz;i++)
     // {
       // ROS_INFO("ix1:[%f] iy1:[%f] iz1:[%f]", ix1[i], iy1[i], iz1[i]);
        // std::cout << i << ' ';
     // }
    //  for (int i=0;i<2*uav2_siz;i++)
    //  {
    //    ROS_INFO("ix2:[%f] iy2:[%f] iz2:[%f]", ix2[i], iy2[i], iz2[i]);
    //     // std::cout << i << ' ';
    //  }
    while (ros::ok())
    {
      //swarm code start
//       switch(0)
//       {
//         case 0: cout << "navigation" << endl;
//                 disst[0]= sqrt(pow((ix[0][uav1_waypoint] -px[0]),2)+pow((iy[0][uav1_waypoint] -py[0]),2)+pow((iz[0][uav1_waypoint] -pz[0]),2));
//                 desx[0]= (ix[0][uav1_waypoint] -px[0])/disst[0]; 
//                 desy[0]= (iy[0][uav1_waypoint] -py[0])/disst[0]; 
//                 desz[0]= (iz[0][uav1_waypoint] -pz[0])/disst[0];
//                 // despitch=1;

//                 if(disst[0]<1)
//                 {
//                   desx[0]=0.1*desx[0];
//                   desy[0]=0.1*desy[0];
//                   desz[0]=0.1*desz[0];
//                   uav1_waypoint+=1;
//                 }
//                 if(uav1_waypoint>=ix[0].size())
//                 {
//                   uav1_waypoint-=1;
//                 }
// //////////////////////////////////////////////////
//                 disst[1]= sqrt(pow((ix[1][uav2_waypoint] -px[1]),2)+pow((iy[1][uav2_waypoint] -py[1]),2)+pow((iz[1][uav2_waypoint] -pz[1]),2));
//                 desx[1]= (ix[1][uav2_waypoint] -px[1])/disst[1]; 
//                 desy[1]= (iy[1][uav2_waypoint] -py[1])/disst[1]; 
//                 desz[1]= (iz[1][uav2_waypoint] -pz[1])/disst[1];
//                 // despitch=1;

//                 if(disst[1]<1)
//                 {
//                   desx[1]=0.1*desx[1];
//                   desy[1]=0.1*desy[1];
//                   desz[1]=0.1*desz[1];
                  
//                 }
//                 if((uav1_waypoint-1)>0)
//                 {
//                   uav2_waypoint+=1;
//                 }
                
//                 if(uav2_waypoint>=ix[1].size())
//                 {
//                   uav2_waypoint-=1;
//                 }
// ///////////////////////////////////////////////////
//                 disst[2]= sqrt(pow((ix[2][uav3_waypoint] -px[2]),2)+pow((iy[2][uav3_waypoint] -py[2]),2)+pow((iz[2][uav3_waypoint] -pz[2]),2));
//                 desx[2]= (ix[2][uav3_waypoint] -px[2])/disst[2]; 
//                 desy[2]= (iy[2][uav3_waypoint] -py[2])/disst[2]; 
//                 desz[2]= (iz[2][uav3_waypoint] -pz[2])/disst[2];
//                 // despitch=1;

//                 if(disst[2]<1)
//                 {
//                   desx[2]=0.1*desx[2];
//                   desy[2]=0.1*desy[2];
//                   desz[2]=0.1*desz[2];
//                 }
//                 if((uav2_waypoint-1)>0)
//                 {
//                   uav3_waypoint+=1;
//                 }
//                 if(uav3_waypoint>=ix[2].size())
//                 {
//                   uav3_waypoint-=1;
//                 }

//                 ROS_INFO("uav waypoint[%f %f %f ]", uav1_waypoint, uav2_waypoint,uav3_waypoint);
// ///////////////////////////////////
//                 // if()
//                 break;
//         case 1: cout << "brick pick up" << endl;
//                 break;
//         case 2: cout << "brick drop" << endl;
//                 break;
//         default: cout<<"Default ";
//                 for (int i = 0; i < 3; ++i)
//                 {
                  
//                   desx[i]=0;
//                   desy[i]=0;
//                   desz[i]=0;
//                 }



//       }
      // if(flagg==0)
      // {
      //   disst= sqrt(pow((10 -pz[0]),2)); 
      //   desz= (10 -pz[0])/disst;
      //   desx= 0;
      //   desy=0;
      //   despitch=1;    ////1= inertial frame     0= body frame
      //   desyaw=0;

      //   if(disst<1)
      //   {
      //     ROS_INFO("innnnnnnnnnnnnnnnnnn");
      //     desz=0;
      //     desyaw=-0.05;
      //   }
      //   if(front_boxx!=0 || front_boxy!=0)
      //   {
      //     ROS_INFO("boxxinnnnnnnnnnnnnnnnnnn");
      //     flagg2=1;
      //     flagg=1;
      //     desyaw=0.05;
      //   }

      //   msg1.linear.x = desx;
      //   msg1.linear.y = desy;
      //   msg1.linear.z = desz;
      //   msg1.angular.y = despitch;
      //   msg1.angular.z = desyaw;
      //   // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);
      //   desired_velocity1.publish(msg1);
      //   ros::spinOnce();
      //   loop_rate.sleep();
      // }
      // else if(flagg==2)
      // {
      //   ROS_INFO("navigation");
      //   disst= sqrt(pow((ix[0][i] -px[0]),2)+pow((iy[0][i] -py[0]),2)+pow((iz[0][i] -pz[0]),2));
      //   desx= (ix[0][i] -px[0])/disst; 
      //   desy= (iy[0][i] -py[0])/disst; 
      //   desz= (iz[0][i] -pz[0])/disst;
      //   despitch=1;

      //   if(disst<0.7)
      //   {
      //     desx=0.1*desx;
      //     desy=0.1*desy;
      //     desz=0.1*desz;
      //     i=i+1;
      //   }
      //   if(desz>=0.5)
      //   {
      //     desz=0.5;
      //   }
      //   else if(desz<=-0.5)
      //   {
      //     desz=-0.5;
      //   }
      //   if((i+1)%3==0)
      //   {
      //     gripper_off.call(srv);
      //   }
      //   if(i%3==0)
      //   {
      //     flagg=1;
      //     flagg1=0;
      //     flagg2=2;
      //   }
        

      //   msg1.linear.x = desx;
      //   msg1.linear.y = desy;
      //   msg1.linear.z = desz;
      //   msg1.angular.y = despitch;
      //   // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);
      //   desired_velocity1.publish(msg1);
      //   ros::spinOnce();
      //   loop_rate.sleep();
      // }
      // else if(flagg2==1)
      // {
      //   ROS_INFO("front");
      //   // Calculation of horizontal offset
      //   // ROS_INFO("bx:[%d] by:[%d] ",boxx,boxy);
      //   if(front_boxx!=0 || front_boxy!=0)
      //   {
      //     // ROS_INFO("bx:[%d] by:[%d] ",boxx,boxy);
      //       float y_offset = (image_w/2)-front_boxx;
      //       float y_deriv_error = y_offset-y_offset_prev; 
      //       y_offset_prev = y_offset;
      //       // Calculation of vertical offset

      //       float z_offset = (image_h/2)-front_boxy;
      //       float z_deriv_error = z_offset-z_offset_prev; 
      //       z_offset_prev = z_offset;
      //       //Calculation of distance or range error
      //       // float FOV = 1.3962634; // FOV of quad cam
      //       // float trgt_w = 0.47;  // width of target quad  
      //       // float fcl_lngth = image_w/(2*tan(FOV/2)); //calculation of camera focal length
      //       // float obj_dist = fcl_lngth*(boxw-trgt_w)/trgt_w;
      //       // float safe_dist = 1; //distance to be maintained with target
      //       // float rng_offset = -(safe_dist-obj_dist);
      //       // float rng_deriv_error = rng_offset-rng_offset_prev; 
      //       // rng_offset_prev = rng_offset;


      //       //PD Controller for yaw

      //       float Kp_y = 0.0005;
      //       float Kd_y = 0.0005;
      //       desyaw = Kp_y*y_offset+Kd_y*(y_deriv_error)/0.01;

      //       //PD Controller for z  
      //       float Kp_z = 0.0005;
      //       float Kd_z = 0.0005;
      //       desz = Kp_z*z_offset+Kd_z*(z_deriv_error)/0.01;

      //       // ROS_INFO("desyaw:[%f] yoffset:[%f] ", desyaw,y_offset);
      //       despitch=0;

      //       msg1.linear.z = 0;
      //       msg1.angular.y = despitch;
      //       msg1.angular.z = desyaw;

      //       // ROS_INFO("boxarea:[%d] ", boxarea);
      //       if((front_boxarea>10000 && front_boxarea<11000) || (flagg1==1))
      //       {
      //         msg1.linear.x = 0;
      //         flagg1=1;
      //         flagg=2;
      //         // float red_block_location[]
      //       }
      //       else
      //       {
      //         if(abs(y_offset)<40)
      //         {
      //           msg1.linear.x = 0.6;
      //         }
      //         else
      //         {
      //           msg1.linear.x = 0;
      //         }
      //       }
      //       if(down_boxx!=0 || down_boxy!=0)
      //         flagg2=2;
      //       // ROS_INFO("mx:[%f] my:[%f] mz:[%f]", msg2.x,msg2.y,msg2.z);
            
      //       desired_velocity1.publish(msg1);

      //       // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y);

      //       ros::spinOnce();
      //       loop_rate.sleep();
      //       // ROS_INFO("tt:[%f]",c_time);
      //       // ROS_INFO("begin:[%f]",gett_time);
      //   //swarm code end
      //   //publish desired x y z to px4 through keyboard_teleop
      //   }
      //   else
      //   {
      //       msg1.linear.y = 0;
      //       msg1.linear.x = 0;
      //       msg1.linear.z = 0;
      //       msg1.angular.z = 0;
      //       msg1.angular.y = 0;
        

      //       // ROS_INFO("mx:[%f] my:[%f] mz:[%f]", msg2.x,msg2.y,msg2.z);
            
      //       desired_velocity1.publish(msg1);

      //       // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);

      //       ros::spinOnce();
      //       loop_rate.sleep();
      //   }
      // }

      // else if(flagg2==2)
      // {
      //   ROS_INFO("down");
      //   if(down_boxx!=0 || down_boxy!=0)
      //   {
      //     // ROS_INFO("bx:[%d] by:[%d] ",boxx,boxy);
      //       float x_offset = (image_w/2)-down_boxx;
      //       float x_deriv_error = x_offset-x_offset_prev; 
      //       x_offset_prev = x_offset;
      //       // Calculation of vertical offset

      //       float y_offset = (image_h/2)-down_boxy;
      //       float y_deriv_error = y_offset-y_offset_prev; 
      //       y_offset_prev = y_offset;
      //       ROS_INFO("x:[%f] y:[%f] ",x_offset,y_offset);
      //       //Calculation of distance or range error
      //       // float FOV = 1.3962634; // FOV of quad cam
      //       // float trgt_w = 0.47;  // width of target quad  
      //       // float fcl_lngth = image_w/(2*tan(FOV/2)); //calculation of camera focal length
      //       // float obj_dist = fcl_lngth*(boxw-trgt_w)/trgt_w;
      //       // float safe_dist = 1; //distance to be maintained with target
      //       // float rng_offset = -(safe_dist-obj_dist);
      //       // float rng_deriv_error = rng_offset-rng_offset_prev; 
      //       // rng_offset_prev = rng_offset;


      //       //PD Controller for x

      //       float Kp_x = 0.004;
      //       float Kd_x = 0.004;
      //       desx = Kp_x*x_offset+Kd_x*(x_deriv_error)/0.01;

      //       //PD Controller for y  
      //       float Kp_y = 0.004;
      //       float Kd_y = 0.004;
      //       desy = Kp_y*y_offset+Kd_y*(y_deriv_error)/0.01;

      //       // ROS_INFO("xoffset:[%f] yoffset:[%f] ", x_offset,y_offset);
      //       despitch=1;

      //       msg1.linear.x = desy;
      //       msg1.linear.y = desx;
      //       msg1.angular.z = 0;
      //       msg1.angular.y = despitch;

            
      //       if(abs(x_offset)<30 && abs(y_offset)<30)
      //       {
      //         msg1.linear.z = -0.2;
      //       }
      //       else
      //       {
      //         msg1.linear.z = 0;
      //       }
            
      //       // if(fgg==1)
      //       // {
      //         // gripper_on.call(srv);
      //         // fgg=0;
      //       // }
      //       gripper_on.call(srv);
      //       if((down_boxarea>8000) || (flagg1==1) || pz[0]<=0.55 || gripper_state==1)
      //       {
      //         msg1.linear.z = 0;
      //         flagg1=1;
      //         flagg=2;

      //         // float red_block_location[]
      //       }
      //       // if(gripper_state==1)
      //       // {

      //       //   flagg1=1;
      //       //   flagg=2;
      //       // }
              
      //         // float red_block_location[]
            
            
      //       ROS_INFO("z:[%f] ", msg1.linear.z);
      //       // ROS_INFO("mx:[%f] my:[%f] mz:[%f]", msg2.x,msg2.y,msg2.z);
            
      //       desired_velocity1.publish(msg1);

      //       // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y);

      //       ros::spinOnce();
      //       loop_rate.sleep();
      //       // ROS_INFO("tt:[%f]",c_time);
      //       // ROS_INFO("begin:[%f]",gett_time);
      //   //swarm code end
      //   //publish desired x y z to px4 through keyboard_teleop
      //   }
      //   else
      //   {
      //       msg1.linear.y = 0;
      //       msg1.linear.x = 0;
      //       msg1.linear.z = 0;
      //       msg1.angular.y = 1;
        

            // ROS_INFO("mx:[%f] my:[%f] mz:[%f]", msg2.x,msg2.y,msg2.z);
            
            msg1.linear.y = desx[0];
            msg1.linear.x = desy[0];
            msg1.linear.z = desz[0];

            msg2.linear.y = desx[1];
            msg2.linear.x = desy[1];
            msg2.linear.z = desz[1];

            msg3.linear.y = desx[2];
            msg3.linear.x = desy[2];
            msg3.linear.z = desz[2];
            desired_velocity1.publish(msg1);
            desired_velocity2.publish(msg2);
            desired_velocity3.publish(msg3);

            // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);

            ros::spinOnce();
            loop_rate.sleep();
      //   }
      // }
  }

  return 0;



} 
