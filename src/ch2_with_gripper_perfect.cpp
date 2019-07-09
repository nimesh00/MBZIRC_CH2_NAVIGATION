//C++ program for visual servoing Method 2
//Abhishek Kashyap, Narayani, Rahul
#include "ros/ros.h"
#include "interface/test1.h"
#include "interface/test2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include<stdio.h>
#include<iostream>
#include<algorithm>
#include<vector>
#include<array>
#include<functional> 
#include<cmath> // use of math.h is permitted but not advisable
#include<Eigen/Dense> // #include<Eigen/Eigenvalues>
#include<Eigen/Geometry>
#include <list> 
#include <iterator> 
#include <stdlib.h>
using namespace std; 
using namespace Eigen;

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


void seq(char sequence[],float ix1[], float iy1[],float iz1[],float ix2[], float iy2[],float iz2[],float ix3[], float iy3[],float iz3[],float brick_pos[],
  float brick_specs[], int siz)
  {
    int m=0,n=0,o=0;
    float brick_initial_posx = brick_pos[0];
    float brick_initial_posy = brick_pos[1];
    float brick_length = brick_specs[0];
    float brick_width = brick_specs[0];
    int height1=5, height2 = 4, height3=6;
    
    for(int l=0;l<siz;l++)
    {
      if(sequence[l]=='R')
      {
        ix1[m]=5;
        iy1[m]=-4;
        iz1[m]=5;
        m+=1;
        ix1[m]=brick_initial_posx;
        iy1[m]=brick_initial_posy;
        iz1[m]=5;
        m+=1;
        ix1[m]=brick_initial_posx;
        iy1[m]=brick_initial_posy;
        iz1[m]=0.5;
        m+=1;
        ix1[m]=5;
        iy1[m]=-4;
        iz1[m]=5;
        m+=1;
      }
      else if(sequence[l]=='G')
      {
        ix2[n]=brick_initial_posx;
        iy2[n]=brick_initial_posy;
        iz2[n]=0.1;
        n+=1;
        ix2[n]=16;
        iy2[n]=-16;
        iz2[n]=height2;
        n+=1;
      }
      else if(sequence[l]=='B')
      {
        ix3[o]=brick_initial_posx;
        iy3[o]=brick_initial_posy;
        iz3[o]=0.1;
        o+=1;
        ix3[o]=16;
        iy3[o]=-10;
        iz3[o]=height3;
        o+=1;
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
  px[1]= msg->pose.pose.position.x;
  py[1]= msg->pose.pose.position.y;
  pz[1]= msg->pose.pose.position.z;
}
void handle_poses3(const nav_msgs::Odometry::ConstPtr& msg)
{
  px[2]= msg->pose.pose.position.x;
  py[2]= msg->pose.pose.position.y;
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
void simple_image_in_front(const interface::test1::ConstPtr& msg)
{
  front_boxx= msg->front_cam_boxx;
  front_boxy= msg->front_cam_boxy;
  front_boxarea = msg->front_cam_boxarea;
}
void simple_image_in_down(const interface::test2::ConstPtr& msg)
{
  down_boxx= msg->down_cam_boxx;
  down_boxy= msg->down_cam_boxy;
  down_boxarea = msg->down_cam_boxarea;
}
void grab(const std_msgs::Bool::ConstPtr& msg)
{
  if(msg->data==true)
  {
    gripper_state=1;
  }
  else
  {
    gripper_state=0;
  }
}


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "forward_vis");
    ros::NodeHandle n;
    ros::Subscriber simple_image1 = n.subscribe("/front_cam", 10, simple_image_in_front);
    ros::Subscriber simple_image2 = n.subscribe("/down_cam", 10, simple_image_in_down);
    ros::Subscriber position1 = n.subscribe("/mavros/global_position/local", 1000, handle_poses1);
    ros::Subscriber velocity1 = n.subscribe("/mavros/local_position/velocity", 1000, handle_vel1);
    ros::Subscriber position2 = n.subscribe("/uav2/mavros/global_position/local", 10, handle_poses2);
    ros::Subscriber velocity2 = n.subscribe("/uav2/mavros/local_position/velocity", 10, handle_vel2);
    ros::Subscriber position3 = n.subscribe("/uav3/mavros/global_position/local", 10, handle_poses3);
    ros::Subscriber velocity3 = n.subscribe("/uav3/mavros/local_position/velocity", 10, handle_vel3);
    ros::Subscriber gripper = n.subscribe("/vacuum_gripper/grasping", 10, grab);
    ros::ServiceClient gripper_on = n.serviceClient<std_srvs::Empty>("/vacuum_gripper/on");
    ros::ServiceClient gripper_off = n.serviceClient<std_srvs::Empty>("/vacuum_gripper/off");
    // ros::Subscriber orientation1 = n.subscribe("/uav1/mavros/imu/data", 1000, handle_orientation1);
    ros::Publisher desired_velocity1= n.advertise<geometry_msgs::Twist>("chatter1", 100);
    geometry_msgs::Twist msg1;

    std_srvs::Empty srv;
  // gripper_on.call(srv);


    ros::Rate loop_rate(100);
    //Quad Quaternion parameters
    float desx;
    float desy;
    float desz;
    float desroll;
    float despitch;
    float desyaw;
    float disst;
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
    int uav1_siz=0,uav2_siz=0,uav3_siz=0;
    int fgg=1;
    
    int i=0, j=0, k=0;
    char sequence[siz]={'R','G','R','B','G','R','B'};

    for(int l=0;l<siz;l++)
    {
      if(sequence[l]=='R')
      {
        uav1_siz = uav1_siz + 1;
      }
      else if(sequence[l]=='G')
      {
        uav2_siz = uav2_siz +1;
      }
      else if(sequence[l]=='B')
      {
        uav3_siz = uav3_siz +1;
      }
    } 

    float ix1[4*uav1_siz]={}, ix2[4*uav2_siz]={}, ix3[4*uav3_siz]={};
    float iy1[4*uav1_siz]={}, iy2[4*uav2_siz]={}, iy3[4*uav3_siz]={};
    float iz1[4*uav1_siz]={},iz2[4*uav2_siz]={},iz3[4*uav3_siz]={};


    float brick_pos[2]={brick_initial_posx,brick_initial_posy};
    float brick_specs[2]={brick_length,brick_width};

    seq(sequence,ix1, iy1,iz1,ix2, iy2,iz2,ix3, iy3,iz3,brick_pos,brick_specs,siz);

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
    //  {
    //    ROS_INFO("ix1:[%f] iy1:[%f] iz1:[%f]", ix1[i], iy1[i], iz1[i]);
    //     // std::cout << i << ' ';
    //  }
    //  for (int i=0;i<2*uav2_siz;i++)
    //  {
    //    ROS_INFO("ix2:[%f] iy2:[%f] iz2:[%f]", ix2[i], iy2[i], iz2[i]);
    //     // std::cout << i << ' ';
    //  }
    while (ros::ok())
    {
      //swarm code start
      // c_time = t_sec+0.000000001*t_nsec;
      // if(c_time!=0)
      // {
      //   next_time=c_time;
      // }
      // // ROS_INFO("next_time:[%f] gett_time:[%f] t_diff:[%f]", next_time, gett_time,t_diff);
      // t_diff= next_time - gett_time;

      // if(c_time!=0)
      // {
      //   gett_time=c_time;
      // }

      if(flagg==0)
      {
        disst= sqrt(pow((10 -pz[0]),2)); 
        desz= (10 -pz[0])/disst;
        desx= 0;
        desy=0;
        despitch=1;    ////1= inertial frame     0= body frame
        desyaw=0;

        if(disst<1)
        {
          ROS_INFO("innnnnnnnnnnnnnnnnnn");
          desz=0;
          desyaw=-0.05;
        }
        if(front_boxx!=0 || front_boxy!=0)
        {
          ROS_INFO("boxxinnnnnnnnnnnnnnnnnnn");
          flagg2=1;
          flagg=1;
          desyaw=0.05;
        }

        msg1.linear.x = desx;
        msg1.linear.y = desy;
        msg1.linear.z = desz;
        msg1.angular.y = despitch;
        msg1.angular.z = desyaw;
        // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);
        desired_velocity1.publish(msg1);
        ros::spinOnce();
        loop_rate.sleep();
      }
      else if(flagg==2)
      {
        ROS_INFO("navigation");
        disst= sqrt(pow((ix1[i] -px[0]),2)+pow((iy1[i] -py[0]),2)+pow((iz1[i] -pz[0]),2));
        desx= (ix1[i] -px[0])/disst; 
        desy= (iy1[i] -py[0])/disst; 
        desz= (iz1[i] -pz[0])/disst;
        despitch=1;

        if(disst<0.7)
        {
          desx=0.1*desx;
          desy=0.1*desy;
          desz=0.1*desz;
          i=i+1;
        }
        if(desz>=0.5)
        {
          desz=0.5;
        }
        else if(desz<=-0.5)
        {
          desz=-0.5;
        }
        if((i+1)%4==0)
        {
          gripper_off.call(srv);
        }
        if(i%4==0)
        {
          flagg=1;
          flagg1=0;
          flagg2=2;
        }
        

        msg1.linear.x = desx;
        msg1.linear.y = desy;
        msg1.linear.z = desz;
        msg1.angular.y = despitch;
        // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);
        desired_velocity1.publish(msg1);
        ros::spinOnce();
        loop_rate.sleep();
      }
      else if(flagg2==1)
      {
        ROS_INFO("front");
        // Calculation of horizontal offset
        // ROS_INFO("bx:[%d] by:[%d] ",boxx,boxy);
        if(front_boxx!=0 || front_boxy!=0)
        {
          // ROS_INFO("bx:[%d] by:[%d] ",boxx,boxy);
            float y_offset = (image_w/2)-front_boxx;
            float y_deriv_error = y_offset-y_offset_prev; 
            y_offset_prev = y_offset;
            // Calculation of vertical offset

            float z_offset = (image_h/2)-front_boxy;
            float z_deriv_error = z_offset-z_offset_prev; 
            z_offset_prev = z_offset;
            //Calculation of distance or range error
            // float FOV = 1.3962634; // FOV of quad cam
            // float trgt_w = 0.47;  // width of target quad  
            // float fcl_lngth = image_w/(2*tan(FOV/2)); //calculation of camera focal length
            // float obj_dist = fcl_lngth*(boxw-trgt_w)/trgt_w;
            // float safe_dist = 1; //distance to be maintained with target
            // float rng_offset = -(safe_dist-obj_dist);
            // float rng_deriv_error = rng_offset-rng_offset_prev; 
            // rng_offset_prev = rng_offset;


            //PD Controller for yaw

            float Kp_y = 0.0005;
            float Kd_y = 0.0005;
            desyaw = Kp_y*y_offset+Kd_y*(y_deriv_error)/0.01;

            //PD Controller for z  
            float Kp_z = 0.0005;
            float Kd_z = 0.0005;
            desz = Kp_z*z_offset+Kd_z*(z_deriv_error)/0.01;

            // ROS_INFO("desyaw:[%f] yoffset:[%f] ", desyaw,y_offset);
            despitch=0;

            msg1.linear.z = 0;
            msg1.angular.y = despitch;
            msg1.angular.z = desyaw;

            // ROS_INFO("boxarea:[%d] ", boxarea);
            if((front_boxarea>10000 && front_boxarea<11000) || (flagg1==1))
            {
              msg1.linear.x = 0;
              flagg1=1;
              flagg=2;
              // float red_block_location[]
            }
            else
            {
              if(abs(y_offset)<40)
              {
                msg1.linear.x = 0.6;
              }
              else
              {
                msg1.linear.x = 0;
              }
            }
            if(down_boxx!=0 || down_boxy!=0)
              flagg2=2;
            // ROS_INFO("mx:[%f] my:[%f] mz:[%f]", msg2.x,msg2.y,msg2.z);
            
            desired_velocity1.publish(msg1);

            // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y);

            ros::spinOnce();
            loop_rate.sleep();
            // ROS_INFO("tt:[%f]",c_time);
            // ROS_INFO("begin:[%f]",gett_time);
        //swarm code end
        //publish desired x y z to px4 through keyboard_teleop
        }
        else
        {
            msg1.linear.y = 0;
            msg1.linear.x = 0;
            msg1.linear.z = 0;
            msg1.angular.z = 0;
            msg1.angular.y = 0;
        

            // ROS_INFO("mx:[%f] my:[%f] mz:[%f]", msg2.x,msg2.y,msg2.z);
            
            desired_velocity1.publish(msg1);

            // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);

            ros::spinOnce();
            loop_rate.sleep();
        }
      }

      else if(flagg2==2)
      {
        ROS_INFO("down");
        if(down_boxx!=0 || down_boxy!=0)
        {
          // ROS_INFO("bx:[%d] by:[%d] ",boxx,boxy);
            float x_offset = (image_w/2)-down_boxx;
            float x_deriv_error = x_offset-x_offset_prev; 
            x_offset_prev = x_offset;
            // Calculation of vertical offset

            float y_offset = (image_h/2)-down_boxy;
            float y_deriv_error = y_offset-y_offset_prev; 
            y_offset_prev = y_offset;
            ROS_INFO("x:[%f] y:[%f] ",x_offset,y_offset);
            //Calculation of distance or range error
            // float FOV = 1.3962634; // FOV of quad cam
            // float trgt_w = 0.47;  // width of target quad  
            // float fcl_lngth = image_w/(2*tan(FOV/2)); //calculation of camera focal length
            // float obj_dist = fcl_lngth*(boxw-trgt_w)/trgt_w;
            // float safe_dist = 1; //distance to be maintained with target
            // float rng_offset = -(safe_dist-obj_dist);
            // float rng_deriv_error = rng_offset-rng_offset_prev; 
            // rng_offset_prev = rng_offset;


            //PD Controller for x

            float Kp_x = 0.004;
            float Kd_x = 0.004;
            desx = Kp_x*x_offset+Kd_x*(x_deriv_error)/0.01;

            //PD Controller for y  
            float Kp_y = 0.004;
            float Kd_y = 0.004;
            desy = Kp_y*y_offset+Kd_y*(y_deriv_error)/0.01;

            // ROS_INFO("xoffset:[%f] yoffset:[%f] ", x_offset,y_offset);
            despitch=1;

            msg1.linear.x = desy;
            msg1.linear.y = desx;
            msg1.angular.z = 0;
            msg1.angular.y = despitch;

            
            if(abs(x_offset)<30 && abs(y_offset)<30)
            {
              msg1.linear.z = -0.2;
            }
            else
            {
              msg1.linear.z = 0;
            }
            
            // if(fgg==1)
            // {
              // gripper_on.call(srv);
              // fgg=0;
            // }
            gripper_on.call(srv);
            if((down_boxarea>8000) || (flagg1==1) || pz[0]<=0.55 || gripper_state==1)
            {
              msg1.linear.z = 0;
              flagg1=1;
              flagg=2;

              // float red_block_location[]
            }
            // if(gripper_state==1)
            // {

            //   flagg1=1;
            //   flagg=2;
            // }
              
              // float red_block_location[]
            
            
            ROS_INFO("z:[%f] ", msg1.linear.z);
            // ROS_INFO("mx:[%f] my:[%f] mz:[%f]", msg2.x,msg2.y,msg2.z);
            
            desired_velocity1.publish(msg1);

            // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y);

            ros::spinOnce();
            loop_rate.sleep();
            // ROS_INFO("tt:[%f]",c_time);
            // ROS_INFO("begin:[%f]",gett_time);
        //swarm code end
        //publish desired x y z to px4 through keyboard_teleop
        }
        else
        {
            msg1.linear.y = 0;
            msg1.linear.x = 0;
            msg1.linear.z = 0;
            msg1.angular.y = 1;
        

            // ROS_INFO("mx:[%f] my:[%f] mz:[%f]", msg2.x,msg2.y,msg2.z);
            
            desired_velocity1.publish(msg1);

            // ROS_INFO("m1x:[%f] m1y:[%f] m1z:[%f]", msg1.linear.x,msg1.linear.y,msg1.linear.z);

            ros::spinOnce();
            loop_rate.sleep();
        }
      }
  }

  return 0;



} 
