#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <math.h>

#define DEBUG 0
#define vel_control 0
#define freq 10
#define dt 1/freq

float targetx = 0.0;
float targety = 0.0;

float x_pos = 0.0;
float y_pos = 0.0;
float robot_angle = 0.0;

float Kv = 1.0;
float Kp = 0.8;

bool goal_margin = false;

void odom_callback(const nav_msgs::Odometry::ConstPtr& data)
{
   x_pos = data->pose.pose.position.x;
   y_pos = data->pose.pose.position.y;
   robot_angle = M_PI  * data->pose.pose.orientation.z;
}  

void goal_callback(const geometry_msgs::Twist::ConstPtr& data)
{
   ROS_INFO("New Goal Command");

   targetx = data->linear.x;
   targety = data->linear.y;
   goal_margin = false;
}

int main(int argc, char** argv)
{
   ROS_INFO("Initializing Goal Controller...");

   ros::init(argc, argv, "Goal_node");
   ros::NodeHandle nh;
   #if vel_control
   ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/vel_controller/target_vel", 1, true);
   #else
   ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1, true);
   #endif
   ros::Publisher reset_pub = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1 , true);
   ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);
   ros::Subscriber goal_sub = nh.subscribe("/goal_controller/target_goal", 10, goal_callback);
   ros::Rate loop_rate = freq;
  
   float goal_range;   
   ros::param::get("~kv", Kv); 
   ros::param::get("~kp", Kp); 
   ros::param::get("~goal_range", goal_range);

   geometry_msgs::Twist new_vel;

   std_msgs::Empty reset_msg;
   reset_pub.publish(reset_msg);

   ROS_INFO("Goal Controller Initialized!");
   ROS_INFO("Starting main goal controller loop...");

   while(ros::ok()) {
      ros::spinOnce();
      float distance = sqrt(pow((targetx - x_pos), 2) + pow((targety - y_pos), 2));
      float angle = atan2((targety - y_pos), (targetx - x_pos));
      
      if(!goal_margin) {
         new_vel.linear.x = Kv * distance;
         new_vel.angular.z = Kp * (angle - robot_angle);
      }else {
         new_vel.linear.x = 0.0;
         new_vel.angular.z = 0.0;
      }
      vel_pub.publish(new_vel);

      if(distance < goal_range) {
         goal_margin = true;
      }
      
      #if DEBUG
      ROS_INFO_STREAM(angle);
      #endif

      loop_rate.sleep();
   }
}
