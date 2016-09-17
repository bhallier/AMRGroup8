#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "geometry_msgs/Twist.h"

bool bump_action = false;
int bumper;
void bumpCallback(const kobuki_msgs::BumperEvent::ConstPtr& data)
{
   ROS_INFO("Bump!");
   bumper = data->bumper;
   bump_action = true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "turtlebot_wander");
   ros::NodeHandle nh;
   ros::Publisher cmd = nh.advertise<geometry_msgs::Twist>("/vel_controller/target_vel", 10);

   ros::Subscriber bump = nh.subscribe("/mobile_base/events/bumper", 10, bumpCallback);
   ros::Rate loop_rate(10);
   int t_count = 0;
   int event = 0;
   geometry_msgs::Twist new_msg;

   new_msg.linear.x = 0.2;
   new_msg.linear.y = 0.0;
   new_msg.linear.z = 0.0;

   new_msg.angular.x = 0.0;
   new_msg.angular.y = 0.0;
   new_msg.angular.z = 0.0;

   while(ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();

      if(!bump_action) {
          cmd.publish(new_msg);

      }else {
          t_count++;
          if(t_count < 10) {
	     new_msg.linear.x = -0.1;
             ROS_INFO("Backing Up...");

          }else if((t_count >= 10) && (t_count < 15)) {
             if(bumper == 0) {
	        new_msg.linear.x = 0.0;
                new_msg.angular.z = -1.0;
                ROS_INFO("Left Bumper Hit");
             }else if(bumper == 1) {
                new_msg.linear.x = 0.0;
                new_msg.angular.z = 4.0;
                ROS_INFO("Center Bumper Hit");
             }else {
                new_msg.linear.x = 0.0;
                new_msg.angular.z = 1.0;
                ROS_INFO("Right Bumper Hit");
             }
          }else {
             new_msg.linear.x = 0.2;
             new_msg.angular.z = 0.0;
             t_count = 0;
             bump_action = false;
             ROS_INFO("Normal Behavior");
          }
      }
      cmd.publish(new_msg);
   }
}
