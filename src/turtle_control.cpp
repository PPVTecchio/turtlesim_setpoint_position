#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>


class TurtleControl
{
private:
  ros::NodeHandle n_;
  ros::Subscriber pose_sub_;
  ros::Publisher cmd_vel_pub_;
  void poseCallback(const turtlesim::Pose::ConstPtr& msg);

public:
  TurtleControl(/* args */);
  ~TurtleControl();
};

TurtleControl::TurtleControl(/* args */)
{
  pose_sub_ = n_.subscribe<turtlesim::Pose>("turtle1/pose", 1000, &TurtleControl::poseCallback,this);

  cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

}

TurtleControl::~TurtleControl()
{
}


void TurtleControl::poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
  ROS_INFO("I heard x: %f x: %f y: %f theta: %f lvel: %f avel: %f ",msg->x,msg->y,msg->theta,msg->linear_velocity,msg->angular_velocity);

  geometry_msgs::Twist twist;
  twist.angular.z = msg->angular_velocity * 1.5;
  twist.linear.x = msg->linear_velocity * 1.5;
  cmd_vel_pub_.publish(twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_control");

  TurtleControl turtle_control;



  ros::spin();

  return 0;
}