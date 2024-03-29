/* COPYRIGHT Pedro P. V. Tecchio 2019
 * Adapted from http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <iostream>

class TurtleControl {
 private:
  ros::NodeHandle n_;
  ros::Subscriber pose_sub_;
  ros::Publisher cmd_vel_pub_;

  turtlesim::Pose pose_;
  turtlesim::Pose goal_pose_;

  float tolerance_;
  const float angular_constant_ = 6.0;
  const float linear_constant_ = 1.5;

  void poseCallback(const turtlesim::Pose::ConstPtr& msg);
  float computeEuclideanDistance(void);
  float computeSteeringAngle(void);
  float computeLinearVelocity(void);
  float computeAngularVelocity(void);

 public:
  TurtleControl(int argc, char **argv);
  ~TurtleControl();
  void move2Goal(void);
};

TurtleControl::TurtleControl(int argc, char **argv) {
  pose_sub_ = n_.subscribe<turtlesim::Pose>("turtle1/pose", 1, &
              TurtleControl::poseCallback, this);

  cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

  ROS_INFO("Hey");

  if (n_.hasParam("x"))
    n_.getParam("x",goal_pose_.x);
  else
    ROS_INFO("No param x");
}

TurtleControl::~TurtleControl() {
  geometry_msgs::Twist twist;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  cmd_vel_pub_.publish(twist);

  ROS_INFO("Bye");
  ros::shutdown();
}

void TurtleControl::poseCallback(const turtlesim::Pose::ConstPtr& msg) {
  pose_.x = msg->x;
  pose_.y = msg->y;
  pose_.theta = msg->theta;
}

float TurtleControl::computeEuclideanDistance(void) {
  return sqrt(pow((pose_.x - goal_pose_.x), 2) +
              pow((pose_.y - goal_pose_.y), 2));
}

float TurtleControl::computeSteeringAngle(void) {
  return atan2((goal_pose_.y - pose_.y), (goal_pose_.x - pose_.x));
}

float TurtleControl::computeLinearVelocity(void) {
  return linear_constant_ * computeEuclideanDistance();
}

float TurtleControl::computeAngularVelocity(void) {
  return angular_constant_ * (computeSteeringAngle() - pose_.theta);
}

void TurtleControl::move2Goal(void) {
  ros::Rate rate(60);

  geometry_msgs::Twist twist;

  std::cout << "X: ";
  std::cin >> goal_pose_.x;
  std::cout << "Y: ";
  std::cin >> goal_pose_.y;
  std::cout << "Tolerance: ";
  std::cin >> tolerance_;

  while (ros::ok()) {
    twist.linear.x = computeLinearVelocity();
    twist.angular.z = computeAngularVelocity();

    cmd_vel_pub_.publish(twist);

    if (computeEuclideanDistance() <= tolerance_)
      break;

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle_control");

  TurtleControl turtle_control(argc, argv);

  turtle_control.move2Goal();

  return 0;
}
