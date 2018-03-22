#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <robosub/DiveAction.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <string>

#include "helpers/Bool.h"

constexpr double kTolerance = 1.0;

// See
// http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer(GoalCallbackMethod)

typedef actionlib::SimpleActionServer<robosub::DiveAction> Server;

class DiveAction {
 public:
  DiveAction(std::string name)
      : depth_(0.0), server_(nh_, name, false), action_name_(name) {
    // Register callback for when a new goal is received
    server_.registerGoalCallback(boost::bind(&DiveAction::goalCallback, this));

    // Register callback for when the current goal is cancelled
    server_.registerPreemptCallback(
        boost::bind(&DiveAction::preemptCallback, this));

    depth_sub_ = nh_.subscribe("/pose", 1, &DiveAction::poseCallback, this);

    // Node namespace makes this $(arg ns)/setpoint instead
    depth_pub_ = nh_.advertise<std_msgs::Float64>("setpoint", 1);
    enable_pub_ = nh_.advertise<std_msgs::Bool>("pid_enable", 1);

    enable_pub_.publish(Bool(false));

    server_.start();
    ROS_INFO("%s: Started", action_name_.c_str());
  }

  ~DiveAction() {}

  void goalCallback() {
    depth_ = server_.acceptNewGoal()->depth;
    ROS_INFO("%s: Received new depth goal %f", action_name_.c_str(), depth_);
    enable_pub_.publish(Bool(true));
  }

  void preemptCallback() {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    enable_pub_.publish(Bool(false));
    server_.setPreempted();
  }

  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    double depth = msg->position.z;
    robosub::DiveFeedback feedback;
    feedback.depth = depth;
    server_.publishFeedback(feedback);

    if (::fabs(depth_ - depth) < kTolerance) {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      robosub::DiveResult result;
      result.depth = depth;
      enable_pub_.publish(Bool(false));
      server_.setSucceeded(result);
    } else {
      // Do PID control here by sending a setpoint to the PID node
      std_msgs::Float64 setpoint;
      setpoint.data = depth_;
      depth_pub_.publish(setpoint);
    }
  }

 private:
  double depth_;
  ros::NodeHandle nh_;
  Server server_;
  std::string action_name_;
  ros::Subscriber depth_sub_;
  ros::Publisher depth_pub_;
  ros::Publisher enable_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dive_action");

  DiveAction action(ros::this_node::getName());

  ros::spin();
  return 0;
}
