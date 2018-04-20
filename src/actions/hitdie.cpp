#include <actionlib/server/simple_action_server.h>
#include <robosub/HitDieAction.h>
#include <geometry_msgs/Twist.h>


typedef actionlib::SimpleActionServer<robosub::HitDieAction> Server;

class HitDie {
 public:
  HitDie(std::string name)
      : depth_(0.0), server_(nh_, name, false), action_name_(name) {
    // Register callback for when a new goal is received
    server_.registerGoalCallback(boost::bind(&HitDie::goalCallback, this));

    // Register callback for when the current goal is cancelled
    server_.registerPreemptCallback(
        boost::bind(&HitDie::preemptCallback, this));

    // Node namespace makes this $(arg ns)/setpoint instead
    motion_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    server_.start();
    ROS_INFO("%s: Started", action_name_.c_str());
  }

  ~HitDie() {}

  void goalCallback() {
    pips_ = server_.acceptNewGoal()->pips;
    ROS_INFO("%s: Received new die pips goal %d", action_name_.c_str(), pips_);
  }

  void preemptCallback() {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    server_.setPreempted();
  }

 private:
  uint8_t pips_;
  ros::NodeHandle nh_;
  Server server_;
  std::string action_name_;
  ros::Publisher motion_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hitdie_action");

  HitDie action(ros::this_node::getName());

  ros::spin();
  return 0;
}
