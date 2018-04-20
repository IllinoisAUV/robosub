#include <actionlib/server/simple_action_server.h>
#include <robosub/PingerAction.h>
#include <geometry_msgs/Twist.h>


typedef actionlib::SimpleActionServer<robosub::PingerAction> Server;

class Pinger {
 public:
  Pinger(std::string name)
      : server_(nh_, name, false), action_name_(name) {
    // Register callback for when a new goal is received
    server_.registerGoalCallback(boost::bind(&Pinger::goalCallback, this));

    // Register callback for when the current goal is cancelled
    server_.registerPreemptCallback(
        boost::bind(&Pinger::preemptCallback, this));

    // Node namespace makes this $(arg ns)/setpoint instead
    motion_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    server_.start();
    ROS_INFO("%s: Started", action_name_.c_str());
  }

  ~Pinger() {}

  void goalCallback() {
    frequency_ = server_.acceptNewGoal()->frequency;
    ROS_INFO("%s: Received new die pips goal %d", action_name_.c_str(), pips_);
  }

  void preemptCallback() {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    server_.setPreempted();
  }

 private:
  uint32_t frequency_;
  ros::NodeHandle nh_;
  Server server_;
  std::string action_name_;
  ros::Publisher motion_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "hitdie_action");

  Pinger action(ros::this_node::getName());

  ros::spin();
  return 0;
}
