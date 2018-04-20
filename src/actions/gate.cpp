#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <robosub/GateAction.h>

typedef actionlib::SimpleActionServer<robosub::GateAction> Server;

class Gate {
 public:
  Gate(std::string name) : server_(nh_, name, false), action_name_(name) {
    // Register callback for when a new goal is received
    server_.registerGoalCallback(boost::bind(&Gate::goalCallback, this));

    // Register callback for when the current goal is cancelled
    server_.registerPreemptCallback(boost::bind(&Gate::preemptCallback, this));

    // Node namespace makes this $(arg ns)/setpoint instead
    motion_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    server_.start();
    ROS_INFO("%s: Started", action_name_.c_str());
  }

  ~Gate() {}

  void goalCallback() {
    red_ = server_.acceptNewGoal()->red;
    ROS_INFO("%s: Received new die pips goal %s", action_name_.c_str(),
             red_ ? "red" : "black");
  }

  void preemptCallback() {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    server_.setPreempted();
  }

 private:
  bool red_;
  ros::NodeHandle nh_;
  Server server_;
  std::string action_name_;
  ros::Publisher motion_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gate_action");

  Gate action(ros::this_node::getName());

  ros::spin();
  return 0;
}
