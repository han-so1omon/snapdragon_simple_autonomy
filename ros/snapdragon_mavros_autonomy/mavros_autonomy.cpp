#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <limits.h>
#include <unistd.h>

#include "mavros_autonomy.h"

ros::Publisher _local_pos_pub;
mavros_msgs::State _curr_state;
TimeoutMutex _pose_mutex;

void vislam_pose_cb
    (const geometry_msgs::PoseStamped::ConstPtr& msg);
void okvis_pose_cb
    (const geometry_msgs::PoseStamped::ConstPtr& msg);

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
  _curr_state = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "snapdragon_mavros_autonomy_node");
  ros::console::set_logger_level(
      ROSCONSOLE_DEFAULT_NAME,
      ros::console::levels::Debug);
  ros::NodeHandle nh;

  // Set _pose_mutex to timeout every second unless
  // mutex holder causes timeout to be reset.
  // Reset occurs in the try_lock function
  // if the caller holds the mutex.
  _pose_mutex.set_timeout_duration(nh,1); 

  /*
  ros::Subscriber state_sub = nh.subscribe
            <mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
  */
  ros::Subscriber vislam_pose_sub = nh.subscribe
            <geometry_msgs::PoseStamped>
            ("/vislam/pose",10,vislam_pose_cb);
  ros::Subscriber okvis_pose_sub = nh.subscribe
            <geometry_msgs::PoseStamped>
            ("/okvis/pose",10,vislam_pose_cb);
  _local_pos_pub = nh.advertise
            <geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10);

  ros::Rate rate(30);

  // wait for FCU connection
  /*
  while(ros::ok() && _curr_state.connected) {
    ROS_INFO("curr state connected %d",
             _curr_state.connected);
    ros::spinOnce();
    rate.sleep();
  }

  */

  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

void vislam_pose_cb
    (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (_pose_mutex.try_lock(VISLAM)) {
    _local_pos_pub.publish(msg);
  }
}

void okvis_pose_cb
    (const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (_pose_mutex.try_lock(OKVIS)) {
    _local_pos_pub.publish(msg);
  }
}

// -----------------------------------------------------------

TimeoutMutex::TimeoutMutex() {
  _holder = NONE;
  _duration_set = false;
}

TimeoutMutex::TimeoutMutex(ros::NodeHandle &nh, int timeout) {
  _holder = NONE;
  _timeout = nh.createTimer(
      ros::Duration(timeout),
      &TimeoutMutex::timeout_cb,
      this);
  _duration_set = true;
}

TimeoutMutex::~TimeoutMutex() {
}

void TimeoutMutex::set_timeout_duration(ros::NodeHandle &nh, int timeout) {
  _timeout = nh.createTimer(
      ros::Duration(timeout),
      &TimeoutMutex::timeout_cb,
      this);
  /*
  _timeout = nh.createTimer(
    ros::Duration(timeout),
    boost::bind(&TimeoutMutex::timeout_cb, this, _1)
  );
  */

  _duration_set = true;
}

int TimeoutMutex::holder_priority(int hold_requester) {
  switch(hold_requester) {
    case NONE:
      return NONE_PRIORITY;
    case VISLAM:
      return VISLAM_PRIORITY;
    case OKVIS:
      return OKVIS_PRIORITY;
  }
}

int TimeoutMutex::compare_holder(int h1, int h2) {
  return holder_priority(h1) - holder_priority(h2);
}

void TimeoutMutex::timeout_cb(const ros::TimerEvent &te) {
  if (_holder != NONE)
    unlock();
}

void TimeoutMutex::reset_timeout() {
  _timeout.stop(); // resets timer
  _timeout.start();
}

bool TimeoutMutex::try_lock(int hold_requester) {
  int holder_res;
  bool ret = true;

  if ((holder_res = compare_holder(_holder, hold_requester))
      > 0) {
    _mutex.unlock();
    usleep(1000);
    _mutex.try_lock();
    _holder = hold_requester;
  } else if (holder_res < 0) {
    ret = false;
  } else {
    reset_timeout();
  }

  return ret;
}

void TimeoutMutex::unlock() {
  _mutex.unlock();
  _holder = NONE;
}
