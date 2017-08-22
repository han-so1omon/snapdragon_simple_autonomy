#ifndef MAVROS_AUTONOMY_H
#define MAVROS_AUTONOMY_H

#include <ros/ros.h>
#include <mutex>

#define NONE 0
#define VISLAM 1
#define OKVIS 2

#define NONE_PRIORITY 0
#define VISLAM_PRIORITY 1
#define OKVIS_PRIORITY 2

class TimeoutMutex {
public:
  ros::Timer _timeout;
  std::mutex _mutex;
  int _holder;
  bool _duration_set;

  TimeoutMutex();
  TimeoutMutex(ros::NodeHandle &nh, int timeout);
  ~TimeoutMutex();

  void set_timeout_duration(ros::NodeHandle &nh, int timeout);
  // Lower return -> higher priority
  int holder_priority(int hold_requester);
  int compare_holder(int h1, int h2);
  void timeout_cb(const ros::TimerEvent &te);
  void reset_timeout();
  // Returns true if hold_requester type is holding
  // mutex at the end of the method
  bool try_lock(int hold_requester);
  void unlock();
};

#endif
