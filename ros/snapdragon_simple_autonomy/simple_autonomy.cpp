#include <inttypes.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>

#include <basic_uart.h>
#include <platform.h>

int _fd;
char _serial_msg[MAX_MSG_LEN];

void vislam_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg);

void test_ipc();

int main(int argc, char **argv) {
  ros::init(argc, argv, "snap_simple_autonomy");

  ros::NodeHandle n;

  ros::Rate rate(20.0);

  _fd = serial_open();

  /*
  ros::Subscriber sub = n.subscribe(
      "/vislam/pose",
      1000,
      vislam_pose_cb);

  ros::spin();
      */

  while(ros::ok()) {
    test_ipc();
    rate.sleep();
  }

  //TODO what to do with return value?
  serial_close(_fd);

  return 0;
}

void vislam_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  uint16_t x,y,z;
  static int len = 10;

  _serial_msg[0] = 0; 
  _serial_msg[1] = 255; 
  _serial_msg[2] = 255; 
  _serial_msg[3] = 6; 

  x = (uint16_t)(msg->pose.position.x*1000);
  y = (uint16_t)(msg->pose.position.y*1000);
  z = (uint16_t)(msg->pose.position.z*1000);

  ROS_INFO("len: %d, x: %" PRIu16 "", len, x);

  _serial_msg[4] = get_high_byte(x);
  _serial_msg[5] = get_low_byte(x);
  _serial_msg[6] = get_high_byte(y);
  _serial_msg[7] = get_low_byte(y);
  _serial_msg[8] = get_high_byte(z);
  _serial_msg[9] = get_low_byte(z);

  if (serial_write(_fd, &(_serial_msg[0]), len) != _fd) {
    _fd = serial_close(_fd);
    _fd = serial_open(); 
  }
}

void test_ipc() {
  static int len = 10;

  _serial_msg[0] = 0; 
  _serial_msg[1] = 255; 
  _serial_msg[2] = 255; 
  _serial_msg[3] = 6; 

  _serial_msg[4] = 42; 
  _serial_msg[5] = 24; 
  _serial_msg[6] = 123; 
  _serial_msg[7] = 213; 
  _serial_msg[8] = 231; 
  _serial_msg[9] = 1; 

  ROS_INFO("len: %d", len);

  if (serial_write(_fd, &(_serial_msg[0]), len) != _fd) {
    _fd = serial_close(_fd);
    _fd = serial_open(); 
  }
}
