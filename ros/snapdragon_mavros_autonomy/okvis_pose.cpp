#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <okvis/VioParametersReader.hpp>
#include <okvis/ThreadedKFVio.hpp>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>

void publishFullStateAsCallback(
    const okvis::Time &t,
    const okvis::kinematics::Transformation &T_WS,
    const Eigen::Matrix<double, 9, 1> &speedAndBiases,
    const Eigen::Matrix<double, 3, 1> &omega_S);

ros::Publisher _pose_pub;
geometry_msgs::PoseStamped _pose;

int main(int argc, char **argv) {
  ros::NodeHandle nh;
  _pose_pub = nh.advertise
            <geometry_msgs::PoseStamped>
            ("/okvis/pose", 10);

  std::string configFilename;
  if (!nh.getParam("/okvis_config", configFilename)) {
    ROS_ERROR("Must pass okvis config file");
  }

  okvis::VioParametersReader vio_parameters_reader(configFilename);
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);

  okvis::ThreadedKFVio okvis_estimator(parameters);

  okvis_estimator.setFullStateCallback(
    std::bind(&publishFullStateAsCallback,
      std::placeholders::_1,std::placeholders::_2,
      std::placeholders::_3,std::placeholders::_4));

  okvis_estimator.setBlocking(true);

  return 0;
}

void publishFullStateAsCallback(
    const okvis::Time &t,
    const okvis::kinematics::Transformation &T_WS,
    const Eigen::Matrix<double, 9, 1> &speedAndBiases,
    const Eigen::Matrix<double, 3, 1> &omega_S) {
  Eigen::Vector3d r = T_WS.r();
  ROS_INFO("Translation\n\tx:\t%f\n\ty:\t%f\n\tz:\t%f",
      r[0],r[1],r[2]);

  _pose.pose.position.x = r[0];
  _pose.pose.position.y = r[1];
  _pose.pose.position.z = r[2];
  _pose_pub.publish(_pose);
}
