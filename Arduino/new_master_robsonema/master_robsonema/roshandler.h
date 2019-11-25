#ifndef ROSHANDLER_H
#define ROSHANDLER_H

#include <ros.h>
#include <geometry_msgs/Vector3.h>

extern volatile float pose_x, pose_y, pose_z, u_x, u_y, u_z;
extern volatile int w1, w2, w3;

void pose_callback(const geometry_msgs::Vector3 & dat);
void pwm_callback(const geometry_msgs::Vector3 & dat);
void publish_encoder();
void publish_imu();
void init_ros();
void ros_routine();

#endif
