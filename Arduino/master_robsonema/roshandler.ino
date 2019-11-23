#include "roshandler.h"
#include "i2cinterface.h"

geometry_msgs::Vector3 enc_val;
geometry_msgs::Vector3 imu_vector;
geometry_msgs::Vector3 pose_update;
ros::Subscriber<geometry_msgs::Vector3> pose("robot/pose", &pose_callback);
ros::Subscriber<geometry_msgs::Vector3> pwm("robot/pwm", &pwm_callback);
ros::Publisher encoder_publisher("robot/encoder", &enc_val);
ros::Publisher imu_publisher("robot/bno055", &imu_vector);
ros::Publisher pose_publisher("robot/update_pose", &pose_update);

volatile float pose_x=0, pose_y=0, pose_z=0, u_x=0, u_y=0, u_z=0;
volatile int w1=0, w2=0, w3=0;

void pose_callback(const geometry_msgs::Vector3 & dat){
  pose_x = dat.x;
  pose_y = dat.y;
  pose_z = dat.z;  
}

void pwm_callback(const geometry_msgs::Vector3 & dat){
  w1 = dat.x; //motor belakang
  w2 = dat.y; //motor kanan
  w3 = dat.z; //motor kiri
}

void publish_encoder(){
  enc_val.x = enc[0];
  enc_val.y = enc[1];
  encoder_publisher.publish(&enc_val);
}

void publish_pose(){
  pose_update.x = u_x;  
  pose_update.y = u_y;
  pose_update.z = u_z;
  pose_publisher.publish(&pose_update);
}

void publish_imu(){
  imu_vector.x = imu_x;    
  imu_vector.y = imu_y;
  imu_vector.z = imu_z;
  if(imu_vector.x>180){
    imu_vector.x = imu_vector.x - 360;
  }
  if(imu_ref>180){
    imu_ref = imu_ref - 360;
  }
  imu_vector.x = -imu_vector.x + imu_ref;
  if(imu_vector.x < -180){
    imu_vector.x = imu_vector.x + 360;  
  }
  else if(imu_vector.x>180){
    imu_vector.x = imu_vector.x - 360;
  }
  imu_publisher.publish(&imu_vector);
}

void init_ros(){
  nh.initNode();
  nh.subscribe(pose);//subscribe data odometry
  nh.subscribe(pwm);//subscribe data odometry
  nh.advertise(encoder_publisher);//publish posisi bola
  nh.advertise(imu_publisher);//publish imu
  nh.advertise(pose_publisher);//publish imu
}

void ros_routine(){
  nh.spinOnce();
  delay(10);
}

