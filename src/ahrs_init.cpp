#include <ros/ros.h>
#include "ekf/initRequest.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace Eigen;

// ****Make a class****
ros::Subscriber sub;
ros::ServiceServer service;
ros::NodeHandle * n_ptr;

int n_imu = 0;
int num_data = 1000;
RowVector3d sum_accel = sum_accel.setZero();
RowVector3d sum_gyro = sum_gyro.setZero();

bool initialization_handle(ekf::initRequest::Request &req,ekf::initRequest::Response &res);

void imu_callback(const sensor_msgs::Imu::ConstPtr& data){

    ros::NodeHandle n = *n_ptr;

    // Getting data from the accelerometer
    RowVector3d accel_data = accel_data.setZero();
    accel_data << data -> linear_acceleration.x, data -> linear_acceleration.y, data -> linear_acceleration.z;

    // Getting data from the gyro
    RowVector3d gyro_data = gyro_data.setZero();
    gyro_data << data -> angular_velocity.x, data -> angular_velocity.y, data -> angular_velocity.z;

    // Making sure we get a 1000 points to get the avg
    if (n_imu < num_data){

        sum_accel -= accel_data;
        sum_gyro += gyro_data; 
        n_imu += 1;
    }
    else {

        service = n.advertiseService("initializing_ahrs", initialization_handle);
        sub.shutdown();
    }   
}


bool initialization_handle(ekf::initRequest::Request &req,ekf::initRequest::Response &res){

    // Estimating gravity vector  
    RowVector3d g_v = g_v.setZero();
    g_v = sum_accel / num_data;

    // Initial roll(phi), pitch(theta), yaw(psi)
    double phi = atan2(-g_v[1], -g_v[2]);
    double theta = atan2(g_v[0], sqrt(pow(g_v[1],2) + pow(g_v[2],2)));
    double psi = 0;
    
    // Creating quaternion from RPY to pass to ekf
    tf2::Quaternion quat;
    quat.setRPY(-phi,-theta, -psi);

    // Computing bias of the gyroscope ****Possibly change srv file to pass Eigen RowVector****
    res.gyro_bias[0] = sum_gyro[0] / num_data;
    res.gyro_bias[1] = sum_gyro[1] / num_data;
    res.gyro_bias[2] = sum_gyro[2] / num_data;
          
    // Converting from tf::Quaternion to geometry_msgs::Quaternion to pass as a msg
    tf2::convert(quat , res.init_orientation);
    ROS_INFO("angles YPR: %f, %f, %f ", (double)psi, (double)phi, (double)theta);
    ROS_INFO("sending back response: \n gyro_bias:[%f, %f, %f]", (double)res.gyro_bias[0], (double)res.gyro_bias[1], (double)res.gyro_bias[2]);
    ROS_INFO("quat x: %f y: %f z: %f w: %f ", (double)quat[0], (double)quat[1], (double)quat[2], (double)quat[3]);
    
    return true;
}


int main(int argc, char **argv)
{
  // Initializing the node and creating the node handle
  ros::init(argc, argv, "ahrs_init");
  ros::NodeHandle n;
  n_ptr = &n;

  // Subscribing to get the data from accel and gyro
  sub = n.subscribe<sensor_msgs::Imu>("/imu/data", 1000, imu_callback);
  
  ros::spin();  
  return 0;
}