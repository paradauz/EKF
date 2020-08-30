#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>

using namespace Eigen;
using namespace std;

ros::Publisher odom_publisher;
ros::NodeHandle * n_ptr;

RowVector4d state = state.setZero();

double ticks_left_old = 0;
double ticks_right_old = 0;
bool first_time = 1;
bool AHRS_init = 0;

Matrix4d cov = Matrix4d::Zero();
double k = 0.1;
double sigma_yaw = 0.001;
double roll, pitch, yaw;
double z;
int c = 0;
bool measurment_update(){
	
	if(!AHRS_init){
		return 1;
	}

    RowVector4d H = H.setZero(); 
    H << 0,0,0,1;
    double Q = pow(sigma_yaw ,2);

    Vector4d K = K.setZero();
    double c = (1.0 / (H * cov * H.transpose() + Q));
    K = cov * H.transpose();
    K = K *c;

    // Measurment residual
    z = yaw - state[3];

    RowVector4d s = s.setZero();
    s << K[0], K[1], K[2], K[3];
    state = state + (s * z);    

    Matrix4d I = Matrix4d::Identity();
    cov = (I - K * H) * cov;

    ros::NodeHandle n = * n_ptr;
    odom_publisher = n.advertise<nav_msgs::Odometry>("/odom", 100);

    // Object for publishing odometry data
    nav_msgs::Odometry odom; 
    
    // Header    
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
     
    // Transforming from Eulers to quaternion   
    tf::Quaternion odom_quat = tf::createQuaternionFromRPY(roll, pitch, state[3]);

    // Setting position and orientation
    odom.pose.pose.position.x = state[0];
    odom.pose.pose.position.y = state[1];
    odom.pose.pose.position.z = state[2];
    odom.pose.pose.orientation.x = odom_quat[0];
    odom.pose.pose.orientation.y = odom_quat[1];
    odom.pose.pose.orientation.z = odom_quat[2];
    odom.pose.pose.orientation.w = odom_quat[3];
    
    odom_publisher.publish(odom);     
}

void AHRScallback(const geometry_msgs::Quaternion::ConstPtr& quat){

	if(!AHRS_init){
		AHRS_init = 1;
		cout << "AHRS initialized" << endl;
	}

	geometry_msgs::Quaternion qu;
	qu.x = quat -> x;
	qu.y = quat -> y;
	qu.z = quat -> z;
	qu.w = quat -> w;

    tf::Quaternion q(qu.x, qu.y, qu.z, qu.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(roll, pitch, yaw);
    
}
void encoderCallback(const std_msgs::Int32MultiArray::ConstPtr& ticks)
{
    if (first_time){ 
        ticks_left_old = ticks->data[0];
        ticks_right_old = ticks->data[1];
        first_time = 0; 
    }
    
    // Wheel radius and base width in m
    double L = 0.6096;
    double R = 0.127;

    // 1440 ticks per revolution
    double ticks_per_m = 1440/(M_PI*2*R);
    
    // Current ticks from subscriber
    double ticks_left_current = ticks->data[0];
    double ticks_right_current = ticks->data[1];

    // Distance traveled by each wheel and center
    double Dl = (ticks_left_current - ticks_left_old)/ticks_per_m;
    double Dr = (ticks_right_current - ticks_right_old)/ticks_per_m;
    double Dc = (Dl+Dr)/2;

    // Update state
    double cpsi = cos(state[2]);
    double spsi = sin(state[2]);
    double dpsi = (Dr-Dl)/L;

    // Pose update
    state[0] = state[0] + Dc*cpsi; // x (m)
	state[1] = state[1] + Dc*spsi; // y (m)
	state[2] = state[2] + dpsi; // psi (rad)
    
    // Updating previous tick counts
    ticks_left_old = ticks_left_current;
    ticks_right_old = ticks_right_current;

    Matrix2d U = Matrix2d::Zero();
    U(0,0) = k * pow( abs(Dl), 2);
    U(1,1) = k * pow( abs(Dr), 2);

    // Precomputed roll, pitch terms
    double ctheta = cos(pitch);
    double stheta = sin(pitch);
    double cphi = cos(roll);

    Matrix4d Fx = Matrix4d::Zero();
    Fx << 1, 0, 0, -Dc*spsi*ctheta,
    	  0, 1, 0, Dc*cpsi*ctheta,
    	  0, 0, 1, 0,
    	  0, 0, 0, 1;

    MatrixXd Fu = MatrixXd::Zero(4,2);  
    Fu << 0.5*cpsi*ctheta, 0.5*cpsi*ctheta,
          0.5*spsi*ctheta, 0.5*spsi*ctheta,
          -1/2.0*stheta,-1/2.0*stheta,
          -1/L*cphi/ctheta, 1/L*cphi/ctheta;

    cov = Fx * cov * Fx.transpose() + Fu * U * Fu.transpose();
    measurment_update();    
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pose_ekf");

    ros::NodeHandle n;
    n_ptr = &n;
    ros::Subscriber sub = n.subscribe("/wheels", 100, encoderCallback);
    ros::Subscriber sub1 = n.subscribe("/AHRS_EKF_quaternion", 100, AHRScallback);

    ros::spin();
    return 0;
}
