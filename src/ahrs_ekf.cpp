#include <ros/ros.h>
#include "ekf/initRequest.h"
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace Eigen;
using namespace std;

// Node Handle pointer for client and publisher
ros::NodeHandle * n_ptr;
ros::ServiceClient client;
ros::Publisher pub;

//State vectors - quaternion, gyro bias, accel bias
RowVector4d state_b0 = RowVector4d::Zero(); 
RowVector3d state_x_g = RowVector3d::Zero();
RowVector3d state_x_a = RowVector3d::Zero();

// Predicted gravity vector
RowVector3d g_pred = g_pred.setZero();
// Covariance matrix 
MatrixXd cov = MatrixXd::Zero(9,9);

//Sample time
double dt = 0;
int counter = 0;
int accel_counter = 0;

// Noise terms ****put in separate file****
double sigma_xg = 0.00000290; // Gyro (rate) random walk
double sigma_nug = 0.00068585;   // rad/s/rt_Hz, Gyro white noise
double sigma_xa = 0.00001483; // Accel (rate) random walk m/s3 1/sqrt(Hz)
double sigma_nua = 0.00220313; // accel white noise
double T = 1000.0/200; // Number of measurments over rate of IMU
double g = 9.81; // Gravity

void measurment_update(){
    // Current state
    RowVector4d b = b.setZero();
    b << state_b0[0], state_b0[1], state_b0[2], state_b0[3];

    tf2::Quaternion b_q(b[0],b[1],b[2],b[3]);
    
    // Current rotation matrix
    tf2::Matrix3x3 rot(b_q);     
    Matrix3d R_body_to_nav = Matrix3d::Zero();

    R_body_to_nav << rot[0].x(), rot[0].y(),rot[0].z(),
                     rot[1].x(), rot[1].y(),rot[1].z(),
                     rot[2].x(), rot[2].y(),rot[2].z();

    R_body_to_nav.transposeInPlace();

    // Measurment matrix for gyro
    MatrixXd Ha = MatrixXd::Zero(3,9);
    Ha.block(0,6,3,3) = R_body_to_nav;
    Ha(0,1) = g;
    Ha(1,0) = -g;    

    // Measurment matricies
    MatrixXd H = MatrixXd::Zero(3,9);
    H = Ha;

    // Noise matrix for measurment
    Matrix3d R = Matrix3d::Zero();
    R << pow(sigma_nua,2),0,0,
         0,pow(sigma_nua,2),0,                                      
         0,0,pow(sigma_nua,2);

    // Kalman gain
    MatrixXd K = MatrixXd::Zero(9,3);
    Matrix3d mid = Matrix3d::Zero();
    mid = R + H * cov * H.transpose();    
    K = (cov * H.transpose()) * (mid.inverse());        

    RowVector3d y_pred = y_pred.setZero();
    y_pred = g_pred;

    // Known gravity vector
    RowVector3d g_vector = g_vector.setZero();
    g_vector << 0, 0, 9.81;
    RowVector3d y = y.setZero();
    y = g_vector;

    // Residual z
    RowVector3d z = z.setZero();
    z = y - y_pred;

    // State correction
    VectorXd dx(9);
    dx = K * z.transpose();

    // Rotation update
    RowVector3d p = p.setZero();
    p = dx.head(3);
    
    Matrix3d P = Matrix3d::Zero();
    P << 0, -p(2), p(1),
         p(2), 0, -p(0),
         -p(1), p(0), 0;

    Matrix3d R_nav_to_body = Matrix3d::Zero();
    R_nav_to_body = R_body_to_nav.transpose();

    Matrix3d R_nav_to_body_next = Matrix3d::Zero();
    Matrix3d i = Matrix3d::Identity();
    R_nav_to_body_next = R_nav_to_body * (i - P);

    // Reorthagonalize matrix
    JacobiSVD<MatrixXd> svd(R_nav_to_body_next, ComputeFullU | ComputeFullV);
    Matrix3d V;
    V = svd.matrixV();
    V.transposeInPlace();
    R_nav_to_body_next = svd.matrixU() * V;
    
    // Updating bias
    RowVector3d dx_g = dx_g.setZero();
    RowVector3d dx_a = dx_g.setZero();
    dx_g << dx(3), dx(4), dx(5);
    dx_a << dx(6), dx(7), dx(8);
    
    // Didn't find a better way to initialize Rotational matrix
    Matrix3d a = Matrix3d::Zero();
    a = R_nav_to_body_next;
    tf2::Matrix3x3 quat_m;
    tf2::Vector3 row0 (a(0,0),a(0,1),a(0,2));
    tf2::Vector3 row1 (a(1,0),a(1,1),a(1,2));
    tf2::Vector3 row2 (a(2,0),a(2,1),a(2,2));
    quat_m[0] = row0;
    quat_m[1] = row1;
    quat_m[2] = row2;    
                           
    tf2::Quaternion b_next;
    double res_yaw;
    double res_pitch;
    double res_roll;
    quat_m.getRPY(res_yaw, res_pitch, res_roll);
    b_next.setRPY(res_yaw, res_pitch, res_roll);       
     
    // Updating state
    state_b0 << b_next[0], b_next[1], b_next[2], b_next[3];
    cout << state_b0<< "\n" << endl; 
    state_x_g = state_x_g + dx_g;
    state_x_a = state_x_a + dx_a;

    // Updating covariance
    MatrixXd I = MatrixXd::Identity(9,9);
    cov = (I - K * H) * cov;
}


void imu_callback(const sensor_msgs::Imu::ConstPtr& data){ 
    
    // Getting data from the accel - a
    RowVector3d a_data = a_data.setZero();
    a_data << data -> linear_acceleration.x, data -> linear_acceleration.y, data -> linear_acceleration.z;

    // Getting data from the gyro - w
    RowVector3d g_data = g_data.setZero();    
    g_data << data -> angular_velocity.x, data -> angular_velocity.y, data -> angular_velocity.z;

    // Accessing current state 
    RowVector4d b = b.setZero();
    b << state_b0[0], state_b0[1], state_b0[2], state_b0[3];

    // Subtract out gyro data
    g_data = -1*(g_data - state_x_g);
    double g_norm = g_data.norm();

    // Differential rotation (quaternion)
    RowVector4d v_db = v_db.setZero();
    v_db << sin(g_norm * dt/2)* g_data[0]/g_norm, sin(g_norm * dt/2) * g_data[1]/g_norm, sin(g_norm * dt/2)* g_data[2]/g_norm , cos(g_norm * dt/2);

    // Previous quat  = state that we got from initialization
    tf2::Quaternion b_prev(b[0],b[1],b[2],b[3]);
    tf2::Quaternion db(v_db[0],v_db[1],v_db[2],v_db[3]);

    // Latest quat(previous one * differential)
    tf2::Quaternion b_next;
    b_next = db * b_prev;    
    b_next.normalize();
   
    // Updating state - time propagation
    state_b0 << b_next[0], b_next[1], b_next[2], b_next[3];

    ros::NodeHandle n = *n_ptr;    

        // Publishing quaternion to the topic used by pose estimation
        tf2::Quaternion b_vis;
        b_vis = b_next.inverse();       
        geometry_msgs::Quaternion p;
        p.x = b_vis[0];
        p.y = b_vis[1];
        p.z = b_vis[2];
        p.w = b_vis[3];
        pub = n.advertise<geometry_msgs::Quaternion>("AHRS_EKF_quaternion", 1);
        pub.publish(p);
        
        // Transform for visualization in Rviz
        tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "ENU";
        ts.child_frame_id = "quat";
        ts.transform.translation.x = 0.0;
        ts.transform.translation.y = 0.0;
        ts.transform.translation.z = 0.0;
        ts.transform.rotation.x = b_vis.x();
        ts.transform.rotation.y = b_vis.y();
        ts.transform.rotation.z = b_vis.z();
        ts.transform.rotation.w = b_vis.w();
        br.sendTransform(ts);    
    
    // Getting a rotation matrix from b (body to navigation frame)    
    tf2::Matrix3x3 rot(b_next); 
    Matrix3d R_body_to_nav = Matrix3d::Zero();
    R_body_to_nav << rot[0].x(), rot[0].y(),rot[0].z(),
                     rot[1].x(), rot[1].y(),rot[1].z(),
                     rot[2].x(), rot[2].y(),rot[2].z();

    R_body_to_nav.transposeInPlace();
    
    double lambda_g = 1.0 / 1000; // Correlation time of gyro
    double labmda_a = 1.0 / 1000; // Correlation time of accel
    
    MatrixXd F = MatrixXd::Zero(9,9);
    Matrix3d F_g = F_g.Identity();
    Matrix3d F_a = F_a.Identity();
    F_g = -1.0 / 1000 * F_g;
    F_a = -1.0 / 1000 * F_a; 
    F.block(0, 3, 3, 3) = -R_body_to_nav; 
    F.block(3, 3, 3, 3) = F_g ;
    F.bottomRightCorner(3,3) = F_a;

    // State transition matrix 9x9
    MatrixXd Phi = MatrixXd::Zero(9,9);
    Phi = F * dt;    
    Phi = Phi.exp();

    MatrixXd G = MatrixXd::Zero(9,12);
    Matrix3d i = i.Identity();
    G.topLeftCorner(3,3) = -R_body_to_nav; 
    G.block(0, 6, 3, 3) = -R_body_to_nav; 
    G.block(3,3,3,3) = i;
    G.bottomRightCorner(3,3) = i;    

    // Noise matrix
    MatrixXd Q = MatrixXd::Zero(12,12);
    double sigma_in = 0.0000;
    Q.topLeftCorner(3,3) = pow(sigma_in, 2) * i;
    Q.block(3,3,3,3) = pow(sigma_xg, 2) * i; 
    Q.block(6,6,3,3) = pow(sigma_nug, 2) * i; 
    Q.block(9,9,3,3) = pow(sigma_xa, 2) * i;  
    
    // Computing Qdk -  9 x 9
    MatrixXd Qdk = MatrixXd::Zero(9,9);
    Qdk = (G*Q)*(G.transpose())*dt;

    // Updating covariance
    cov = (Phi * cov) * (Phi.transpose()) + Qdk;       
    
    // Predicting gravity in nav frame
    g_pred = R_body_to_nav * (state_x_a - a_data).transpose();
    
    // Normalizing quaternion periodically
    if (counter == 1000){
        tf2::Quaternion state_b(b[0],b[1],b[2],b[3]);
        state_b.normalize(); 
        state_b0 << state_b[0], state_b[1], state_b[2],state_b[3];
    }

    // Doing measurment update if the robot is stationary (accel readings are close to predicted g_vector)
    if ((abs(a_data.norm() - 9.8021)) < 0.05){
        accel_counter++;
    }else {
        accel_counter = 0;
    }
    if (accel_counter == 200){
        cout << "Measurment update" << endl;
        measurment_update();
        accel_counter = 0;
    }
    counter++;
}

bool initializing_ahrs_client(){

    ros::NodeHandle n = *n_ptr;
    ros::ServiceClient client = n.serviceClient<ekf::initRequest>("initializing_ahrs");

    ekf::initRequest srv;
    // make it wait forever?
    if(!client.waitForExistence(ros::Duration(200))){
        ROS_ERROR("Server didn't show up");
    } 
        
    if (client.call(srv))
    {
        ROS_INFO("Received initialization data");
        geometry_msgs::Quaternion b;
        b = srv.response.init_orientation;  
      
        // Storing init_orientation quat as a vector
        state_b0 << b.x, b.y, b.z, b.w;        
        
        // Init gyro biases / Accel biases are 0 for now
        state_x_g << srv.response.gyro_bias[0], srv.response.gyro_bias[1], srv.response.gyro_bias[2];
      
        // imu freq
        int imu_hz = 200;
        //Sample time
        dt = 1.0/imu_hz;
  
        // Covariance matrix 
        cov = MatrixXd::Identity(9,9);
        Matrix3d i = Matrix3d::Identity();
        cov(0, 0) = pow(sigma_nua / g, 2 )/ T;
        cov(1, 1) = pow(sigma_nua / g, 2 )/ T;
        cov(2, 2) = pow(0, 2) / T;
        cov.block(3,3,3,3) = i * pow(sigma_nug, 2 )/ T;
        cov(0, 7) = sqrt(cov(0, 0) * cov(7, 7));
        cov(1, 8) = sqrt(cov(1, 1) * cov(8, 8));
        cov(7, 0) = cov(0, 7);
        cov(8, 1) = cov(1, 8);        
    }
    
    else{
        ROS_ERROR("Failed to call service initializing_ahrs");
        return 1;
    }    
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ahrs_ekf");
    ros::NodeHandle n;
    n_ptr = &n;

    ROS_INFO("Initializing IMU");
    initializing_ahrs_client();

    ros::Subscriber sub = n.subscribe<sensor_msgs::Imu>("/imu/data", 1000, imu_callback);    
        
    ros::spin();
    return 0;
}


