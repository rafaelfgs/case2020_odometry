#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>

/*
Universidade Federal de Minas Gerais (UFMG) - 2020
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
*/

using namespace Eigen;

class EKF_Filter{
  private:
    VectorXd states, imu_data;
    //----- ----- states ----- -----
    //x
    //y
    //psi (yaw angle)
    //v_x (forward velocity)
    //----- ----- ------ ----- -----

    // Pose Measurement (6)
    VectorXd read_pos;
    // Time stamp
    double dt;
    // Range Finder
    double rangeF_data;


    double rangeF_data_prev;
    double rf_time_prev;

    // Covariation Matrix
    MatrixXd P, Q, R, H, Q_bar, H_yolo, H_slam, R_yolo, R_slam;
    // Kalman Gain
    MatrixXd K;


  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EKF_Filter(VectorXd, double, double); //Construtor
    EKF_Filter(VectorXd, double, MatrixXd, MatrixXd, MatrixXd, MatrixXd, MatrixXd);
    ~EKF_Filter(); //Destructor

    //callback imu Data (imu, dt)
    void callback_imu(VectorXd, VectorXd, double);

    // Model fuction (states, imu)
    VectorXd discrete_model(VectorXd, VectorXd, double);



    // Facobian of the model functionwith respect to the states (VectorXd x, VectorXd u)
    MatrixXd Jacobian_F(VectorXd, VectorXd, double);
    //Facobian of the model functionwith respect to the IMU (VectorXd x, VectorXd u)
    MatrixXd Jacobian_G(VectorXd, VectorXd, double);


    // Prediction Step
    void prediction();


    // Actualization position measurement (pos)
    void callback_position(VectorXd);

    // Actualization velocity cmd (vel_x)
    void callback_vel_x(VectorXd);



    // Convert Euler to Quaternion
    VectorXd EulertoQuaternion(VectorXd);
    VectorXd quat2eulerangle(VectorXd);
    MatrixXd angle2rotm(VectorXd);
    VectorXd rotm2angle(MatrixXd);

    // Get EKF States Data
    VectorXd get_states();
    // Get Filtered IMU Data
    VectorXd get_IMU();
};
