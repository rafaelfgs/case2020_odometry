#include "EKF_espeleo.h"

/*
Universidade Federal de Minas Gerais (UFMG) - 2020
Laboraorio CORO
Instituto Tecnologico Vale (ITV)
Contact:
Adriano M. C. Rezende, <adrianomcr18@gmail.com>
*/



#define PI 3.1415926535

#define N_STATES 4


using namespace std;
using namespace Eigen;




// Constructor
EKF_Filter::EKF_Filter(VectorXd initial_pos, double Frequency, double init_angle){

  VectorXd state_init(N_STATES), imu_init(6), read_pos_init(2);
  state_init.setZero();
  read_pos_init.setZero();
  imu_init << 0,0,0,  0,0,9.81;

  state_init.block(0,0,2,1) = initial_pos;
  state_init(2) = init_angle;
  state_init(3) = 0.0;

  read_pos_init.block(0,0,2,1) = initial_pos;

  states = state_init; // (x, y, z, phi, theta, psi, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz)
  imu_data = imu_init; // (ang_vel, acc_linear)
  read_pos = read_pos_init; // (x, y, z, phi, theta, psi)

  dt = 1.0/Frequency;



  // Jacobian of the measurement model
  H = MatrixXd::Zero(3,N_STATES);
  H << 1.0, 0.0, 0.0, 0.0, //gps
       0.0, 1.0, 0.0, 0.0, //gps
       0.0, 0.0, 0.0, 1.0; //vel_x cmd




  // Model Propagation Covariance
  Q = MatrixXd::Zero(N_STATES,N_STATES);
  Q << 1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0;


  //Covariance matrix of the IMU
  Q_bar = MatrixXd::Zero(6,6);
  Q_bar << 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, // gyro
           0.0, 5.0, 0.0, 0.0, 0.0, 0.0, // gyro
           0.0, 0.0, 5.0, 0.0, 0.0, 0.0, // gyro
           0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, // acc
           0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, // acc
           0.0, 0.0, 0.0, 0.0, 0.0, 1000.0; // acc
           // Q_bar = Q_bar*1000.0;
           //  Q_bar = Q_bar/10.0;



  // Measurement Covariance
  R = MatrixXd::Zero(3,3);
  R << 1.5, 0.0,  0.0, //gps x
       0.0, 1.5,  0.0, //gps x
       0.0, 0.0, 1.0; //velocity x cmd
       // R = R*10;
       // R = R/10.0;


  // Initial covariance matrix
  P = MatrixXd::Zero(N_STATES,N_STATES);
  P << 1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0;

  //P = P + 0.01;
  for (int k1 = 0; k1<N_STATES; k1++){
    for (int k2 = 0; k2<N_STATES; k2++){
      if(k1!=k2){
        P(k1,k2) = P(k1,k2) + 0.01*0;
      }
    }
  }

  // Kalman Gain
  K = MatrixXd::Zero(N_STATES,N_STATES);
}




EKF_Filter::EKF_Filter(VectorXd states_0, double Frequency, MatrixXd H_, MatrixXd Q_, MatrixXd Q_bar_, MatrixXd R_, MatrixXd P_){


    VectorXd imu_init(6);
    imu_init << 0,0,0,  0,0,9.81;

    states = states_0; // (x, y, z, phi, theta, psi, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz)
    imu_data = imu_init; // (ang_vel, acc_linear)

    dt = 1.0/Frequency;

    // Jacobian of the measurement model
    H = H_;

    // Model Propagation Covariance
    Q = Q_;


    //Covariance matrix of the IMU
    Q_bar = Q_bar_;
    // Q_bar = Q_bar*1000.0;
    // Q_bar = Q_bar/10.0;



    // Measurement Covariance
    R = R_;


    // Initial covariance matrix
    P = P_;
    //P = P + 0.01;
    for (int k1 = 0; k1<N_STATES; k1++){
      for (int k2 = 0; k2<N_STATES; k2++){
        if(k1!=k2){
          P(k1,k2) = P(k1,k2) + 0.01*0;
        }
      }
    }

    // Kalman Gain
    K = MatrixXd::Zero(N_STATES,N_STATES);


}




















// Callback_imu
void EKF_Filter::callback_imu(VectorXd gyro_read, VectorXd acc_read, double Ts){
  // Set dt
  dt = Ts;

  // Low-pass filter
  double alpha = 0.51;  //We can make alpha be a function of Ts
  double beta = 0.51;  //We can make alpha be a function of Ts
  imu_data.block(0,0,3,1) = (1.0-beta)*imu_data.block(0,0,3,1) + beta*gyro_read;
  imu_data.block(3,0,3,1) = (1.0-alpha)*imu_data.block(3,0,3,1) + alpha*acc_read;
}







// Jacobian of the model with respect to the states
MatrixXd EKF_Filter::Jacobian_F(VectorXd x, VectorXd u, double dt){

  MatrixXd F(N_STATES,N_STATES);

  VectorXd f0(N_STATES);
  VectorXd f1(N_STATES);

  VectorXd state_now(N_STATES), imu_now(6);

  state_now = x;
  imu_now = u;
  VectorXd state_plus(N_STATES), state_diff(N_STATES);
  double delta = 0.0001;

  f0 = discrete_model(state_now, imu_data, dt);

  for (int k = 0; k<N_STATES; k++){
    state_plus = state_now;
    state_plus(k) = state_plus(k) + delta;
    f1 = discrete_model(state_plus, imu_now, dt);
    state_diff = f1-f0;
    state_diff(2) = sin(state_diff(2)); //yaw
    F.block(0,k,N_STATES,1) = state_diff/delta;
  }

  return F;
}



// Jacobian of the model with respect to the IMU
MatrixXd EKF_Filter::Jacobian_G(VectorXd x, VectorXd u, double dt){

  MatrixXd G(N_STATES,6);

  VectorXd f0(N_STATES);
  VectorXd f1(N_STATES);

  VectorXd state_now(N_STATES), imu_now(6);

  state_now = x;
  imu_now = u;
  VectorXd imu_plus(6), state_diff(N_STATES);
  double delta = 0.0001;

  f0 = discrete_model(state_now, imu_now, dt);


  for (int k = 0; k<6; k++){
    imu_plus = imu_now;
    imu_plus(k) = imu_plus(k) + delta;
    f1 = discrete_model(state_now, imu_plus, dt);
    state_diff = f1-f0;
    state_diff(2) = sin(state_diff(2)); //Orientation
    G.block(0,k,N_STATES,1) = state_diff/delta;
  }

  return G;
}





// Propagation model
VectorXd EKF_Filter::discrete_model(VectorXd x, VectorXd u, double dt){

  //Create output vector
  VectorXd f(N_STATES);

  VectorXd u_imu(6);
  u_imu = u;


  // Position
  f(0) = x(0) + ( x(3)*cos(x(2)) ) *dt;
  f(1) = x(1) + ( x(3)*sin(x(2)) ) *dt;
  // Orientation
  f(2) = x(2) + u_imu(2)*dt;
  // Velocity
  f(3) = x(3) + u_imu(4)*dt;

  return f;
}




// Prediction Step
void EKF_Filter::prediction(){

  // Next state function
  VectorXd f(N_STATES);
  // Jacobian of the next state with respect to the states
  MatrixXd F(N_STATES,N_STATES);
  // Jacobian of the next state with respect to the IMU
  MatrixXd G(N_STATES,2);


  //Compute the next state
  f = discrete_model(states, imu_data, dt);
  //Compute the Jacobian matrix
  F = Jacobian_F(states, imu_data, dt);
  //Compute the Jacobian matrix
  G = Jacobian_G(states, imu_data, dt);


  //Propagate the states
  states = f;

  //Compute the covariance of the propagation given the current state and the covariance of the IMU
  Q = G*Q_bar*G.transpose();


  //Propagate the covariance matrix
  P = F*P*F.transpose() + Q;
  // // Resymetrize the matrix
  // P = (P+P.transpose())/2.0;


  //Avoid loops in the angle coordinate
  if(states(2)>PI){
    states(2) = states(2)-2*PI;
  }
  if(states(2)<-PI){
    states(2) = states(2)+2*PI;
  }

}




















// EKF update - position from GPS
void EKF_Filter::callback_position(VectorXd pos){
  // Measurements
  read_pos = pos;

  //Measurement model Jacobial only for position
  MatrixXd H_aux(2,N_STATES);
  H_aux = H.block(0,0,2,N_STATES);

  //Measurement covariance only for position
  MatrixXd R_aux(2,2);
  R_aux = R.block(0,0,2,2);


  // Compute Inovation
  VectorXd inovation(2);
  inovation = read_pos - H_aux*states;


  // Compute Kalman Gain
  MatrixXd S(2,2);
  S = H_aux*P*H_aux.transpose() + R_aux;
  K = P*H_aux.transpose()*S.inverse();


  // Actualization of the states
  states = states + K*inovation;

  // Actualization of covariance matrix
  P = (MatrixXd::Identity(N_STATES,N_STATES) - K*H_aux)*P;
}











// EKF update - forward velocity from cmd_vel
void EKF_Filter::callback_vel_x(VectorXd vel_x){

  //Measurement model Jacobial only for velocity
  MatrixXd H_aux(1,N_STATES);
  H_aux = H.block(2,0,1,N_STATES);

  // Compute Inovation
  VectorXd inovation(1);
  inovation = vel_x - H_aux*states;

  //Measurement covariance only for velocity
  MatrixXd R_aux(1,1);
  R_aux = R.block(2,2,1,1);

  // Compute Kalman Gain
  MatrixXd S(1,1);
  S = H_aux*P*H_aux.transpose() + R_aux;
  K = P*H_aux.transpose()*S.inverse();

  // Actualization of the states
  states = states + K*inovation;

  // Actualization of covariance matrix
  P = (MatrixXd::Identity(N_STATES,N_STATES) - K*H_aux)*P;
}








// destructor
EKF_Filter::~EKF_Filter(){
}





VectorXd EKF_Filter::EulertoQuaternion( VectorXd rpy) // yaw (Z), pitch (Y), roll (X)
{

  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);

  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  double w = cy * cp * cr + sy * sp * sr;
  double x = cy * cp * sr - sy * sp * cr;
  double y = sy * cp * sr + cy * sp * cr;
  double z = sy * cp * cr - cy * sp * sr;

  VectorXd q(4);
  q << w,x,y,z;
  return q;
}





MatrixXd EKF_Filter::angle2rotm( VectorXd rpy) // yaw (Z), pitch (Y), roll (X)
{
  MatrixXd Rot(3,3);
  double phi = rpy(0);
  double theta = rpy(1);
  double psi = rpy(2);
  // Get rotation matrix
  Rot << (cos(theta)*cos(psi)), (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)), (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)),
         (cos(theta)*sin(psi)), (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)), (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)),
         (-sin(theta)), (sin(phi)*cos(theta)), (cos(phi)*cos(theta));

  return Rot;
}



VectorXd EKF_Filter::rotm2angle( MatrixXd Rot){
  VectorXd rpy(3);
  Matrix3d Rot2;

  Rot2 = Rot;

  Quaterniond quat1(Rot2);

  VectorXd quat2(4);

  quat2 << quat1.w(), quat1.x(), quat1.y(), quat1.z();

  rpy = quat2eulerangle(quat2);

  return rpy;
}






// Unit Quaternion to Euler angle
VectorXd EKF_Filter::quat2eulerangle(VectorXd q){
  // w x y z

  VectorXd angle(3,1);
  angle.setZero();

  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q[0] * q[1] + q[2] * q[3]);
  double cosr_cosp = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
  angle[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q[0] * q[2] - q[3] * q[1]);
  if (fabs(sinp) >= 1){
    angle[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  }
  else{
    angle[1] = asin(sinp);
  }

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q[0] * q[3] + q[1] * q[2]);
  double cosy_cosp = +1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
  angle[2] = atan2(siny_cosp, cosy_cosp);

  return angle;
}



VectorXd EKF_Filter::get_states(){

  Eigen::VectorXd states_return(N_STATES);

  states_return = states;

  return states_return;
}




VectorXd EKF_Filter::get_IMU(){
  return imu_data;
}
