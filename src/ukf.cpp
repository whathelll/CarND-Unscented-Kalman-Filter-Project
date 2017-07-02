#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  x_ << 0, 0, 0, 0, 0; //px, py, v, psi, psi_d
  P_ <<  0.15,    0, 0, 0, 0,
            0, 0.15, 0, 0, 0,
            0,    0, 1, 0, 0,
            0,    0, 0, 1, 0,
            0,    0, 0, 0, 1;


  //create vector for weights
  //used in "Predicted Mean and Covariance"
  weights_ = VectorXd(2*n_aug_+1);
  //set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  double weight = 0.5/(n_aug_+lambda_);
  for (int i=1; i<2*n_aug_+1; i++) {
    weights_(i) = weight;
  }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!is_initialized_){
    std::cout << "Time: " << meas_package.timestamp_ << std::endl;
    std::cout << "Laser Measurement: " << meas_package.raw_measurements_ << std::endl;

    if(meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_[0] = meas_package.raw_measurements_[0];
      x_[1] = meas_package.raw_measurements_[1];
      time_us_ = meas_package.timestamp_;

    } else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      //rho phi, pho_dot
      auto rho = meas_package.raw_measurements_[0];
      auto phi = meas_package.raw_measurements_[1];
      x_[0] = rho * cos(phi);
      x_[1] = rho * sin(phi);
      time_us_ = meas_package.timestamp_;
    }
    is_initialized_ = true;
    return;
  }

  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  if(use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
    std::cout << "Time: " << meas_package.timestamp_ << std::endl;
    std::cout << "Laser Measurement: " << meas_package.raw_measurements_ << std::endl;

  } else if(use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    std::cout << "Time: " << meas_package.timestamp_ << std::endl;
    std::cout << "Radar Measurement: " << meas_package.raw_measurements_ << std::endl;
    Prediction(delta_t);
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  std::cout << "------------prediction" << std::endl;
  /*//////////////////////////////////////////////////////////////////////
    //////////////////Generate Sigma Matrix
  */
  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set first column of sigma point matrix
  Xsig.col(0)  = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++) {
    Xsig.col(i+1)     = x_ + sqrt(lambda_+n_x_) * A.col(i);
    Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_) * A.col(i);
  }


  /*//////////////////////////////////////////////////////////////////////
    //////////////////Augment the Sigma matrix
  */
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;
  //create square root matrix
  A = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
//  std::cout << "debug: " << std::endl << A.col(1) << std::endl;
  double multiplier = sqrt(lambda_ + n_aug_);
  for(int i = 0; i < n_aug_; i++){
    Xsig_aug.col(i+1) = x_aug + multiplier * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - multiplier * A.col(i);
  }

//  std::cout << "debug: " << std::endl << Xsig_aug << std::endl;
  /*//////////////////////////////////////////////////////////////////////
    //////////////////Sigma point prediction
  */

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++) {
    VectorXd x = Xsig_aug.col(i);

    double px = x(0);
    double py = x(1);
    double v = x(2);
    double psi = x(3);
    double psi_d = x(4);
    double nu_a = x(5);
    double nu_a_dd = x(6);

    double newX, newY;
    //avoid division by zero
    if(fabs(psi_d) > 0.001) {
      newX = px + v/psi_d * (sin(psi + psi_d*delta_t) - sin(psi));
      newY = py + v/psi_d * (-cos(psi + psi_d*delta_t) + cos(psi));
    } else {
      newX = px + v * cos(psi) * delta_t;
      newY = py + v * sin(psi) * delta_t;
    }
    double newV = v;
    double newPsi = psi + psi_d * delta_t;
    double newPsi_d = psi_d;

    //add noise
    newX += 0.5 * delta_t * delta_t * cos(psi) * nu_a;
    newY += 0.5 * delta_t * delta_t * sin(psi) * nu_a;
    newV += delta_t * nu_a;
    newPsi += 0.5 * delta_t * delta_t * nu_a_dd;
    newPsi_d += delta_t * nu_a_dd;

    //write predicted sigma points into right column
    Xsig_aug(0, i) = newX;
    Xsig_aug(1, i) = newY;
    Xsig_aug(2, i) = newV;
    Xsig_aug(3, i) = newPsi;
    Xsig_aug(4, i) = newPsi_d;
  }

  /*//////////////////////////////////////////////////////////////////////
    //////////////////Predicted Mean and Covariance
  */
  //predict state mean
//  x_ = Xsig_pred_ * weights_;
  x_.fill(0.0);             //******* necessary? *********
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

  std::cout << "predicted x: " << std::endl << x_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  //extract measurement as VectorXd
  VectorXd z = meas_package.raw_measurements_;

  //set measurement dimension, lidar can measure p_x and p_y
  int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    // measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S = S + R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);


    /**
      UKF update for lidar
    */
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //calculate NIS
//  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  std::cout << "------------update radar" << std::endl;
  /*//////////////////////////////////////////////////////////////////////
    //////////////////Predict Radar Measurement and Measurement Covariance
  */
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //transform sigma points into measurement space
  for(int i = 0; i < 2*n_aug_+1; i++){
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double psi = Xsig_pred_(3, i);

//    std::cout << "px|py|v|psi: " << px << "|" << py << "|" << v << "|" << psi << std::endl;
    Zsig(0, i) = sqrt(px*px + py*py);
    Zsig(1, i) = atan2(py, px);
    if(Zsig(0, i) < 0.001) {
      Zsig(2, i) = 0;
    } else {
      Zsig(2, i) = (px*cos(psi)*v + py*sin(psi)*v) / Zsig(0, i);
    }
  }

  //calculate mean predicted measurement
  z_pred.fill(0);
  for(int i = 0; i < 2*n_aug_+1; i++){
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
//  std::cout << "z_sig: " << std::endl << z_pred << std::endl;
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

//  std::cout << "debug S: " << std::endl << S << std::endl;

  /*//////////////////////////////////////////////////////////////////////
    //////////////////Predict Radar Measurement and Measurement Covariance
  */
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0);
  for(int i=0; i<2*n_aug_+1; i++) {
    MatrixXd Xdiff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (Xdiff(3)> M_PI) Xdiff(3)-=2.*M_PI;
    while (Xdiff(3)<-M_PI) Xdiff(3)+=2.*M_PI;

    MatrixXd Zdiff = Zsig.col(i) - z_pred;
    //angle normalization
    while (Zdiff(1)> M_PI) Zdiff(1)-=2.*M_PI;
    while (Zdiff(1)<-M_PI) Zdiff(1)+=2.*M_PI;

    Tc = Tc + weights_(i) * Xdiff * Zdiff.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();   //5x3 * 3x3 = 5x3

  //update state mean and covariance matrix
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

  x_ = x_ + K * z_diff;  //5x3 * 3x1 = 5x1
  P_ = P_ - K * S * K.transpose();

  /// NIS = (Zk+1 - Zk+1|k)T * Sinv * (Zk+1 - Zk+1|k)
  double NIS = z_diff.transpose() * S.inverse() * z_diff; // 1x3 * 3x3 * 3x1 = 1x1

  std::cout << "debug NIS: " << NIS << std::endl;
  std::cout << "debug X: " << std::endl << x_ << std::endl;
  std::cout << "debug P: " << std::endl << P_ << std::endl;
}
