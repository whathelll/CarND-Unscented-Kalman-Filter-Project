#include "gtest/gtest.h"
#include "ukf.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

MeasurementPackage getLidarPackage(long time, double x, double y) {
  MeasurementPackage meas_package;
  meas_package.sensor_type_ = MeasurementPackage::LASER;
  meas_package.raw_measurements_ = VectorXd(2);
  meas_package.raw_measurements_ << x, y;
  meas_package.timestamp_ = time;
  return meas_package;
}

MeasurementPackage getRadarPackage(long time, double rho, double phi, double rho_dot) {
  MeasurementPackage meas_package;
  meas_package.sensor_type_ = MeasurementPackage::RADAR;
  meas_package.raw_measurements_ = VectorXd(3);
  meas_package.raw_measurements_ << rho, phi, rho_dot;
  meas_package.timestamp_ = time;
  return meas_package;
}




TEST(UKF, initialize) {
  UKF ukf;

  auto package = getLidarPackage(1477010443000000, 0.312243, 0.58034);
  ukf.ProcessMeasurement(package);
  EXPECT_EQ(0.312243, ukf.x_[0]);
  EXPECT_EQ(0.58034, ukf.x_[1]);
}


TEST(UKF, radar_measurements) {
  UKF ukf;
//  Ground Truth 0:
//      0.6
//      0.6
  auto package = getLidarPackage(1477010443000000, 0.312243, 0.58034);
  ukf.ProcessMeasurement(package);
  EXPECT_DOUBLE_EQ(0.312243, ukf.x_[0]);
  EXPECT_DOUBLE_EQ(0.58034, ukf.x_[1]);

//  Ground Truth 1:
//  0.859997
//    0.600045
  package = getRadarPackage(1477010443050000, 1.01489, 0.554329, 4.89281);
  ukf.ProcessMeasurement(package);

//  Ground Truth 2:
//  1.37996
//   0.600629
  package = getRadarPackage(1477010443150000, 1.04751, 0.38924, 4.51132);
  ukf.ProcessMeasurement(package);

//  Ground Truth 3:
//  1.89982
//    0.60247
  package = getRadarPackage(1477010443250000, 1.6983, 0.29828, 5.20999);
  ukf.ProcessMeasurement(package);

  package = getRadarPackage(1477010443350000, 2.04438, 0.276002, 5.04387);
  ukf.ProcessMeasurement(package);

  package = getRadarPackage(1477010443450000, 2.99092, 0.217668, 5.19181);
  ukf.ProcessMeasurement(package);

  package = getRadarPackage(1477010443550000, 3.59388, 0.135452, 5.16175);
  ukf.ProcessMeasurement(package);
  package = getRadarPackage(1477010443650000, 4.25555, 0.16484, 5.43333);
  ukf.ProcessMeasurement(package);
  package = getRadarPackage(1477010443750000, 4.67026, 0.14818, 5.12085);
  ukf.ProcessMeasurement(package);
  package = getRadarPackage(1477010443850000, 5.25142, 0.127163, 4.82591);
  ukf.ProcessMeasurement(package);
}




TEST(UKF, laser_measurements) {
  UKF ukf;
//  Time: 1477010443000000
//  Laser Measurement: 0.312243
//   0.58034
//  Ground Truth:
//     1.11998
//    0.600225
//     5.19943
//  0.00538996

  auto package = getLidarPackage(1477010443000000, 0.312243, 0.58034);
  ukf.ProcessMeasurement(package);
  EXPECT_DOUBLE_EQ(0.312243, ukf.x_[0]);
  EXPECT_DOUBLE_EQ(0.58034, ukf.x_[1]);

//  -----------------------------------------
//  Time: 1477010443100000
//  Laser Measurement:  1.17385
//  0.481073
//  Ground Truth:
//     1.6399
//   0.601347
//    5.19839
//  0.0179597
  package = getLidarPackage(1477010443100000, 1.17385, 0.481073);
  ukf.ProcessMeasurement(package);
//  -----------------------------------------
//  Time: 1477010443200000
//  Laser Measurement: 1.65063
//  0.62469
//  Ground Truth:
//     2.1597
//   0.604086
//    5.19678
//  0.0376932
  package = getLidarPackage(1477010443200000, 1.65063, 0.62469);
  ukf.ProcessMeasurement(package);
//  -----------------------------------------
//  Time: 1477010443300000
//  Laser Measurement:  2.18882
//  0.648739
//  Ground Truth:
//    2.67932
//   0.609155
//     5.1945
//  0.0645654
  package = getLidarPackage(1477010443300000, 2.18882, 0.648739);
  ukf.ProcessMeasurement(package);
//  -----------------------------------------
//  Time: 1477010443400000
//  Laser Measurement: 2.65526
//  0.66598
//  Ground Truth:
//    3.19869
//   0.617267
//    5.19147
//  0.0985415
  package = getLidarPackage(1477010443400000, 2.65526, 0.66598);
  ukf.ProcessMeasurement(package);
//  -----------------------------------------
//  Time: 1477010443500000
//  Laser Measurement:  3.01222
//  0.637046
//  Ground Truth:
//   3.71772
//   0.62913
//   5.18754
//  0.139576
//  -----------------------------------------
//  Time: 1477010443600000
//  Laser Measurement:  3.89365
//  0.311793
//  Ground Truth:
//   4.23632
//  0.645449
//   5.18256
//  0.187614
//  -----------------------------------------
//  Time: 1477010443700000
//  Laser Measurement:  4.30935
//  0.578564
//  Ground Truth:
//   4.75437
//  0.666921
//   5.17634
//  0.242587
//  -----------------------------------------
//  Time: 1477010443800000
//  Laser Measurement:  4.35143
//  0.899174
//  Ground Truth:
//   5.27175
//  0.694234
//   5.16867
//  0.304413
//  -----------------------------------------
//  Time: 1477010443900000
//  Laser Measurement:  5.51894
//  0.648233
//  Ground Truth:
//   5.78828
//  0.728072
//   5.15932
//  0.372999
//  -----------------------------------------
//  Time: 1477010444000000
//  Laser Measurement:    6.022
//  0.708619
//  Ground Truth:
//   6.30379
//  0.769103
//   5.14803
//  0.448233
//  -----------------------------------------
//  Time: 1477010444100000
//  Laser Measurement:  6.34249
//  0.948833
//  Ground Truth:
//   6.81808
//  0.817988
//   5.13452
//   0.52999

}
