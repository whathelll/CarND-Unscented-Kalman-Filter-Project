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
  package = getRadarPackage(1477010443650000, 4.25555, 0.16484, 5.43333);
  package = getRadarPackage(1477010443750000, 4.67026, 0.14818, 5.12085);
  package = getRadarPackage(1477010443850000, 5.25142, 0.127163, 4.82591);
}
