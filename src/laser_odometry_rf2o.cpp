#include <rf2o_laser_odometry/CLaserOdometry2D.h>

#include <laser_odometry_core/laser_odometry_utils.h>
#include <laser_odometry_rf2o/laser_odometry_rf2o.h>

#include <pluginlib/class_list_macros.h>

namespace laser_odometry {

OdomType LaserOdometryRf2o::odomType() const noexcept
{
  return OdomType::Odom2DCov;
}

bool LaserOdometryRf2o::configureImpl()
{
  return true;
}

bool LaserOdometryRf2o::process_impl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                     const Transform& /*prediction*/)
{
  rf2o_.odometryCalculation(*laser_msg);

  increment_ = rf2o_.getIncrement().cast<Scalar>();

  increment_covariance_ = utils::covariance2dTo3d(rf2o_.getIncrementCovariance().cast<Scalar>());

  return true;
}

bool LaserOdometryRf2o::initialize(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  geometry_msgs::Pose identity;

  rf2o_.init(*scan_msg, identity);

  return rf2o_.is_initialized();
}

} /* namespace laser_odometry */

PLUGINLIB_EXPORT_CLASS(laser_odometry::LaserOdometryRf2o, laser_odometry::LaserOdometryBase);
