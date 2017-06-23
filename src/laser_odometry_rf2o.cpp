#include <pluginlib/class_list_macros.h>

#include <laser_odometry_core/laser_odometry_utils.h>
#include <laser_odometry_rf2o/laser_odometry_rf2o.h>

#include <boost/assign/list_of.hpp>

namespace laser_odometry {

OdomType LaserOdometryRf2o::odomType() const noexcept
{
  return OdomType::Odom2D;
}

bool LaserOdometryRf2o::configureImpl()
{
  return true;
}

bool LaserOdometryRf2o::process_impl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                                      const tf::Transform& prediction)
{
  //convert(laser_msg, current_ldp_scan_);


  updateLaserPose();

  return true;
}

/*void LaserOdometryRf2o::convert(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                 LDP& ldp_scan)
{
  unsigned int n = scan_msg->ranges.size();
  ldp_scan = ld_alloc_new(n);

  if (n != theta_.size()) cache(scan_msg);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame
    double r = scan_msg->ranges[i];

    if (r > input_.min_reading && r < input_.max_reading)
    {
      ldp_scan->valid[i]    = 1;
      ldp_scan->readings[i] = r;
    }
    else
    {
      ldp_scan->valid[i]    =  0;
      ldp_scan->readings[i] = -1;  // for invalid range
    }

    ldp_scan->theta[i]    = theta_[i];
    ldp_scan->cluster[i]  = -1;
  }

  ldp_scan->min_theta = *theta_.begin();
  ldp_scan->max_theta = theta_.back();

  ldp_scan->odometry[0] = 0.0;
  ldp_scan->odometry[1] = 0.0;
  ldp_scan->odometry[2] = 0.0;

  ldp_scan->true_pose[0] = 0.0;
  ldp_scan->true_pose[1] = 0.0;
  ldp_scan->true_pose[2] = 0.0;
}*/

bool LaserOdometryRf2o::initialize(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  // convert(scan_msg, prev_scan_);

  return true;
}

bool LaserOdometryRf2o::isKeyFrame(const tf::Transform& tf)
{
  if (fabs(tf::getYaw(tf.getRotation())) > kf_dist_angular_) return true;

  double x = tf.getOrigin().getX();
  double y = tf.getOrigin().getY();

  if (x*x + y*y > kf_dist_linear_sq_) return true;

  return false;
}

void LaserOdometryRf2o::isKeyFrame()
{

}

void LaserOdometryRf2o::isNotKeyFrame()
{

}

} /* namespace laser_odometry */

PLUGINLIB_EXPORT_CLASS(laser_odometry::LaserOdometryRf2o, laser_odometry::LaserOdometryBase);
