#ifndef _LASER_ODOMETRY_RF2O_LASER_ODOMETRY_RF2O_H_
#define _LASER_ODOMETRY_RF2O_LASER_ODOMETRY_RF2O_H_

#include <laser_odometry_core/laser_odometry_core.h>

#include <laser_odometry_csm/LaserOdometryRf2oParameters.h>

namespace laser_odometry
{

  class LaserOdometryRf2o : public LaserOdometryBase
  {
    using Base = LaserOdometryBase;

    using Parameters = laser_odometry_polar::LaserOdometryRf2oParameters;
    using ParametersPtr = std::shared_ptr<Parameters>;

  public:

    LaserOdometryRf2o()  = default;
    ~LaserOdometryRf2o() = default;

    OdomType odomType() const noexcept override;

  protected:

    bool process_impl(const sensor_msgs::LaserScanConstPtr& laser_msg,
                      const tf::Transform& prediction) override;

  protected:

    bool initialized_ = false;

    ParametersPtr params_ptr_;

    //void convert(const sensor_msgs::LaserScanConstPtr& scan_msg,
    //             LDP& ldp_scan);

    bool configureImpl() override;

    bool initialize(const sensor_msgs::LaserScanConstPtr& scan_msg) override;

    void updateLaserPose();

    bool isKeyFrame(const Transform& tf) override;
    void isKeyFrame() override;
    void isNotKeyFrame() override;
  };

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_RF2O_LASER_ODOMETRY_RF2O_H_ */
