#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace perception {

class MotionDistortionCorrector {
public:
    MotionDistortionCorrector(double vx = 0.0, double vy = 0.0, double omega = 0.0)
        : vx_(vx), vy_(vy), omega_(omega) {}

    void setEgoMotion(double vx, double vy, double omega) {
        vx_ = vx;
        vy_ = vy;
        omega_ = omega;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr correctMotionDistortion(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
        double scan_duration_sec = 0.1) const {

        auto corrected = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        corrected->reserve(cloud->size());

        if (cloud->empty()) return corrected;

        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& pt = cloud->at(i);

            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                corrected->push_back(pt);
                continue;
            }

            double t_frac = static_cast<double>(i) / cloud->size();
            double t = t_frac * scan_duration_sec;

            Eigen::Vector3d p_ego(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_corrected = correctPointMotion(p_ego, t, scan_duration_sec);

            pcl::PointXYZ corrected_pt;
            corrected_pt.x = p_corrected.x();
            corrected_pt.y = p_corrected.y();
            corrected_pt.z = p_corrected.z();
            corrected->push_back(corrected_pt);
        }

        return corrected;
    }

private:
    double vx_, vy_, omega_;

    Eigen::Vector3d correctPointMotion(const Eigen::Vector3d& p_scan,
                                       double t_point,
                                       double t_end) const {
        double dt = t_end - t_point;

        double cos_theta = std::cos(omega_ * dt);
        double sin_theta = std::sin(omega_ * dt);

        Eigen::Matrix3d R;
        R << cos_theta, -sin_theta, 0,
             sin_theta,  cos_theta, 0,
             0,          0,         1;

        Eigen::Vector3d trans(vx_ * dt, vy_ * dt, 0);
        return R * p_scan + trans;
    }
};

} // namespace perception
