#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>

namespace perception {

// How the LiDAR sweeps. Used only when the cloud has no per-point "time" field.
struct SweepModel {
    double period_s = 0.1;                       // one rotation
    double start_azimuth_rad = 1.5707963267948966;  // atan2(y, x) of the first point; pi/2 for the Velodyne driver
    bool counter_clockwise = false;              // direction of increasing atan2(y, x) over the sweep
    double sensor_x_m = 0.0;                     // LiDAR position in base_footprint (lever arm)
    double sensor_y_m = 0.0;
};

// Corrects one sweep to the sensor pose at the scan stamp (the sweep start).
//
// Each point is measured at sweep time tau (s after the scan stamp), from the cloud's "time"
// field when present, else from its azimuth via SweepModel. The Velodyne ROS 2 driver writes
// that field as FLOAT32 seconds relative to the scan start (velodyne_pointcloud rawdata.cpp:
// time_diff_start_to_this_packet + timing offset), and stamps the cloud with the scan start.
//
// The body twist (vx, vy, omega) is constant over the sweep, in base_footprint. With
// v_s = (vx - omega*y_s, vy + omega*x_s) the sensor velocity in its own frame, and
// Delta = t_ref - tau = -tau (t_ref = 0), the sensor displacement over Delta is
//     theta = omega * Delta
//     d(Delta) = integral_0^Delta R(omega*s) v_s ds = (a*u - b*w, b*u + a*w)
//     a = sin(theta) / omega,  b = (1 - cos(theta)) / omega   (a = Delta, b = 0 at omega = 0)
// with (u, w) = v_s, and the point in the scan-start frame is
//     p_ref = R(-theta) (p - d(Delta)).
class MotionDistortionCorrector {
public:
    explicit MotionDistortionCorrector(const SweepModel& model = SweepModel())
        : model_(model) {}

    void setEgoMotion(double vx, double vy, double omega) {
        vx_ = vx;
        vy_ = vy;
        omega_ = omega;
    }

    // Non-finite points are dropped, as the NaN strip they replace did.
    pcl::PointCloud<pcl::PointXYZ>::Ptr correct(const sensor_msgs::msg::PointCloud2& cloud) const {
        pcl::PointCloud<pcl::PointXYZ> in;
        pcl::fromROSMsg(cloud, in);
        const int time_offset = floatFieldOffset(cloud, "time");

        auto corrected = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        corrected->reserve(in.size());
        for (size_t i = 0; i < in.size(); ++i) {
            const auto& pt = in[i];
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

            const double tau = time_offset >= 0
                ? readFloat(cloud, i, static_cast<size_t>(time_offset))
                : sweepTime(std::atan2(pt.y, pt.x));
            if (!std::isfinite(tau)) continue;

            const Eigen::Vector3d p = correctPoint(pt, tau);
            corrected->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
        }
        return corrected;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kSeamTol = 1e-6;     // rad; far below the VLP-16 azimuth step
    static constexpr double kYawRateEps = 1e-9;  // rad/s

    SweepModel model_;
    double vx_ = 0.0, vy_ = 0.0, omega_ = 0.0;

    // Sweep time in [0, period) from the azimuth. Rays on the start edge a rounding error
    // below it stay at time 0.
    double sweepTime(double azimuth) const {
        const double delta = model_.counter_clockwise ? azimuth - model_.start_azimuth_rad
                                                      : model_.start_azimuth_rad - azimuth;
        double swept = std::fmod(delta + kSeamTol, 2.0 * kPi);
        if (swept < 0.0) swept += 2.0 * kPi;
        return std::max(swept - kSeamTol, 0.0) / (2.0 * kPi) * model_.period_s;
    }

    Eigen::Vector3d correctPoint(const pcl::PointXYZ& pt, double tau) const {
        const double delta = -tau;  // t_ref - tau
        const double theta = omega_ * delta;
        double a = delta;
        double b = 0.0;
        if (std::abs(omega_) >= kYawRateEps) {
            a = std::sin(theta) / omega_;
            b = (1.0 - std::cos(theta)) / omega_;
        }

        const double u = vx_ - omega_ * model_.sensor_y_m;
        const double w = vy_ + omega_ * model_.sensor_x_m;
        const double dx = a * u - b * w;
        const double dy = b * u + a * w;

        const double qx = pt.x - dx;
        const double qy = pt.y - dy;
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        // R(-theta) = [[c, s], [-s, c]]
        return Eigen::Vector3d(c * qx + s * qy, -s * qx + c * qy, pt.z);
    }

    // Byte offset of a FLOAT32 field with one element, or -1.
    static int floatFieldOffset(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
        for (const auto& f : cloud.fields)
            if (f.name == name && f.datatype == sensor_msgs::msg::PointField::FLOAT32 && f.count == 1)
                return static_cast<int>(f.offset);
        return -1;
    }

    static double readFloat(const sensor_msgs::msg::PointCloud2& cloud, size_t point, size_t offset) {
        float value;
        std::memcpy(&value, &cloud.data[point * cloud.point_step + offset], sizeof(value));
        return value;
    }
};

} // namespace perception
