// MotionDistortionCorrector against an independent forward model of a moving sensor.
//
// The forward model measures a static point p0 (sensor frame at the scan start) at sweep time
// tau as p_tau = R(-omega tau) (p0 - d(tau)), with d(tau) = integral_0^tau R(omega s) v_s ds
// evaluated numerically. The corrector must invert it.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include <Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "lidar_cone_detector/motion_distortion.hpp"

#ifdef LIDAR_GPU_TEST
#include "lidar_cone_detector/cuda_cone_pipeline.hpp"
#endif

namespace {

constexpr double kPeriod = 0.1;
constexpr double kStart = -M_PI;
constexpr double kSpeed = 10.0;
constexpr int kRingPoints = 720;
constexpr double kRingRadius = 8.0;
constexpr double kTolM = 1e-3;

perception::SweepModel sweepModel(double sensor_x = 0.0, double sensor_y = 0.0) {
    perception::SweepModel m;
    m.period_s = kPeriod;
    m.start_azimuth_rad = kStart;
    m.counter_clockwise = true;
    m.sensor_x_m = sensor_x;
    m.sensor_y_m = sensor_y;
    return m;
}

struct Twist {
    double vx, vy, omega, sensor_x, sensor_y;
};

Eigen::Matrix2d rot(double a) {
    Eigen::Matrix2d r;
    r << std::cos(a), -std::sin(a),
         std::sin(a),  std::cos(a);
    return r;
}

// Sensor-frame point p0 at the scan start, as measured at sweep time tau. Height is unchanged.
Eigen::Vector3d measure(const Eigen::Vector3d& p0, double tau, const Twist& tw) {
    const Eigen::Vector2d v(tw.vx - tw.omega * tw.sensor_y, tw.vy + tw.omega * tw.sensor_x);
    const int steps = 4000;
    const double h = tau / steps;
    Eigen::Vector2d d = Eigen::Vector2d::Zero();
    for (int k = 0; k < steps; ++k) {
        const double s = (k + 0.5) * h;
        d += h * rot(tw.omega * s) * v;
    }
    const Eigen::Vector2d xy = rot(-tw.omega * tau) * (p0.head<2>() - d);
    return Eigen::Vector3d(xy.x(), xy.y(), p0.z());
}

// Sweep time of a measured point from its azimuth, as MotionDistortionCorrector does without
// a time field.
double azimuthTime(const Eigen::Vector3d& p) {
    const double swept = std::fmod(std::atan2(p.y(), p.x()) - kStart, 2.0 * M_PI);
    return (swept < 0.0 ? swept + 2.0 * M_PI : swept) / (2.0 * M_PI) * kPeriod;
}

// Static scene in the scan-start frame, measured by a sensor that moves with the twist.
// withTime: the sweep time of each point is known (time field). Otherwise the sweep time is
// the azimuth of the measured point, solved as a fixed point (its azimuth in the sensor frame
// at its time is its sweep angle).
struct Scene {
    std::vector<Eigen::Vector3d> instant;   // points in the scan-start frame
    std::vector<Eigen::Vector3d> measured;  // points as the moving sensor reports them
    std::vector<double> times;
};

Scene sceneOf(const std::vector<Eigen::Vector3d>& instant, const Twist& tw, bool withTime) {
    Scene sc;
    for (const auto& p0 : instant) {
        double tau = azimuthTime(p0);
        Eigen::Vector3d m = measure(p0, tau, tw);
        if (!withTime) {
            for (int it = 0; it < 20; ++it) {
                tau = azimuthTime(m);
                m = measure(p0, tau, tw);
            }
        }
        sc.instant.push_back(p0);
        sc.measured.push_back(m);
        sc.times.push_back(tau);
    }
    return sc;
}

// Ring of cone-top points at 8 m, height 0.5 m.
std::vector<Eigen::Vector3d> ringPoints() {
    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i < kRingPoints; ++i) {
        const double az = kStart + (i + 0.5) * 2.0 * M_PI / kRingPoints;
        pts.emplace_back(kRingRadius * std::cos(az), kRingRadius * std::sin(az), 0.5);
    }
    return pts;
}

// Cloud with x, y, z (float32) and optionally a float32 "time" field, as the Velodyne driver writes.
sensor_msgs::msg::PointCloud2 makeCloud(const Scene& sc, bool withTime) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(sc.measured.size());
    const uint32_t step = withTime ? 16 : 12;
    auto field = [](const char* name, uint32_t offset) {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count = 1;
        return f;
    };
    msg.fields = {field("x", 0), field("y", 4), field("z", 8)};
    if (withTime) msg.fields.push_back(field("time", 12));
    msg.point_step = step;
    msg.row_step = step * msg.width;
    msg.is_dense = true;
    msg.data.resize(static_cast<size_t>(step) * msg.width);
    for (size_t i = 0; i < sc.measured.size(); ++i) {
        const float vals[4] = {static_cast<float>(sc.measured[i].x()),
                               static_cast<float>(sc.measured[i].y()),
                               static_cast<float>(sc.measured[i].z()),
                               static_cast<float>(sc.times[i])};
        std::memcpy(&msg.data[i * step], vals, step);
    }
    return msg;
}

void expectRecoversScene(const Scene& sc, const pcl::PointCloud<pcl::PointXYZ>& out) {
    ASSERT_EQ(out.size(), sc.instant.size());
    double worst = 0.0;
    for (size_t i = 0; i < out.size(); ++i) {
        const double ex = out[i].x - sc.instant[i].x();
        const double ey = out[i].y - sc.instant[i].y();
        worst = std::max(worst, std::hypot(ex, ey));
    }
    EXPECT_LT(worst, kTolM) << "worst point error (m)";
}

}  // namespace

TEST(MotionDistortionCorrector, ZeroMotionLeavesPointsUnchanged) {
    const Twist still{0.0, 0.0, 0.0, 0.0, 0.0};
    const Scene sc = sceneOf(ringPoints(), still, true);
    perception::MotionDistortionCorrector corrector(sweepModel());
    corrector.setEgoMotion(0.0, 0.0, 0.0);
    expectRecoversScene(sc, *corrector.correct(makeCloud(sc, true)));
}

TEST(MotionDistortionCorrector, TimeFieldRecoversStraightLineSweep) {
    const Twist tw{kSpeed, 0.0, 0.0, 0.0, 0.0};
    const Scene sc = sceneOf(ringPoints(), tw, true);
    perception::MotionDistortionCorrector corrector(sweepModel());
    corrector.setEgoMotion(tw.vx, tw.vy, tw.omega);
    expectRecoversScene(sc, *corrector.correct(makeCloud(sc, true)));
}

TEST(MotionDistortionCorrector, TimeFieldRecoversYawingSweepWithLeverArm) {
    const Twist tw{kSpeed, 0.0, 0.3, -0.15, 0.0};
    const Scene sc = sceneOf(ringPoints(), tw, true);
    perception::MotionDistortionCorrector corrector(sweepModel(-0.15, 0.0));
    corrector.setEgoMotion(tw.vx, tw.vy, tw.omega);
    expectRecoversScene(sc, *corrector.correct(makeCloud(sc, true)));
}

TEST(MotionDistortionCorrector, AzimuthFallbackRecoversStraightLineSweep) {
    const Twist tw{kSpeed, 0.0, 0.0, 0.0, 0.0};
    const Scene sc = sceneOf(ringPoints(), tw, false);
    perception::MotionDistortionCorrector corrector(sweepModel());
    corrector.setEgoMotion(tw.vx, tw.vy, tw.omega);
    expectRecoversScene(sc, *corrector.correct(makeCloud(sc, false)));
}

TEST(MotionDistortionCorrector, AzimuthFallbackRecoversYawingSweep) {
    const Twist tw{kSpeed, 0.0, 0.3, -0.15, 0.0};
    const Scene sc = sceneOf(ringPoints(), tw, false);
    perception::MotionDistortionCorrector corrector(sweepModel(-0.15, 0.0));
    corrector.setEgoMotion(tw.vx, tw.vy, tw.omega);
    expectRecoversScene(sc, *corrector.correct(makeCloud(sc, false)));
}

TEST(MotionDistortionCorrector, NonFinitePointsAreDropped) {
    const Twist tw{kSpeed, 0.0, 0.3, 0.0, 0.0};
    const Scene sc = sceneOf(ringPoints(), tw, true);
    const float nan = std::nanf("");
    auto msg = makeCloud(sc, true);
    const float bad[4] = {nan, nan, nan, nan};
    msg.data.insert(msg.data.begin() + 16, reinterpret_cast<const uint8_t*>(bad),
                    reinterpret_cast<const uint8_t*>(bad) + 16);
    msg.width += 1;
    msg.row_step += 16;
    perception::MotionDistortionCorrector corrector(sweepModel());
    corrector.setEgoMotion(tw.vx, tw.vy, tw.omega);
    expectRecoversScene(sc, *corrector.correct(msg));
}

#ifdef LIDAR_GPU_TEST
// The GPU raw path (motion correction on the device, then the shared pipeline) must match the
// CPU path on the same cloud. Both feed the same downstream stages, so only motion differs.
TEST(GpuMotionCorrection, MatchesCpuWithinOneMillimetre) {
    const Twist tw{kSpeed, 0.0, 0.3, -0.15, 0.0};
    // Ground grid at z = 0 around the ring, so RANSAC fits the ground and the ring is the band.
    std::vector<Eigen::Vector3d> pts = ringPoints();
    for (double x = -6.0; x <= 6.0; x += 0.25)
        for (double y = -6.0; y <= 6.0; y += 0.25) pts.emplace_back(x, y, 0.0);
    const Scene sc = sceneOf(pts, tw, true);
    const auto msg = makeCloud(sc, true);

    GpuMotion mo;
    mo.enable = true;
    mo.vx = static_cast<float>(tw.vx);
    mo.vy = static_cast<float>(tw.vy);
    mo.omega = static_cast<float>(tw.omega);
    mo.sensor_x = static_cast<float>(tw.sensor_x);
    mo.scan_period = static_cast<float>(kPeriod);
    mo.start_azimuth = static_cast<float>(kStart);
    mo.counter_clockwise = true;
    mo.time_offset = 12;

    GpuConeParams gp;
    gp.roi_x_min = -100.0f;
    gp.roi_x_max = 100.0f;
    gp.y_max = 100.0f;

    GpuConePipeline gpu;
    GpuConeResult gpu_res;
    gpu.runRaw(msg.data.data(), msg.width, msg.point_step, 0, 4, 8, mo, gp, gpu_res);

    perception::MotionDistortionCorrector corrector(sweepModel(tw.sensor_x, tw.sensor_y));
    corrector.setEgoMotion(tw.vx, tw.vy, tw.omega);
    const auto cpu_cloud = corrector.correct(msg);
    std::vector<float> cpu_xyzp;
    for (const auto& p : cpu_cloud->points) {
        cpu_xyzp.insert(cpu_xyzp.end(), {p.x, p.y, p.z, 0.0f});
    }
    GpuConeResult cpu_res;
    gpu.run(cpu_xyzp.data(), static_cast<unsigned>(cpu_cloud->size()), gp, cpu_res);

    ASSERT_FALSE(cpu_res.points.empty());
    ASSERT_EQ(gpu_res.points.size(), cpu_res.points.size());
    double worst = 0.0;
    for (size_t i = 0; i < gpu_res.points.size(); ++i) {
        worst = std::max(worst, static_cast<double>(std::abs(gpu_res.points[i] - cpu_res.points[i])));
    }
    EXPECT_LT(worst, kTolM) << "worst GPU/CPU coordinate difference (m)";
}
#endif
