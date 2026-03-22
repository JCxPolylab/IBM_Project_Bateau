#include "Lidar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

#if defined(__has_include)
#  if __has_include("sl_lidar.h") && __has_include("sl_lidar_driver.h")
#    include "sl_lidar.h"
#    include "sl_lidar_driver.h"
#    define CATJ_RPLIDAR_SDK_AVAILABLE 1
#  endif
#endif

namespace CATJ_lidar {

namespace {

constexpr float kInfinity = std::numeric_limits<float>::infinity();

float normalizeAngleDeg(float a)
{
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

float angularDiffDeg(float a, float b)
{
    float d = std::fabs(normalizeAngleDeg(a) - normalizeAngleDeg(b));
    if (d > 180.0f) d = 360.0f - d;
    return d;
}

std::string serialToString(const unsigned char* serial, int len)
{
    std::ostringstream oss;
    oss << std::hex << std::uppercase << std::setfill('0');
    for (int i = 0; i < len; ++i) {
        oss << std::setw(2) << static_cast<int>(serial[i]);
    }
    return oss.str();
}

} // namespace

RplidarC1::RplidarC1() = default;

RplidarC1::~RplidarC1()
{
    close();
}

bool RplidarC1::open(const RplidarConfig& cfg)
{
    close();
    cfg_ = cfg;

    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        snapshot_ = Snapshot{};
        snapshot_.mockMode = cfg_.useMockData;
    }

    if (cfg_.autoStart) {
        return startScan();
    }
    return true;
}

void RplidarC1::close()
{
    stopScan();
    std::lock_guard<std::mutex> lock(dataMutex_);
    snapshot_ = Snapshot{};
}

bool RplidarC1::startScan()
{
    if (workerRunning_.load()) return true;
    workerRunning_.store(true);
    worker_ = std::thread(&RplidarC1::workerLoop_, this);
    return true;
}

void RplidarC1::stopScan()
{
    workerRunning_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
    std::lock_guard<std::mutex> lock(dataMutex_);
    snapshot_.scanning = false;
    snapshot_.connected = false;
}

bool RplidarC1::isConnected() const
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return snapshot_.connected;
}

bool RplidarC1::isScanning() const
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return snapshot_.scanning;
}

bool RplidarC1::isMockMode() const
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return snapshot_.mockMode;
}

Snapshot RplidarC1::snapshot() const
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return snapshot_;
}

SectorDistances RplidarC1::sectors() const
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    return snapshot_.sectors;
}

bool RplidarC1::collisionImminente(float thresholdMm) const
{
    const auto s = sectors();
    return std::isfinite(s.frontMm) && s.frontMm < thresholdMm;
}

bool RplidarC1::obstacleEnApproche(float thresholdMm) const
{
    const auto s = sectors();
    return std::isfinite(s.frontMm) && s.frontMm < thresholdMm;
}

void RplidarC1::workerLoop_()
{
#if defined(CATJ_RPLIDAR_SDK_AVAILABLE)
    if (!cfg_.useMockData) {
        using namespace sl;

        while (workerRunning_.load()) {
            IChannel* channel = nullptr;
            ILidarDriver* drv = nullptr;

            try {
                channel = *createSerialPortChannel(cfg_.port.c_str(), cfg_.baudrate);
                drv = *createLidarDriver();
            } catch (...) {
                channel = nullptr;
                drv = nullptr;
            }

            if (!channel || !drv) {
                std::cerr << "[LIDAR] Impossible d'initialiser le SDK SLAMTEC, bascule en mock." << std::endl;
                cfg_.useMockData = true;
                runMockLoop_();
                return;
            }

            auto res = drv->connect(channel);
            if (SL_IS_FAIL(res)) {
                {
                    std::lock_guard<std::mutex> lock(dataMutex_);
                    snapshot_.connected = false;
                    snapshot_.scanning = false;
                    snapshot_.mockMode = false;
                }
                delete drv;
                delete channel;
                std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnectDelayMs));
                continue;
            }

            sl_lidar_response_device_info_t devinfo{};
            sl_lidar_response_device_health_t health{};

            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                snapshot_.connected = true;
                snapshot_.scanning = false;
                snapshot_.mockMode = false;
            }

            if (SL_IS_OK(drv->getDeviceInfo(devinfo))) {
                std::lock_guard<std::mutex> lock(dataMutex_);
                snapshot_.device.valid = true;
                snapshot_.device.model = devinfo.model;
                snapshot_.device.firmwareMajor = devinfo.firmware_version >> 8;
                snapshot_.device.firmwareMinor = devinfo.firmware_version & 0xFFu;
                snapshot_.device.hardwareVersion = devinfo.hardware_version;
                snapshot_.device.serialNumber = serialToString(devinfo.serialnum, 16);
            }

            if (SL_IS_OK(drv->getHealth(health))) {
                std::lock_guard<std::mutex> lock(dataMutex_);
                snapshot_.device.healthStatus = health.status;
                snapshot_.device.healthErrorCode = health.error_code;
            }

            drv->setMotorSpeed();
            res = drv->startScan(0, 1);
            if (SL_IS_FAIL(res)) {
                std::cerr << "[LIDAR] startScan a échoué, tentative de reconnexion." << std::endl;
                drv->setMotorSpeed(0);
                delete drv;
                delete channel;
                std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnectDelayMs));
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                snapshot_.scanning = true;
            }

            auto lastTs = std::chrono::steady_clock::now();
            while (workerRunning_.load()) {
                sl_lidar_response_measurement_node_hq_t nodes[8192];
                std::size_t count = sizeof(nodes) / sizeof(nodes[0]);
                const auto ans = drv->grabScanDataHq(nodes, count);
                if (SL_IS_FAIL(ans) && ans != SL_RESULT_OPERATION_TIMEOUT) {
                    break;
                }

                drv->ascendScanData(nodes, count);
                std::vector<ScanPoint> points;
                points.reserve(count);
                for (std::size_t i = 0; i < count; ++i) {
                    const float distanceMm = nodes[i].dist_mm_q2 / 4.0f;
                    if (distanceMm <= 1.0f || distanceMm > cfg_.maxDistanceMm) continue;

                    ScanPoint p;
                    p.angleDeg = nodes[i].angle_z_q14 * 90.0f / 16384.0f;
                    p.distanceMm = distanceMm;
                    p.quality = static_cast<int>(nodes[i].quality);
                    p.sync = (nodes[i].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) != 0;
                    points.push_back(p);
                }

                const auto now = std::chrono::steady_clock::now();
                const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTs).count() / 1000.0;
                lastTs = now;
                const double scanHz = (dt > 1e-6) ? (1.0 / dt) : 0.0;
                updateSnapshot_(std::move(points), scanHz, true, true);
            }

            drv->stop();
            drv->setMotorSpeed(0);
            delete drv;
            delete channel;

            {
                std::lock_guard<std::mutex> lock(dataMutex_);
                snapshot_.connected = false;
                snapshot_.scanning = false;
            }

            if (workerRunning_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnectDelayMs));
            }
        }
        return;
    }
#endif

    runMockLoop_();
}

void RplidarC1::runMockLoop_()
{
    std::uint64_t tick = 0;
    auto lastTs = std::chrono::steady_clock::now();

    while (workerRunning_.load()) {
        std::vector<ScanPoint> points;
        points.reserve(360);

        const float phase = static_cast<float>(tick % 360);
        for (int a = 0; a < 360; ++a) {
            float d = 2400.0f;

            // obstacle avant oscillant
            if (angularDiffDeg(static_cast<float>(a), 0.0f) < 16.0f) {
                d = 450.0f + 160.0f * std::sin((phase + a) * 0.04f);
            }
            // mur latéral droit plus proche
            else if (angularDiffDeg(static_cast<float>(a), 90.0f) < 35.0f) {
                d = 900.0f + 60.0f * std::sin((phase + a) * 0.03f);
            }
            // obstacle à gauche périodique
            else if (angularDiffDeg(static_cast<float>(a), 285.0f) < 18.0f) {
                d = 1300.0f + 120.0f * std::cos((phase + a) * 0.05f);
            }

            ScanPoint p;
            p.angleDeg = static_cast<float>(a);
            p.distanceMm = std::clamp(d, 120.0f, cfg_.maxDistanceMm);
            p.quality = 90;
            p.sync = (a == 0);
            points.push_back(p);
        }

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTs).count() / 1000.0;
        lastTs = now;
        const double scanHz = (dt > 1e-6) ? (1.0 / dt) : 0.0;

        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            snapshot_.mockMode = true;
            snapshot_.device.valid = false;
        }

        updateSnapshot_(std::move(points), scanHz, true, true);
        ++tick;
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.mockPeriodMs));
    }
}

void RplidarC1::updateSnapshot_(std::vector<ScanPoint> points, double scanHz, bool connected, bool scanning)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    snapshot_.connected = connected;
    snapshot_.scanning = scanning;
    snapshot_.scanHz = scanHz;
    snapshot_.sampleCount = points.size();
    snapshot_.sectors = computeSectors_(points);
    snapshot_.points = std::move(points);
    ++snapshot_.scanId;
}

SectorDistances RplidarC1::computeSectors_(const std::vector<ScanPoint>& points) const
{
    SectorDistances out;
    out.frontMm = sectorMin_(points, 0.0f, cfg_.frontHalfAngleDeg);
    out.frontRightMm = sectorMin_(points, 45.0f, cfg_.frontHalfAngleDeg);
    out.rightMm = sectorMin_(points, 90.0f, cfg_.lateralHalfAngleDeg);
    out.rearMm = sectorMin_(points, 180.0f, cfg_.frontHalfAngleDeg);
    out.leftMm = sectorMin_(points, 270.0f, cfg_.lateralHalfAngleDeg);
    out.frontLeftMm = sectorMin_(points, 315.0f, cfg_.frontHalfAngleDeg);
    out.minDistanceMm = kInfinity;
    for (const auto& p : points) {
        if (p.distanceMm > 1.0f) {
            out.minDistanceMm = std::min(out.minDistanceMm, p.distanceMm);
        }
    }
    return out;
}

float RplidarC1::sectorMin_(const std::vector<ScanPoint>& points, float centerDeg, float halfWidthDeg) const
{
    float out = kInfinity;
    for (const auto& p : points) {
        if (p.distanceMm <= 1.0f) continue;
        if (angularDiffDeg(p.angleDeg, centerDeg) <= halfWidthDeg) {
            out = std::min(out, p.distanceMm);
        }
    }
    return out;
}

} // namespace CATJ_lidar
