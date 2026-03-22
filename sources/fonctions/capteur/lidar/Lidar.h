#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace CATJ_lidar {

struct ScanPoint {
    float angleDeg = 0.0f;
    float distanceMm = 0.0f;
    int quality = 0;
    bool sync = false;
};

struct SectorDistances {
    float frontMm = std::numeric_limits<float>::infinity();
    float frontRightMm = std::numeric_limits<float>::infinity();
    float rightMm = std::numeric_limits<float>::infinity();
    float rearMm = std::numeric_limits<float>::infinity();
    float leftMm = std::numeric_limits<float>::infinity();
    float frontLeftMm = std::numeric_limits<float>::infinity();
    float minDistanceMm = std::numeric_limits<float>::infinity();
};

struct DeviceInfo {
    bool valid = false;
    int model = 0;
    int firmwareMajor = 0;
    int firmwareMinor = 0;
    int hardwareVersion = 0;
    int healthStatus = 0;
    int healthErrorCode = 0;
    std::string serialNumber;
};

struct Snapshot {
    bool connected = false;
    bool scanning = false;
    bool mockMode = false;
    double scanHz = 0.0;
    uint64_t scanId = 0;
    std::size_t sampleCount = 0;
    SectorDistances sectors;
    DeviceInfo device;
    std::vector<ScanPoint> points;
};

struct RplidarConfig {
    std::string port = "/dev/ttyUSB0";
    std::uint32_t baudrate = 460800;
    bool autoStart = true;
    bool useMockData = false;
    int pollTimeoutMs = 120;
    int reconnectDelayMs = 1200;
    int mockPeriodMs = 80;
    float maxDistanceMm = 8000.0f;
    float frontHalfAngleDeg = 20.0f;
    float lateralHalfAngleDeg = 50.0f;
};

class RplidarC1 {
public:
    RplidarC1();
    ~RplidarC1();

    RplidarC1(const RplidarC1&) = delete;
    RplidarC1& operator=(const RplidarC1&) = delete;

    bool open(const RplidarConfig& cfg);
    void close();

    bool startScan();
    void stopScan();

    bool isConnected() const;
    bool isScanning() const;
    bool isMockMode() const;

    Snapshot snapshot() const;
    SectorDistances sectors() const;
    bool collisionImminente(float thresholdMm) const;
    bool obstacleEnApproche(float thresholdMm) const;

private:
    void workerLoop_();
    void runMockLoop_();
    void updateSnapshot_(std::vector<ScanPoint> points, double scanHz, bool connected, bool scanning);
    SectorDistances computeSectors_(const std::vector<ScanPoint>& points) const;
    float sectorMin_(const std::vector<ScanPoint>& points, float centerDeg, float halfWidthDeg) const;

    RplidarConfig cfg_{};

    mutable std::mutex dataMutex_;
    Snapshot snapshot_{};

    std::atomic<bool> workerRunning_{false};
    std::thread worker_;
};

} // namespace CATJ_lidar
