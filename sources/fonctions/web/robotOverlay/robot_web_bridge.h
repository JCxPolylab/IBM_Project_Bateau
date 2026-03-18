#pragma once

#include "../webui/webui_rt.h"

#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

namespace CATJ_robot_web {

enum class RobotMode {
    Manual,
    Auto
};

struct Detection {
    cv::Rect box;
    std::string label;
    double confidence = 0.0;
    cv::Scalar color = cv::Scalar(0, 255, 0);
    int trackId = -1;
    bool primary = false;
};

struct ImuTelemetry {
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double yawDeg = 0.0;
    double yawRateDps = 0.0;
    double gyroX_dps = 0.0;
    double gyroY_dps = 0.0;
    double gyroZ_dps = 0.0;
};

struct CompassTelemetry {
    double headingDeg = 0.0;
    double magneticNorthDeg = 0.0;
};

struct MotorTelemetry {
    int leftPct = 0;
    int rightPct = 0;
    int conveyorPct = 0;
    int camPanDeg = 0;
    int camTiltDeg = 0;
};

struct CameraTelemetry {
    int exposure = 0;
    int brightness = 50;
    int contrast = 50;
};

struct VisionTelemetry {
    double fps = 0.0;
    std::string target = "none";
    double confidence = 0.0;
};

struct RobotTelemetry {
    RobotMode mode = RobotMode::Manual;
    bool missionEnabled = false;
    bool overlayEnabled = true;
    bool autoEngaged = false;
    bool remoteControlEnabled = true;
    bool driveControlsLocked = false;
    double batteryV = 0.0;
    std::string autoState = "idle";
    std::string statusText = "ready";
    ImuTelemetry imu;
    CompassTelemetry compass;
    MotorTelemetry motors;
    CameraTelemetry camera;
    VisionTelemetry vision;
};

enum class ControlEventType {
    SetMode,
    Mission,
    SetThrusters,
    SetConveyor,
    CameraStep,
    CameraCenter,
    CameraSetting,
    ToggleOverlay,
    StopAll,
    DrivePreset,
    SetRemoteControlEnabled,
    RawWebCommand
};

struct ControlEvent {
    uint64_t sequence = 0;
    std::string remoteIp;
    uint16_t remotePort = 0;
    ControlEventType type = ControlEventType::RawWebCommand;
    std::string name;
    std::string value;
    int valueA = 0;
    int valueB = 0;
    bool accepted = true;
    std::unordered_map<std::string, std::string> rawFields;
    std::string rawJson;
};

using ControlHandler = std::function<void(const ControlEvent&)>;

class RobotWebBridge {
public:
    RobotWebBridge();
    ~RobotWebBridge();

    RobotWebBridge(const RobotWebBridge&) = delete;
    RobotWebBridge& operator=(const RobotWebBridge&) = delete;

    bool start(const CATJ_webui_rt::WebUiRtConfig& cfg,
               const std::string& documentRoot = "./robot_web_demo");
    void stop();
    bool isRunning() const;

    CATJ_webui_rt::WebUiRtServer& server() { return web_; }
    const CATJ_webui_rt::WebUiRtServer& server() const { return web_; }

    void setControlHandler(ControlHandler cb);
    bool popControlEvent(ControlEvent& outEvent);
    size_t queuedControlCount() const;

    void updateTelemetry(const RobotTelemetry& telemetry);
    RobotTelemetry telemetry() const;

    void updateDetections(const std::vector<Detection>& detections);
    std::vector<Detection> detections() const;

    bool updateVideoFrame(const cv::Mat& frameBgr);
    bool updateVideoFrame(const cv::Mat& frameBgr, const std::vector<Detection>& detections);

    void setMode(RobotMode mode);
    void setMissionEnabled(bool enabled);
    void setOverlayEnabled(bool enabled);
    void setStatusText(const std::string& text);
    void setAutoState(const std::string& text);

private:
    CATJ_webui_rt::WebUiRtServer web_;

    mutable std::mutex stateMutex_;
    RobotTelemetry telemetry_{};
    std::vector<Detection> detections_;

    mutable std::mutex queueMutex_;
    std::queue<ControlEvent> queue_;
    ControlHandler controlHandler_;

    void setupWebBindings_();
    void handleWebCommand_(const CATJ_webui_rt::WebCommandEvent& ev);
    void emitControlEvent_(ControlEvent ev);
    bool isManualAction_(const std::string& action) const;

    cv::Mat buildOverlayFrame_(const cv::Mat& frameBgr) const;
    std::string buildTelemetryJson_() const;

    static std::string modeToString_(RobotMode mode);
    static std::string jsonBool_(bool v) { return v ? "true" : "false"; }
    static int toInt_(const std::unordered_map<std::string, std::string>& fields,
                      const std::string& key,
                      int fallback = 0);
    static bool toBool_(const std::unordered_map<std::string, std::string>& fields,
                        const std::string& key,
                        bool fallback = false);
};

} // namespace CATJ_robot_web
