#include "robot_web_bridge.h"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>

namespace CATJ_robot_web {

namespace {

std::string trimLowerCopy(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
            out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }
    }
    return out;
}

void drawLabelBox(cv::Mat& frame,
    const std::string& text,
    const cv::Point& anchor,
    const cv::Scalar& bg,
    const cv::Scalar& fg = cv::Scalar(255, 255, 255),
    double scale = 0.42,
    int thickness = 1)
{
    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(
        text,
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        thickness,
        &baseline
    );

    const int padX = 6;
    const int padY = 5;

    cv::Rect rect(
        anchor.x,
        anchor.y - textSize.height - 2 * padY,
        textSize.width + 2 * padX,
        textSize.height + 2 * padY
    );

    rect.x = std::max(0, std::min(rect.x, frame.cols - rect.width));
    rect.y = std::max(0, std::min(rect.y, frame.rows - rect.height));

    cv::rectangle(frame, rect, bg, cv::FILLED);
    cv::putText(frame,
        text,
        { rect.x + padX, rect.y + rect.height - padY },
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        fg,
        thickness,
        cv::LINE_AA);
}

} // namespace

RobotWebBridge::RobotWebBridge()
{
    setupWebBindings_();
}

RobotWebBridge::~RobotWebBridge()
{
    stop();
}

bool RobotWebBridge::start(const CATJ_webui_rt::WebUiRtConfig& cfg,
                           const std::string& documentRoot)
{
    CATJ_webui_rt::WebUiRtConfig localCfg = cfg;
    if (!documentRoot.empty()) {
        localCfg.documentRoot = documentRoot;
    }
    return web_.start(localCfg);
}

void RobotWebBridge::stop()
{
    web_.stop();
}

bool RobotWebBridge::isRunning() const
{
    return web_.isRunning();
}

void RobotWebBridge::setControlHandler(ControlHandler cb)
{
    std::lock_guard<std::mutex> lock(queueMutex_);
    controlHandler_ = std::move(cb);
}

bool RobotWebBridge::popControlEvent(ControlEvent& outEvent)
{
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (queue_.empty()) return false;
    outEvent = queue_.front();
    queue_.pop();
    return true;
}

size_t RobotWebBridge::queuedControlCount() const
{
    std::lock_guard<std::mutex> lock(queueMutex_);
    return queue_.size();
}

void RobotWebBridge::updateTelemetry(const RobotTelemetry& telemetry)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    telemetry_ = telemetry;
}

RobotTelemetry RobotWebBridge::telemetry() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return telemetry_;
}

void RobotWebBridge::updateDetections(const std::vector<Detection>& detections)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    detections_ = detections;
}

std::vector<Detection> RobotWebBridge::detections() const
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    return detections_;
}

bool RobotWebBridge::updateVideoFrame(const cv::Mat& frameBgr)
{
    return web_.updateFrame(buildOverlayFrame_(frameBgr));
}

bool RobotWebBridge::updateVideoFrame(const cv::Mat& frameBgr, const std::vector<Detection>& detections)
{
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        detections_ = detections;
    }
    return updateVideoFrame(frameBgr);
}

void RobotWebBridge::setMode(RobotMode mode)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    telemetry_.mode = mode;
    telemetry_.driveControlsLocked = (mode == RobotMode::Auto);
}

void RobotWebBridge::setMissionEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    telemetry_.missionEnabled = enabled;
}

void RobotWebBridge::setOverlayEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    telemetry_.overlayEnabled = enabled;
}

void RobotWebBridge::setStatusText(const std::string& text)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    telemetry_.statusText = text;
}

void RobotWebBridge::setAutoState(const std::string& text)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    telemetry_.autoState = text;
}

void RobotWebBridge::setupWebBindings_()
{
    web_.setTelemetryProvider([this]() {
        return buildTelemetryJson_();
    });

    web_.setCommandHandler([this](const CATJ_webui_rt::WebCommandEvent& ev) {
        handleWebCommand_(ev);
    });

    web_.registerGet("/api/robot_state", [this](const CATJ_webui_rt::HttpRequest&) {
        CATJ_webui_rt::HttpResponse res;
        res.contentType = "application/json; charset=utf-8";
        res.body = buildTelemetryJson_();
        return res;
    });
}

void RobotWebBridge::handleWebCommand_(const CATJ_webui_rt::WebCommandEvent& ev)
{
    ControlEvent out;
    out.sequence = ev.sequence;
    out.remoteIp = ev.remoteIp;
    out.remotePort = ev.remotePort;
    out.rawFields = ev.fields;
    out.rawJson = ev.rawJson;
    out.accepted = true;

    RobotMode currentMode = RobotMode::Manual;
    bool remoteEnabled = true;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        currentMode = telemetry_.mode;
        remoteEnabled = telemetry_.remoteControlEnabled;
    }

    if (!remoteEnabled) 
    {
        out.type = ControlEventType::RawWebCommand;
        out.name = ev.action;
        out.accepted = false;
        emitControlEvent_(std::move(out));
        return;
    }

    if (ev.action == "mode_set") 
    {
        const std::string mode = ev.fields.count("mode") ? trimLowerCopy(ev.fields.at("mode")) : "manual";
        out.type = ControlEventType::SetMode;
        out.name = mode;

        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            telemetry_.mode = (mode == "auto") ? RobotMode::Auto : RobotMode::Manual;
            telemetry_.autoEngaged = (telemetry_.mode == RobotMode::Auto);
            telemetry_.driveControlsLocked = telemetry_.autoEngaged;
            telemetry_.statusText = (telemetry_.mode == RobotMode::Auto)
                ? "auto mode engaged"
                : "manual mode engaged";
        }
    }
    else if (ev.action == "mission") {
        const std::string cmd = ev.fields.count("cmd") ? trimLowerCopy(ev.fields.at("cmd")) : "";
        out.type = ControlEventType::Mission;
        out.name = cmd;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            telemetry_.missionEnabled = (cmd == "start");
            telemetry_.statusText = telemetry_.missionEnabled ? "mission running" : "mission stopped";
        }
    }
    else if (ev.action == "overlay_toggle") {
        const bool enabled = toBool_(ev.fields, "enabled", true);
        out.type = ControlEventType::ToggleOverlay;
        out.valueA = enabled ? 1 : 0;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            telemetry_.overlayEnabled = enabled;
        }
    }
    else if (ev.action == "remote_control") {
        const bool enabled = toBool_(ev.fields, "enabled", true);
        out.type = ControlEventType::SetRemoteControlEnabled;
        out.valueA = enabled ? 1 : 0;
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            telemetry_.remoteControlEnabled = enabled;
            telemetry_.statusText = enabled ? "remote control enabled" : "remote control disabled";
        }
    }
    else if (ev.action == "stop_all") {
        out.type = ControlEventType::StopAll;
        if (currentMode == RobotMode::Auto) {
            out.accepted = false;
        } else {
            std::lock_guard<std::mutex> lock(stateMutex_);
            telemetry_.motors.leftPct = 0;
            telemetry_.motors.rightPct = 0;
            telemetry_.motors.conveyorPct = 0;
            telemetry_.statusText = "all motors stopped";
        }
    }
    else if (ev.action == "drive_preset") {
        out.type = ControlEventType::DrivePreset;
        out.name = ev.fields.count("preset") ? ev.fields.at("preset") : "";
        if (currentMode == RobotMode::Auto) {
            out.accepted = false;
        }
    }
    else if (ev.action == "thrusters") {
        out.type = ControlEventType::SetThrusters;
        out.valueA = std::clamp(toInt_(ev.fields, "left_pct", 0), -100, 100);
        out.valueB = std::clamp(toInt_(ev.fields, "right_pct", 0), -100, 100);
        if (currentMode == RobotMode::Auto) {
            out.accepted = false;
        } else {
            std::lock_guard<std::mutex> lock(stateMutex_);
            telemetry_.motors.leftPct = out.valueA;
            telemetry_.motors.rightPct = out.valueB;
        }
    }
    else if (ev.action == "conveyor") {
        out.type = ControlEventType::SetConveyor;
        out.valueA = std::clamp(toInt_(ev.fields, "pct", 0), -100, 100);
        if (currentMode == RobotMode::Auto) {
            out.accepted = false;
        } else {
            std::lock_guard<std::mutex> lock(stateMutex_);
            telemetry_.motors.conveyorPct = out.valueA;
        }
    }
    else if (ev.action == "camera_step") {
        out.type = ControlEventType::CameraStep;
        out.name = ev.fields.count("axis") ? ev.fields.at("axis") : "pan";
        out.valueA = std::clamp(toInt_(ev.fields, "steps", 0), -50, 50);
        if (currentMode == RobotMode::Auto) {
            out.accepted = false;
        } else {
            std::lock_guard<std::mutex> lock(stateMutex_);
            if (out.name == "pan") {
                telemetry_.motors.camPanDeg = std::clamp(telemetry_.motors.camPanDeg + out.valueA, -90, 90);
            } else {
                telemetry_.motors.camTiltDeg = std::clamp(telemetry_.motors.camTiltDeg + out.valueA, -45, 45);
            }
        }
    }
    else if (ev.action == "camera_center") {
        out.type = ControlEventType::CameraCenter;
        if (currentMode == RobotMode::Auto) {
            out.accepted = false;
        } else {
            std::lock_guard<std::mutex> lock(stateMutex_);
            telemetry_.motors.camPanDeg = 0;
            telemetry_.motors.camTiltDeg = 0;
        }
    }
    else if (ev.action == "camera_setting") {
        out.type = ControlEventType::CameraSetting;
        out.name = ev.fields.count("setting") ? ev.fields.at("setting") : "";
        out.valueA = toInt_(ev.fields, "value", 0);
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (out.name == "expRange") telemetry_.camera.exposure = out.valueA;
        if (out.name == "brightRange") telemetry_.camera.brightness = out.valueA;
        if (out.name == "contrastRange") telemetry_.camera.contrast = out.valueA;
    }
    else {
        out.type = ControlEventType::RawWebCommand;
        out.name = ev.action;
    }

    emitControlEvent_(std::move(out));
}

void RobotWebBridge::emitControlEvent_(ControlEvent ev)
{
    ControlHandler handler;
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        queue_.push(ev);
        handler = controlHandler_;
    }
    if (handler) handler(ev);
}

bool RobotWebBridge::isManualAction_(const std::string& action) const
{
    return action == "thrusters"
        || action == "conveyor"
        || action == "camera_step"
        || action == "camera_center"
        || action == "drive_preset"
        || action == "stop_all";
}

cv::Mat RobotWebBridge::buildOverlayFrame_(const cv::Mat& frameBgr) const
{
    if (frameBgr.empty()) return frameBgr;

    RobotTelemetry telem;
    std::vector<Detection> detectionsLocal;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        telem = telemetry_;
        detectionsLocal = detections_;
    }

    cv::Mat out = frameBgr.clone();
    if (!telem.overlayEnabled) {
        return out;
    }

    const cv::Scalar manualColor(52, 211, 153);
    const cv::Scalar autoColor(59, 130, 246);
    const cv::Scalar warnColor(59, 59, 239);
    const cv::Scalar panelBg(12, 18, 28);

    const bool smallFrame = (out.cols <= 640 || out.rows <= 480);

    const double labelScale = smallFrame ? 0.34 : 0.46;
    const double textScale1 = smallFrame ? 0.42 : 0.58;
    const double textScale2 = smallFrame ? 0.38 : 0.52;

    const int textThickness = 1;
    const int boxThickness = smallFrame ? 1 : 2;
    const int boxThicknessPrimary = smallFrame ? 2 : 3;

    const int margin = 10;
    const int line1Y = smallFrame ? 58 : 78;
    const int line2Y = smallFrame ? 78 : 106;

    // Détections
    for (const auto& det : detectionsLocal) {
        cv::Rect box = det.box & cv::Rect(0, 0, out.cols, out.rows);
        if (box.width <= 1 || box.height <= 1) continue;

        const cv::Scalar color = det.primary ? cv::Scalar(0, 255, 255) : det.color;
        cv::rectangle(out, box, color, det.primary ? boxThicknessPrimary : boxThickness, cv::LINE_AA);

        std::ostringstream label;
        label << det.label << ' '
            << std::fixed << std::setprecision(0)
            << (det.confidence * 100.0) << '%';

        drawLabelBox(out,
            label.str(),
            { box.x, std::max(18, box.y) },
            color,
            cv::Scalar(255, 255, 255),
            labelScale,
            1);

        cv::circle(out,
            { box.x + box.width / 2, box.y + box.height / 2 },
            smallFrame ? 3 : 4,
            color,
            cv::FILLED,
            cv::LINE_AA);
    }

    // Badges compacts
    drawLabelBox(out,
        telem.mode == RobotMode::Auto ? "AUTO" : "MANUAL",
        { margin, 28 },
        telem.mode == RobotMode::Auto ? autoColor : manualColor,
        cv::Scalar(255, 255, 255),
        labelScale,
        1);

    if (telem.missionEnabled) {
        drawLabelBox(out,
            "MISSION",
            { smallFrame ? 95 : 120, 28 },
            cv::Scalar(34, 197, 94),
            cv::Scalar(255, 255, 255),
            labelScale,
            1);
    }

    if (!telem.remoteControlEnabled) {
        drawLabelBox(out,
            "REMOTE OFF",
            { smallFrame ? 175 : 220, 28 },
            warnColor,
            cv::Scalar(255, 255, 255),
            labelScale,
            1);
    }

    // Petit bandeau haut uniquement
    std::ostringstream line1;
    line1 << std::fixed << std::setprecision(0)
        << "HDG " << telem.compass.headingDeg
        << " | MAG " << telem.compass.magneticNorthDeg
        << " | FPS " << std::setprecision(1) << telem.vision.fps;

    std::ostringstream line2;
    line2 << "Target: " << telem.vision.target
        << " (" << std::fixed << std::setprecision(0)
        << (telem.vision.confidence * 100.0) << "%)"
        << " | L " << telem.motors.leftPct
        << " R " << telem.motors.rightPct
        << " C " << telem.motors.conveyorPct;

    int baseline = 0;
    cv::Size s1 = cv::getTextSize(line1.str(), cv::FONT_HERSHEY_SIMPLEX, textScale1, textThickness, &baseline);
    cv::Size s2 = cv::getTextSize(line2.str(), cv::FONT_HERSHEY_SIMPLEX, textScale2, textThickness, &baseline);

    int panelW = std::max(s1.width, s2.width) + 16;
    int panelH = smallFrame ? 52 : 66;
    panelW = std::min(panelW, out.cols - 2 * margin);

    cv::rectangle(out,
        cv::Rect(margin, 34, panelW, panelH),
        panelBg,
        cv::FILLED);

    cv::putText(out,
        line1.str(),
        { margin + 8, line1Y },
        cv::FONT_HERSHEY_SIMPLEX,
        textScale1,
        cv::Scalar(255, 255, 255),
        textThickness,
        cv::LINE_AA);

    cv::putText(out,
        line2.str(),
        { margin + 8, line2Y },
        cv::FONT_HERSHEY_SIMPLEX,
        textScale2,
        cv::Scalar(210, 220, 235),
        textThickness,
        cv::LINE_AA);

    return out;
}

std::string RobotWebBridge::buildTelemetryJson_() const
{
    RobotTelemetry t;
    std::vector<Detection> dets;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        t = telemetry_;
        dets = detections_;
    }

    auto esc = CATJ_webui_rt::WebUiRtServer::jsonEscape;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << '{';
    oss << "\"ok\":true,";
    oss << "\"mode\":\"" << esc(modeToString_(t.mode)) << "\",";
    oss << "\"mission_enabled\":" << jsonBool_(t.missionEnabled) << ',';
    oss << "\"overlay_enabled\":" << jsonBool_(t.overlayEnabled) << ',';
    oss << "\"auto_engaged\":" << jsonBool_(t.autoEngaged) << ',';
    oss << "\"remote_control_enabled\":" << jsonBool_(t.remoteControlEnabled) << ',';
    oss << "\"drive_controls_locked\":" << jsonBool_(t.driveControlsLocked) << ',';
    oss << "\"battery_v\":" << t.batteryV << ',';
    oss << "\"status_text\":\"" << esc(t.statusText) << "\",";
    oss << "\"auto_state\":\"" << esc(t.autoState) << "\",";

    oss << "\"compass\":{";
    oss << "\"heading_deg\":" << t.compass.headingDeg << ',';
    oss << "\"mag_north_deg\":" << t.compass.magneticNorthDeg;
    oss << "},";

    oss << "\"imu\":{";
    oss << "\"roll_deg\":" << t.imu.rollDeg << ',';
    oss << "\"pitch_deg\":" << t.imu.pitchDeg << ',';
    oss << "\"yaw_deg\":" << t.imu.yawDeg << ',';
    oss << "\"yaw_rate_dps\":" << t.imu.yawRateDps << ',';
    oss << "\"gyro_x_dps\":" << t.imu.gyroX_dps << ',';
    oss << "\"gyro_y_dps\":" << t.imu.gyroY_dps << ',';
    oss << "\"gyro_z_dps\":" << t.imu.gyroZ_dps;
    oss << "},";

    oss << "\"motors\":{";
    oss << "\"left_pct\":" << t.motors.leftPct << ',';
    oss << "\"right_pct\":" << t.motors.rightPct << ',';
    oss << "\"conveyor_pct\":" << t.motors.conveyorPct << ',';
    oss << "\"cam_pan_deg\":" << t.motors.camPanDeg << ',';
    oss << "\"cam_tilt_deg\":" << t.motors.camTiltDeg;
    oss << "},";

    oss << "\"camera\":{";
    oss << "\"exposure\":" << t.camera.exposure << ',';
    oss << "\"brightness\":" << t.camera.brightness << ',';
    oss << "\"contrast\":" << t.camera.contrast;
    oss << "},";

    oss << "\"vision\":{";
    oss << "\"fps\":" << t.vision.fps << ',';
    oss << "\"target\":\"" << esc(t.vision.target) << "\",";
    oss << "\"confidence\":" << t.vision.confidence << ',';
    oss << "\"detections\":[";
    for (size_t i = 0; i < dets.size(); ++i) {
        const auto& d = dets[i];
        if (i) oss << ',';
        oss << '{';
        oss << "\"label\":\"" << esc(d.label) << "\",";
        oss << "\"confidence\":" << d.confidence << ',';
        oss << "\"x\":" << d.box.x << ',';
        oss << "\"y\":" << d.box.y << ',';
        oss << "\"w\":" << d.box.width << ',';
        oss << "\"h\":" << d.box.height << ',';
        oss << "\"track_id\":" << d.trackId << ',';
        oss << "\"primary\":" << jsonBool_(d.primary);
        oss << '}';
    }
    oss << ']';
    oss << '}';
    oss << '}';
    return oss.str();
}

std::string RobotWebBridge::modeToString_(RobotMode mode)
{
    return mode == RobotMode::Auto ? "auto" : "manual";
}

int RobotWebBridge::toInt_(const std::unordered_map<std::string, std::string>& fields,
                           const std::string& key,
                           int fallback)
{
    auto it = fields.find(key);
    if (it == fields.end()) return fallback;
    try {
        return std::stoi(it->second);
    } catch (...) {
        return fallback;
    }
}

bool RobotWebBridge::toBool_(const std::unordered_map<std::string, std::string>& fields,
                             const std::string& key,
                             bool fallback)
{
    auto it = fields.find(key);
    if (it == fields.end()) return fallback;
    const std::string v = trimLowerCopy(it->second);
    if (v == "1" || v == "true" || v == "on" || v == "yes") return true;
    if (v == "0" || v == "false" || v == "off" || v == "no") return false;
    return fallback;
}

} // namespace CATJ_robot_web
