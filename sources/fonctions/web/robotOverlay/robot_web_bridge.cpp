#include "robot_web_bridge.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
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
                {rect.x + padX, rect.y + rect.height - padY},
                cv::FONT_HERSHEY_SIMPLEX,
                scale,
                fg,
                thickness,
                cv::LINE_8);
}

std::string fmm(float v)
{
    if (!std::isfinite(v) || v <= 0.0f) return "--";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(0) << v;
    return oss.str();
}

// Minimal x-www-form-urlencoded parser for our small REST endpoints.
static std::string urlDecode_(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        char c = s[i];
        if (c == '+') { out.push_back(' '); continue; }
        if (c == '%' && i + 2 < s.size()) {
            auto hex = [](char h) -> int {
                if (h >= '0' && h <= '9') return h - '0';
                if (h >= 'a' && h <= 'f') return 10 + (h - 'a');
                if (h >= 'A' && h <= 'F') return 10 + (h - 'A');
                return -1;
            };
            int hi = hex(s[i + 1]);
            int lo = hex(s[i + 2]);
            if (hi >= 0 && lo >= 0) {
                out.push_back((char)((hi << 4) | lo));
                i += 2;
                continue;
            }
        }
        out.push_back(c);
    }
    return out;
}

static std::unordered_map<std::string, std::string> parseUrlEncoded_(const std::string& body)
{
    std::unordered_map<std::string, std::string> out;
    size_t start = 0;
    while (start < body.size()) {
        size_t amp = body.find('&', start);
        if (amp == std::string::npos) amp = body.size();
        const std::string pair = body.substr(start, amp - start);
        size_t eq = pair.find('=');
        std::string k = (eq == std::string::npos) ? pair : pair.substr(0, eq);
        std::string v = (eq == std::string::npos) ? "" : pair.substr(eq + 1);
        k = urlDecode_(k);
        v = urlDecode_(v);
        if (!k.empty()) out[k] = v;
        start = amp + 1;
    }
    return out;
}

static bool toBoolField_(const std::unordered_map<std::string, std::string>& f,
                         const std::string& k,
                         bool fallback)
{
    auto it = f.find(k);
    if (it == f.end()) return fallback;
    const std::string v = trimLowerCopy(it->second);
    return (v == "1" || v == "true" || v == "on" || v == "yes");
}

static int toIntField_(const std::unordered_map<std::string, std::string>& f,
                       const std::string& k,
                       int fallback)
{
    auto it = f.find(k);
    if (it == f.end()) return fallback;
    try { return std::stoi(it->second); } catch (...) { return fallback; }
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
    // Ensure we stop background services before stopping the web server.
    gpioService_.stopAll();
    listenService_.stop();
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

    // =====================================================================
    //  COMMS "ACTIVE LISTEN" (UART / Bluetooth / WiFi / Ethernet)
    //  This is meant as a debug tool: receive frames and optionally auto-reply.
    // =====================================================================
    listenService_.setEventCallback([this](const CATJ_comms_listen::ListenEvent& ev) {
        auto esc = [](const std::string& s) { return CATJ_webui_rt::WebUiRtServer::jsonEscape(s); };
        std::ostringstream payload;
        payload << "{";
        payload << "\"ts_ms\":" << ev.tsMs << ",";
        payload << "\"dir\":\"" << esc(ev.dir) << "\",";
        payload << "\"transport\":\"" << esc(ev.transport) << "\",";
        payload << "\"device\":\"" << esc(ev.device) << "\",";
        payload << "\"encoding\":\"" << esc(ev.encoding) << "\",";
        payload << "\"data\":\"" << esc(ev.data) << "\"";
        payload << "}";

        web_.broadcastJsonEnvelope("comms_event", payload.str());
    });

    web_.registerGet("/api/comms_listen_scan", [](const CATJ_webui_rt::HttpRequest&) {
        CATJ_webui_rt::HttpResponse res;
        res.contentType = "application/json; charset=utf-8";
        res.body = CATJ_comms_listen::CommsListenService::scanJson();
        return res;
    });

    web_.registerGet("/api/comms_listen_status", [this](const CATJ_webui_rt::HttpRequest&) {
        CATJ_webui_rt::HttpResponse res;
        res.contentType = "application/json; charset=utf-8";
        res.body = listenService_.statusJson();
        return res;
    });

    web_.registerPost("/api/comms_listen_stop", [this](const CATJ_webui_rt::HttpRequest&) {
        listenService_.stop();
        CATJ_webui_rt::HttpResponse res;
        res.contentType = "application/json; charset=utf-8";
        res.body = "{\"ok\":true}";
        return res;
    });

    web_.registerPost("/api/comms_listen_start", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto fields = parseUrlEncoded_(req.body);

        CATJ_comms_listen::ListenRequest lr;
        const std::string transport = fields.count("transport") ? trimLowerCopy(fields.at("transport")) : "uart";
        if (transport == "usb") lr.transport = CATJ_comms_listen::Transport::Usb;
        else if (transport == "bluetooth" || transport == "bt") lr.transport = CATJ_comms_listen::Transport::Bluetooth;
        else if (transport == "wifi" || transport == "tcp" || transport == "udp" || transport == "net") lr.transport = CATJ_comms_listen::Transport::Wifi;
        else if (transport == "ethernet" || transport == "rj45") lr.transport = CATJ_comms_listen::Transport::Ethernet;
        else if (transport == "i2c") lr.transport = CATJ_comms_listen::Transport::I2c;
        else if (transport == "spi") lr.transport = CATJ_comms_listen::Transport::Spi;
        else lr.transport = CATJ_comms_listen::Transport::Uart;

        lr.deviceSpec = fields.count("device") ? fields.at("device") : "";
        lr.rxMode = fields.count("rx_mode") ? trimLowerCopy(fields.at("rx_mode")) : "bytes";
        lr.pollTimeoutMs = std::clamp(toIntField_(fields, "poll_ms", 80), 5, 2000);
        lr.maxBytes = (size_t)std::clamp(toIntField_(fields, "max_bytes", 512), 1, 8192);
        lr.autoReply = toBoolField_(fields, "auto_reply", false);
        lr.echoReply = toBoolField_(fields, "echo_reply", false);
        lr.replyPayload = fields.count("reply_payload") ? fields.at("reply_payload") : "ACK";
        lr.replyEncoding = fields.count("reply_encoding") ? trimLowerCopy(fields.at("reply_encoding")) : "ascii";
        // Front-end key is "reply_append_nl" (compat: also accept "reply_newline").
        lr.replyAppendNewline = toBoolField_(fields, "reply_append_nl",
                                            toBoolField_(fields, "reply_newline", true));
        lr.pollTxPayload = fields.count("poll_tx_payload") ? fields.at("poll_tx_payload") : "";
        lr.pollTxEncoding = fields.count("poll_tx_encoding") ? trimLowerCopy(fields.at("poll_tx_encoding")) : "hex";
        lr.pollIntervalMs = std::clamp(toIntField_(fields, "poll_interval_ms", 250), 20, 5000);

        std::string err;
        const bool ok = listenService_.start(lr, &err);

        CATJ_webui_rt::HttpResponse res;
        res.contentType = "application/json; charset=utf-8";
        if (!ok) {
            res.status = 400;
            res.body = "{\"ok\":false,\"error\":\"" + CATJ_webui_rt::WebUiRtServer::jsonEscape(err) + "\"}";
        } else {
            res.body = "{\"ok\":true}";
        }
        return res;
    });

    // =====================================================================
    //  GPIO control / sampling (Linux sysfs best-effort)
    // =====================================================================
    gpioService_.setEventCallback([this](const CATJ_gpio_web::GpioEvent& ev) {
        auto esc = [](const std::string& s) { return CATJ_webui_rt::WebUiRtServer::jsonEscape(s); };
        std::ostringstream payload;
        payload << "{";
        payload << "\"ts_ms\":" << ev.tsMs << ",";
        payload << "\"pin\":" << ev.pin << ",";
        payload << "\"kind\":\"" << esc(ev.kind) << "\",";
        payload << "\"message\":\"" << esc(ev.message) << "\",";
        payload << "\"value\":" << ev.value;
        payload << "}";
        web_.broadcastJsonEnvelope("gpio_event", payload.str());
    });

    web_.registerGet("/api/gpio_status", [this](const CATJ_webui_rt::HttpRequest&) {
        CATJ_webui_rt::HttpResponse res;
        res.contentType = "application/json; charset=utf-8";
        res.body = gpioService_.statusJson();
        return res;
    });

    web_.registerPost("/api/gpio_mode", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto f = parseUrlEncoded_(req.body);
        const int pin = toIntField_(f, "pin", -1);
        const std::string mode = f.count("mode") ? trimLowerCopy(f.at("mode")) : "input";
        std::string err;
        const bool ok = gpioService_.configurePin(pin, mode, &err);
        CATJ_webui_rt::HttpResponse res; res.contentType = "application/json; charset=utf-8";
        res.status = ok ? 200 : 400;
        res.body = ok ? "{\"ok\":true}" : (std::string("{\"ok\":false,\"error\":\"") + CATJ_webui_rt::WebUiRtServer::jsonEscape(err) + "\"}");
        return res;
    });

    web_.registerPost("/api/gpio_write", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto f = parseUrlEncoded_(req.body);
        const int pin = toIntField_(f, "pin", -1);
        const int value = toIntField_(f, "value", 0);
        std::string err;
        const bool ok = gpioService_.writePin(pin, value, &err);
        CATJ_webui_rt::HttpResponse res; res.contentType = "application/json; charset=utf-8";
        res.status = ok ? 200 : 400;
        res.body = ok ? "{\"ok\":true}" : (std::string("{\"ok\":false,\"error\":\"") + CATJ_webui_rt::WebUiRtServer::jsonEscape(err) + "\"}");
        return res;
    });

    web_.registerPost("/api/gpio_read", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto f = parseUrlEncoded_(req.body);
        const int pin = toIntField_(f, "pin", -1);
        int v = 0; std::string err;
        const bool ok = gpioService_.readPin(pin, &v, &err);
        CATJ_webui_rt::HttpResponse res; res.contentType = "application/json; charset=utf-8";
        if (ok) { res.body = std::string("{\"ok\":true,\"value\":") + std::to_string(v) + "}"; }
        else { res.status = 400; res.body = std::string("{\"ok\":false,\"error\":\"") + CATJ_webui_rt::WebUiRtServer::jsonEscape(err) + "\"}"; }
        return res;
    });

    web_.registerPost("/api/gpio_pwm_start", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto f = parseUrlEncoded_(req.body);
        const int pin = toIntField_(f, "pin", -1);
        const double freq = (double)toIntField_(f, "freq_hz", 50);
        const double duty = (double)toIntField_(f, "duty_pct", 50);
        std::string err;
        const bool ok = gpioService_.startPwm(pin, freq, duty, &err);
        CATJ_webui_rt::HttpResponse res; res.contentType = "application/json; charset=utf-8";
        res.status = ok ? 200 : 400;
        res.body = ok ? "{\"ok\":true}" : (std::string("{\"ok\":false,\"error\":\"") + CATJ_webui_rt::WebUiRtServer::jsonEscape(err) + "\"}");
        return res;
    });

    web_.registerPost("/api/gpio_pwm_update", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto f = parseUrlEncoded_(req.body);
        const int pin = toIntField_(f, "pin", -1);
        const double freq = (double)toIntField_(f, "freq_hz", 50);
        const double duty = (double)toIntField_(f, "duty_pct", 50);
        std::string err;
        const bool ok = gpioService_.updatePwm(pin, freq, duty, &err);
        CATJ_webui_rt::HttpResponse res; res.contentType = "application/json; charset=utf-8";
        res.status = ok ? 200 : 400;
        res.body = ok ? "{\"ok\":true}" : (std::string("{\"ok\":false,\"error\":\"") + CATJ_webui_rt::WebUiRtServer::jsonEscape(err) + "\"}");
        return res;
    });

    web_.registerPost("/api/gpio_pwm_stop", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto f = parseUrlEncoded_(req.body);
        const int pin = toIntField_(f, "pin", -1);
        std::string err;
        gpioService_.stopPwm(pin, &err);
        CATJ_webui_rt::HttpResponse res; res.contentType = "application/json; charset=utf-8";
        res.body = "{\"ok\":true}";
        return res;
    });

    web_.registerPost("/api/gpio_sample_start", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto f = parseUrlEncoded_(req.body);
        const int pin = toIntField_(f, "pin", -1);
        const double hz = (double)toIntField_(f, "hz", 5);
        std::string err;
        const bool ok = gpioService_.startSampling(pin, hz, &err);
        CATJ_webui_rt::HttpResponse res; res.contentType = "application/json; charset=utf-8";
        res.status = ok ? 200 : 400;
        res.body = ok ? "{\"ok\":true}" : (std::string("{\"ok\":false,\"error\":\"") + CATJ_webui_rt::WebUiRtServer::jsonEscape(err) + "\"}");
        return res;
    });

    web_.registerPost("/api/gpio_sample_stop", [this](const CATJ_webui_rt::HttpRequest& req) {
        const auto f = parseUrlEncoded_(req.body);
        const int pin = toIntField_(f, "pin", -1);
        std::string err;
        gpioService_.stopSampling(pin, &err);
        CATJ_webui_rt::HttpResponse res; res.contentType = "application/json; charset=utf-8";
        res.body = "{\"ok\":true}";
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

    if (!remoteEnabled) {
        out.type = ControlEventType::RawWebCommand;
        out.name = ev.action;
        out.accepted = false;
        emitControlEvent_(std::move(out));
        return;
    }

    /*========================================================================================*/
    /*                                  EVENT TABLE                                           */   
    /*========================================================================================*/
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
    else if (ev.action == "mission") 
    {
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
    else if (ev.action == "programme")
    {
        std::cout << "action : " << ev.action << std::endl;
        const std::string cmd = ev.fields.count("cmd") ? trimLowerCopy(ev.fields.at("cmd")) : "";

        if (cmd == "stop")
        {
            out.type = ControlEventType::quitProgram;
            std::cout << "STOP programme" << std::endl;
            std::lock_guard<std::mutex> lock(stateMutex_);
        }
    }
    else if (ev.action == "camera")
    {
        const std::string cmd = ev.fields.count("cmd") ? trimLowerCopy(ev.fields.at("cmd")) : "";
        if (cmd == "calibration")
        {
            out.type = ControlEventType::CameraSetting;
            out.name = "calibration";
			std::cout << "Camera calibration requested" << std::endl;
        }
    }
    else {
		std::cout << "unknown web command: " << ev.action << std::endl;
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
    const cv::Scalar detectColor(34, 197, 94);
    const cv::Scalar noDetectColor(80, 90, 110);

    const bool smallFrame = (out.cols <= 640 || out.rows <= 480);
    const bool hasObject = !detectionsLocal.empty() || telem.vision.objectDetected || telem.vision.detectionCount > 0;

    const double labelScale = smallFrame ? 0.34 : 0.46;
    const double textScale1 = smallFrame ? 0.42 : 0.58;
    const double textScale2 = smallFrame ? 0.38 : 0.52;

    const int textThickness = 1;
    const int boxThickness = smallFrame ? 1 : 2;
    const int boxThicknessPrimary = smallFrame ? 2 : 3;

    const int margin = 10;
    const int line1Y = smallFrame ? 58 : 78;
    const int line2Y = smallFrame ? 78 : 106;

    for (const auto& det : detectionsLocal) {
        cv::Rect box = det.box & cv::Rect(0, 0, out.cols, out.rows);
        if (box.width <= 1 || box.height <= 1) continue;

        const cv::Scalar color = det.primary ? cv::Scalar(0, 255, 255) : det.color;
        cv::rectangle(out, box, color, det.primary ? boxThicknessPrimary : boxThickness, cv::LINE_8);

        std::ostringstream label;
        label << det.label << ' ' << std::fixed << std::setprecision(0)
              << (det.confidence * 100.0) << '%';

        drawLabelBox(out,
                     label.str(),
                     {box.x, std::max(18, box.y)},
                     color,
                     cv::Scalar(255, 255, 255),
                     labelScale,
                     1);

        cv::circle(out,
                   {box.x + box.width / 2, box.y + box.height / 2},
                   smallFrame ? 3 : 4,
                   color,
                   cv::FILLED,
                   cv::LINE_8);

        if (det.primary) {
            const int cx = box.x + box.width / 2;
            const int cy = box.y + box.height / 2;
            const int half = smallFrame ? 8 : 12;
            cv::line(out, {cx - half, cy}, {cx + half, cy}, color, 1, cv::LINE_8);
            cv::line(out, {cx, cy - half}, {cx, cy + half}, color, 1, cv::LINE_8);
        }
    }

    drawLabelBox(out,
                 telem.mode == RobotMode::Auto ? "AUTO" : "MANUAL",
                 {margin, 28},
                 telem.mode == RobotMode::Auto ? autoColor : manualColor,
                 cv::Scalar(255, 255, 255),
                 labelScale,
                 1);

    if (telem.missionEnabled) {
        drawLabelBox(out,
                     "MISSION",
                     {smallFrame ? 95 : 120, 28},
                     cv::Scalar(34, 197, 94),
                     cv::Scalar(255, 255, 255),
                     labelScale,
                     1);
    }

    if (!telem.remoteControlEnabled) {
        drawLabelBox(out,
                     "REMOTE OFF",
                     {smallFrame ? 175 : 220, 28},
                     warnColor,
                     cv::Scalar(255, 255, 255),
                     labelScale,
                     1);
    }

    std::ostringstream detBadge;
    detBadge << (hasObject ? "OBJ DETECTED" : "NO OBJECT") << " | N=" << detectionsLocal.size();
    const int detBadgeX = smallFrame ? std::max(10, out.cols - 175) : std::max(10, out.cols - 240);
    drawLabelBox(out,
                 detBadge.str(),
                 {detBadgeX, 28},
                 hasObject ? detectColor : noDetectColor,
                 cv::Scalar(255, 255, 255),
                 labelScale,
                 1);

    std::ostringstream line1;
    line1 << std::fixed << std::setprecision(0)
          << "HDG " << telem.compass.headingDeg
          << " | MAG " << telem.compass.magneticNorthDeg
          << " | FPS " << std::setprecision(1) << telem.vision.fps;

    if (telem.lidar.enabled) {
        line1 << " | LID F " << fmm(telem.lidar.frontMm);
    }

    std::ostringstream line2;
    line2 << "Target: " << telem.vision.target
          << " (" << std::fixed << std::setprecision(0)
          << (telem.vision.confidence * 100.0) << "%)"
          << " | Dets " << detectionsLocal.size()
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
                {margin + 8, line1Y},
                cv::FONT_HERSHEY_SIMPLEX,
                textScale1,
                cv::Scalar(255, 255, 255),
                textThickness,
                cv::LINE_8);

    cv::putText(out,
                line2.str(),
                {margin + 8, line2Y},
                cv::FONT_HERSHEY_SIMPLEX,
                textScale2,
                cv::Scalar(210, 220, 235),
                textThickness,
                cv::LINE_8);

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
    oss << "\"object_detected\":" << jsonBool_(t.vision.objectDetected) << ',';
    oss << "\"detection_count\":" << t.vision.detectionCount << ',';
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
    oss << "},";

    oss << "\"lidar\":{";
    oss << "\"enabled\":" << jsonBool_(t.lidar.enabled) << ',';
    oss << "\"connected\":" << jsonBool_(t.lidar.connected) << ',';
    oss << "\"scanning\":" << jsonBool_(t.lidar.scanning) << ',';
    oss << "\"mock_mode\":" << jsonBool_(t.lidar.mockMode) << ',';
    oss << "\"scan_hz\":" << t.lidar.scanHz << ',';
    oss << "\"sample_count\":" << t.lidar.sampleCount << ',';
    oss << "\"min_distance_mm\":" << t.lidar.minDistanceMm << ',';
    oss << "\"front_mm\":" << t.lidar.frontMm << ',';
    oss << "\"front_right_mm\":" << t.lidar.frontRightMm << ',';
    oss << "\"right_mm\":" << t.lidar.rightMm << ',';
    oss << "\"rear_mm\":" << t.lidar.rearMm << ',';
    oss << "\"left_mm\":" << t.lidar.leftMm << ',';
    oss << "\"front_left_mm\":" << t.lidar.frontLeftMm << ',';
    oss << "\"max_distance_mm\":" << t.lidar.maxDistanceMm << ',';
    oss << "\"status_text\":\"" << esc(t.lidar.statusText) << "\",";
    oss << "\"points\":[";
    for (size_t i = 0; i < t.lidar.points.size(); ++i) {
        if (i) oss << ',';
        const auto& p = t.lidar.points[i];
        oss << '{';
        oss << "\"a\":" << p.angleDeg << ',';
        oss << "\"d\":" << p.distanceMm << ',';
        oss << "\"q\":" << p.quality;
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
    const std::string value = trimLowerCopy(it->second);
    if (value == "1" || value == "true" || value == "on" || value == "yes") return true;
    if (value == "0" || value == "false" || value == "off" || value == "no") return false;
    return fallback;
}

} // namespace CATJ_robot_web
