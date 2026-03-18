#include "webui_rt.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

namespace {
std::atomic<bool> g_running{ true };

void onSignal(int)
{
    g_running.store(false);
}

double nowSeconds()
{
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - t0).count();
    return static_cast<double>(dt) / 1000.0;
}
}

int main()
{
    std::signal(SIGINT, onSignal);
#if defined(SIGTERM)
    std::signal(SIGTERM, onSignal);
#endif

    std::atomic<int> leftPct{ 0 };
    std::atomic<int> rightPct{ 0 };
    std::atomic<int> conveyorPct{ 0 };
    std::atomic<int> camPanDeg{ 0 };
    std::atomic<int> camTiltDeg{ 0 };
    std::atomic<bool> missionEnabled{ false };
    std::atomic<double> headingDeg{ 0.0 };
    std::atomic<double> rollDeg{ 0.0 };
    std::atomic<double> pitchDeg{ 0.0 };
    std::atomic<double> yawRateDps{ 0.0 };

    std::mutex visMutex;
    std::string lastDetectedTarget = "none";
    double lastConfidence = 0.0;
    double fps = 0.0;

    CATJ_webui_rt::WebUiRtServer web;
    CATJ_webui_rt::WebUiRtConfig cfg;
    cfg.bindAddress = "0.0.0.0";
    cfg.port = 8080;
    cfg.documentRoot = "./webui_rt_demo";
    cfg.indexFile = "index.html";
    cfg.telemetryPeriodMs = 100;
    cfg.jpegQuality = 80;

    web.setTelemetryProvider([&]() {
        std::ostringstream oss;
        std::lock_guard<std::mutex> lock(visMutex);
        oss << std::fixed << std::setprecision(2);
        oss << "{";
        oss << "\"ok\":true,";
        oss << "\"mission_enabled\":" << (missionEnabled.load() ? "true" : "false") << ',';
        oss << "\"heading_deg\":" << headingDeg.load() << ',';
        oss << "\"mag_north_deg\":" << headingDeg.load() << ',';
        oss << "\"gyro\":{";
        oss << "\"roll_deg\":" << rollDeg.load() << ',';
        oss << "\"pitch_deg\":" << pitchDeg.load() << ',';
        oss << "\"yaw_rate_dps\":" << yawRateDps.load();
        oss << "},";
        oss << "\"motors\":{";
        oss << "\"left_pct\":" << leftPct.load() << ',';
        oss << "\"right_pct\":" << rightPct.load() << ',';
        oss << "\"conveyor_pct\":" << conveyorPct.load() << ',';
        oss << "\"cam_pan_deg\":" << camPanDeg.load() << ',';
        oss << "\"cam_tilt_deg\":" << camTiltDeg.load();
        oss << "},";
        oss << "\"vision\":{";
        oss << "\"fps\":" << fps << ',';
        oss << "\"target\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(lastDetectedTarget) << "\",";
        oss << "\"confidence\":" << lastConfidence;
        oss << "}";
        oss << "}";
        return oss.str();
    });

    web.setCommandHandler([&](const CATJ_webui_rt::WebCommandEvent& ev) {
        auto getInt = [&](const char* key, int fallback = 0) {
            auto it = ev.fields.find(key);
            if (it == ev.fields.end()) return fallback;
            try { return std::stoi(it->second); }
            catch (...) { return fallback; }
        };

        if (ev.action == "thrusters") {
            leftPct = std::clamp(getInt("left_pct"), -100, 100);
            rightPct = std::clamp(getInt("right_pct"), -100, 100);
        }
        else if (ev.action == "conveyor") {
            conveyorPct = std::clamp(getInt("pct"), -100, 100);
        }
        else if (ev.action == "camera_step") {
            const int steps = std::clamp(getInt("steps"), -30, 30);
            const std::string axis = ev.fields.count("axis") ? ev.fields.at("axis") : "pan";
            if (axis == "pan") camPanDeg = std::clamp(camPanDeg.load() + steps, -90, 90);
            if (axis == "tilt") camTiltDeg = std::clamp(camTiltDeg.load() + steps, -45, 45);
        }
        else if (ev.action == "camera_center") {
            camPanDeg = 0;
            camTiltDeg = 0;
        }
        else if (ev.action == "mission") {
            const std::string cmd = ev.fields.count("cmd") ? ev.fields.at("cmd") : "";
            if (cmd == "start") missionEnabled = true;
            if (cmd == "stop") missionEnabled = false;
        }
        else if (ev.action == "camera_setting") {
            std::cout << "[CAMERA_SETTING] " << ev.rawJson << std::endl;
        }

        std::cout << "[WS CMD] " << ev.remoteIp << ':' << ev.remotePort
                  << " action=" << ev.action << " raw=" << ev.rawJson << std::endl;
    });

    if (!web.start(cfg)) {
        std::cerr << "Impossible de lancer le serveur WebUI RT" << std::endl;
        return -1;
    }

    std::cout << "Web dashboard: http://<IP_RASPI>:" << cfg.port << std::endl;
    std::cout << "Flux video MJPEG: /stream.mjpg" << std::endl;
    std::cout << "WebSocket: ws://<IP_RASPI>:" << cfg.port << cfg.wsRoute << std::endl;

    cv::VideoCapture cap;
    cap.open(0);
    if (cap.isOpened()) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 540);
    }

    auto lastFrameTime = std::chrono::steady_clock::now();

    while (g_running.load()) {
        cv::Mat frame;

        if (cap.isOpened()) {
            cap >> frame;
        }

        if (frame.empty()) {
            frame = cv::Mat(540, 960, CV_8UC3, cv::Scalar(24, 24, 28));
            const double t = nowSeconds();
            const int cx = 480 + static_cast<int>(std::cos(t) * 220.0);
            const int cy = 270 + static_cast<int>(std::sin(0.7 * t) * 120.0);
            cv::circle(frame, {cx, cy}, 34, cv::Scalar(0, 255, 0), -1);
            cv::putText(frame, "Synthetic OpenCV stream", {28, 48}, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        }

        const double t = nowSeconds();
        headingDeg = std::fmod(t * 22.0, 360.0);
        rollDeg = std::sin(t * 1.8) * 12.0;
        pitchDeg = std::cos(t * 1.2) * 8.0;
        yawRateDps = std::sin(t * 2.4) * 18.0;

        {
            std::lock_guard<std::mutex> lock(visMutex);
            lastDetectedTarget = ((static_cast<int>(t) / 4) % 2 == 0) ? "green_ball" : "red_ball";
            lastConfidence = 0.70 + 0.25 * (0.5 + 0.5 * std::sin(t * 1.3));
        }

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastFrameTime).count() / 1000.0;
        lastFrameTime = now;
        if (dt > 1e-6) fps = 1.0 / dt;

        std::ostringstream overlay;
        overlay << std::fixed << std::setprecision(1)
                << "HDG " << headingDeg.load() << " deg"
                << " | Roll " << rollDeg.load() << " deg"
                << " | Pitch " << pitchDeg.load() << " deg"
                << " | L/R " << leftPct.load() << "% / " << rightPct.load() << '%';

        cv::rectangle(frame, {14, frame.rows - 62}, {frame.cols - 14, frame.rows - 16}, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(frame, overlay.str(), {24, frame.rows - 30}, cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);

        web.updateFrame(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    web.stop();
    return 0;
}
