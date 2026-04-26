#include "main.h"
#include "../fonctions/web/robotOverlay/robot_web_bridge.h"
#include "../fonctions/capteur/lidar/Lidar.h"
#include "../fonctions/Communication/comms/comms_manager.h"
#include "../fonctions/Communication/I2C/I2C.h"
#include "../fonctions/Communication/SPI/SPI.h"
#include "../fonctions/motherboard/motherboard_protocol.h"
#include "../fonctions/mode/ball/ball_logic.h"
#include "../fonctions/mode/course/mode_course.h"
#include "../fonctions/mode/labyrinthe/mode_labyrinthe.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <unordered_map>
#include <iomanip>

#define DIR_NAME "IBM_Project_Bateau"

// Dir raspi Jerry
#define DIR_NAME_LINUX "IBM_Bateau"

// Dir raspi EPF
//#define DIR_NAME_LINUX "jerryCamera"

#ifdef WIN32
#include <windows.h>
#endif

bool flagRecording = false;
bool showImage = false;
int vKey = 0;
std::string imgCodec;
std::string imgExt;
CATJ_utility::MyString trimTime;
std::string recordingPath;

namespace {

    std::string ballColorToLabel(CATJ_camera::BallColor c)
    {
        switch (c) {
        case CATJ_camera::BallColor::Red:   return "red_ball";
        case CATJ_camera::BallColor::Blue:  return "blue_ball";
        case CATJ_camera::BallColor::White: return "white_ball";
        default:                            return "unknown";
        }
    }

    cv::Scalar ballColorToScalar(CATJ_camera::BallColor c)
    {
        switch (c) {
        case CATJ_camera::BallColor::Red:   return cv::Scalar(30, 30, 230);
        case CATJ_camera::BallColor::Blue:  return cv::Scalar(230, 30, 30);
        case CATJ_camera::BallColor::White: return cv::Scalar(220, 220, 220);
        default:                            return cv::Scalar(60, 200, 60);
        }
    }

    std::vector<CATJ_robot_web::Detection>
        convertDetections(const std::vector<CATJ_camera::BallDetection>& dets)
    {
        std::vector<CATJ_robot_web::Detection> out;
        out.reserve(dets.size());

        int bestIdx = -1;
        float bestConf = -1.0f;
        for (size_t i = 0; i < dets.size(); ++i) {
            if (dets[i].conf > bestConf) {
                bestConf = dets[i].conf;
                bestIdx = static_cast<int>(i);
            }
        }

        for (size_t i = 0; i < dets.size(); ++i) {
            CATJ_robot_web::Detection d;
            d.box = dets[i].box;
            d.label = ballColorToLabel(dets[i].color);
            d.confidence = dets[i].conf;
            d.color = ballColorToScalar(dets[i].color);
            d.trackId = static_cast<int>(i);
            d.primary = (static_cast<int>(i) == bestIdx);
            out.push_back(std::move(d));
        }

        return out;
    }

    std::vector<CATJ_robot_web::LidarPointTelemetry>
        convertLidarPoints(const std::vector<CATJ_lidar::ScanPoint>& pts, std::size_t maxPoints)
    {
        std::vector<CATJ_robot_web::LidarPointTelemetry> out;
        if (pts.empty() || maxPoints == 0) return out;

        const std::size_t step = std::max<std::size_t>(1, (pts.size() + maxPoints - 1) / maxPoints);
        out.reserve(std::min<std::size_t>(maxPoints, pts.size()));

        for (std::size_t i = 0; i < pts.size(); i += step) {
            CATJ_robot_web::LidarPointTelemetry p;
            p.angleDeg = pts[i].angleDeg;
            p.distanceMm = pts[i].distanceMm;
            p.quality = pts[i].quality;
            out.push_back(p);
        }

        return out;
    }

    void fillWebLidarTelemetry(CATJ_robot_web::LidarTelemetry& out,
        const CATJ_lidar::Snapshot& snap,
        std::size_t webMaxPoints,
        float maxDistanceMm)
    {
        out.enabled = true;
        out.connected = snap.connected;
        out.scanning = snap.scanning;
        out.mockMode = snap.mockMode;
        out.scanHz = snap.scanHz;
        out.sampleCount = static_cast<int>(snap.sampleCount);
        out.minDistanceMm = std::isfinite(snap.sectors.minDistanceMm) ? snap.sectors.minDistanceMm : 0.0f;
        out.frontMm = std::isfinite(snap.sectors.frontMm) ? snap.sectors.frontMm : 0.0f;
        out.frontRightMm = std::isfinite(snap.sectors.frontRightMm) ? snap.sectors.frontRightMm : 0.0f;
        out.rightMm = std::isfinite(snap.sectors.rightMm) ? snap.sectors.rightMm : 0.0f;
        out.rearMm = std::isfinite(snap.sectors.rearMm) ? snap.sectors.rearMm : 0.0f;
        out.leftMm = std::isfinite(snap.sectors.leftMm) ? snap.sectors.leftMm : 0.0f;
        out.frontLeftMm = std::isfinite(snap.sectors.frontLeftMm) ? snap.sectors.frontLeftMm : 0.0f;
        out.maxDistanceMm = maxDistanceMm;
        out.statusText = !snap.connected ? "disconnected" : (snap.mockMode ? "mock stream" : "live stream");
        out.points = convertLidarPoints(snap.points, webMaxPoints);
    }

    CATJ_camera::BallColor parseBallColorName_(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });

        s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) {
            return std::isspace(c) != 0 || c == '_' || c == '-';
        }), s.end());

        if (s == "red" || s == "rouge") return CATJ_camera::BallColor::Red;
        if (s == "blue" || s == "bleu") return CATJ_camera::BallColor::Blue;
        if (s == "white" || s == "blanc") return CATJ_camera::BallColor::White;
        return CATJ_camera::BallColor::Unknown;
    }

    CATJ_camera::BallColor ballColorFromHexRgb_(uint32_t rgb)
    {
        const int r = static_cast<int>((rgb >> 16) & 0xFFu);
        const int g = static_cast<int>((rgb >> 8) & 0xFFu);
        const int b = static_cast<int>(rgb & 0xFFu);

        const int maxc = std::max({ r, g, b });
        const int minc = std::min({ r, g, b });

        if (maxc >= 180 && (maxc - minc) <= 45) return CATJ_camera::BallColor::White;
        if (r >= 100 && r > g + 30 && r > b + 30) return CATJ_camera::BallColor::Red;
        if (b >= 100 && b > r + 20 && b > g + 20) return CATJ_camera::BallColor::Blue;
        return CATJ_camera::BallColor::Unknown;
    }

    CATJ_camera::BallColor parseConfiguredBallColor_(const std::string& raw,
        CATJ_camera::BallColor fallback)
    {
        const auto byName = parseBallColorName_(raw);
        if (byName != CATJ_camera::BallColor::Unknown) return byName;

        uint32_t rgb = 0;
        if (CATJ_utility::parseHexU32(CATJ_utility::trim_copy(raw), rgb)) {
            const auto byHex = ballColorFromHexRgb_(rgb);
            if (byHex != CATJ_camera::BallColor::Unknown) return byHex;
        }

        return fallback;
    }

    const char* ballColorName_(CATJ_camera::BallColor c)
    {
        switch (c) {
        case CATJ_camera::BallColor::Red: return "red";
        case CATJ_camera::BallColor::Blue: return "blue";
        case CATJ_camera::BallColor::White: return "white";
        default: return "unknown";
        }
    }

    struct MotionCommandMain_ {
        CATJ_robot::MoveCommand move = CATJ_robot::MoveCommand::Stop;
        int speedPct = 0;
        float turnDeg = 0.0f;
    };

    struct MotionCommandCache_ {
        CATJ_robot::MoveCommand move = CATJ_robot::MoveCommand::Stop;
        int speedPct = std::numeric_limits<int>::min();
        float turnDeg = std::numeric_limits<float>::quiet_NaN();
    };

    void applyMotionCommand_(CATJ_robot::MotherboardLink& board,
        MotionCommandCache_& cache,
        const MotionCommandMain_& cmd)
    {
        const int clampedSpeed = std::clamp(cmd.speedPct, 0, 100);
        const float clampedTurn = std::clamp(cmd.turnDeg, -180.0f, 180.0f);

        if (cache.speedPct != clampedSpeed) {
            board.sendSpeedPct(clampedSpeed);
            cache.speedPct = clampedSpeed;
        }

        if (!std::isfinite(cache.turnDeg) || std::fabs(cache.turnDeg - clampedTurn) >= 1.0f) {
            board.sendTurnDeg(clampedTurn);
            cache.turnDeg = clampedTurn;
        }

        if (cache.move != cmd.move) {
            board.sendMove(cmd.move);
            cache.move = cmd.move;
        }
    }




    static int hexVal_(char c)
    {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
        if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
        return -1;
    }

    static std::string urlDecode_(const std::string& in)
    {
        std::string out;
        out.reserve(in.size());
        for (size_t i = 0; i < in.size(); ++i) {
            const char c = in[i];
            if (c == '+') { out.push_back(' '); continue; }
            if (c == '%' && i + 2 < in.size()) {
                const int hi = hexVal_(in[i + 1]);
                const int lo = hexVal_(in[i + 2]);
                if (hi >= 0 && lo >= 0) {
                    out.push_back(static_cast<char>((hi << 4) | lo));
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
            const size_t eq = pair.find('=');
            std::string k = (eq == std::string::npos) ? pair : pair.substr(0, eq);
            std::string v = (eq == std::string::npos) ? "" : pair.substr(eq + 1);
            k = urlDecode_(k);
            v = urlDecode_(v);
            if (!k.empty()) out[k] = v;
            start = amp + 1;
        }
        return out;
    }

    static uint32_t parseU32Loose_(std::string s, uint32_t fallback)
    {
        try {
            s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c) != 0; }), s.end());
            int base = 10;
            if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) { s = s.substr(2); base = 16; }
            return static_cast<uint32_t>(std::stoul(s, nullptr, base));
        } catch (...) { return fallback; }
    }

    static std::string hex8_(uint32_t v)
    {
        std::ostringstream oss;
        oss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (v & 0xFFu);
        return oss.str();
    }

    static std::string hex16_(uint32_t v)
    {
        std::ostringstream oss;
        oss << "0x" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << (v & 0xFFFFu);
        return oss.str();
    }


    static bool parseI2cSpecMain_(const std::string& spec, std::string& outDev, uint16_t& outAddr)
    {
        std::string s = spec;
        s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c) != 0; }), s.end());
        if (s.empty()) return false;
        outDev = "/dev/i2c-1";
        outAddr = 0x42;
        const auto at = s.find('@');
        if (at == std::string::npos) { outDev = s; return true; }
        outDev = s.substr(0, at);
        std::string a = s.substr(at + 1);
        try {
            int base = 10;
            if (a.rfind("0x", 0) == 0 || a.rfind("0X", 0) == 0) { a = a.substr(2); base = 16; }
            outAddr = static_cast<uint16_t>(std::stoul(a, nullptr, base));
        } catch (...) { return false; }
        return !outDev.empty();
    }

    static bool parseSpiSpecMain_(const std::string& spec, std::string& outDev, uint32_t& outSpeed, uint8_t& outMode, uint8_t& outBits)
    {
        std::string s = spec;
        s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c) != 0; }), s.end());
        if (s.empty()) return false;
        outDev = "/dev/spidev0.0";
        outSpeed = 1000000;
        outMode = 0;
        outBits = 8;
        const auto at = s.find('@');
        if (at == std::string::npos) { outDev = s; return true; }
        outDev = s.substr(0, at);
        std::string rest = s.substr(at + 1);
        std::string speedPart = rest, modePart, bitsPart;
        const auto hash = rest.find('#');
        if (hash != std::string::npos) {
            speedPart = rest.substr(0, hash);
            const auto plus = rest.find('+', hash + 1);
            if (plus == std::string::npos) modePart = rest.substr(hash + 1);
            else {
                modePart = rest.substr(hash + 1, plus - (hash + 1));
                bitsPart = rest.substr(plus + 1);
            }
        }
        try { if (!speedPart.empty()) outSpeed = static_cast<uint32_t>(std::stoul(speedPart)); } catch (...) {}
        try { if (!modePart.empty()) outMode = static_cast<uint8_t>(std::stoi(modePart)); } catch (...) {}
        try { if (!bitsPart.empty()) outBits = static_cast<uint8_t>(std::stoi(bitsPart)); } catch (...) {}
        outMode = std::min<uint8_t>(outMode, 3);
        if (outBits == 0) outBits = 8;
        return !outDev.empty();
    }

    static bool parseHexBytesMain_(const std::string& text, std::vector<uint8_t>& out)
    {
        out.clear();
        std::string s = text;
        for (char& c : s) if (c == ',' || c == ';' || c == ':') c = ' ';
        std::istringstream iss(s);
        std::string tok;
        while (iss >> tok) {
            if (tok.rfind("0x", 0) == 0 || tok.rfind("0X", 0) == 0) tok = tok.substr(2);
            if (tok.size() == 1) tok = "0" + tok;
            if (tok.size() != 2) return false;
            const int hi = hexVal_(tok[0]);
            const int lo = hexVal_(tok[1]);
            if (hi < 0 || lo < 0) return false;
            out.push_back(static_cast<uint8_t>((hi << 4) | lo));
        }
        return !out.empty();
    }

    static std::string bytesToHexMain_(const uint8_t* data, size_t n)
    {
        static const char* kHex = "0123456789ABCDEF";
        std::string out;
        out.reserve(n * 3);
        for (size_t i = 0; i < n; ++i) {
            const uint8_t b = data[i];
            out.push_back(kHex[(b >> 4) & 0xF]);
            out.push_back(kHex[b & 0xF]);
            if (i + 1 < n) out.push_back(' ');
        }
        return out;
    }

    CATJ_utility::fs::path resolveProjectPath(CATJ_utility::fs::path exeDir)
    {
#ifdef WIN32
        const char* expectedName = DIR_NAME;
#else
        const char* expectedName = DIR_NAME_LINUX;
#endif
        while (!exeDir.empty() && exeDir.filename() != expectedName)
        {
            std::cout << "dossier path check : " << exeDir.filename() << std::endl;
            exeDir = exeDir.parent_path();
            if (exeDir.filename().compare("") == 0)
                break;
        }
        return exeDir;
    }

    int runRemoteWebMode(const std::filesystem::path& prjPath,
        CATJ_utility::iniReader& iniFile,
        CATJ_camera::Camera& cam,
        CATJ_utility::programme_mode baseMode,
        bool enableRecording,
        const std::string& recPath,
        const std::string& codec)
    {
        // --- WEB ---
        int webPort = 8080;
        iniFile.get("WEB", "port", webPort);

        const bool allowNetworkControl = iniFile.getOr("WEB", "allow network control", true);
        const int telemetryPeriodMs = iniFile.getOr("WEB", "telemetry period ms", 100);
        const int jpegQuality = iniFile.getOr("WEB", "jpeg quality", 80);

        // stream camera : permet un remote LIDAR only sans dépendre de la caméra.
        // vision enabled : active / désactive la détection + overlay.
        const bool streamCamera = iniFile.getOr("WEB", "stream camera", true);
        const bool visionEnabled = iniFile.getOr("WEB", "vision enabled", (baseMode == CATJ_utility::programme_mode::camera));


        // --- LIDAR ---
        const bool lidarEnabled = iniFile.getOr("LIDAR", "enabled", true);
        const std::string lidarPort = iniFile.getOr<std::string>("LIDAR", "port", "/dev/ttyUSB0");
        const int lidarBaud = iniFile.getOr("LIDAR", "baudrate", 460800);
        const bool lidarMock = iniFile.getOr("LIDAR", "mock", false);
        const int lidarWebMaxPoints = iniFile.getOr("LIDAR", "web max points", 180);
        const float lidarMaxDistMm = iniFile.getOr("LIDAR", "max distance mm", 8000.0f);
        const float lidarFrontHalfDeg = iniFile.getOr("LIDAR", "front half angle deg", 20.0f);
        const float lidarLateralHalfDeg = iniFile.getOr("LIDAR", "lateral half angle deg", 50.0f);
        const float lidarAvoidMm = iniFile.getOr("LIDAR", "avoid distance mm", 650.0f);

        CATJ_webui_rt::WebUiRtConfig webCfg;
        webCfg.bindAddress = "0.0.0.0";
        webCfg.port = static_cast<uint16_t>(std::clamp(webPort, 1, 65535));
        webCfg.documentRoot = (prjPath / "sources" / "fonctions" / "web" / "frontend" / "view").string();
        webCfg.indexFile = "index.html";
        webCfg.telemetryPeriodMs = telemetryPeriodMs;
        webCfg.jpegQuality = jpegQuality;
        webCfg.allowNetworkControl = allowNetworkControl;

        CATJ_robot_web::RobotWebBridge bridge;
        if (!bridge.start(webCfg, webCfg.documentRoot)) {
            std::cerr << "Impossible de lancer l'interface web distante" << std::endl;
            return -40;
        }

        std::cout << "Dashboard disponible sur : http://" << "raspberrypi:" << webCfg.port << std::endl;

        // --- COMMS (UART / Bluetooth / WiFi / RJ45) ---
        CATJ_comms::CommsManager comms;
        {
            std::string commsErr;
            const auto commsIni = prjPath / "parametres" / "motherboard_trame.ini";
            if (!comms.loadIni(commsIni, &commsErr)) {
                std::cerr << "[COMMS] " << commsErr << std::endl;
            } else {
                std::cout << "[COMMS] Presets chargés: " << commsIni << std::endl;
            }
        }

        // API pour la page Comms (chargement config + historique + scan)
        bridge.server().registerGet("/api/comms_config", [&](const CATJ_webui_rt::HttpRequest&) {
            CATJ_webui_rt::HttpResponse r;
            r.status = 200;
            r.contentType = "application/json; charset=utf-8";
            r.body = comms.configJson();
            return r;
        });

        bridge.server().registerGet("/api/comms_history", [&](const CATJ_webui_rt::HttpRequest& req) {
            size_t limit = 200;
            if (auto it = req.query.find("limit"); it != req.query.end()) {
                try { limit = static_cast<size_t>(std::stoul(it->second)); } catch (...) {}
                limit = std::clamp<size_t>(limit, 10, 2000);
            }
            CATJ_webui_rt::HttpResponse r;
            r.status = 200;
            r.contentType = "application/json; charset=utf-8";
            r.body = comms.historyJson(limit);
            return r;
        });

        bridge.server().registerGet("/api/system_profile", [&](const CATJ_webui_rt::HttpRequest&) {
            const auto esc = CATJ_webui_rt::WebUiRtServer::jsonEscape;

            const std::string camReference = iniFile.getOr<std::string>("CAMERA", "reference", "");
            const int camDevice = iniFile.getOr("CAMERA", "device", 0);
            const int camWidth = iniFile.getOr("CAMERA", "width", 0);
            const int camHeight = iniFile.getOr("CAMERA", "height", 0);
            const int camFps = iniFile.getOr("CAMERA", "fps", 0);
            const int camAiFps = iniFile.getOr("CAMERA", "Aifps", 0);
            const std::string camBackend = iniFile.getOr<std::string>("CAMERA", "backend", "auto");
            const bool camShowImage = iniFile.getOr("CAMERA", "show image", false);
            const bool camRecording = iniFile.getOr("CAMERA", "flag recording", false);
            const std::string camRecordingPathIni = iniFile.getOr<std::string>("CAMERA", "recording file path", "");

            const float scoreThreshold = iniFile.getOr("CAMERA", "score threshold", 0.45f);
            const float nmsThreshold = iniFile.getOr("CAMERA", "non maximum suppression threshold", 0.35f);

            std::string onnxModel = iniFile.getOr<std::string>("IA", "ONNX model path", "");
            if (!onnxModel.empty()) {
                std::filesystem::path modelPath = std::filesystem::path(onnxModel).is_absolute()
                    ? std::filesystem::path(onnxModel)
                    : (prjPath / onnxModel);
                onnxModel = modelPath.lexically_normal().string();
            }

            std::ostringstream js;
            js << std::fixed << std::setprecision(2);
            js << '{';
            js << "\"ok\":true,";
            js << "\"base_mode\":\"" << esc(CATJ_utility::modeToStr(baseMode)) << "\",";
            js << "\"stream_camera\":" << (streamCamera ? "true" : "false") << ',';
            js << "\"vision_enabled\":" << (visionEnabled ? "true" : "false") << ',';
            js << "\"overlay_default\":true,";
            js << "\"allow_network_control\":" << (allowNetworkControl ? "true" : "false") << ',';
            js << "\"telemetry_period_ms\":" << telemetryPeriodMs << ',';
            js << "\"jpeg_quality\":" << jpegQuality << ',';
            js << "\"image_codec\":\"" << esc(codec) << "\",";

            js << "\"camera\":{";
            js << "\"reference\":\"" << esc(camReference) << "\",";
            js << "\"device\":" << camDevice << ',';
            js << "\"width\":" << camWidth << ',';
            js << "\"height\":" << camHeight << ',';
            js << "\"fps\":" << camFps << ',';
            js << "\"ai_fps\":" << camAiFps << ',';
            js << "\"backend\":\"" << esc(camBackend) << "\",";
            js << "\"show_image\":" << (camShowImage ? "true" : "false") << ',';
            js << "\"recording_enabled\":" << (camRecording ? "true" : "false") << ',';
            js << "\"recording_path\":\"" << esc(!recordingPath.empty() ? recordingPath : camRecordingPathIni) << "\"";
            js << "},";

            js << "\"vision\":{";
            js << "\"onnx_model\":\"" << esc(onnxModel) << "\",";
            js << "\"score_threshold\":" << scoreThreshold << ',';
            js << "\"nms_threshold\":" << nmsThreshold;
            js << "},";

            js << "\"lidar\":{";
            js << "\"enabled\":" << (lidarEnabled ? "true" : "false") << ',';
            js << "\"port\":\"" << esc(lidarPort) << "\",";
            js << "\"baudrate\":" << lidarBaud << ',';
            js << "\"mock\":" << (lidarMock ? "true" : "false") << ',';
            js << "\"web_max_points\":" << lidarWebMaxPoints << ',';
            js << "\"max_distance_mm\":" << lidarMaxDistMm << ',';
            js << "\"avoid_distance_mm\":" << lidarAvoidMm;
            js << '}';

            js << '}';

            CATJ_webui_rt::HttpResponse r;
            r.status = 200;
            r.contentType = "application/json; charset=utf-8";
            r.body = js.str();
            return r;
        });

        auto doScan = [&]() {
            std::string scanErr;
            comms.refreshDevices(&scanErr);
            // broadcast du nouveau listing (si une page Comms est déjà ouverte)
            bridge.server().broadcastJsonEnvelope("comms_config", comms.configJson());
            if (!scanErr.empty()) {
                const auto msg = CATJ_webui_rt::WebUiRtServer::jsonEscape(scanErr);
                bridge.server().broadcastText(std::string("{\"type\":\"error\",\"message\":\"") + msg + "\"}");
            } else {
                bridge.server().broadcastText("{\"type\":\"info\",\"message\":\"COMMS scan OK\"}");
            }
        };

        bridge.server().registerPost("/api/comms_scan", [&](const CATJ_webui_rt::HttpRequest&) {
            doScan();
            CATJ_webui_rt::HttpResponse r;
            r.status = 200;
            r.contentType = "application/json; charset=utf-8";
            r.body = comms.configJson();
            return r;
        });

        bridge.server().registerGet("/api/comms_scan", [&](const CATJ_webui_rt::HttpRequest&) {
            doScan();
            CATJ_webui_rt::HttpResponse r;
            r.status = 200;
            r.contentType = "application/json; charset=utf-8";
            r.body = comms.configJson();
            return r;
        });

        // Broadcast config au démarrage (utile si la page est déjà connectée)
        bridge.server().broadcastJsonEnvelope("comms_config", comms.configJson());


        // -----------------------------------------------------------------
        // Dedicated Bus API: I2C address scan + register read/write + SPI reg R/W
        // -----------------------------------------------------------------
        bridge.server().registerGet("/api/bus_i2c_scan", [&](const CATJ_webui_rt::HttpRequest& req) {
            CATJ_webui_rt::HttpResponse r;
            r.status = 200;
            r.contentType = "application/json; charset=utf-8";

            const std::string dev = [&]() {
                auto it = req.query.find("device");
                return (it == req.query.end() || it->second.empty()) ? std::string("/dev/i2c-1") : it->second;
            }();
            const int a0 = [&]() { auto it = req.query.find("start"); return it == req.query.end() ? 0x03 : (int)parseU32Loose_(it->second, 0x03); }();
            const int a1 = [&]() { auto it = req.query.find("end");   return it == req.query.end() ? 0x77 : (int)parseU32Loose_(it->second, 0x77); }();

            std::ostringstream js;
            js << "{\"ok\":";
#if defined(_WIN32)
            js << "false,\"error\":\"I2C scan unsupported on Windows\"}";
#else
            js << "true,\"device\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(dev) << "\",\"addresses\":[";
            bool first = true;
            for (int addr = std::max(0x03, a0); addr <= std::min(0x77, a1); ++addr) {
                CATJ_i2c::I2cConfig ic;
                ic.device = dev;
                ic.slaveAddress = static_cast<uint16_t>(addr);
                ic.timeoutMs = 30;
                CATJ_i2c::I2cDevice d;
                if (!d.open(ic)) continue;
                uint8_t tmp = 0;
                bool ok = d.writeThenRead(nullptr, 0, &tmp, 1);
                if (!ok) {
                    // Some devices NACK reads but still exist. Try a zero-byte-like probe via reg write 0x00.
                    const uint8_t z = 0;
                    ok = (d.writeBytes(&z, 1) >= 0);
                }
                d.close();
                if (!ok) continue;
                if (!first) js << ',';
                first = false;
                js << "{\"addr\":" << addr << ",\"hex\":\"" << hex8_(static_cast<uint32_t>(addr)) << "\"}";
            }
            js << "]}";
#endif
            r.body = js.str();
            return r;
        });

        bridge.server().registerPost("/api/bus_i2c_reg_read", [&](const CATJ_webui_rt::HttpRequest& req) {
            const auto f = parseUrlEncoded_(req.body);
            const std::string deviceSpec = f.count("device_spec") ? f.at("device_spec") : std::string("/dev/i2c-1@0x42");
            const uint32_t reg = parseU32Loose_(f.count("reg") ? f.at("reg") : "0", 0);
            const size_t size = static_cast<size_t>(std::clamp<int>((int)parseU32Loose_(f.count("size") ? f.at("size") : "1", 1), 1, 256));

            CATJ_webui_rt::HttpResponse r;
            r.contentType = "application/json; charset=utf-8";
#if defined(_WIN32)
            r.status = 400;
            r.body = "{\"ok\":false,\"error\":\"I2C unsupported on Windows\"}";
#else
            std::string dev = "/dev/i2c-1"; uint16_t addr = 0x42;
            if (!parseI2cSpecMain_(deviceSpec, dev, addr)) {
                r.status = 400; r.body = "{\"ok\":false,\"error\":\"bad i2c spec\"}"; return r;
            }
            CATJ_i2c::I2cConfig ic; ic.device = dev; ic.slaveAddress = addr; ic.timeoutMs = 80;
            CATJ_i2c::I2cDevice d;
            if (!d.open(ic)) {
                r.status = 400; r.body = "{\"ok\":false,\"error\":\"I2C open failed\"}"; return r;
            }
            std::vector<uint8_t> rx; bool ok = d.readBlock(static_cast<uint8_t>(reg & 0xFFu), rx, size); d.close();
            if (!ok) {
                r.status = 400; r.body = "{\"ok\":false,\"error\":\"I2C register read failed\"}"; return r;
            }
            std::ostringstream js;
            js << "{\"ok\":true,\"device_spec\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(deviceSpec)
               << "\",\"reg\":\"" << hex8_(reg) << "\",\"hex\":\"" << bytesToHexMain_(rx.data(), rx.size())
               << "\",\"ascii\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(std::string(rx.begin(), rx.end())) << "\"}";
            r.body = js.str();
#endif
            return r;
        });

        bridge.server().registerPost("/api/bus_i2c_reg_write", [&](const CATJ_webui_rt::HttpRequest& req) {
            const auto f = parseUrlEncoded_(req.body);
            const std::string deviceSpec = f.count("device_spec") ? f.at("device_spec") : std::string("/dev/i2c-1@0x42");
            const uint32_t reg = parseU32Loose_(f.count("reg") ? f.at("reg") : "0", 0);
            const std::string payload = f.count("payload") ? f.at("payload") : std::string();
            const std::string encoding = f.count("encoding") ? f.at("encoding") : std::string("hex");
            CATJ_webui_rt::HttpResponse r; r.contentType = "application/json; charset=utf-8";
#if defined(_WIN32)
            r.status = 400; r.body = "{\"ok\":false,\"error\":\"I2C unsupported on Windows\"}";
#else
            std::string dev = "/dev/i2c-1"; uint16_t addr = 0x42;
            if (!parseI2cSpecMain_(deviceSpec, dev, addr)) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"bad i2c spec\"}"; return r; }
            std::vector<uint8_t> tx;
            bool okEnc = false;
            if (!encoding.empty() && (encoding == "hex" || encoding == "HEX")) okEnc = parseHexBytesMain_(payload, tx);
            else { std::string s = payload; tx.assign(s.begin(), s.end()); okEnc = true; }
            if (!okEnc) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"bad payload encoding\"}"; return r; }
            CATJ_i2c::I2cConfig ic; ic.device = dev; ic.slaveAddress = addr; ic.timeoutMs = 80;
            CATJ_i2c::I2cDevice d; if (!d.open(ic)) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"I2C open failed\"}"; return r; }
            const bool ok = d.writeBlock(static_cast<uint8_t>(reg & 0xFFu), tx); d.close();
            r.status = ok ? 200 : 400;
            r.body = ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"I2C register write failed\"}";
#endif
            return r;
        });

        bridge.server().registerPost("/api/bus_spi_reg_read", [&](const CATJ_webui_rt::HttpRequest& req) {
            const auto f = parseUrlEncoded_(req.body);
            const std::string deviceSpec = f.count("device_spec") ? f.at("device_spec") : std::string("/dev/spidev0.0@1000000#0+8");
            const uint32_t reg = parseU32Loose_(f.count("reg") ? f.at("reg") : "0", 0);
            const size_t size = static_cast<size_t>(std::clamp<int>((int)parseU32Loose_(f.count("size") ? f.at("size") : "1", 1), 1, 256));
            const int regWidth = std::clamp<int>((int)parseU32Loose_(f.count("reg_width") ? f.at("reg_width") : "8", 8), 8, 16);
            CATJ_webui_rt::HttpResponse r; r.contentType = "application/json; charset=utf-8";
#if defined(_WIN32)
            r.status = 400; r.body = "{\"ok\":false,\"error\":\"SPI unsupported on Windows\"}";
#else
            std::string dev = "/dev/spidev0.0"; uint32_t speed = 1000000; uint8_t mode = 0, bits = 8;
            if (!parseSpiSpecMain_(deviceSpec, dev, speed, mode, bits)) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"bad spi spec\"}"; return r; }
            CATJ_spi::SpiConfig sc; sc.device = dev; sc.speedHz = speed; sc.mode = mode; sc.bitsPerWord = bits;
            CATJ_spi::SpiDevice d; if (!d.open(sc)) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"SPI open failed\"}"; return r; }
            std::vector<uint8_t> rx; bool ok = false;
            if (regWidth <= 8) ok = d.readRegisterBlock8(static_cast<uint8_t>(reg & 0xFFu), rx, size);
            else ok = d.readRegisterBlock16(static_cast<uint16_t>(reg & 0xFFFFu), rx, size);
            d.close();
            if (!ok) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"SPI register read failed\"}"; return r; }
            std::ostringstream js;
            js << "{\"ok\":true,\"device_spec\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(deviceSpec)
               << "\",\"reg\":\"" << (regWidth <= 8 ? hex8_(reg) : hex16_(reg)) << "\",\"hex\":\"" << bytesToHexMain_(rx.data(), rx.size())
               << "\",\"ascii\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(std::string(rx.begin(), rx.end())) << "\"}";
            r.body = js.str();
#endif
            return r;
        });

        bridge.server().registerPost("/api/bus_spi_reg_write", [&](const CATJ_webui_rt::HttpRequest& req) {
            const auto f = parseUrlEncoded_(req.body);
            const std::string deviceSpec = f.count("device_spec") ? f.at("device_spec") : std::string("/dev/spidev0.0@1000000#0+8");
            const uint32_t reg = parseU32Loose_(f.count("reg") ? f.at("reg") : "0", 0);
            const int regWidth = std::clamp<int>((int)parseU32Loose_(f.count("reg_width") ? f.at("reg_width") : "8", 8), 8, 16);
            const std::string payload = f.count("payload") ? f.at("payload") : std::string();
            const std::string encoding = f.count("encoding") ? f.at("encoding") : std::string("hex");
            CATJ_webui_rt::HttpResponse r; r.contentType = "application/json; charset=utf-8";
#if defined(_WIN32)
            r.status = 400; r.body = "{\"ok\":false,\"error\":\"SPI unsupported on Windows\"}";
#else
            std::string dev = "/dev/spidev0.0"; uint32_t speed = 1000000; uint8_t mode = 0, bits = 8;
            if (!parseSpiSpecMain_(deviceSpec, dev, speed, mode, bits)) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"bad spi spec\"}"; return r; }
            std::vector<uint8_t> tx;
            bool okEnc = false;
            if (!encoding.empty() && (encoding == "hex" || encoding == "HEX")) okEnc = parseHexBytesMain_(payload, tx);
            else { std::string s = payload; tx.assign(s.begin(), s.end()); okEnc = true; }
            if (!okEnc) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"bad payload encoding\"}"; return r; }
            CATJ_spi::SpiConfig sc; sc.device = dev; sc.speedHz = speed; sc.mode = mode; sc.bitsPerWord = bits;
            CATJ_spi::SpiDevice d; if (!d.open(sc)) { r.status = 400; r.body = "{\"ok\":false,\"error\":\"SPI open failed\"}"; return r; }
            bool ok = false;
            if (regWidth <= 8) ok = d.writeRegisterBlock8(static_cast<uint8_t>(reg & 0xFFu), tx);
            else ok = d.writeRegisterBlock16(static_cast<uint16_t>(reg & 0xFFFFu), tx);
            d.close();
            r.status = ok ? 200 : 400;
            r.body = ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"SPI register write failed\"}";
#endif
            return r;
        });

        bridge.server().registerPost("/api/bus_raw_transfer", [&](const CATJ_webui_rt::HttpRequest& req) {
            const auto f = parseUrlEncoded_(req.body);
            CATJ_comms::SendRequest sreq;
            std::string transport = f.count("transport") ? f.at("transport") : std::string("i2c");
            for (auto& c : transport) c = (char)std::tolower((unsigned char)c);
            if (transport == "spi") sreq.transport = CATJ_comms::Transport::Spi;
            else sreq.transport = CATJ_comms::Transport::I2c;
            sreq.deviceSpec = f.count("device_spec") ? f.at("device_spec") : "";
            sreq.payload = f.count("payload") ? f.at("payload") : "";
            sreq.encoding = f.count("encoding") ? f.at("encoding") : "hex";
            sreq.appendNewline = false;
            CATJ_comms::ReplyOptions ro;
            ro.expectReply = true;
            ro.mode = "bytes";
            ro.timeoutMs = std::clamp((int)parseU32Loose_(f.count("timeout_ms") ? f.at("timeout_ms") : "200", 200), 10, 5000);
            ro.maxBytes = static_cast<size_t>(std::clamp<int>((int)parseU32Loose_(f.count("max_bytes") ? f.at("max_bytes") : "64", 64), 1, 4096));
            ro.clearRxBeforeSend = false;
            const auto rr = comms.sendEx(sreq, ro);
            CATJ_webui_rt::HttpResponse r; r.contentType = "application/json; charset=utf-8";
            if (!rr.ok) {
                r.status = 400;
                r.body = std::string("{\"ok\":false,\"error\":\"") + CATJ_webui_rt::WebUiRtServer::jsonEscape(rr.error) + "\"}";
            } else {
                std::ostringstream js;
                js << "{\"ok\":true,\"reply_hex\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(rr.replyHex)
                   << "\",\"reply_ascii\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(rr.replyAscii) << "\"}";
                r.body = js.str();
            }
            return r;
        });

        CATJ_lidar::RplidarC1 lidar;
        bool lidarOk = false;
        std::cout << "activation du LiDAR : " << (lidarEnabled ? "OUI" : "NON") << std::endl;
        if (lidarEnabled)
        {
            std::cout << "Initialisation du LiDAR..." << std::endl;
            CATJ_lidar::RplidarConfig lidarCfg;
            lidarCfg.port = lidarPort;
            lidarCfg.baudrate = static_cast<std::uint32_t>(std::max(115200, lidarBaud));
            lidarCfg.useMockData = lidarMock;
            lidarCfg.maxDistanceMm = lidarMaxDistMm;
            lidarCfg.frontHalfAngleDeg = lidarFrontHalfDeg;
            lidarCfg.lateralHalfAngleDeg = lidarLateralHalfDeg;

            lidarOk = lidar.open(lidarCfg);
            if (lidarOk) {
                lidarOk = lidar.startScan();
            }

            std::cout << "LiDAR initialisé sur " << lidarPort << " @" << lidarBaud << " bauds"
                << (lidarMock ? " (mock forcé)" : "") << std::endl;
        }

        bool camOk = false;
        if (streamCamera) {
            std::cout << "Initialisation de la caméra..." << std::endl;
            camOk = cam.startCapture(cam.getFps());
            if (!camOk) {
                std::cerr << "Impossible de lancer la capture camera" << std::endl;

                const bool cameraRequired =
                    (baseMode == CATJ_utility::programme_mode::camera) ||
                    (baseMode == CATJ_utility::programme_mode::calibration) ||
                    (baseMode == CATJ_utility::programme_mode::mesure) ||
                    (baseMode == CATJ_utility::programme_mode::debug);

                if (cameraRequired) {
                    if (lidarEnabled) lidar.close();
                    bridge.stop();
                    return -41;
                }
            }

            std::cout << "enable recording : " << (enableRecording ? "OUI" : "NON") << std::endl;
            if (enableRecording && camOk)
            {
                cam.startRecording(recPath, cam.getFps(), codec);
            }
        }
        else {
            std::cout << "Initialisation de la caméra... SKIP (WEB.stream camera = false)" << std::endl;
        }

        std::vector<cv::Point3f> calibObjTemplate;
        if (baseMode == CATJ_utility::programme_mode::calibration) {
            calibObjTemplate = cam.makeChessboard3D(cam.getEchiquierSizeCalibration(),
                cam.getSquareSizeCalibration());
        }
        cv::Mat calibOut;

        CATJ_robot_web::RobotTelemetry telem;
        telem.mode = CATJ_robot_web::RobotMode::Manual;
        telem.overlayEnabled = true;
        telem.remoteControlEnabled = allowNetworkControl;
        telem.statusText = std::string("remote: ") + CATJ_utility::modeToStr(baseMode);
        telem.autoState = "idle";
        telem.lidar.enabled = lidarEnabled;
        bridge.updateTelemetry(telem);

        int manualLeftPct = 0;
        int manualRightPct = 0;
        int manualConveyorPct = 0;
        int manualPanDeg = 0;
        int manualTiltDeg = 0;

        auto lastTs = std::chrono::steady_clock::now();

        std::cout << "Entrée dans la boucle principale. Appuyez sur Ctrl+C pour quitter." << std::endl;

        bool shouldQuit = false;

        while (bridge.isRunning() && !shouldQuit)
        {
            CATJ_robot_web::ControlEvent ev;
            while (bridge.popControlEvent(ev))
            {
                if (!ev.accepted) {
                    std::cout << "Commande refusée: " << ev.name << std::endl;
                    continue;
                }

                switch (ev.type) {
                case CATJ_robot_web::ControlEventType::SetMode:
                    std::cout << "Mode demandé via web: " << ev.name << std::endl;
                    break;

                case CATJ_robot_web::ControlEventType::Mission:
                    std::cout << "Mission: " << ev.name << std::endl;
                    break;

                case CATJ_robot_web::ControlEventType::SetThrusters:
                    manualLeftPct = ev.valueA;
                    manualRightPct = ev.valueB;
                    break;

                case CATJ_robot_web::ControlEventType::SetConveyor:
                    manualConveyorPct = ev.valueA;
                    break;

                case CATJ_robot_web::ControlEventType::CameraStep:
                    if (ev.name == "pan") {
                        manualPanDeg = std::clamp(manualPanDeg + ev.valueA, -90, 90);
                    }
                    else {
                        manualTiltDeg = std::clamp(manualTiltDeg + ev.valueA, -45, 45);
                    }
                    break;

                case CATJ_robot_web::ControlEventType::CameraCenter:
                    manualPanDeg = 0;
                    manualTiltDeg = 0;
                    break;

                case CATJ_robot_web::ControlEventType::CameraSetting:
                    std::cout << "Réglage caméra: " << ev.name << " = " << ev.valueA << std::endl;
                    break;

                case CATJ_robot_web::ControlEventType::ToggleOverlay:
                    std::cout << "Overlay: " << (ev.valueA ? "ON" : "OFF") << std::endl;
                    break;

                case CATJ_robot_web::ControlEventType::StopAll:
                    manualLeftPct = 0;
                    manualRightPct = 0;
                    manualConveyorPct = 0;
                    break;

                case CATJ_robot_web::ControlEventType::DrivePreset:
                    if (ev.name == "forward") {
                        manualLeftPct = 40;
                        manualRightPct = 40;
                    }
                    else if (ev.name == "backward") {
                        manualLeftPct = -40;
                        manualRightPct = -40;
                    }
                    else if (ev.name == "left") {
                        manualLeftPct = -25;
                        manualRightPct = 25;
                    }
                    else if (ev.name == "right") {
                        manualLeftPct = 25;
                        manualRightPct = -25;
                    }
                    break;


                case CATJ_robot_web::ControlEventType::CommsScan: {
                    // Scan devices (UART/BT) + broadcast config
                    doScan();
                    break;
                }

                case CATJ_robot_web::ControlEventType::CommsSend: {
                    // Envoi de trame depuis l'UI web (comms.html)
                    auto getS = [&](const char* k) -> std::string {
                        auto it = ev.rawFields.find(k);
                        return (it != ev.rawFields.end()) ? it->second : std::string();
                    };
                    auto getI = [&](const char* k, int def) -> int {
                        auto it = ev.rawFields.find(k);
                        if (it == ev.rawFields.end()) return def;
                        try { return std::stoi(it->second); } catch (...) { return def; }
                    };
                    auto getB = [&](const char* k, bool def) -> bool {
                        auto it = ev.rawFields.find(k);
                        if (it == ev.rawFields.end()) return def;
                        std::string v = it->second;
                        for (auto& c : v) c = (char)std::tolower((unsigned char)c);
                        v.erase(std::remove_if(v.begin(), v.end(), ::isspace), v.end());
                        if (v == "1" || v == "true" || v == "yes" || v == "on") return true;
                        if (v == "0" || v == "false" || v == "no" || v == "off") return false;
                        return def;
                    };

                    std::string transport = getS("transport");
                    for (auto& c : transport) c = (char)std::tolower((unsigned char)c);
                    const std::string device = getS("device");
                    const std::string payload = getS("payload");
                    std::string encoding = getS("encoding");
                    if (encoding.empty()) encoding = "ascii";
                    for (auto& c : encoding) c = (char)std::tolower((unsigned char)c);
                    const bool appendNl = getB("append_nl", false);

                    CATJ_comms::SendRequest req;
                    req.deviceSpec = device;
                    req.payload = payload;
                    req.encoding = encoding;
                    req.appendNewline = appendNl;

                    if (transport == "usb") req.transport = CATJ_comms::Transport::Usb;
                    else if (transport == "bluetooth" || transport == "bt") req.transport = CATJ_comms::Transport::Bluetooth;
                    else if (transport == "wifi") req.transport = CATJ_comms::Transport::Wifi;
                    else if (transport == "ethernet" || transport == "rj45") req.transport = CATJ_comms::Transport::Ethernet;
                    else if (transport == "i2c") req.transport = CATJ_comms::Transport::I2c;
                    else if (transport == "spi") req.transport = CATJ_comms::Transport::Spi;
                    else req.transport = CATJ_comms::Transport::Uart;

                    CATJ_comms::ReplyOptions ro;
                    ro.expectReply = getB("expect_reply", false);
                    ro.mode = getS("reply_mode");
                    if (ro.mode.empty()) ro.mode = "line";
                    ro.timeoutMs = std::clamp(getI("timeout_ms", 250), 10, 5000);
                    ro.maxBytes = (size_t)std::clamp(getI("max_bytes", 512), 16, 8192);
                    ro.clearRxBeforeSend = getB("clear_rx", true);

                    auto r = comms.sendEx(req, ro);
                    if (!r.ok) {
                        const auto msg = CATJ_webui_rt::WebUiRtServer::jsonEscape(r.error);
                        bridge.server().broadcastText(std::string("{\"type\":\"error\",\"message\":\"COMMS send failed: ") + msg + "\"}");
                    } else {
                        bridge.server().broadcastText("{\"type\":\"info\",\"message\":\"COMMS send OK\"}");
                        // Push last RX immediately (history est aussi dispo via /api/comms_history)
                        if (r.haveReply) {
                            std::ostringstream js;
                            js << "{\"transport\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(transport)
                               << "\",\"device\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(device)
                               << "\",\"reply_ascii\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(r.replyAscii)
                               << "\",\"reply_hex\":\"" << CATJ_webui_rt::WebUiRtServer::jsonEscape(r.replyHex)
                               << "\"}";
                            bridge.server().broadcastJsonEnvelope("comms_rx", js.str());
                        }
                        // broadcast config/histo optional
                    }
                    break;
                }
                case CATJ_robot_web::ControlEventType::quitProgram:
                    std::cout << "ControlEventType : quitProgram" << std::endl;
                    shouldQuit = true;
                    break;

                case CATJ_robot_web::ControlEventType::SetRemoteControlEnabled:
                case CATJ_robot_web::ControlEventType::RawWebCommand:
                default:
                    break;
                }

                if (shouldQuit) break;
            }

            cv::Mat frame;
            bool haveFrame = false;
            if (streamCamera && camOk) {
                haveFrame = cam.getLatestFrame(frame);
            }


            if (haveFrame && baseMode == CATJ_utility::programme_mode::calibration && !calibObjTemplate.empty()) {
                if (cam.calibrateCamera(true, calibObjTemplate, frame, &calibOut) && !calibOut.empty()) {
                    frame = calibOut;
                }
            }

            std::vector<CATJ_robot_web::Detection> webDets;
            if (haveFrame && visionEnabled) {
                std::vector<CATJ_camera::BallDetection> dets;
                cam.detectBalls(dets);
                webDets = convertDetections(dets);
                bridge.updateDetections(webDets);
            }

            telem = bridge.telemetry();
            telem.lidar.enabled = lidarEnabled;

            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTs).count() / 1000.0;
            lastTs = now;
            if (dt > 1e-6) {
                telem.vision.fps = 1.0 / dt;
            }

            // TODO: brancher IMU/compas ici
            telem.compass.headingDeg = 0.0;
            telem.compass.magneticNorthDeg = 0.0;
            telem.imu.rollDeg = 0.0;
            telem.imu.pitchDeg = 0.0;
            telem.imu.yawDeg = 0.0;
            telem.imu.yawRateDps = 0.0;
            telem.imu.gyroX_dps = 0.0;
            telem.imu.gyroY_dps = 0.0;
            telem.imu.gyroZ_dps = 0.0;
            telem.batteryV = 0.0;

            if (lidarEnabled && lidarOk) {
                fillWebLidarTelemetry(
                    telem.lidar,
                    lidar.snapshot(),
                    static_cast<std::size_t>(std::max(16, lidarWebMaxPoints)),
                    lidarMaxDistMm
                );
            }

            telem.vision.objectDetected = !webDets.empty();
            telem.vision.detectionCount = static_cast<int>(webDets.size());

            if (!webDets.empty()) {
                const auto& best = *std::max_element(webDets.begin(), webDets.end(),
                    [](const auto& a, const auto& b) { return a.confidence < b.confidence; });
                telem.vision.target = best.label;
                telem.vision.confidence = best.confidence;
            }
            else {
                telem.vision.target = "none";
                telem.vision.confidence = 0.0;
            }

            if (baseMode == CATJ_utility::programme_mode::camera && telem.mode == CATJ_robot_web::RobotMode::Auto) {
                telem.autoEngaged = true;
                telem.autoState = "tracking";

                const bool lidarAvoid = telem.lidar.enabled && telem.lidar.connected
                    && telem.lidar.frontMm > 0.0f && telem.lidar.frontMm < lidarAvoidMm;

                if (lidarAvoid) {
                    const int base = 18;
                    const bool goLeft = telem.lidar.leftMm > telem.lidar.rightMm;
                    telem.motors.leftPct = goLeft ? -base : base;
                    telem.motors.rightPct = goLeft ? base : -base;
                    telem.motors.conveyorPct = 0;
                    telem.statusText = "auto lidar avoidance";
                    telem.autoState = goLeft ? "avoid left" : "avoid right";
                }
                else if (!webDets.empty() && telem.missionEnabled && haveFrame) {
                    const auto& best = *std::max_element(webDets.begin(), webDets.end(),
                        [](const auto& a, const auto& b) { return a.confidence < b.confidence; });

                    const double boxCx = best.box.x + best.box.width * 0.5;
                    const double imageCx = frame.cols * 0.5;
                    const double err = (boxCx - imageCx) / std::max(1.0, imageCx);
                    const int turn = std::clamp(static_cast<int>(err * 70.0), -35, 35);
                    const int base = 35;

                    telem.motors.leftPct = std::clamp(base + turn, -100, 100);
                    telem.motors.rightPct = std::clamp(base - turn, -100, 100);
                    telem.motors.conveyorPct = 25;
                    telem.statusText = "auto tracking target";
                }
                else {
                    telem.motors.leftPct = 0;
                    telem.motors.rightPct = 0;
                    telem.motors.conveyorPct = 0;
                    telem.statusText = "auto waiting for target";
                }
            }
            else {
                telem.autoEngaged = false;
                telem.autoState = "manual";
                telem.motors.leftPct = manualLeftPct;
                telem.motors.rightPct = manualRightPct;
                telem.motors.conveyorPct = manualConveyorPct;
                telem.motors.camPanDeg = manualPanDeg;
                telem.motors.camTiltDeg = manualTiltDeg;
                telem.statusText = "manual remote control";
            }

            bridge.updateTelemetry(telem);

            if (haveFrame) {
                if (visionEnabled) bridge.updateVideoFrame(frame, webDets);
                else bridge.updateVideoFrame(frame);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "Arrêt du programme..." << std::endl;

        if (lidarEnabled) {
            lidar.close();
        }
        bridge.stop();
        cam.stopCapture();
        return 0;
    }


    int runRamassageMode(const std::filesystem::path& prjPath,
        CATJ_utility::iniReader& iniFile,
        CATJ_camera::Camera& cam,
        bool enableRecording,
        const std::string& recPath,
        const std::string& codec)
    {
        (void)prjPath;

        struct RamassageConfig {
            int durationSec = 300;
            int returnReserveSec = 30;
            int predictedScore = 100;
            int loopSleepMs = 60;
            int logPeriodMs = 1000;
            int heartbeatPeriodMs = 1000;
            int searchSpeedPct = 24;
            int trackSpeedPct = 40;
            int approachSpeedPct = 28;
            int avoidSpeedPct = 26;
            int returnSpeedPct = 20;
            float searchTurnDeg = 14.0f;
            float avoidTurnDeg = 28.0f;
            float targetTurnClampDeg = 35.0f;
            float targetTurnDeadbandDeg = 3.0f;
            float approachDistanceM = 0.75f;
            float rearPontoonDetectMm = 300.0f;
            float rearSlowdownMm = 700.0f;
            float frontAvoidMm = 650.0f;
            int searchFlipMs = 1200;
            float ballMinConfidence = 0.50f;
            float ballMinDistanceM = 0.10f;
            float ballMaxDistanceM = 6.00f;
            float ballOrangeDiameterM = 0.040f;
            float ballPiscineDiameterM = 0.070f;
            float hfovDeg = 62.2f;
        };

        RamassageConfig cfg;
        cfg.durationSec = iniFile.getOr("RAMASSAGE", "duration sec", cfg.durationSec);
        cfg.returnReserveSec = iniFile.getOr("RAMASSAGE", "return reserve sec", cfg.returnReserveSec);
        cfg.predictedScore = iniFile.getOr("RAMASSAGE", "predicted score", cfg.predictedScore);
        cfg.loopSleepMs = iniFile.getOr("RAMASSAGE", "loop sleep ms", cfg.loopSleepMs);
        cfg.logPeriodMs = iniFile.getOr("RAMASSAGE", "log period ms", cfg.logPeriodMs);
        cfg.heartbeatPeriodMs = iniFile.getOr("RAMASSAGE", "heartbeat period ms", cfg.heartbeatPeriodMs);
        cfg.searchSpeedPct = iniFile.getOr("RAMASSAGE", "search speed pct", cfg.searchSpeedPct);
        cfg.trackSpeedPct = iniFile.getOr("RAMASSAGE", "track speed pct", cfg.trackSpeedPct);
        cfg.approachSpeedPct = iniFile.getOr("RAMASSAGE", "approach speed pct", cfg.approachSpeedPct);
        cfg.avoidSpeedPct = iniFile.getOr("RAMASSAGE", "avoid speed pct", cfg.avoidSpeedPct);
        cfg.returnSpeedPct = iniFile.getOr("RAMASSAGE", "return speed pct", cfg.returnSpeedPct);
        cfg.searchTurnDeg = iniFile.getOr("RAMASSAGE", "search turn deg", cfg.searchTurnDeg);
        cfg.avoidTurnDeg = iniFile.getOr("RAMASSAGE", "avoid turn deg", cfg.avoidTurnDeg);
        cfg.targetTurnClampDeg = iniFile.getOr("RAMASSAGE", "target turn clamp deg", cfg.targetTurnClampDeg);
        cfg.targetTurnDeadbandDeg = iniFile.getOr("RAMASSAGE", "target turn deadband deg", cfg.targetTurnDeadbandDeg);
        cfg.approachDistanceM = iniFile.getOr("RAMASSAGE", "approach distance m", cfg.approachDistanceM);
        cfg.rearPontoonDetectMm = iniFile.getOr("RAMASSAGE", "rear pontoon detect mm", cfg.rearPontoonDetectMm);
        cfg.rearSlowdownMm = iniFile.getOr("RAMASSAGE", "rear slowdown mm", cfg.rearSlowdownMm);
        cfg.frontAvoidMm = iniFile.getOr("RAMASSAGE", "front avoid mm", cfg.frontAvoidMm);
        cfg.searchFlipMs = iniFile.getOr("RAMASSAGE", "search flip ms", cfg.searchFlipMs);
        cfg.ballMinConfidence = iniFile.getOr("RAMASSAGE", "ball min confidence", cfg.ballMinConfidence);
        cfg.ballMinDistanceM = iniFile.getOr("RAMASSAGE", "ball min distance m", cfg.ballMinDistanceM);
        cfg.ballMaxDistanceM = iniFile.getOr("RAMASSAGE", "ball max distance m", cfg.ballMaxDistanceM);
        cfg.ballOrangeDiameterM = iniFile.getOr("RAMASSAGE", "ball orange diameter m", cfg.ballOrangeDiameterM);
        cfg.ballPiscineDiameterM = iniFile.getOr("RAMASSAGE", "ball piscine diameter m", cfg.ballPiscineDiameterM);
        cfg.hfovDeg = iniFile.getOr("RAMASSAGE", "camera hfov deg", cfg.hfovDeg);

        const bool boardEnabled = iniFile.getOr("MOTHERBOARD", "enabled", true);
        const std::string boardPort = iniFile.getOr<std::string>("MOTHERBOARD", "port", "/dev/serial0");
        const int boardBaud = iniFile.getOr("MOTHERBOARD", "baudrate", 115200);
        const int boardReadTimeoutMs = iniFile.getOr("MOTHERBOARD", "read timeout ms", 20);
        const int boardWriteTimeoutMs = iniFile.getOr("MOTHERBOARD", "write timeout ms", 50);
        const int boardStatusTimeoutMs = iniFile.getOr("MOTHERBOARD", "status timeout ms", 10);

        if (!boardEnabled) {
            std::cerr << "Mode RAMASSAGE: carte mère désactivée dans [MOTHERBOARD]." << std::endl;
            return -60;
        }

        const bool lidarEnabled = iniFile.getOr("LIDAR", "enabled", true);
        const std::string lidarPort = iniFile.getOr<std::string>("LIDAR", "port", "/dev/ttyUSB0");
        const int lidarBaud = iniFile.getOr("LIDAR", "baudrate", 460800);
        const bool lidarMock = iniFile.getOr("LIDAR", "mock", false);
        const float lidarMaxDistMm = iniFile.getOr("LIDAR", "max distance mm", 8000.0f);
        const float lidarFrontHalfDeg = iniFile.getOr("LIDAR", "front half angle deg", 20.0f);
        const float lidarLateralHalfDeg = iniFile.getOr("LIDAR", "lateral half angle deg", 50.0f);

        CATJ_robot::MotherboardConfig boardCfg;
        boardCfg.uart.port = boardPort;
        boardCfg.uart.baudrate = boardBaud;
        boardCfg.uart.readTimeoutMs = boardReadTimeoutMs;
        boardCfg.uart.writeTimeoutMs = boardWriteTimeoutMs;
        boardCfg.statusTimeoutMs = boardStatusTimeoutMs;

        CATJ_robot::MotherboardLink board;
        if (!board.open(boardCfg)) {
            std::cerr << "Impossible d'ouvrir l'UART de la carte mère: " << boardPort << std::endl;
            return -61;
        }

        CATJ_lidar::RplidarC1 lidar;
        bool lidarOk = false;
        if (lidarEnabled) {
            CATJ_lidar::RplidarConfig lcfg;
            lcfg.port = lidarPort;
            lcfg.baudrate = static_cast<std::uint32_t>(std::max(115200, lidarBaud));
            lcfg.useMockData = lidarMock;
            lcfg.maxDistanceMm = lidarMaxDistMm;
            lcfg.frontHalfAngleDeg = lidarFrontHalfDeg;
            lcfg.lateralHalfAngleDeg = lidarLateralHalfDeg;
            lidarOk = lidar.open(lcfg);
            if (lidarOk) {
                lidar.startScan();
            }
            else {
                std::cerr << "LIDAR indisponible, poursuite sans évitement LIDAR." << std::endl;
            }
        }

        if (!cam.startCapture(cam.getFps())) {
            if (lidarEnabled && lidarOk) lidar.close();
            board.close();
            std::cerr << "Impossible de lancer la capture caméra." << std::endl;
            return -62;
        }

        if (enableRecording) {
            cam.startRecording(recPath, cam.getFps(), codec);
        }

        CATJ_robot::BallFilterConfig ballCfg;
        ballCfg.minConfidence = cfg.ballMinConfidence;
        ballCfg.minDistanceM = cfg.ballMinDistanceM;
        ballCfg.maxDistanceM = cfg.ballMaxDistanceM;
        ballCfg.orangeDiameterM = cfg.ballOrangeDiameterM;
        ballCfg.piscineDiameterM = cfg.ballPiscineDiameterM;
        ballCfg.hfovDeg = cfg.hfovDeg;

        CATJ_robot::BallLogic ballLogic(ballCfg);

        CATJ_robot::ModeCourse::Config courseCfg;
        courseCfg.distCollecteRapideM = iniFile.getOr("RAMASSAGE", "collect fast distance m", 0.50f);
        courseCfg.distBalleCollecteeM = iniFile.getOr("RAMASSAGE", "collect confirm distance m", 0.20f);
        courseCfg.scoreBaseCollecte = iniFile.getOr("RAMASSAGE", "score base collecte", 20);
        courseCfg.scoreBonusPonton = iniFile.getOr("RAMASSAGE", "score bonus ponton", 100);
        courseCfg.scoreBonusAfficheur = iniFile.getOr("RAMASSAGE", "score bonus afficheur", 50);
        courseCfg.conveyorNormalPct = iniFile.getOr("RAMASSAGE", "conveyor normal pct", 80);
        courseCfg.conveyorRapidePct = iniFile.getOr("RAMASSAGE", "conveyor rapide pct", 100);
        courseCfg.antiRebond = std::chrono::milliseconds(iniFile.getOr("RAMASSAGE", "anti rebond ms", 1500));

        CATJ_robot::ModeCourse course(courseCfg);
        course.prepare(cfg.predictedScore);
        course.start();
        course.setPontoonMonitoring(false);

        board.sendHeartbeat();
        board.sendMode(CATJ_robot::RobotMode::Ramassage);
        board.sendObjective(cfg.predictedScore);
        board.sendStopAll();

        std::cout << "Mode RAMASSAGE autonome" << std::endl;
        std::cout << "  - UART carte mère : " << boardPort << " @ " << boardBaud << std::endl;
        std::cout << "  - LIDAR           : " << (lidarEnabled ? (lidarMock ? "mock" : lidarPort) : "désactivé") << std::endl;
        std::cout << "  - Durée max       : " << cfg.durationSec << " s" << std::endl;
        std::cout << "  - Retour réserve  : " << cfg.returnReserveSec << " s" << std::endl;

        enum class CollectState {
            Search,
            Track,
            Avoid,
            ReturnHome,
            Finished
        };

        MotionCommandCache_ motionCache;
        CollectState state = CollectState::Search;
        int searchSign = 1;

        const auto tStart = std::chrono::steady_clock::now();
        auto tNextFlip = tStart + std::chrono::milliseconds(cfg.searchFlipMs);
        auto tNextLog = tStart;
        auto tNextHeartbeat = tStart;

        bool shouldQuit = false;

        while (!shouldQuit) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsedSec = std::chrono::duration_cast<std::chrono::milliseconds>(now - tStart).count() / 1000.0;
            const double remainingSec = std::max(0.0, static_cast<double>(cfg.durationSec) - elapsedSec);

            const bool returnPhase = remainingSec <= static_cast<double>(cfg.returnReserveSec);
            course.setPontoonMonitoring(returnPhase);

            if (now >= tNextHeartbeat) {
                board.sendHeartbeat();
                tNextHeartbeat = now + std::chrono::milliseconds(cfg.heartbeatPeriodMs);
            }

            cv::Mat frame;
            const bool haveFrame = cam.getLatestFrame(frame);

            std::vector<CATJ_camera::BallDetection> rawDetections;
            std::vector<CATJ_robot::RobotBall> robotBalls;
            if (haveFrame && cam.detectBalls(rawDetections)) {
                robotBalls = ballLogic.convert(cam, rawDetections, frame.cols, frame.rows);
            }

            CATJ_lidar::SectorDistances sectors;
            if (lidarEnabled && lidarOk) {
                sectors = lidar.snapshot().sectors;
            }

            course.update(robotBalls, std::isfinite(sectors.rearMm) ? sectors.rearMm : -1.0f, board);

            while (auto st = board.readStatusOnce(1)) {
                if (!st->rawLine.empty()) {
                    std::cout << "[MB] " << st->rawLine << std::endl;
                }
            }

            const auto bestTarget = ballLogic.bestTarget(robotBalls);

            MotionCommandMain_ cmd;
            cmd.move = CATJ_robot::MoveCommand::Stop;
            cmd.speedPct = 0;
            cmd.turnDeg = 0.0f;

            const bool obstacleFront = std::isfinite(sectors.frontMm) && sectors.frontMm > 0.0f && sectors.frontMm < cfg.frontAvoidMm;
            const bool pontoonRear = std::isfinite(sectors.rearMm) && sectors.rearMm > 0.0f && sectors.rearMm < cfg.rearPontoonDetectMm;

            if (returnPhase && (course.pontonDetected() || pontoonRear)) {
                state = CollectState::Finished;
                cmd.move = CATJ_robot::MoveCommand::Stop;
                cmd.speedPct = 0;
                cmd.turnDeg = 0.0f;
                shouldQuit = true;
            }
            else if (returnPhase) {
                state = CollectState::ReturnHome;
                if (now >= tNextFlip) {
                    searchSign = -searchSign;
                    tNextFlip = now + std::chrono::milliseconds(cfg.searchFlipMs);
                }

                cmd.move = CATJ_robot::MoveCommand::Backward;
                cmd.speedPct = (std::isfinite(sectors.rearMm) && sectors.rearMm < cfg.rearSlowdownMm)
                    ? std::max(10, cfg.returnSpeedPct / 2)
                    : cfg.returnSpeedPct;
                cmd.turnDeg = searchSign * (cfg.searchTurnDeg * 0.5f);
            }
            else if (obstacleFront) {
                state = CollectState::Avoid;
                const bool goLeft = !std::isfinite(sectors.leftMm) || (std::isfinite(sectors.rightMm) && sectors.leftMm > sectors.rightMm);
                cmd.move = CATJ_robot::MoveCommand::Forward;
                cmd.speedPct = cfg.avoidSpeedPct;
                cmd.turnDeg = goLeft ? -cfg.avoidTurnDeg : +cfg.avoidTurnDeg;
            }
            else if (bestTarget) {
                state = CollectState::Track;
                float targetTurn = std::clamp(bestTarget->angleDeg, -cfg.targetTurnClampDeg, cfg.targetTurnClampDeg);
                if (std::fabs(targetTurn) < cfg.targetTurnDeadbandDeg) targetTurn = 0.0f;
                cmd.move = CATJ_robot::MoveCommand::Forward;
                cmd.speedPct = (bestTarget->distanceM <= cfg.approachDistanceM) ? cfg.approachSpeedPct : cfg.trackSpeedPct;
                if (std::fabs(bestTarget->angleDeg) > 20.0f && bestTarget->distanceM < 0.80f) {
                    cmd.speedPct = std::max(12, cfg.approachSpeedPct / 2);
                }
                cmd.turnDeg = targetTurn;
            }
            else {
                state = CollectState::Search;
                if (now >= tNextFlip) {
                    searchSign = -searchSign;
                    tNextFlip = now + std::chrono::milliseconds(cfg.searchFlipMs);
                }
                cmd.move = CATJ_robot::MoveCommand::Forward;
                cmd.speedPct = cfg.searchSpeedPct;
                cmd.turnDeg = searchSign * cfg.searchTurnDeg;
            }

            applyMotionCommand_(board, motionCache, cmd);

            if (now >= tNextLog) {
                const int positivesVisible = static_cast<int>(std::count_if(robotBalls.begin(), robotBalls.end(), [](const auto& b) {
                    return b.isCollectible();
                }));

                std::cout << "[RAMASSAGE] t=" << std::fixed << std::setprecision(1) << elapsedSec
                    << "s | remaining=" << remainingSec
                    << "s | score=" << course.score()
                    << " | visibles=" << robotBalls.size()
                    << " | positives=" << positivesVisible
                    << " | front=" << (std::isfinite(sectors.frontMm) ? sectors.frontMm : 0.0f)
                    << "mm | rear=" << (std::isfinite(sectors.rearMm) ? sectors.rearMm : 0.0f)
                    << "mm | state=";

                switch (state) {
                case CollectState::Search: std::cout << "search"; break;
                case CollectState::Track: std::cout << "track"; break;
                case CollectState::Avoid: std::cout << "avoid"; break;
                case CollectState::ReturnHome: std::cout << "return"; break;
                case CollectState::Finished: std::cout << "finished"; break;
                }

                if (bestTarget) {
                    std::cout << " | target=" << CATJ_robot::BallLogic::toString(bestTarget->type)
                        << " @ " << bestTarget->distanceM << "m / " << bestTarget->angleDeg << "deg";
                }
                std::cout << std::endl;

                tNextLog = now + std::chrono::milliseconds(cfg.logPeriodMs);
            }

            const int key = cv::pollKey();
            if (key == 'q' || key == 'Q') {
                std::cout << "Arrêt demandé par l'utilisateur (q)." << std::endl;
                shouldQuit = true;
            }

            if (remainingSec <= 0.0) {
                std::cout << "Temps écoulé pour l'épreuve de ramassage." << std::endl;
                shouldQuit = true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(10, cfg.loopSleepMs)));
        }

        course.stop();
        applyMotionCommand_(board, motionCache, MotionCommandMain_{});
        board.sendStopAll();
        board.sendMode(CATJ_robot::RobotMode::Stop);

        std::cout << "Fin RAMASSAGE | score estimé=" << course.score()
            << " | retour ponton=" << (course.pontonDetected() ? "OUI" : "NON")
            << std::endl;

        cam.stopCapture();
        if (lidarEnabled && lidarOk) lidar.close();
        board.close();
        return 0;
    }


    int runLabyrintheMode(const std::filesystem::path& prjPath,
        CATJ_utility::iniReader& iniFile)
    {
        (void)prjPath;

        struct LabyrintheRuntimeConfig {
            int maxDurationSec = 180;
            int loopSleepMs = 60;
            int logPeriodMs = 1000;
            int heartbeatPeriodMs = 1000;
            int contactPenaltySec = 2;
        };

        LabyrintheRuntimeConfig runtimeCfg;
        runtimeCfg.maxDurationSec = iniFile.getOr("LABYRINTHE", "max duration sec", runtimeCfg.maxDurationSec);
        runtimeCfg.loopSleepMs = iniFile.getOr("LABYRINTHE", "loop sleep ms", runtimeCfg.loopSleepMs);
        runtimeCfg.logPeriodMs = iniFile.getOr("LABYRINTHE", "log period ms", runtimeCfg.logPeriodMs);
        runtimeCfg.heartbeatPeriodMs = iniFile.getOr("LABYRINTHE", "heartbeat period ms", runtimeCfg.heartbeatPeriodMs);
        runtimeCfg.contactPenaltySec = iniFile.getOr("LABYRINTHE", "contact penalty sec", runtimeCfg.contactPenaltySec);

        const bool boardEnabled = iniFile.getOr("MOTHERBOARD", "enabled", true);
        const std::string boardPort = iniFile.getOr<std::string>("MOTHERBOARD", "port", "/dev/serial0");
        const int boardBaud = iniFile.getOr("MOTHERBOARD", "baudrate", 460800);
        const int boardReadTimeoutMs = iniFile.getOr("MOTHERBOARD", "read timeout ms", 20);
        const int boardWriteTimeoutMs = iniFile.getOr("MOTHERBOARD", "write timeout ms", 50);
        const int boardStatusTimeoutMs = iniFile.getOr("MOTHERBOARD", "status timeout ms", 10);

        if (!boardEnabled) {
            std::cerr << "Mode LABYRINTHE: carte mère désactivée dans [MOTHERBOARD]." << std::endl;
            return -70;
        }

        const bool lidarEnabled = iniFile.getOr("LIDAR", "enabled", true);
        if (!lidarEnabled) {
            std::cerr << "Mode LABYRINTHE: le LIDAR doit être activé." << std::endl;
            return -71;
        }

        const std::string lidarPort = iniFile.getOr<std::string>("LIDAR", "port", "/dev/ttyUSB0");
        const int lidarBaud = iniFile.getOr("LIDAR", "baudrate", 460800);
        const bool lidarMock = iniFile.getOr("LIDAR", "mock", false);
        const float lidarMaxDistMm = iniFile.getOr("LIDAR", "max distance mm", 8000.0f);
        const float lidarFrontHalfDeg = iniFile.getOr("LIDAR", "front half angle deg", 20.0f);
        const float lidarLateralHalfDeg = iniFile.getOr("LIDAR", "lateral half angle deg", 50.0f);

        CATJ_robot::MotherboardConfig boardCfg;
        boardCfg.uart.port = boardPort;
        boardCfg.uart.baudrate = boardBaud;
        boardCfg.uart.readTimeoutMs = boardReadTimeoutMs;
        boardCfg.uart.writeTimeoutMs = boardWriteTimeoutMs;
        boardCfg.statusTimeoutMs = boardStatusTimeoutMs;

        CATJ_robot::MotherboardLink board;
        if (!board.open(boardCfg)) {
            std::cerr << "Impossible d'ouvrir l'UART de la carte mère: " << boardPort << std::endl;
            return -72;
        }

        CATJ_lidar::RplidarC1 lidar;
        CATJ_lidar::RplidarConfig lidarCfg;
        lidarCfg.port = lidarPort;
        lidarCfg.baudrate = static_cast<std::uint32_t>(std::max(115200, lidarBaud));
        lidarCfg.useMockData = lidarMock;
        lidarCfg.maxDistanceMm = lidarMaxDistMm;
        lidarCfg.frontHalfAngleDeg = lidarFrontHalfDeg;
        lidarCfg.lateralHalfAngleDeg = lidarLateralHalfDeg;

        if (!lidar.open(lidarCfg) || !lidar.startScan()) {
            board.close();
            std::cerr << "Impossible d'initialiser le LIDAR pour le mode labyrinthe." << std::endl;
            return -73;
        }

        CATJ_robot::ModeLabyrinthe::Config modeCfg;
        modeCfg.frontAvoidMm = iniFile.getOr("LABYRINTHE", "front avoid mm", modeCfg.frontAvoidMm);
        modeCfg.frontCriticalMm = iniFile.getOr("LABYRINTHE", "front critical mm", modeCfg.frontCriticalMm);
        modeCfg.sideContactMm = iniFile.getOr("LABYRINTHE", "side contact mm", modeCfg.sideContactMm);
        modeCfg.wallFollowMm = iniFile.getOr("LABYRINTHE", "wall follow mm", modeCfg.wallFollowMm);
        modeCfg.rearReturnMm = iniFile.getOr("LABYRINTHE", "rear return mm", modeCfg.rearReturnMm);
        modeCfg.wallToleranceMm = iniFile.getOr("LABYRINTHE", "wall tolerance mm", modeCfg.wallToleranceMm);
        modeCfg.turnAngleDeg = iniFile.getOr("LABYRINTHE", "turn angle deg", modeCfg.turnAngleDeg);
        modeCfg.wallCorrectionDeg = iniFile.getOr("LABYRINTHE", "wall correction deg", modeCfg.wallCorrectionDeg);
        modeCfg.speedPct = iniFile.getOr("LABYRINTHE", "speed pct", modeCfg.speedPct);
        modeCfg.avoidSpeedPct = iniFile.getOr("LABYRINTHE", "avoid speed pct", modeCfg.avoidSpeedPct);
        modeCfg.minRunBeforeReturnSec = iniFile.getOr("LABYRINTHE", "min run before return sec", modeCfg.minRunBeforeReturnSec);
        modeCfg.returnConfirmCycles = iniFile.getOr("LABYRINTHE", "return confirm cycles", modeCfg.returnConfirmCycles);
        modeCfg.contactDebounce = std::chrono::milliseconds(iniFile.getOr("LABYRINTHE", "contact debounce ms", 1000));

        CATJ_robot::ModeLabyrinthe laby(modeCfg);

        board.sendHeartbeat();
        board.sendMode(CATJ_robot::RobotMode::Labyrinthe);
        board.sendObjective(0);
        board.sendStopAll();

        std::cout << "Mode LABYRINTHE autonome" << std::endl;
        std::cout << "  - UART carte mère : " << boardPort << " @ " << boardBaud << std::endl;
        std::cout << "  - LIDAR           : " << (lidarMock ? "mock" : lidarPort) << " @ " << lidarBaud << std::endl;
        std::cout << "  - Timeout sécurité: " << runtimeCfg.maxDurationSec << " s" << std::endl;

        laby.start();

        const auto tStart = std::chrono::steady_clock::now();
        auto tNextLog = tStart;
        auto tNextHeartbeat = tStart;
        auto tFinish = tStart;

        bool shouldQuit = false;
        bool timedOut = false;

        while (!shouldQuit && laby.running()) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsedSec = std::chrono::duration_cast<std::chrono::milliseconds>(now - tStart).count() / 1000.0;

            if (now >= tNextHeartbeat) {
                board.sendHeartbeat();
                tNextHeartbeat = now + std::chrono::milliseconds(runtimeCfg.heartbeatPeriodMs);
            }

            const auto snap = lidar.snapshot();
            CATJ_robot::LidarDirections d;
            d.frontMm = snap.sectors.frontMm;
            d.frontRightMm = snap.sectors.frontRightMm;
            d.rightMm = snap.sectors.rightMm;
            d.leftMm = snap.sectors.leftMm;
            d.frontLeftMm = snap.sectors.frontLeftMm;
            d.rearMm = snap.sectors.rearMm;

            laby.update(d, board);

            auto status = board.readStatusOnce(1);
            (void)status;

            if (now >= tNextLog) {
                std::cout << "[LABYRINTHE] t=" << std::fixed << std::setprecision(1) << elapsedSec
                    << "s | state=" << laby.stateName()
                    << " | contacts=" << laby.contacts()
                    << " | front=" << (std::isfinite(d.frontMm) ? d.frontMm : 0.0f)
                    << "mm | right=" << (std::isfinite(d.rightMm) ? d.rightMm : 0.0f)
                    << "mm | left=" << (std::isfinite(d.leftMm) ? d.leftMm : 0.0f)
                    << "mm | rear=" << (std::isfinite(d.rearMm) ? d.rearMm : 0.0f)
                    << "mm" << std::endl;
                tNextLog = now + std::chrono::milliseconds(runtimeCfg.logPeriodMs);
            }

            const int key = cv::pollKey();
            if (key == 'q' || key == 'Q') {
                std::cout << "Arrêt demandé par l'utilisateur (q)." << std::endl;
                shouldQuit = true;
                break;
            }

            if (elapsedSec >= static_cast<double>(runtimeCfg.maxDurationSec)) {
                std::cout << "Timeout sécurité atteint en mode labyrinthe." << std::endl;
                timedOut = true;
                shouldQuit = true;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(10, runtimeCfg.loopSleepMs)));
        }

        tFinish = std::chrono::steady_clock::now();
        const double rawTimeSec = std::chrono::duration_cast<std::chrono::milliseconds>(tFinish - tStart).count() / 1000.0;
        const int contacts = laby.contacts();
        const double penaltySec = static_cast<double>(contacts * std::max(0, runtimeCfg.contactPenaltySec));
        const double finalTimeSec = rawTimeSec + penaltySec;
        const bool complete = laby.circuitComplete();

        laby.stop();
        board.sendStopAll();
        board.sendMode(CATJ_robot::RobotMode::Stop);
        lidar.close();
        board.close();

        if (complete) {
            std::cout << "Fin LABYRINTHE | chrono brut=" << std::fixed << std::setprecision(2) << rawTimeSec
                << " s | contacts=" << contacts
                << " | malus=" << penaltySec
                << " s | chrono final=" << finalTimeSec << " s" << std::endl;
            return 0;
        }

        std::cout << "Fin LABYRINTHE | circuit incomplet"
            << (timedOut ? " (timeout)" : "")
            << " | chrono brut=" << std::fixed << std::setprecision(2) << rawTimeSec
            << " s | contacts=" << contacts
            << " | malus théorique=" << penaltySec << " s" << std::endl;
        return 2;
    }

    int runCourseMode(const std::filesystem::path& prjPath,
        CATJ_utility::iniReader& iniFile)
    {
        (void)prjPath;

        struct CourseRuntimeConfig {
            int maxDurationSec = 120;
            int loopSleepMs = 60;
            int logPeriodMs = 1000;
            int heartbeatPeriodMs = 1000;
        };

        CourseRuntimeConfig runtimeCfg;
        runtimeCfg.maxDurationSec = iniFile.getOr("COURSE", "max duration sec", runtimeCfg.maxDurationSec);
        runtimeCfg.loopSleepMs = iniFile.getOr("COURSE", "loop sleep ms", runtimeCfg.loopSleepMs);
        runtimeCfg.logPeriodMs = iniFile.getOr("COURSE", "log period ms", runtimeCfg.logPeriodMs);
        runtimeCfg.heartbeatPeriodMs = iniFile.getOr("COURSE", "heartbeat period ms", runtimeCfg.heartbeatPeriodMs);

        const bool boardEnabled = iniFile.getOr("MOTHERBOARD", "enabled", true);
        const std::string boardPort = iniFile.getOr<std::string>("MOTHERBOARD", "port", "/dev/serial0");
        const int boardBaud = iniFile.getOr("MOTHERBOARD", "baudrate", 460800);
        const int boardReadTimeoutMs = iniFile.getOr("MOTHERBOARD", "read timeout ms", 20);
        const int boardWriteTimeoutMs = iniFile.getOr("MOTHERBOARD", "write timeout ms", 50);
        const int boardStatusTimeoutMs = iniFile.getOr("MOTHERBOARD", "status timeout ms", 10);

        if (!boardEnabled) {
            std::cerr << "Mode COURSE: carte mère désactivée dans [MOTHERBOARD]." << std::endl;
            return -80;
        }

        const bool lidarEnabled = iniFile.getOr("LIDAR", "enabled", true);
        if (!lidarEnabled) {
            std::cerr << "Mode COURSE: le LIDAR doit être activé." << std::endl;
            return -81;
        }

        const std::string lidarPort = iniFile.getOr<std::string>("LIDAR", "port", "/dev/ttyUSB0");
        const int lidarBaud = iniFile.getOr("LIDAR", "baudrate", 460800);
        const bool lidarMock = iniFile.getOr("LIDAR", "mock", false);
        const float lidarMaxDistMm = iniFile.getOr("LIDAR", "max distance mm", 8000.0f);
        const float lidarFrontHalfDeg = iniFile.getOr("LIDAR", "front half angle deg", 20.0f);
        const float lidarLateralHalfDeg = iniFile.getOr("LIDAR", "lateral half angle deg", 50.0f);

        CATJ_robot::MotherboardConfig boardCfg;
        boardCfg.uart.port = boardPort;
        boardCfg.uart.baudrate = boardBaud;
        boardCfg.uart.readTimeoutMs = boardReadTimeoutMs;
        boardCfg.uart.writeTimeoutMs = boardWriteTimeoutMs;
        boardCfg.statusTimeoutMs = boardStatusTimeoutMs;

        CATJ_robot::MotherboardLink board;
        if (!board.open(boardCfg)) {
            std::cerr << "Impossible d'ouvrir l'UART de la carte mère: " << boardPort << std::endl;
            return -82;
        }

        CATJ_lidar::RplidarC1 lidar;
        CATJ_lidar::RplidarConfig lidarCfg;
        lidarCfg.port = lidarPort;
        lidarCfg.baudrate = static_cast<std::uint32_t>(std::max(115200, lidarBaud));
        lidarCfg.useMockData = lidarMock;
        lidarCfg.maxDistanceMm = lidarMaxDistMm;
        lidarCfg.frontHalfAngleDeg = lidarFrontHalfDeg;
        lidarCfg.lateralHalfAngleDeg = lidarLateralHalfDeg;

        if (!lidar.open(lidarCfg) || !lidar.startScan()) {
            board.close();
            std::cerr << "Impossible d'initialiser le LIDAR pour le mode course." << std::endl;
            return -83;
        }

        CATJ_robot::ModeCourseDemiTour::Config modeCfg;
        modeCfg.markerApproachMm = iniFile.getOr("COURSE", "marker approach mm", modeCfg.markerApproachMm);
        modeCfg.markerCaptureMm = iniFile.getOr("COURSE", "marker capture mm", modeCfg.markerCaptureMm);
        modeCfg.markerTooCloseMm = iniFile.getOr("COURSE", "marker too close mm", modeCfg.markerTooCloseMm);
        modeCfg.homeDetectMm = iniFile.getOr("COURSE", "home detect mm", modeCfg.homeDetectMm);
        modeCfg.straightWallBalanceMm = iniFile.getOr("COURSE", "straight wall balance mm", modeCfg.straightWallBalanceMm);
        modeCfg.straightHeadingCorrectionDeg = iniFile.getOr("COURSE", "straight heading correction deg", modeCfg.straightHeadingCorrectionDeg);
        modeCfg.arcTurnDeg = iniFile.getOr("COURSE", "arc turn deg", modeCfg.arcTurnDeg);
        modeCfg.arcTightTurnDeg = iniFile.getOr("COURSE", "arc tight turn deg", modeCfg.arcTightTurnDeg);
        modeCfg.counterTurnDeg = iniFile.getOr("COURSE", "counter turn deg", modeCfg.counterTurnDeg);
        modeCfg.searchTurnDeg = iniFile.getOr("COURSE", "search turn deg", modeCfg.searchTurnDeg);
        modeCfg.frontContactMm = iniFile.getOr("COURSE", "front contact mm", modeCfg.frontContactMm);
        modeCfg.sideContactMm = iniFile.getOr("COURSE", "side contact mm", modeCfg.sideContactMm);
        modeCfg.speedStraightPct = iniFile.getOr("COURSE", "speed straight pct", modeCfg.speedStraightPct);
        modeCfg.speedApproachPct = iniFile.getOr("COURSE", "speed approach pct", modeCfg.speedApproachPct);
        modeCfg.speedArcPct = iniFile.getOr("COURSE", "speed arc pct", modeCfg.speedArcPct);
        modeCfg.speedCounterPct = iniFile.getOr("COURSE", "speed counter pct", modeCfg.speedCounterPct);
        modeCfg.speedReturnPct = iniFile.getOr("COURSE", "speed return pct", modeCfg.speedReturnPct);
        modeCfg.speedBackupPct = iniFile.getOr("COURSE", "speed backup pct", modeCfg.speedBackupPct);
        modeCfg.speedSearchPct = iniFile.getOr("COURSE", "speed search pct", modeCfg.speedSearchPct);
        modeCfg.clockwiseTurn = iniFile.getOr("COURSE", "clockwise turn", modeCfg.clockwiseTurn);
        modeCfg.fallbackMarkerSec = iniFile.getOr("COURSE", "fallback marker sec", modeCfg.fallbackMarkerSec);
        modeCfg.minHomeDetectSec = iniFile.getOr("COURSE", "min home detect sec", modeCfg.minHomeDetectSec);
        modeCfg.minReturnTravelSec = iniFile.getOr("COURSE", "min return travel sec", modeCfg.minReturnTravelSec);
        modeCfg.homeConfirmCycles = iniFile.getOr("COURSE", "home confirm cycles", modeCfg.homeConfirmCycles);
        modeCfg.backupDuration = std::chrono::milliseconds(iniFile.getOr("COURSE", "backup duration ms", 500));
        modeCfg.arcDuration = std::chrono::milliseconds(iniFile.getOr("COURSE", "arc duration ms", 2600));
        modeCfg.counterDuration = std::chrono::milliseconds(iniFile.getOr("COURSE", "counter duration ms", 1100));
        modeCfg.searchFlipPeriod = std::chrono::milliseconds(iniFile.getOr("COURSE", "search flip ms", 1200));
        modeCfg.contactDebounce = std::chrono::milliseconds(iniFile.getOr("COURSE", "contact debounce ms", 1000));

        CATJ_robot::ModeCourseDemiTour course(modeCfg);

        board.sendHeartbeat();
        board.sendMode(CATJ_robot::RobotMode::Course);
        board.sendObjective(0);
        board.sendStopAll();

        std::cout << "Mode COURSE autonome (demi-tour autour d'une balise)" << std::endl;
        std::cout << "  - UART carte mère : " << boardPort << " @ " << boardBaud << std::endl;
        std::cout << "  - LIDAR           : " << (lidarMock ? "mock" : lidarPort) << " @ " << lidarBaud << std::endl;
        std::cout << "  - Timeout sécurité: " << runtimeCfg.maxDurationSec << " s" << std::endl;
        std::cout << "  - Sens rotation   : " << (modeCfg.clockwiseTurn ? "horaire" : "anti-horaire") << std::endl;

        course.start();

        const auto tStart = std::chrono::steady_clock::now();
        auto tNextLog = tStart;
        auto tNextHeartbeat = tStart;
        auto tFinish = tStart;

        bool shouldQuit = false;
        bool timedOut = false;

        while (!shouldQuit && course.running()) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsedSec = std::chrono::duration_cast<std::chrono::milliseconds>(now - tStart).count() / 1000.0;

            if (now >= tNextHeartbeat) {
                board.sendHeartbeat();
                tNextHeartbeat = now + std::chrono::milliseconds(runtimeCfg.heartbeatPeriodMs);
            }

            const auto snap = lidar.snapshot();
            CATJ_robot::CourseLidarDirections d;
            d.frontMm = snap.sectors.frontMm;
            d.frontRightMm = snap.sectors.frontRightMm;
            d.rightMm = snap.sectors.rightMm;
            d.leftMm = snap.sectors.leftMm;
            d.frontLeftMm = snap.sectors.frontLeftMm;
            d.rearMm = snap.sectors.rearMm;

            course.update(d, board);

            while (auto status = board.readStatusOnce(1)) {
                if (!status->rawLine.empty()) {
                    std::cout << "[MB] " << status->rawLine << std::endl;
                }
            }

            if (now >= tNextLog) {
                std::cout << "[COURSE] t=" << std::fixed << std::setprecision(1) << elapsedSec
                    << "s | state=" << course.stateName()
                    << " | marker=" << (course.markerSeen() ? "yes" : "no")
                    << " | home=" << (course.homeSeen() ? "yes" : "no")
                    << " | contacts=" << course.contacts()
                    << " | front=" << (std::isfinite(d.frontMm) ? d.frontMm : 0.0f)
                    << "mm | right=" << (std::isfinite(d.rightMm) ? d.rightMm : 0.0f)
                    << "mm | left=" << (std::isfinite(d.leftMm) ? d.leftMm : 0.0f)
                    << "mm | rear=" << (std::isfinite(d.rearMm) ? d.rearMm : 0.0f)
                    << "mm" << std::endl;
                tNextLog = now + std::chrono::milliseconds(runtimeCfg.logPeriodMs);
            }

            const int key = cv::pollKey();
            if (key == 'q' || key == 'Q') {
                std::cout << "Arrêt demandé par l'utilisateur (q)." << std::endl;
                shouldQuit = true;
                break;
            }

            if (elapsedSec >= static_cast<double>(runtimeCfg.maxDurationSec)) {
                std::cout << "Timeout sécurité atteint en mode course." << std::endl;
                timedOut = true;
                shouldQuit = true;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(std::max(10, runtimeCfg.loopSleepMs)));
        }

        tFinish = std::chrono::steady_clock::now();
        const double rawTimeSec = std::chrono::duration_cast<std::chrono::milliseconds>(tFinish - tStart).count() / 1000.0;
        const int contacts = course.contacts();
        const bool complete = course.courseComplete();

        course.stop();
        board.sendStopAll();
        board.sendMode(CATJ_robot::RobotMode::Stop);
        lidar.close();
        board.close();

        if (complete) {
            std::cout << "Fin COURSE | demi-tour validé | chrono=" << std::fixed << std::setprecision(2) << rawTimeSec
                << " s | contacts estimés=" << contacts
                << " | balise=" << (course.markerSeen() ? "OUI" : "NON")
                << " | retour=" << (course.homeSeen() ? "OUI" : "NON")
                << std::endl;
            return 0;
        }

        std::cout << "Fin COURSE | parcours incomplet"
            << (timedOut ? " (timeout)" : "")
            << " | chrono=" << std::fixed << std::setprecision(2) << rawTimeSec
            << " s | contacts estimés=" << contacts
            << " | balise=" << (course.markerSeen() ? "OUI" : "NON")
            << " | retour=" << (course.homeSeen() ? "OUI" : "NON")
            << std::endl;
        return 2;
    }


    int runLidarMode(CATJ_utility::iniReader& iniFile)
    {
        const std::string lidarPort = iniFile.getOr<std::string>("LIDAR", "port", "/dev/ttyUSB0");
        const int lidarBaud = iniFile.getOr("LIDAR", "baudrate", 460800);
        const bool lidarMock = iniFile.getOr("LIDAR", "mock", false);
        const float lidarMaxDistMm = iniFile.getOr("LIDAR", "max distance mm", 8000.0f);
        const float lidarFrontHalfDeg = iniFile.getOr("LIDAR", "front half angle deg", 20.0f);
        const float lidarLateralHalfDeg = iniFile.getOr("LIDAR", "lateral half angle deg", 50.0f);

        CATJ_lidar::RplidarC1 lidar;
        CATJ_lidar::RplidarConfig cfg;
        cfg.port = lidarPort;
        cfg.baudrate = static_cast<std::uint32_t>(std::max(115200, lidarBaud));
        cfg.useMockData = lidarMock;
        cfg.maxDistanceMm = lidarMaxDistMm;
        cfg.frontHalfAngleDeg = lidarFrontHalfDeg;
        cfg.lateralHalfAngleDeg = lidarLateralHalfDeg;

        lidar.open(cfg);
        lidar.startScan();

        std::cout << "Mode LIDAR (q pour quitter dans la console via Ctrl+C si besoin)" << std::endl;

        for (int i = 0; i < 200; ++i) {
            const auto snap = lidar.snapshot();
            std::cout << "scan=" << snap.scanId
                << " hz=" << snap.scanHz
                << " samples=" << snap.sampleCount
                << " front=" << (std::isfinite(snap.sectors.frontMm) ? snap.sectors.frontMm : 0.0f)
                << " right=" << (std::isfinite(snap.sectors.rightMm) ? snap.sectors.rightMm : 0.0f)
                << " left=" << (std::isfinite(snap.sectors.leftMm) ? snap.sectors.leftMm : 0.0f)
                << (snap.mockMode ? " [mock]" : "")
                << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }

        lidar.close();
        return 0;
    }

} // namespace

int main()
{
    std::cout << "démarrage du programme" << std::endl;

    std::cout << "Initialisation des variables et objets" << std::endl;
    CATJ_utility::iniReader iniFile;
    CATJ_utility::fs::path prjPath = CATJ_utility::executable_dir();
    CATJ_utility::programme_mode mode = CATJ_utility::programme_mode::unknown;
    CATJ_error::error erreur;
    cv::Mat frame;
    cv::Mat calibOut;
    std::vector<cv::Point3f> objTemplate;
    std::vector<std::vector<cv::Point3f>> objPts;
    std::string str;
    MeasureState measTools;
    std::vector<CATJ_camera::BallDetection> dets;

    bool debugMode = false;
    int valA = 0, valB = 0, valC = 0;
    float fvalA = 0.0f, fvalB = 0.0f;

    std::cout << "Resolve project path : " << prjPath << std::endl;
    prjPath = resolveProjectPath(prjPath);
    if (prjPath.empty()) {
        std::cerr << "Erreur : impossible de trouver le répertoire du projet." << std::endl;
        return -10;
    }

    std::cout << "Récupération des paramètres dans le fichier .ini" << std::endl;
    check_negzerror_ret(iniFile.load((prjPath / "parametres" / "config.ini").string()),
        "Impossible de charger config.ini");

    iniFile.get("GENERAL", "debug flag", debugMode);
    iniFile.get("GENERAL", "programme mode", str);
    mode = CATJ_utility::strToMode(str);

	//mode camera par défaut si non spécifié, pour compatibilité avec les anciennes config.ini
    bool remoteEnabled = iniFile.getOr("WEB", "remote", false);

    if (mode == CATJ_utility::programme_mode::remote) {
        remoteEnabled = true;

        const std::string baseStr = iniFile.getOr<std::string>("GENERAL", "remote base mode", "camera");
        auto baseMode = CATJ_utility::strToMode(baseStr);
        if (baseMode == CATJ_utility::programme_mode::unknown ||
            baseMode == CATJ_utility::programme_mode::remote) {
            baseMode = CATJ_utility::programme_mode::camera;
        }

        std::cout << "Mode legacy 'remote' détecté. Base mode => "
            << CATJ_utility::modeToStr(baseMode) << std::endl;

        mode = baseMode;
    }

    const bool modeSupportsRemoteWeb =
        (mode == CATJ_utility::programme_mode::camera)
        || (mode == CATJ_utility::programme_mode::ramassage)
        || (mode == CATJ_utility::programme_mode::labyrinthe)
        || (mode == CATJ_utility::programme_mode::course)
        || (mode == CATJ_utility::programme_mode::calibration);

    if (remoteEnabled && !modeSupportsRemoteWeb) {
        std::cout << "WEB remote ignoré pour le mode "
            << CATJ_utility::modeToStr(mode)
            << " (mode local prioritaire)" << std::endl;
        remoteEnabled = false;
    }

    iniFile.get("CAMERA", "reference", str);
    iniFile.get("CAMERA", "device", valA);
    iniFile.get("CAMERA", "width", valB);
    iniFile.get("CAMERA", "height", valC);

    std::string camBackendStr = "auto";
    iniFile.get("CAMERA", "backend", camBackendStr);
    std::string customPipeline;
    iniFile.get("CAMERA", "csi pipeline", customPipeline);

    CATJ_camera::CameraBackend camBackend =
        CATJ_camera::cameraBackendFromString(camBackendStr);

    CATJ_camera::Camera cam(valA, valB, valC, camBackend, customPipeline);

    iniFile.get("CAMERA", "fps", valA);
    cam.setFps(valA);
    iniFile.get("CAMERA", "Aifps", valA);
    cam.setAiFps(valA);

    iniFile.get("CAMERA", "calibration file", str);
    cam.setCalibPath((prjPath / str).string());

    iniFile.get("CAMERA", "echiquier size", str);
    {
        const auto comma = str.find(',');
        if (comma != std::string::npos) {
            const int sx = std::stoi(str.substr(0, comma));
            const int sy = std::stoi(str.substr(comma + 1));
            cam.setEchiquierSizeCalibration({ sx, sy });
        }
    }

    iniFile.get("CAMERA", "square size", fvalA);
    cam.setSquareSizeCalibration(fvalA);

    iniFile.get("CAMERA", "flag recording", flagRecording);
    cam.setRecording(flagRecording);

    iniFile.get("CAMERA", "show image", showImage);
    cam.setShow(showImage);

    iniFile.get("CAMERA", "non maximum suppression threshold", fvalA);
    iniFile.get("CAMERA", "score threshold", fvalB);
    cam.setThresholds(fvalB, fvalA);

    std::string targetColorRaw = iniFile.getOr<std::string>("CAMERA", "target color", "red");
    std::string ignoreColorRaw = iniFile.getOr<std::string>("CAMERA", "ignore color", "blue");

    CATJ_camera::BallColor configuredTargetColor = parseConfiguredBallColor_(targetColorRaw, CATJ_camera::BallColor::Red);
    CATJ_camera::BallColor configuredIgnoreColor = parseConfiguredBallColor_(ignoreColorRaw, CATJ_camera::BallColor::Blue);

    if (configuredIgnoreColor == configuredTargetColor) {
        configuredIgnoreColor = (configuredTargetColor == CATJ_camera::BallColor::Red)
            ? CATJ_camera::BallColor::Blue
            : CATJ_camera::BallColor::Red;
    }

    cam.setColorDecision(configuredTargetColor, configuredIgnoreColor);
    std::cout << "Couleurs métier caméra : target=" << ballColorName_(configuredTargetColor)
        << " | ignore=" << ballColorName_(configuredIgnoreColor) << std::endl;

    if (iniFile.get("CAMERA", "hardware focal mm", fvalA)
        || iniFile.get("CAMERA", "hardware focale", fvalA)) {
        cam.setHWFocaleMm(fvalA);
    }

    iniFile.get("CAMERA", "nombres de prise pour la calibration", valA);
    cam.setNeededViews(valA);

    iniFile.get("CAMERA", "video codec", imgCodec);
    iniFile.get("CAMERA", "video ext", imgExt);
    iniFile.get("CAMERA", "recording file path", str);

    std::cout << "Initialisation du timestamp pour le nom de fichier d'enregistrement" << std::endl;
    trimTime.fromStdString(CATJ_utility::now_timestamp());
    trimTime.trim();

    std::cout << "Initialisation du nom de fichier d'enregistrement" << std::endl;
    const std::filesystem::path recDir = prjPath / str;
    const std::filesystem::path recFile = std::string("recording_") + trimTime + "." + imgExt;
    recordingPath = (recDir / recFile).string();

    std::cout << "Enregistrement vidéo : " << (flagRecording ? ("ON, path: " + recordingPath) : "OFF") << std::endl;

    const bool needsCalibration = (mode == CATJ_utility::programme_mode::camera)
        || (mode == CATJ_utility::programme_mode::mesure)
        || (mode == CATJ_utility::programme_mode::debug)
        || (mode == CATJ_utility::programme_mode::calibration)
        || (mode == CATJ_utility::programme_mode::ramassage);

    std::cout << "Calibration caméra : " << (needsCalibration ? "ON" : "OFF") << std::endl;

    const bool webVisionEnabled = iniFile.getOr("WEB", "vision enabled", (mode == CATJ_utility::programme_mode::camera));

    // L'IA est requise :
    //  - en mode caméra,
    //  - en mode ramassage,
    //  - ou dès que l'interface web distante demande la vision.
    // Important : cela permet de tester le modèle depuis la page web même si le mode programme
    // est encore "calibration" ou "debug".
    const bool needsDetection = (mode == CATJ_utility::programme_mode::camera)
        || (mode == CATJ_utility::programme_mode::ramassage)
        || (remoteEnabled && webVisionEnabled);

    if (needsCalibration) {
        cam.loadCalibration(cam.getcalibPath());
    }

    if (needsDetection) {
        std::cout << "Chargement du modèle de détection ONNX" << std::endl;
        iniFile.get("IA", "ONNX model path", str);
        std::filesystem::path modelPath = std::filesystem::path(str).is_absolute()
            ? std::filesystem::path(str)
            : (prjPath / str);

        check_negzerror_ret(std::filesystem::exists(modelPath),
            "ONNX introuvable: " + modelPath.string());

        check_negzerror_ret(cam.loadBallDetectorONNX(modelPath.string(), cam.getOnnx_InputSize()),
            "Impossible de charger le modèle ONNX: " + modelPath.string());
    }

    std::cout << "Mode sélectionné : " << CATJ_utility::modeToStr(mode)
        << " | remote=" << (remoteEnabled ? "ON" : "OFF") << std::endl;

    if (remoteEnabled) {
        std::cout << "Remote activé => démarrage de l'interface web" << std::endl;
        return runRemoteWebMode(prjPath, iniFile, cam, mode, flagRecording, recordingPath, imgCodec);
    }

    switch (mode)
    {
    case CATJ_utility::programme_mode::camera:
        std::cout << "Mode CAMERA\n";
        cam.startCapture(cam.getFps());
        if (flagRecording) {
            cam.startRecording(recordingPath, cam.getFps(), imgCodec);
        }

        cam.setThresholds(0.35f, 0.45f);
        cam.setAIFps(5);
        cam.setColorDecision(CATJ_camera::BallColor::Red, CATJ_camera::BallColor::Blue);

        while (cam.isOpen()) {
			std::cout << "debug" << std::endl;
            if (cam.detectBalls(dets)) {
                std::cout << "Balle détectée !\n" << std::endl;
            }
        }
        break;

    case CATJ_utility::programme_mode::calibration:
        std::cout << "Mode CALIBRATION\n";
        cam.startCapture(cam.getFps());
        if (flagRecording) {
            cam.startRecording(recordingPath, cam.getFps(), imgCodec);
        }

        objTemplate = cam.makeChessboard3D(cam.getEchiquierSizeCalibration(),
            cam.getSquareSizeCalibration());
        objPts = { objTemplate };

        std::cout << "Appuyez sur la touche 'q' pour quitter la calibration\n" << std::endl;
        while (cam.getKeyPolled() != 'q') {
            while (!cam.getLatestFrame(frame))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                std::cerr << "Mode calibration: aucune frame disponible pour l'instant\n";
            }

            if (cam.calibrateCamera(true, objTemplate, frame, &calibOut)) {
                std::cout << "Calibration success !\n";
                cv::imwrite((prjPath / "calibration_result.jpg").string(), calibOut);
                break;
            }
            else {
                std::cerr << "Echec calibration.\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;

    case CATJ_utility::programme_mode::labyrinthe:
        std::cout << "Mode LABYRINTHE\n";
        return runLabyrintheMode(prjPath, iniFile);

    case CATJ_utility::programme_mode::course:
        std::cout << "Mode COURSE\n";
        return runCourseMode(prjPath, iniFile);

    case CATJ_utility::programme_mode::ramassage:
        std::cout << "Mode RAMASSAGE\n";
        return runRamassageMode(prjPath, iniFile, cam, flagRecording, recordingPath, imgCodec);

    case CATJ_utility::programme_mode::lidar:
        std::cout << "Mode LIDAR\n";
        return runLidarMode(iniFile);

    case CATJ_utility::programme_mode::colorsensor:
        std::cout << "Mode COLORSENSOR\n";
        break;

    case CATJ_utility::programme_mode::gyro:
        std::cout << "Mode GYRO\n";
        break;

    case CATJ_utility::programme_mode::navigation:
        std::cout << "Mode NAVIGATION\n";
        break;

    case CATJ_utility::programme_mode::mesure:
        std::cout << "Mode MESURE\n";
        cam.startCapture(cam.getFps());
        measTools.drawing = false;
        measTools.hasLine = false;
        measTools.realSizeMm = 10;
        cam.runMeasureTool(cam, measTools);

        while (cam.isOpen()) {
            std::cout << "Dernière mesure : " << measTools.lastPx
                << " px, distance estimée : " << measTools.lastDistMm
                << " mm\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        break;

    case CATJ_utility::programme_mode::debug:
        std::cout << "Mode DEBUG/capture loop\n";
        cam.startCapture(cam.getFps());
        if (flagRecording) {
            cam.startRecording(recordingPath, cam.getFps(), imgCodec);
        }

        std::cout << "Appuyez sur la touche 'q' pour quitter le programme\n" << std::endl;
        while (cam.isOpen()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (cv::pollKey() == 'q') {
                cam.stopCapture();
                break;
            }
        }
        break;

    case CATJ_utility::programme_mode::remote:
        // Ne devrait plus arriver: remote est géré comme un flag + compat dans le parsing.
        std::cout << "Mode REMOTE (legacy)\n";
        return runRemoteWebMode(prjPath, iniFile, cam, CATJ_utility::programme_mode::camera, flagRecording, recordingPath, imgCodec);

    case CATJ_utility::programme_mode::unknown:
        set_error(-20, "Mode inconnu dans le config.ini\n");
        break;

    default:
        set_error(-30, "Mode inconnu dans le config.ini\n");
        break;
    }

    return 0;

err:
    return erreur.code;
}
