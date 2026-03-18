#include "main.h"
#include "../fonctions/web/robotOverlay/robot_web_bridge.h"

#include <algorithm>
#include <chrono>
#include <thread>

#define DIR_NAME "IBM_robot_SimuPC"

// Dir raspi Jerry
/*
#define DIR_NAME_LINUX "IBM_Bateau"
*/

// Dir raspi EPF
#define DIR_NAME_LINUX "jerryCamera"

#ifdef WIN32
#include <windows.h>
#endif

// Déclaration des variables globales historiques du projet
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

CATJ_utility::fs::path resolveProjectPath(CATJ_utility::fs::path exeDir)
{
#ifdef WIN32
    const char* expectedName = DIR_NAME;
#else
    const char* expectedName = DIR_NAME_LINUX;
#endif

    while (!exeDir.empty() && exeDir.filename() != expectedName) {
        exeDir = exeDir.parent_path();
    }
    return exeDir;
}

int runRemoteWebMode(const std::filesystem::path& prjPath,
                     CATJ_utility::iniReader& iniFile,
                     CATJ_camera::Camera& cam,
                     bool enableRecording,
                     const std::string& recPath,
                     const std::string& codec)
{
    int webPort = 8080;
    iniFile.get("WEB", "port", webPort);

    CATJ_webui_rt::WebUiRtConfig webCfg;
    webCfg.bindAddress = "0.0.0.0";
    webCfg.port = static_cast<uint16_t>(std::clamp(webPort, 1, 65535));
    webCfg.documentRoot = (prjPath / "sources" / "fonctions" / "web" / "frontend" / "robot_web_demo").string();
    webCfg.indexFile = "index.html";
    webCfg.telemetryPeriodMs = 100;
    webCfg.jpegQuality = 80;
    webCfg.allowNetworkControl = true;

    CATJ_robot_web::RobotWebBridge bridge;
    if (!bridge.start(webCfg, webCfg.documentRoot)) {
        std::cerr << "Impossible de lancer l'interface web distante" << std::endl;
        return -40;
    }

    std::cout << "Dashboard disponible sur : http://<IP_RASPI>:" << webCfg.port << std::endl;

    if (!cam.startCapture(cam.getFps())) {
        std::cerr << "Impossible de lancer la capture camera" << std::endl;
        bridge.stop();
        return -41;
    }

    if (enableRecording) {
        cam.startRecording(recPath, cam.getFps(), codec);
    }

    CATJ_robot_web::RobotTelemetry telem;
    telem.mode = CATJ_robot_web::RobotMode::Manual;
    telem.overlayEnabled = true;
    telem.remoteControlEnabled = true;
    telem.statusText = "remote ready";
    telem.autoState = "idle";
    bridge.updateTelemetry(telem);

    int manualLeftPct = 0;
    int manualRightPct = 0;
    int manualConveyorPct = 0;
    int manualPanDeg = 0;
    int manualTiltDeg = 0;

    auto lastTs = std::chrono::steady_clock::now();

    while (cam.isOpen() && bridge.isRunning()) {
        CATJ_robot_web::ControlEvent ev;
        while (bridge.popControlEvent(ev)) {
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
                // TODO: brancher ici le vrai driver des 2 moteurs d'hélices.
                break;

            case CATJ_robot_web::ControlEventType::SetConveyor:
                manualConveyorPct = ev.valueA;
                // TODO: brancher ici le vrai driver du tapis roulant.
                break;

            case CATJ_robot_web::ControlEventType::CameraStep:
                if (ev.name == "pan") {
                    manualPanDeg = std::clamp(manualPanDeg + ev.valueA, -90, 90);
                } else {
                    manualTiltDeg = std::clamp(manualTiltDeg + ev.valueA, -45, 45);
                }
                // TODO: brancher ici le moteur pas à pas / servo du support caméra.
                break;

            case CATJ_robot_web::ControlEventType::CameraCenter:
                manualPanDeg = 0;
                manualTiltDeg = 0;
                // TODO: recentrer physiquement le support caméra.
                break;

            case CATJ_robot_web::ControlEventType::CameraSetting:
                // TODO: raccorder aux vrais réglages caméra si tu exposes ces setters.
                std::cout << "Réglage caméra: " << ev.name << " = " << ev.valueA << std::endl;
                break;

            case CATJ_robot_web::ControlEventType::ToggleOverlay:
                std::cout << "Overlay: " << (ev.valueA ? "ON" : "OFF") << std::endl;
                break;

            case CATJ_robot_web::ControlEventType::StopAll:
                manualLeftPct = 0;
                manualRightPct = 0;
                manualConveyorPct = 0;
                // TODO: arrêt physique des moteurs.
                break;

            case CATJ_robot_web::ControlEventType::DrivePreset:
                if (ev.name == "forward") {
                    manualLeftPct = 40;
                    manualRightPct = 40;
                } else if (ev.name == "backward") {
                    manualLeftPct = -40;
                    manualRightPct = -40;
                } else if (ev.name == "left") {
                    manualLeftPct = -25;
                    manualRightPct = 25;
                } else if (ev.name == "right") {
                    manualLeftPct = 25;
                    manualRightPct = -25;
                }
                break;

            case CATJ_robot_web::ControlEventType::SetRemoteControlEnabled:
            case CATJ_robot_web::ControlEventType::RawWebCommand:
            default:
                break;
            }
        }

        cv::Mat frame;
        if (!cam.getLatestFrame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        std::vector<CATJ_camera::BallDetection> dets;
        cam.detectBalls(dets);
        const auto webDets = convertDetections(dets);

        telem = bridge.telemetry();

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTs).count() / 1000.0;
        lastTs = now;
        if (dt > 1e-6) {
            telem.vision.fps = 1.0 / dt;
        }

        // TODO: remplacer ces valeurs simulées par les vraies mesures IMU / compas.
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

        if (!webDets.empty()) {
            const auto& best = *std::max_element(webDets.begin(), webDets.end(),
                [](const auto& a, const auto& b) { return a.confidence < b.confidence; });
            telem.vision.target = best.label;
            telem.vision.confidence = best.confidence;
        } else {
            telem.vision.target = "none";
            telem.vision.confidence = 0.0;
        }

        if (telem.mode == CATJ_robot_web::RobotMode::Auto) {
            telem.autoEngaged = true;
            telem.autoState = "tracking";

            if (!webDets.empty() && telem.missionEnabled) {
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

                // TODO: brancher ici la vraie logique auto vers les moteurs.
            } else {
                telem.motors.leftPct = 0;
                telem.motors.rightPct = 0;
                telem.motors.conveyorPct = 0;
                telem.statusText = "auto waiting for target";
            }
        } else {
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
        bridge.updateVideoFrame(frame, webDets);

        if (cv::pollKey() == 'q') {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bridge.stop();
    cam.stopCapture();
    return 0;
}

} // namespace

int main()
{
    std::cout << "démarrage du programme" << std::endl;

    CATJ_utility::iniReader iniFile;
    CATJ_utility::fs::path prjPath = CATJ_utility::executable_dir();
    CATJ_utility::programme_mode mode = CATJ_utility::programme_mode::unknown;
    CATJ_error::error erreur;
    cv::Mat frame;
    cv::Mat calibOut;
    UINT32 colorTargetHex = 0;
    UINT32 colorIgnoreHex = 0;
    std::vector<cv::Point3f> objTemplate;
    std::vector<std::vector<cv::Point3f>> objPts;
    std::string str;
    MeasureState measTools;
    std::vector<CATJ_camera::BallDetection> dets;

    bool debugMode = false;
    int valA = 0, valB = 0, valC = 0;
    float fvalA = 0.0f, fvalB = 0.0f;

    prjPath = resolveProjectPath(prjPath);
    if (prjPath.empty()) {
        std::cerr << "Erreur : impossible de trouver le répertoire du projet." << std::endl;
        return -10;
    }

    std::cout << "Récupération des paramètres dans le fichier .ini" << std::endl;
    check_negzerror_ret(iniFile.load((prjPath / "parametres" / "config.ini").string()),
                        "Impossible de charger config.ini");

    // Récupération des données de config
    iniFile.get("GENERAL", "debug flag", debugMode);
    iniFile.get("GENERAL", "programme mode", str);
    mode = CATJ_utility::strToMode(str);

    // CAMERA
    iniFile.get("CAMERA", "reference", str);
    iniFile.get("CAMERA", "device", valA);
    iniFile.get("CAMERA", "width", valB);
    iniFile.get("CAMERA", "height", valC);
    CATJ_camera::Camera cam(valA, valB, valC);

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

    if (iniFile.get("CAMERA", "target color", str)) {
        colorTargetHex = 0;
        CATJ_utility::xstoi(str, colorTargetHex);
    }
    if (iniFile.get("CAMERA", "ignore color", str)) {
        colorIgnoreHex = 0;
        CATJ_utility::xstoi(str, colorIgnoreHex);
    }

    // Compatibilité avec les clés déjà utilisées dans ton config.ini.
    if (iniFile.get("CAMERA", "hardware focal mm", fvalA)
        || iniFile.get("CAMERA", "hardware focale", fvalA)) {
        cam.setHWFocaleMm(fvalA);
    }

    iniFile.get("CAMERA", "nombres de prise pour la calibration", valA);
    cam.setNeededViews(valA);

    iniFile.get("CAMERA", "video codec", imgCodec);
    iniFile.get("CAMERA", "video ext", imgExt);
    iniFile.get("CAMERA", "recording file path", str);

    trimTime.fromStdString(CATJ_utility::now_timestamp());
    trimTime.trim();

    const std::filesystem::path recDir = prjPath / str;
    const std::filesystem::path recFile = std::string("recording_") + trimTime + "." + imgExt;
    recordingPath = (recDir / recFile).string();

    // Chargement calibration / IA uniquement si nécessaire.
    const bool needsCalibration = (mode == CATJ_utility::programme_mode::camera)
        || (mode == CATJ_utility::programme_mode::remote)
        || (mode == CATJ_utility::programme_mode::mesure)
        || (mode == CATJ_utility::programme_mode::debug)
        || (mode == CATJ_utility::programme_mode::calibration);

    const bool needsDetection = (mode == CATJ_utility::programme_mode::camera)
        || (mode == CATJ_utility::programme_mode::remote);

    if (needsCalibration) {
        cam.loadCalibration(cam.getcalibPath());
    }

    if (needsDetection) {
        iniFile.get("IA", "ONNX model path", str);
        std::filesystem::path modelPath = std::filesystem::path(str).is_absolute()
            ? std::filesystem::path(str)
            : (prjPath / str);

        check_negzerror_ret(std::filesystem::exists(modelPath),
                            "ONNX introuvable: " + modelPath.string());

        check_negzerror_ret(cam.loadBallDetectorONNX(modelPath.string(), cam.getOnnx_InputSize()),
                            "Impossible de charger le modèle ONNX: " + modelPath.string());
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
            while (!cam.getLatestFrame(frame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if (cam.calibrateCamera(true, objTemplate, frame, &calibOut)) {
                std::cout << "Calibration success !\n";
                cv::imwrite((prjPath / "calibration_result.jpg").string(), calibOut);
                break;
            } else {
                std::cerr << "Echec calibration.\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        break;

    case CATJ_utility::programme_mode::course:
        std::cout << "Mode COURSE\n";
        break;

    case CATJ_utility::programme_mode::ramassage:
        std::cout << "Mode RAMASSAGE\n";
        break;

    case CATJ_utility::programme_mode::lidar:
        std::cout << "Mode LIDAR\n";
        break;

    case CATJ_utility::programme_mode::colorsensor:
        std::cout << "Mode COLORSENSOR\n";
        break;

    case CATJ_utility::programme_mode::gyro:
        std::cout << "Mode GYRO\n";
        break;

    case CATJ_utility::programme_mode::navigation:
        std::cout << "Mode NAVIGATION\n";
        break;

    case CATJ_utility::programme_mode::remote:
        std::cout << "Mode REMOTE\n";
        return runRemoteWebMode(prjPath, iniFile, cam, flagRecording, recordingPath, imgCodec);

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

    case CATJ_utility::programme_mode::unknown:
        set_error(-20, "Mode inconnu dans le config.ini\n");

    default:
        set_error(-30, "Mode inconnu dans le config.ini\n");
    }

    return 0;

err:
    return erreur.code;
}
