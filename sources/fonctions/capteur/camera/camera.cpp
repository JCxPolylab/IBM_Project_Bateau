#include "camera.h"

#include <algorithm>
#include <limits>

using namespace CATJ_camera;

static inline int makeOdd(int k) { return (k % 2 == 0) ? k + 1 : k; }

Camera::Camera(int device, int w, int h,
    CameraBackend backend,
    const std::string& customPipeline)
    : w_(w), h_(h), backend_(backend), customPipeline_(customPipeline)
{
#ifdef _WIN32
    if (!cap_.open(device)) {
        std::cerr << "Impossible d'ouvrir la camera (index=" << device << ")\n";
        return;
    }
#else
    bool opened = false;

    switch (backend_) {
    case CameraBackend::UsbV4L2:
        opened = openUsbV4L2_(device);
        break;

    case CameraBackend::CsiGStreamer:
        opened = openCsiGStreamer_(device, customPipeline_);
        break;

    case CameraBackend::Auto:
    default:
        // Auto : tente d'abord CSI, puis USB
        opened = openCsiGStreamer_(device, customPipeline_);
        if (!opened) {
            std::cerr << "Auto backend: fallback vers USB/V4L2\n";
            opened = openUsbV4L2_(device);
        }
        break;
    }

    if (!opened) {
        std::cerr << "Aucun backend camera n'a pu être ouvert.\n";
        return;
    }
#endif

    cv::Mat f;
    bool gotFrame = false;
    for (int i = 0; i < 20; ++i) {
        if (cap_.read(f) && !f.empty()) {
            gotFrame = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (gotFrame) {
        std::lock_guard<std::mutex> lk(m_);
        latest_ = f.clone();
        std::cout << "Constructor: première frame OK "
            << f.cols << "x" << f.rows
            << " type=" << f.type()
            << " channels=" << f.channels()
            << std::endl;
    }
    else {
        std::cerr << "Constructor: camera ouverte mais aucune frame initiale.\n";
    }
}


Camera::~Camera() {
    stopRecording();
    stopCapture();
    cap_.release();
    cv::destroyAllWindows();
}

bool Camera::openUsbV4L2_(int device)
{
    std::string devPath = "/dev/video" + std::to_string(device);
    std::cout << "Trying to open camera USB/V4L2 at " << devPath << " ...\n";

    if (!cap_.open(devPath, cv::CAP_V4L2)) {
        std::cerr << "Impossible d'ouvrir la camera USB/V4L2 (" << devPath << ")\n";
        return false;
    }

    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, w_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, h_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    return true;
}

std::string Camera::buildDefaultCsiPipeline_(int w, int h, int fps)
{
    return "libcamerasrc ! "
        "video/x-raw,width=" + std::to_string(w) +
        ",height=" + std::to_string(h) +
        ",framerate=" + std::to_string(std::max(1, fps)) + "/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false";
}

bool Camera::openCsiGStreamer_(int device, const std::string& customPipeline)
{
    (void)device; // pas utilisé pour la CSI via pipeline

    const std::string pipeline = customPipeline.empty()
        ? buildDefaultCsiPipeline_(w_, h_, static_cast<int>(fps_))
        : customPipeline;

    std::cout << "Trying to open camera CSI via GStreamer/libcamerasrc...\n";
    std::cout << pipeline << std::endl;

    if (!cap_.open(pipeline, cv::CAP_GSTREAMER)) {
        std::cerr << "Impossible d'ouvrir la camera CSI via GStreamer/libcamerasrc\n";
        return false;
    }

    return true;
}

bool Camera::isOpen() const { return cap_.isOpened();}

bool Camera::close() 
{
    if (isOpen()) 
    {
        cap_.release();
        return true;
    }
    return false;
}

bool Camera::startCapture(double fps)
{
    if (!isOpen() || running_) {
        std::cerr << "Camera non ouverte ou déjà en capture\n";
        return false;
    }

    fps_ = fps;

    cv::Mat test;
    bool gotFrame = false;
    for (int i = 0; i < 10; ++i) {
        if (cap_.read(test) && !test.empty()) {
            gotFrame = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!gotFrame) {
        std::cerr << "startCapture: première frame vide\n";
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(m_);
        latest_ = test.clone();
    }

    std::cout << "startCapture: première frame OK "
        << test.cols << "x" << test.rows
        << " channels=" << test.channels()
        << " type=" << test.type() << std::endl;

    running_ = true;
    th_ = std::thread(&Camera::captureLoop_, this);
    return true;
}

void Camera::stopCapture() {
    if (!running_) return;
    running_ = false;
    if (th_.joinable()) th_.join();
}

void Camera::captureLoop_()
{
    bool calibrationMode = true; // mets false si tu veux activer via une touche
    cv::Mat frame;
    auto old_time = std::chrono::steady_clock::now();
    auto new_time = std::chrono::steady_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(new_time - old_time).count();
	long long fps_tick = (int)(1000 / this->fps_);

	std::cout << "Image shown status : " << isImageShown_ << std::endl;
	std::cout << "save record status : " << recording_ << std::endl;

    while (running_)
    {
        old_time = std::chrono::steady_clock::now();
        cap_ >> frame;

        if (frame.empty()) {
            std::cerr << "captureLoop_: frame vide\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        cv::Mat view = frame;
        
        // 1) Si on a calibr�, on peut undistort pour stabiliser les mesures
        if (isCalibrated_ && undistortEnabled_) undistordFrame_(frame, view);

        if (!isCalibrated_ && !view.empty())
        {
            auto objTemplate = makeChessboard3D(EchiquierSize_, calib_squareSize_);
            if (!objTemplate.empty())
            {
                isCalibrated_ = calibrateCamera(1, objTemplate, view, &frame);
            }
        }
        
        if (isImageShown_)
        {
            cv::putText(frame, "Focal avg : " + std::to_string(focalavg_),
                { 460, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 2);
            cv::imshow("capture loop", frame);
            cv::putText(frame, "Focal x : " + std::to_string(focal_x_),
                { 460, 50 }, cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0,255,0 }, 2);
            cv::imshow("capture loop", frame);
            cv::putText(frame, "Focal y : " + std::to_string(focal_y_),
                { 460, 80 }, cv::FONT_HERSHEY_SIMPLEX, 0.5, { 0,255,0 }, 2);
            cv::imshow("capture loop", frame);

            keyPolled_.store(cv::pollKey());
        }

        std::lock_guard<std::mutex> lk(m_);
        latest_ = frame.clone();

        if (recording_ && writer_.isOpened()) writer_.write(frame);

		new_time = std::chrono::steady_clock::now();
        interval = std::chrono::duration_cast<std::chrono::milliseconds>(new_time - old_time).count();
        if(fps_tick > interval)
            std::this_thread::sleep_for(std::chrono::milliseconds(fps_tick - interval));
    }
}


bool Camera::getLatestFrame(cv::Mat& out) {
    std::lock_guard<std::mutex> lk(m_);
    if (latest_.empty()) return false;
    out = latest_.clone();
    return true;
}

bool Camera::startRecording(const std::string& filename, double fps, const std::string& fourcc)
{
    if (!isOpen()) return false;

    // Il faut déjà avoir une frame donc startCapture AVANT startRecording
    cv::Mat f;
    cap_ >> f;
    if (f.empty()) {
        std::cerr << "startRecording: frame vide (camera pas prête)\n";
        return false;
    }

    // fourcc doit faire 4 chars
    std::string cc = fourcc;
    if (cc.size() != 4) {
        std::cerr << "startRecording: FOURCC invalide '" << cc << "' (attendu 4 chars)\n";
        return false;
    }

    int four = cv::VideoWriter::fourcc(cc[0], cc[1], cc[2], cc[3]);

    std::cout << "Starting recording to " << filename
        << " at " << fps << " FPS with codec " << cc << "\n";

    bool ok = writer_.open(filename, four, fps, cv::Size(f.cols, f.rows), true);

    if (!ok) {
        std::cerr << "VideoWriter non ouvert (codec/format)\n";
        return false;
    }

    recording_ = true;
    return true;
}

// Pour la calibration voire : https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
// Fonction permettant de calibrer la camera � partir d'un �chiquier.
// objTemplate repr�sente les coordonn�es 3D des coins de l'�chiquier (g�n�ralement z=0, x et y selon la taille des cases).
// objTemplate peut �tre g�n�rer avec la fonction makeChessboard3D qui cr�e un template 3D bas� sur la taille de l'�chiquier et la taille des cases.
// frame est l'image actuelle de la cam�ra (utilis�e pour d�tecter les coins).
// out est optionnel, si fourni, la fonction y dessinera les coins d�tect�s et le nombre de vues prises.
bool Camera::calibrateCamera(bool savePicCalib , std::vector<cv::Point3f> objTemplate , cv::Mat frame , cv::Mat* out)
{
    // Detection de l'�chiquier de calibration
    std::vector<cv::Point2f> corners;
    bool found = false;

    cv::Mat gray;
    cv::Mat view;
    view = frame;


    cv::cvtColor(view, gray, cv::COLOR_BGR2GRAY);

    found = cv::findChessboardCorners(
        gray, EchiquierSize_, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH |
        cv::CALIB_CB_NORMALIZE_IMAGE |
        cv::CALIB_CB_FAST_CHECK
    );

    if (!found) {return false; }

    // Optimisation de la d�tection des coins pour + de pr�cision
    cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001)
    );
    cv::drawChessboardCorners(view, EchiquierSize_, corners, found);

    // Affiche combien de vues ont �t� prises
    cv::putText(view,
        "Calib views: " + std::to_string((int)imgPts_.size()) + "/" + std::to_string(neededViews_) +
        "  (press 's' to save view)",
        { 10, 30 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, { 0,255,0 }, 2);

    if (out != NULL)
    {
        *out = view;
    }

// 's' = sauvegarde une vue si �chiquier d�tect�
    if (!savePicCalib) return true;
   
    imgPts_.push_back(corners);
    objPts_.push_back(objTemplate);
    std::cout << "[CALIB] View saved: " << imgPts_.size() << "/" << neededViews_ << "\n";

    // Si on a assez de vues => calibrate
    if ((int)imgPts_.size() >= neededViews_)
    {
        K_ = cv::Mat::eye(3, 3, CV_64F);
        dist_ = cv::Mat::zeros(8, 1, CV_64F);
        std::vector<cv::Mat> rvecs, tvecs;

        double rms = cv::calibrateCamera(
            objPts_, imgPts_, frame.size(),
            K_, dist_, rvecs, tvecs
        );

        isCalibrated_ = true;
        map1_.release(); map2_.release(); // sera r�g�n�r� au besoin

        focal_x_ = K_.at<double>(0, 0);
        focal_y_ = K_.at<double>(1, 1);
        focalavg_ = 0.5 * (focal_x_ + focal_y_);

        std::cout << "\n[CALIB DONE] RMS=" << rms << "\n";
        std::cout << "K=\n" << K_ << "\n";
        std::cout << "dist=\n" << dist_.t() << "\n";
        std::cout << "fx=" << focal_x_ << "  fy=" << focal_y_ << "\n\n";

        // Sauvegarde pour r�utiliser sans recalibrer � chaque lancement
		if (calibPath_.empty()) return true;
        cv::FileStorage fs(calibPath_, cv::FileStorage::WRITE);
        fs << "K" << K_;
        fs << "dist" << dist_;
        fs.release();

        return true;
    }
    return false; 
}

bool Camera::loadCalibration(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["K"] >> K_;
    fs["dist"] >> dist_;
    fs.release();

    if (K_.empty() || dist_.empty()) return false;

    focal_x_ = K_.at<double>(0, 0);
    focal_y_ = K_.at<double>(1, 1);
    focalavg_ = 0.5 * (focal_x_ + focal_y_);

    isCalibrated_ = true;
    map1_.release(); map2_.release();
    return true;
}


bool Camera::undistordFrame_(const cv::Mat& in, cv::Mat& out)
{
    if (map1_.empty() || map2_.empty())
    {
        cv::initUndistortRectifyMap(K_, dist_, cv::Mat(), K_,
            in.size(), CV_16SC2, map1_, map2_);
    }
    cv::remap(in, out, map1_, map2_, cv::INTER_LINEAR);
    return true;
}


void Camera::stopRecording() {
    recording_ = false;
    if (writer_.isOpened()) writer_.release();
}


bool Camera::loadBallDetectorONNX(const std::string& onnxPath, int inputSize)
{
#ifdef CATJ_USE_ORT
    // Fallback si le mod�le ne donne pas de taille fixe
    onnx_inputSize_ = inputSize;

    try {
        if (!ortEnv_) {
            ortEnv_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "CATJ");
        }

        ortOpts_ = Ort::SessionOptions{};
        ortOpts_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        // Pour Pi4 / CPU : �vite trop de threads
        ortOpts_.SetIntraOpNumThreads(2);
        ortOpts_.SetInterOpNumThreads(1);

#ifdef _WIN32
        std::wstring wpath = std::filesystem::path(onnxPath).wstring();
        ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, wpath.c_str(), ortOpts_);
#else
        ortSession_ = std::make_unique<Ort::Session>(*ortEnv_, onnxPath.c_str(), ortOpts_);
#endif

        // R�cup�re les noms d'IO
        {
            auto in0 = ortSession_->GetInputNameAllocated(0, ortAllocator_);
            auto out0 = ortSession_->GetOutputNameAllocated(0, ortAllocator_);
            ortInputName_ = in0.get();
            ortOutputName_ = out0.get();
        }

        // ---- D�tection automatique de la taille attendue par le mod�le ----
        // On lit la shape de l'input 0. Typiquement: [1, 3, 320, 320]
        {
            Ort::TypeInfo ti = ortSession_->GetInputTypeInfo(0);
            auto tensorInfo = ti.GetTensorTypeAndShapeInfo();
            std::vector<int64_t> dims = tensorInfo.GetShape();

            std::cout << "[ORT] Input dims (" << ortInputName_ << "): ";
            for (auto d : dims) std::cout << d << " ";
            std::cout << "\n";

            // dims[2] = H, dims[3] = W (si mod�le NCHW)
            // Si dims sont dynamiques, ONNX renvoie souvent -1
            if (dims.size() == 4 && dims[2] > 0 && dims[3] > 0) {
                if (dims[2] != dims[3]) {
                    std::cout << "[ORT] Warning: input H != W (" << dims[2] << "x" << dims[3]
                        << "), je prends H.\n";
                }
                onnx_inputSize_ = static_cast<int>(dims[2]); // ex: 320
            }
            else {
                std::cout << "[ORT] Input size dynamique/indisponible -> fallback = "
                    << onnx_inputSize_ << "\n";
            }
        }

        ortLoaded_ = true;
        std::cout << "[ORT] Model loaded. Using inputSize=" << onnx_inputSize_ << "\n";
        return true;
    }
    catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime error: " << e.what() << "\n";
        ortLoaded_ = false;
        return false;
    }
#else
    (void)onnxPath; (void)inputSize;
    std::cerr << "ORT disabled at build time (CATJ_USE_ORT not defined)\n";
    return false;
#endif
}


cv::Mat Camera::letterbox_(const cv::Mat& src, int newSize, LetterboxInfo& info) const {
    int w = src.cols, h = src.rows;
    float r = std::min((float)newSize / w, (float)newSize / h);
    int newW = (int)std::round(w * r);
    int newH = (int)std::round(h * r);

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(newW, newH));

    int dw = newSize - newW;
    int dh = newSize - newH;
    int top = dh / 2, bottom = dh - top;
    int left = dw / 2, right = dw - left;

    cv::Mat out;
    cv::copyMakeBorder(resized, out, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    info = { r, left, top, newW, newH };
    return out;
}

cv::Rect Camera::mapBoxBack_(const cv::Rect& b, const LetterboxInfo& info, int origW, int origH) const {
    // bbox b est en coords "letterbox onnx_inputSize_"
    float x = (b.x - info.dw) / info.r;
    float y = (b.y - info.dh) / info.r;
    float w = b.width / info.r;
    float h = b.height / info.r;

    int x1 = (int)std::round(x);
    int y1 = (int)std::round(y);
    int x2 = (int)std::round(x + w);
    int y2 = (int)std::round(y + h);

    x1 = std::clamp(x1, 0, origW - 1);
    y1 = std::clamp(y1, 0, origH - 1);
    x2 = std::clamp(x2, 0, origW - 1);
    y2 = std::clamp(y2, 0, origH - 1);

    return cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
}

void Camera::decodeDetections_(const cv::Mat& outRaw, const LetterboxInfo& info, int origW, int origH,
    std::vector<cv::Rect>& boxes, std::vector<float>& scores) const
{
    // Formats supportes :
    // 1) YOLOv8 / YOLOv11 brut : (1, 4+nc, N) -> [cx,cy,w,h, cls0, cls1, ...]
    // 2) YOLOv5-like brut :      (1, N, 5+nc) -> [cx,cy,w,h, obj, cls0, cls1, ...]
    // 3) Export NMS/end-to-end : (1, N, 6)    -> [x1,y1,x2,y2, score, class]
    // 4) Variante NMS :          (1, N, 6)    -> [x1,y1,x2,y2, class, score]


    boxes.clear();
    scores.clear();

    if (outRaw.empty()) {
        return;
    }


    cv::Mat out;

    if (outRaw.dims == 3) {
        const int d1 = outRaw.size[1];
        const int d2 = outRaw.size[2];

        if (d1 <= 512 && d2 > d1) {
            // [1,C,N] -> [N,C]
            cv::Mat m(d1, d2, CV_32F, const_cast<float*>(outRaw.ptr<float>()));
            cv::transpose(m, out);
        } else {
            // [1,N,C] -> [N,C]
            out = cv::Mat(d1, d2, CV_32F, const_cast<float*>(outRaw.ptr<float>())).clone();
        }
    } else if (outRaw.dims == 2) {
        out = outRaw;
    } else {
        return;
    }

    const int N = out.rows;
    const int C = out.cols;

    if (C < 5 || N <= 0) {
        return;
    }

    auto isNormalizedCoord = [](float v) -> bool {
        return std::isfinite(v) && v >= -0.25f && v <= 1.25f;
    };

    auto scaleCoord = [&](float v) -> float {
        if (isNormalizedCoord(v)) return v * static_cast<float>(onnx_inputSize_);
        return v;
    };

    auto pushBoxCxCyWh = [&](float cx, float cy, float w, float h, float score) {
        cx = scaleCoord(cx);
        cy = scaleCoord(cy);
        w  = scaleCoord(w);
        h  = scaleCoord(h);
        if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h)) return;
        if (w <= 1.0f || h <= 1.0f) return;
        cv::Rect boxLB(
            static_cast<int>(std::round(cx - w * 0.5f)),
            static_cast<int>(std::round(cy - h * 0.5f)),
            static_cast<int>(std::round(w)),
            static_cast<int>(std::round(h))
        );
        boxLB &= cv::Rect(0, 0, onnx_inputSize_, onnx_inputSize_);
        if (boxLB.area() <= 0) return;
        cv::Rect box = mapBoxBack_(boxLB, info, origW, origH);
        if (box.area() <= 0) return;
        boxes.push_back(box);
        scores.push_back(score);
    };

    auto pushBoxXyXy = [&](float x1, float y1, float x2, float y2, float score) {
        x1 = scaleCoord(x1);
        y1 = scaleCoord(y1);
        x2 = scaleCoord(x2);
        y2 = scaleCoord(y2);
        if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(x2) || !std::isfinite(y2)) return;
        if (x2 <= x1 || y2 <= y1) return;
        cv::Rect boxLB(
            cv::Point(static_cast<int>(std::round(x1)), static_cast<int>(std::round(y1))),
            cv::Point(static_cast<int>(std::round(x2)), static_cast<int>(std::round(y2)))
        );
        boxLB &= cv::Rect(0, 0, onnx_inputSize_, onnx_inputSize_);
        if (boxLB.area() <= 0) return;
        cv::Rect box = mapBoxBack_(boxLB, info, origW, origH);
        if (box.area() <= 0) return;
        boxes.push_back(box);
        scores.push_back(score);
    };


    for (int i = 0; i < N; ++i) {
        const float* p = out.ptr<float>(i);
        float score = 0.0f;
        bool coordsAreXyxy = false;
        int classId = -1;

        if (C == 5) {
            score = p[4];
            coordsAreXyxy = false;
            classId = 0;
        } else if (C == 6) {
            const bool looksLikeXyxy = (scaleCoord(p[2]) > scaleCoord(p[0])) && (scaleCoord(p[3]) > scaleCoord(p[1]));
            if (looksLikeXyxy) {
                const bool p4LooksScore = std::isfinite(p[4]) && p[4] >= 0.0f && p[4] <= 1.5f;
                const bool p5LooksScore = std::isfinite(p[5]) && p[5] >= 0.0f && p[5] <= 1.5f;
                if (p4LooksScore && !p5LooksScore) {
                    score = p[4];
                    classId = static_cast<int>(std::round(p[5]));
                } else if (!p4LooksScore && p5LooksScore) {
                    score = p[5];
                    classId = static_cast<int>(std::round(p[4]));
                } else {
                    score = p[4];
                    classId = static_cast<int>(std::round(p[5]));
                }
                coordsAreXyxy = true;
            } else {
                score = std::max(p[4], p[4] * p[5]);
                classId = 0;
                coordsAreXyxy = false;
            }
        } else {
            float yolo8Max = 0.0f;
            int yolo8Class = -1;
            for (int c = 4; c < C; ++c) {
                if (p[c] > yolo8Max) {
                    yolo8Max = p[c];
                    yolo8Class = c - 4;
                }
            }
            float clsMaxAfterObj = 0.0f;
            int yolo5Class = -1;
            for (int c = 5; c < C; ++c) {
                if (p[c] > clsMaxAfterObj) {
                    clsMaxAfterObj = p[c];
                    yolo5Class = c - 5;
                }
            }
            const float yolo5Score = p[4] * clsMaxAfterObj;
            if (yolo8Max >= yolo5Score) {
                score = yolo8Max;
                classId = yolo8Class;
            } else {
                score = yolo5Score;
                classId = yolo5Class;
            }
            coordsAreXyxy = false;
        }

        if (!std::isfinite(score) || score < confTh_) continue;

        if (coordsAreXyxy) pushBoxXyXy(p[0], p[1], p[2], p[3], score);
        else pushBoxCxCyWh(p[0], p[1], p[2], p[3], score);
    }

}

BallColor Camera::classifyColorHSV_(const cv::Mat& frame, const cv::Rect& box) const {
    cv::Rect b = box & cv::Rect(0, 0, frame.cols, frame.rows);
    if (b.area() <= 0) return BallColor::Unknown;

    cv::Mat roi = frame(b);
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red1, red2, red, blue, white;

    // Rouge (2 plages en Hue)
    cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), red1);
    cv::inRange(hsv, cv::Scalar(170, 80, 80), cv::Scalar(179, 255, 255), red2);
    red = red1 | red2;

    // Bleu
    cv::inRange(hsv, cv::Scalar(95, 80, 80), cv::Scalar(130, 255, 255), blue);

    // Blanc : saturation faible + valeur �lev�e
    cv::inRange(hsv, cv::Scalar(0, 0, 170), cv::Scalar(179, 60, 255), white);

    auto ratio = [&](const cv::Mat& m) {
        return (float)cv::countNonZero(m) / (float)(m.rows * m.cols);
        };

    float rR = ratio(red);
    float rB = ratio(blue);
    float rW = ratio(white);

    // seuils � ajuster selon tes balles r�elles
    if (rR > 0.06f) return BallColor::Red;
    if (rB > 0.06f) return BallColor::Blue;
    if (rW > 0.10f) return BallColor::White;
    return BallColor::Unknown;
}

BallDecision Camera::decide_(BallColor c) const {
    if (c == targetColor_) return BallDecision::Target;
    if (c == ignoreColor_) return BallDecision::Ignore;
    return BallDecision::Unknown;
}

/*
bool Camera::detectBalls(std::vector<BallDetection>& outDets) {
    outDets.clear();
    if (!netLoaded_) return false;

    // --- Throttle IA : 5 FPS -> une inf�rence toutes les 200 ms ---
    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(1000 / aiFps_);

    if (lastInfer_ != std::chrono::steady_clock::time_point::min() &&
        (now - lastInfer_) < period) {
        // trop t�t -> renvoyer cache
        outDets = lastDets_;
        return true;
    }
    lastInfer_ = now;

    cv::Mat frame;
    if (!getLatestFrame(frame)) return false;

    // --- inf�rence ---
    LetterboxInfo info{};
    cv::Mat lb = letterbox_(frame, onnx_inputSize_, info);

    cv::Mat blob = cv::dnn::blobFromImage(lb, 1.0 / 255.0, cv::Size(onnx_inputSize_, onnx_inputSize_),
        cv::Scalar(), true, false);

    net_.setInput(blob);

    std::vector<cv::Mat> outs;
    net_.forward(outs, net_.getUnconnectedOutLayersNames());
    if (outs.empty()) return false;

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    decodeDetections_(outs[0], info, frame.cols, frame.rows, boxes, scores);

    std::vector<int> idx;
    cv::dnn::NMSBoxes(boxes, scores, confTh_, nmsTh_, idx);

    std::vector<BallDetection> dets;
    dets.reserve(idx.size());

    for (int i : idx) {
        BallDetection d;
        d.box = boxes[i];
        d.conf = scores[i];
        d.color = classifyColorHSV_(frame, d.box);
        d.decision = decide_(d.color);
        dets.push_back(d);
    }

    // cache
    lastDets_ = dets;
    outDets = std::move(dets);
    return true;
}*/


//ORT
/*
bool Camera::detectBalls(std::vector<BallDetection>& outDets)
{
    outDets.clear();
    if (!ortLoaded_) return false;

    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(1000 / aiFps_);

    // renvoie le cache, mais le bool indique s'il y a une d�tection
    if (lastInfer_ != std::chrono::steady_clock::time_point::min() &&
        (now - lastInfer_) < period)
    {
        outDets = lastDets_;
        return !outDets.empty();
    }

    cv::Mat frame;
    if (!getLatestFrame(frame)) return false;

    LetterboxInfo info{};
    cv::Mat lb = letterbox_(frame, onnx_inputSize_, info);

    cv::Mat blob = cv::dnn::blobFromImage(
        lb, 1.0 / 255.0, cv::Size(onnx_inputSize_, onnx_inputSize_),
        cv::Scalar(), true, false, CV_32F
    );

    float* inputData = (float*)blob.ptr<float>();
    const size_t inputTensorSize = (size_t)blob.total();

    std::array<int64_t, 4> inputShape = { 1, 3, onnx_inputSize_, onnx_inputSize_ };

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        ortMemInfo_, inputData, inputTensorSize,
        inputShape.data(), inputShape.size()
    );

    const char* inNames[] = { ortInputName_.c_str() };
    const char* outNames[] = { ortOutputName_.c_str() };

    std::vector<Ort::Value> outputs;
    try {
        outputs = ortSession_->Run(
            Ort::RunOptions{ nullptr },
            inNames, &inputTensor, 1,
            outNames, 1
        );
    }
    catch (const Ort::Exception& e) {
        std::cerr << "ORT Run error: " << e.what() << "\n";
        return false; // pas de cache renvoy� ici
    }

    if (outputs.empty()) return false;

    // Wrap output -> cv::Mat
    auto& out0 = outputs[0];
    auto ti = out0.GetTensorTypeAndShapeInfo();
    auto shape = ti.GetShape();
    float* outData = out0.GetTensorMutableData<float>();

    cv::Mat outMat;
    if (shape.size() == 3) {
        int sz[3] = { (int)shape[0], (int)shape[1], (int)shape[2] };
        outMat = cv::Mat(3, sz, CV_32F, outData);
    }
    else if (shape.size() == 2) {
        outMat = cv::Mat((int)shape[0], (int)shape[1], CV_32F, outData);
    }
    else {
        std::cerr << "Shape output ORT inattendue\n";
        return false;
    }

    // Decode + NMS
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    decodeDetections_(outMat, info, frame.cols, frame.rows, boxes, scores);

    std::vector<int> idx;
    cv::dnn::NMSBoxes(boxes, scores, confTh_, nmsTh_, idx);

    std::vector<BallDetection> dets;
    dets.reserve(idx.size());

    for (int i : idx) {
        BallDetection d;
        d.box = boxes[i];
        d.conf = scores[i];
        d.color = classifyColorHSV_(frame, d.box);
        d.decision = decide_(d.color);
        dets.push_back(d);
    }

    // inf�rence OK -> on met � jour le cache et le timestamp
    lastInfer_ = now;
    lastDets_ = dets;

    outDets = std::move(dets);
    return !outDets.empty();
}
*/

bool Camera::detectBalls(std::vector<BallDetection>& outDets)
{
    outDets.clear();


    // Si aucun backend n'est chargé, pas de détection possible
#ifdef CATJ_USE_ORT
    if (!ortLoaded_ && !netLoaded_) {
        return false;
    }
#else
    if (!netLoaded_) {
        return false;
    }
#endif

    // --- throttle IA ---
    const auto now = std::chrono::steady_clock::now();
    const int safeAiFps = std::max(1, aiFps_);
    const auto period = std::chrono::milliseconds(1000 / safeAiFps);

    if (lastInfer_ != std::chrono::steady_clock::time_point::min() &&
        (now - lastInfer_) < period)
    {
        outDets = lastDets_;
        return !outDets.empty();
    }

    cv::Mat frame;
    if (!getLatestFrame(frame)) {
        return false;
    }


    // --- preprocess (letterbox + blob) ---
    LetterboxInfo info{};
    cv::Mat lb = letterbox_(frame, onnx_inputSize_, info);

    cv::Mat blob = cv::dnn::blobFromImage(
        lb, 1.0 / 255.0, cv::Size(onnx_inputSize_, onnx_inputSize_),
        cv::Scalar(), true /*swapRB*/, false /*crop*/, CV_32F
    );

    cv::Mat outMat; // sortie réseau sous forme Mat (2D/3D selon modèle)

    // ==========================
    //  A) ORT (si compilé)
    // ==========================
#ifdef CATJ_USE_ORT
    if (ortLoaded_)
    {
        float* inputData = (float*)blob.ptr<float>();
        const size_t inputTensorSize = (size_t)blob.total();
        std::array<int64_t, 4> inputShape = { 1, 3, onnx_inputSize_, onnx_inputSize_ };

        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            ortMemInfo_, inputData, inputTensorSize,
            inputShape.data(), inputShape.size()
        );

        const char* inNames[]  = { ortInputName_.c_str() };
        const char* outNames[] = { ortOutputName_.c_str() };

        std::vector<Ort::Value> outputs;
        try {
            outputs = ortSession_->Run(
                Ort::RunOptions{ nullptr },
                inNames, &inputTensor, 1,
                outNames, 1
            );
        }
        catch (const Ort::Exception& e) {
            std::cerr << "ORT Run error: " << e.what() << "\n";
            return false;
        }

        if (outputs.empty()) return false;

        auto& out0 = outputs[0];
        auto ti = out0.GetTensorTypeAndShapeInfo();
        auto shape = ti.GetShape();


        float* outData = out0.GetTensorMutableData<float>();

        if (shape.size() == 3) {
            int sz[3] = { (int)shape[0], (int)shape[1], (int)shape[2] };
            outMat = cv::Mat(3, sz, CV_32F, outData);
        }
        else if (shape.size() == 2) {
            outMat = cv::Mat((int)shape[0], (int)shape[1], CV_32F, outData);
        }
        else {
            std::cerr << "ORT output shape inattendue\n";
            return false;
        }
    }
    else
#endif
    // ==========================
    //  B) OpenCV DNN (fallback)
    // ==========================
    {
        // Ici, on utilise cv::dnn::Net si chargé
        if (!netLoaded_) return false;

        net_.setInput(blob);

        std::vector<cv::Mat> outs;
        net_.forward(outs, net_.getUnconnectedOutLayersNames());
        if (outs.empty()) return false;

        outMat = outs[0]; // outMat peut être dims=2 ou dims=3, ton decode gère
    }

    // --- Decode + NMS ---
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    decodeDetections_(outMat, info, frame.cols, frame.rows, boxes, scores);

    std::vector<int> idx;
    cv::dnn::NMSBoxes(boxes, scores, confTh_, nmsTh_, idx);

    std::vector<BallDetection> dets;
    dets.reserve(idx.size());

    for (int i : idx) {
        BallDetection d;
        d.box = boxes[i];
        d.conf = scores[i];
        d.color = classifyColorHSV_(frame, d.box);
        d.decision = decide_(d.color);
        dets.push_back(d);
    }


    // cache + timestamp
    lastInfer_ = now;
    lastDets_ = dets;   
    outDets = std::move(dets);

    return !outDets.empty();
}



std::vector<cv::Point3f> Camera::makeChessboard3D(cv::Size boardSize, float squareSize)
{
    std::vector<cv::Point3f> pts;
    pts.reserve(boardSize.area());
    for (int y = 0; y < boardSize.height; ++y)
        for (int x = 0; x < boardSize.width; ++x)
            pts.emplace_back(x * squareSize, y * squareSize, 0.0f);
    return pts;
}

double Camera::apparentDiameterPxFromBox_(const cv::Rect& box)
{
    // moyenne largeur/hauteur pour compenser un peu les impr�cisions
    return 0.5 * (box.width + box.height);
}


double Camera::computeDistance_mm(double realDiameterMm, double apparentDiameterPx) const
{
    if (realDiameterMm <= 0.0 || apparentDiameterPx <= 0.0 || focalavg_ <= 0.0)
        return std::numeric_limits<double>::quiet_NaN();

    return (focalavg_ * realDiameterMm) / apparentDiameterPx; // r�sultat en mm
}



void Camera::onMouse(int event, int x, int y, int /*flags*/, void* userdata)
{
    auto* st = reinterpret_cast<MeasureState*>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) {
        st->drawing = true;
        st->hasLine = false;
        st->p0 = { x, y };
        st->p1 = st->p0;
    }
    else if (event == cv::EVENT_MOUSEMOVE) {
        if (st->drawing) {
            st->p1 = { x, y };
        }
    }
    else if (event == cv::EVENT_LBUTTONUP) {
        st->drawing = false;
        st->p1 = { x, y };
        st->hasLine = true;
    }
}


void Camera::runMeasureTool(CATJ_camera::Camera& cam , MeasureState& st)
{
    const std::string win = "Measure tool (drag line)";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::setMouseCallback(win, onMouse, &st);

    std::cout << "Outil mesure:\n"
        << " - Clique+drag : tracer une ligne (taille px)\n"
        << " - c : clear\n"
        << " - 1 : objet = balle golf (42.67mm)\n"
        << " - 2 : objet = 100mm (test)\n"
        << " - q / ESC : quitter\n";

    cv::Mat frame;
    while (cam.isOpen())
    {
        if (!cam.getLatestFrame(frame)) {
            cv::waitKey(1);
            continue;
        }

        cv::Mat view = frame.clone();

        // Dessin ligne en cours
        if (st.drawing || st.hasLine) {
            cv::line(view, st.p0, st.p1, cv::Scalar(0, 255, 0), 2);
            cv::circle(view, st.p0, 4, cv::Scalar(0, 255, 0), -1);
            cv::circle(view, st.p1, 4, cv::Scalar(0, 255, 0), -1);

            const double pxLen = cv::norm(st.p1 - st.p0);
            st.lastPx = pxLen;

            // Distance si calibration OK + pxLen > 0
            if (this->isCalibrated_ && cam.focalavg_ > 0.0 && pxLen > 1.0) {
                st.lastDistMm = cam.computeDistance_mm(st.realSizeMm, pxLen);
            }
            else {
                st.lastDistMm = 0.0;
            }

            std::string s1 = "px = " + std::to_string((int)std::round(pxLen));
            std::string s2 = "real(mm) = " + std::to_string(st.realSizeMm);

            cv::putText(view, s1, { 10, 30 }, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);
            cv::putText(view, s2, { 10, 60 }, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);

            if (this->isCalibrated_ && st.lastDistMm > 0.0) {
                std::string s3 = "Dist ~= " + std::to_string((int)std::round(st.lastDistMm)) + " mm";
                cv::putText(view, s3, { 10, 90 }, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    cv::Scalar(0, 255, 255), 2);
            }
            else {
                cv::putText(view, "Dist: (need calibration + line)", { 10, 90 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
        }

        if(isImageShown_)   cv::imshow(win, view);
        int key = cv::waitKey(1);

        if (key == 27 || key == 'q') break;
        if (key == 'c') { st.hasLine = false; st.drawing = false; }
        if (key == '1') st.realSizeMm = 42.67; // golf ball diameter
        if (key == '2') st.realSizeMm = 100.0; // objet de 100mm
    }

    cv::destroyWindow(win);
}