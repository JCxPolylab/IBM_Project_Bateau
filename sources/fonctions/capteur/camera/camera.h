#pragma once

#include "../../utility/myUtil.h"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include <atomic>
#include <mutex>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <algorithm>
#ifdef CATJ_USE_ORT
#include <onnxruntime_cxx_api.h>
/*
#ifdef WIN32
        #include <onnxruntime_cxx_api.h>
    #else
        #include <../onnxruntime/onnxruntime_cxx_api.h>
    #endif
*/
  #include <memory>
  #include <array>
#endif

/*
    Pour la mesure de distance de l'objet il faut utiliser la relation : 
    Distance = Focale * (HauteurObj / HauteurPixel)
*/

struct MeasureState {
    bool drawing = false;
    bool hasLine = false;
    cv::Point p0{}, p1{};
    double realSizeMm = 0.0;   // ex: diam�tre balle de golf = 42.67 mm
    double lastPx = 0.0;
    double lastDistMm = 0.0;
};

namespace CATJ_camera {

    enum class CameraBackend {
        Auto,
        UsbV4L2,
        CsiGStreamer
    };

    inline CameraBackend cameraBackendFromString(const std::string& s)
    {
        if (s == "usb" || s == "v4l2" || s == "usb_v4l2") return CameraBackend::UsbV4L2;
        if (s == "csi" || s == "gstreamer" || s == "csi_gstreamer") return CameraBackend::CsiGStreamer;
        return CameraBackend::Auto;
    }

    enum class BallColor { Unknown, Red, Blue, White, Orange };
    enum class BallDecision { Unknown, Target, Ignore };

    struct BallDetection {
        cv::Rect box;          // bbox dans l'image originale
        float conf = 0.f;      // confiance IA
        BallColor color = BallColor::Unknown;
        BallDecision decision = BallDecision::Unknown;
    };

    class Camera {
    public:
        Camera(int device = 0, int w = 640, int h = 480,
            CameraBackend backend = CameraBackend::Auto,
            const std::string& customPipeline = "");
        ~Camera();

        bool isOpen() const;

        //Structure mesure
        MeasureState MeasDrawing;

        // Capture
        bool startCapture(double fps = 20.0);
        void stopCapture();
        bool close();
        bool getLatestFrame(cv::Mat& out); // thread-safe
        double computeDistance_mm(double realDiameterMm, double apparentDiameterPx) const;
        bool calibrateCamera(bool savePicCalib, std::vector<cv::Point3f> objTemplate, cv::Mat frame, cv::Mat* out);
        bool loadCalibration(const std::string& path);
        std::vector<cv::Point3f> makeChessboard3D(cv::Size boardSize, float squareSize);
        void runMeasureTool(CATJ_camera::Camera& cam, MeasureState& st);

        // Enregistrement
        bool startRecording(const std::string& filename, double fps, const std::string& fourcc);
        void stopRecording();
        void setShow(bool v) { isImageShown_ = v; }

        // IA (Option A)
        bool loadBallDetectorONNX(const std::string& onnxPath, int inputSize = 640);
        void setThresholds(float conf, float nms) { confTh_ = conf; nmsTh_ = nms; }
        void setColorDecision(BallColor target, BallColor ignore) { targetColor_ = target; ignoreColor_ = ignore; }
        void setCalibrationMode(bool enabled) { calibrationMode_ = enabled; }
        bool calibrationMode() const { return calibrationMode_.load(); }
        void setUndistortEnabled(bool enabled) { undistortEnabled_ = enabled; }
        bool undistortEnabled() const { return undistortEnabled_; }
        bool setCaptureProperty(int propId, double value);

        // Ex�cute IA (si net charg�) + couleur HSV
        bool detectBalls(std::vector<BallDetection>& out);
        bool detectBalls(const cv::Mat& frame, std::vector<BallDetection>& out);

        double hw_focale_mm = 3.04;

        //geteurs
		BallColor getTargetColor() const { return targetColor_; }
		BallColor getIgnoreColor() const { return ignoreColor_; }
		double getFps() const { return fps_; }
		std::pair<int, int> getResolution() const { return { w_, h_ }; }
		double getHWFocaleMm() const { return hw_focale_mm; }
		std::pair<double, double> getFocalFromCalibration() const { return { focal_x_, focal_y_ }; }
		float getSquareSizeCalibration() const { return calib_squareSize_; }
		cv::Size getEchiquierSizeCalibration() const { return EchiquierSize_; }
		int getAiFps() const { return aiFps_; }
        float getConfThreshold() const { return confTh_; }
        float getNmsThreshold() const { return nmsTh_; }
		int getDevice() const { return _device; }
        std::string getcalibPath() const { return calibPath_; }
        std::vector<std::vector<cv::Point3f>> getObj() const { return objPts_; }
		std::vector<std::vector<cv::Point2f>> getImg() const { return imgPts_; }
        std::atomic <int> getKeyPolled() const { return keyPolled_.load(); }
		int getNeededViews() const { return neededViews_; }
        int getOnnx_InputSize() const { return onnx_inputSize_; }
		bool getRecording() const { return recording_.load(); }

        //seteurs
		void setTargetColor(BallColor c) { targetColor_ = c; }
		void setIgnoreColor(BallColor c) { ignoreColor_ = c; }
        void setFps(double fps) { fps_ = fps; if (fps <= 0) fps_ = 1; }
		void setResolution(int w, int h) { w_ = w; h_ = h; }
		void setHWFocaleMm(double f) { hw_focale_mm = f; }
		void setFocalFromCalibration(double focal_x, double focal_y) { focal_x_ = focal_x; focal_y_ = focal_y; focalavg_ = (focal_x + focal_y) / 2.0; isCalibrated_ = true; }
		void setSquareSizeCalibration(float s) { calib_squareSize_ = s; }
		void setEchiquierSizeCalibration(cv::Size s) { EchiquierSize_ = s; }
		void setAiFps(int fps) { aiFps_ = std::max(1, fps); }
		void setDevice(int device) { _device = device; }
		void setCalibPath(const std::string& path) { calibPath_ = path; }
		void setObj(std::vector<std::vector<cv::Point3f>> obj) { objPts_ = obj; }
		void setImg(std::vector<std::vector<cv::Point2f>> img) { imgPts_ = img; }
		void setNeededViews(int n) { neededViews_ = n; }
        void setOnnx_InputSize(int input) { onnx_inputSize_ = input; }
		void setRecording(bool v) { recording_ = v; }  

    private:
		void captureLoop_(); //Fonction de capture video exectuer en multithreading

        bool openUsbV4L2_(int device);
        bool openCsiGStreamer_(int device, const std::string& customPipeline);

        static std::string buildDefaultCsiPipeline_(int w, int h, int fps);
        bool undistordFrame_(const cv::Mat& in, cv::Mat& out);
        double apparentDiameterPxFromBox_(const cv::Rect& box);
        static void onMouse(int event, int x, int y, int /*flags*/, void* userdata);

        // Pr�process letterbox + mapping bbox -> image originale
        struct LetterboxInfo { float r; int dw; int dh; int newW; int newH; };
        cv::Mat letterbox_(const cv::Mat& src, int newSize, LetterboxInfo& info) const;
        cv::Rect mapBoxBack_(const cv::Rect& boxLB, const LetterboxInfo& info, int origW, int origH) const;

        // Postprocess ONNX Ultralytics (g�re 2 formats courants)
        void decodeDetections_(const cv::Mat& out, const LetterboxInfo& info, int origW, int origH,
            std::vector<cv::Rect>& boxes, std::vector<float>& scores) const;

        // Couleur
        BallColor classifyColorHSV_(const cv::Mat& frame, const cv::Rect& box) const;
        BallDecision decide_(BallColor c) const;

        // --- ORT ---
#ifdef CATJ_USE_ORT
        std::unique_ptr<Ort::Env> ortEnv_;
        std::unique_ptr<Ort::Session> ortSession_;
        Ort::SessionOptions ortOpts_;
        Ort::AllocatorWithDefaultOptions ortAllocator_;
        Ort::MemoryInfo ortMemInfo_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
#endif
        std::string ortInputName_;
        std::string ortOutputName_;

        bool ortLoaded_ = false;

        // --- throttling IA ---
        int aiFps_ = 5; // 5 FPS par d�faut
        std::chrono::steady_clock::time_point lastInfer_ = std::chrono::steady_clock::time_point::min();
        std::vector<BallDetection> lastDets_;

        cv::VideoCapture cap_;
        cv::VideoWriter writer_;
        cv::dnn::Net net_;
        bool netLoaded_ = false;

        int w_ = 640, h_ = 480;
        int onnx_inputSize_ = 320;
        double fps_ = 20.0;
        int _device = 0;

        /* Calibration */
		std::string calibPath_;
        double focalavg_ = 0.0;
        double focal_x_ = 0.0;
        double focal_y_ = 0.0;
        bool isCalibrated_ = false;
        float calib_squareSize_ = 2.6; //2.5 cm
        cv::Size EchiquierSize_{ 9,6 };
        cv::Mat K_, dist_;
        cv::Mat map1_, map2_;              // pour undistort rapide via remap
        bool undistortEnabled_ = true;
        std::vector<std::vector<cv::Point2f>> imgPts_;
        std::vector<std::vector<cv::Point3f>> objPts_;
        int neededViews_ = 20;             // nombre d'images � capturer

        /* Capture en multiThreading */
        std::atomic<bool> running_{ false };
        std::thread th_;
        mutable std::mutex m_;
        cv::Mat latest_;
        bool isImageShown_ = false;
        std::atomic<bool> recording_{ false };
        float confTh_ = 0.35f;
        float nmsTh_ = 0.45f;
		std::atomic<int> keyPolled_ = -1;
        std::atomic<bool> calibrationMode_{ false };

        BallColor targetColor_ = BallColor::Red;   
        BallColor ignoreColor_ = BallColor::Blue;  

        CameraBackend backend_ = CameraBackend::Auto;
        std::string customPipeline_;
    };

} // namespace CATJ_camera
