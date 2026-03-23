#pragma once

#include <optional>
#include <string>
#include <vector>

#include "../../capteur/camera/camera.h"

namespace CATJ_robot {

    enum class BallType {
        Unknown,
        PingPongOrange,
        PiscineRouge,
        PiscineAutre
    };

    struct RobotBall {
        BallType type = BallType::Unknown;
        int score = 0;
        float distanceM = 0.0f;
        float angleDeg = 0.0f;
        int cx = 0;
        int cy = 0;
        float radiusPx = 0.0f;
        float confidence = 0.0f;

        bool isCollectible() const { return score > 0; }
        float priority() const;
    };

    struct BallFilterConfig {
        float minConfidence = 0.50f;
        float minDistanceM = 0.10f;
        float maxDistanceM = 6.00f;
        float orangeDiameterM = 0.040f;
        float piscineDiameterM = 0.070f;
        float hfovDeg = 62.2f;
    };

    class BallLogic {
    public:
        BallLogic();
        explicit BallLogic(const BallFilterConfig& cfg);

        std::optional<RobotBall> fromCameraDetection(const CATJ_camera::Camera& cam,
            const CATJ_camera::BallDetection& det,
            int frameWidth,
            int frameHeight) const;

        std::vector<RobotBall> convert(const CATJ_camera::Camera& cam,
            const std::vector<CATJ_camera::BallDetection>& dets,
            int frameWidth,
            int frameHeight) const;

        std::optional<RobotBall> bestTarget(const std::vector<RobotBall>& balls) const;
        std::optional<RobotBall> nearestBall(const std::vector<RobotBall>& balls) const;

        static BallType classifyBallType(const CATJ_camera::BallDetection& det);
        static const char* toString(BallType t);
        static int scoreFor(BallType t);

    private:
        float estimateAngleDeg_(int cx, int frameWidth) const;
        float estimateDistanceM_(const CATJ_camera::Camera& cam,
            BallType type,
            float apparentDiameterPx) const;
        static float apparentDiameterPx_(const cv::Rect& box);

        BallFilterConfig cfg_{};
    };

} // namespace CATJ_robot