#include "ball_logic.h"

#include <algorithm>
#include <cmath>

namespace CATJ_robot {

float RobotBall::priority() const
{
    if (score <= 0 || distanceM <= 0.0f) return 0.0f;
    return static_cast<float>(score) / distanceM;
}

BallLogic::BallLogic() : cfg_{} {}

BallLogic::BallLogic(const BallFilterConfig& cfg)
    : cfg_(cfg) {
}

std::optional<RobotBall> BallLogic::fromCameraDetection(const CATJ_camera::Camera& cam,
                                                        const CATJ_camera::BallDetection& det,
                                                        int frameWidth,
                                                        int frameHeight) const
{
    (void)frameHeight;
    RobotBall out;
    out.type = classifyBallType(det);
    if (out.type == BallType::PingPongWhite && !cfg_.whiteIsPingPong) {
        out.type = BallType::PiscineAutre;
    }
    if (out.type == BallType::Unknown) {
        return std::nullopt;
    }

    out.score = scoreFor_(out.type);
    out.confidence = det.conf;
    if (out.confidence < cfg_.minConfidence) {
        return std::nullopt;
    }

    out.cx = det.box.x + det.box.width / 2;
    out.cy = det.box.y + det.box.height / 2;
    out.radiusPx = 0.5f * apparentDiameterPx_(det.box);
    out.angleDeg = estimateAngleDeg_(out.cx, frameWidth);
    out.distanceM = estimateDistanceM_(cam, out.type, out.radiusPx * 2.0f);

    if (out.distanceM < cfg_.minDistanceM || out.distanceM > cfg_.maxDistanceM) {
        return std::nullopt;
    }

    return out;
}

std::vector<RobotBall> BallLogic::convert(const CATJ_camera::Camera& cam,
                                          const std::vector<CATJ_camera::BallDetection>& dets,
                                          int frameWidth,
                                          int frameHeight) const
{
    std::vector<RobotBall> out;
    out.reserve(dets.size());
    for (const auto& d : dets) {
        auto b = fromCameraDetection(cam, d, frameWidth, frameHeight);
        if (b) out.push_back(*b);
    }
    return out;
}

std::optional<RobotBall> BallLogic::bestTarget(const std::vector<RobotBall>& balls) const
{
    std::optional<RobotBall> best;
    for (const auto& b : balls) {
        if (!b.isCollectible()) continue;
        if (!best || b.priority() > best->priority()) best = b;
    }
    return best;
}

std::optional<RobotBall> BallLogic::nearestBall(const std::vector<RobotBall>& balls) const
{
    std::optional<RobotBall> best;
    for (const auto& b : balls) {
        if (!best || b.distanceM < best->distanceM) best = b;
    }
    return best;
}

BallType BallLogic::classifyBallType(const CATJ_camera::BallDetection& det)
{
    using namespace CATJ_camera;

    // La couleur brute HSV est volontairement prioritaire : la decision Target/Ignore
    // sert au dashboard, mais le type metier doit rester coherent avec la table de score.
    switch (det.color) {
    case BallColor::Orange:
        return BallType::PingPongOrange;
    case BallColor::White:
        return BallType::PingPongWhite;
    case BallColor::Red:
        return BallType::PiscineRouge;
    case BallColor::Blue:
        return BallType::PiscineAutre;
    case BallColor::Unknown:
    default:
        return BallType::Unknown;
    }
}

const char* BallLogic::toString(BallType t)
{
    switch (t) {
    case BallType::PingPongOrange: return "pingpong_orange";
    case BallType::PingPongWhite:  return "pingpong_white";
    case BallType::PiscineRouge:   return "piscine_rouge";
    case BallType::PiscineAutre:   return "piscine_autre";
    default: return "unknown";
    }
}

int BallLogic::scoreFor(BallType t)
{
    switch (t) {
    case BallType::PingPongOrange: return -5;
    case BallType::PingPongWhite:  return -5;
    case BallType::PiscineRouge:   return +10;
    case BallType::PiscineAutre:   return -10;
    default: return 0;
    }
}

int BallLogic::scoreFor_(BallType t) const
{
    switch (t) {
    case BallType::PingPongOrange: return cfg_.scorePingPongOrange;
    case BallType::PingPongWhite:  return cfg_.scorePingPongWhite;
    case BallType::PiscineRouge:   return cfg_.scorePiscineRouge;
    case BallType::PiscineAutre:   return cfg_.scorePiscineAutre;
    default: return 0;
    }
}

float BallLogic::estimateAngleDeg_(int cx, int frameWidth) const
{
    if (frameWidth <= 0) return 0.0f;
    const float centered = (static_cast<float>(cx) / static_cast<float>(frameWidth)) - 0.5f;
    return centered * cfg_.hfovDeg;
}

float BallLogic::estimateDistanceM_(const CATJ_camera::Camera& cam,
                                    BallType type,
                                    float apparentDiameterPx) const
{
    float realDiameterM = cfg_.piscineDiameterM;
    if (type == BallType::PingPongOrange) realDiameterM = cfg_.orangeDiameterM;
    else if (type == BallType::PingPongWhite && cfg_.whiteIsPingPong) realDiameterM = cfg_.whiteDiameterM;
    const double distMm = cam.computeDistance_mm(realDiameterM * 1000.0f, apparentDiameterPx);
    return static_cast<float>(distMm / 1000.0);
}

float BallLogic::apparentDiameterPx_(const cv::Rect& box)
{
    return static_cast<float>(0.5 * (box.width + box.height));
}

} // namespace CATJ_robot
