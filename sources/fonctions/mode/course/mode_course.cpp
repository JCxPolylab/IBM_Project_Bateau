#include "mode_course.h"

#include <algorithm>

namespace CATJ_robot {

ModeCourse::ModeCourse(Config cfg) : cfg_(cfg)
{
    counts_[BallType::PingPongOrange] = 0;
    counts_[BallType::PiscineRouge] = 0;
    counts_[BallType::PiscineAutre] = 0;
}

void ModeCourse::prepare(int predictedScore)
{
    predictedScore_ = predictedScore;
}

void ModeCourse::start()
{
    running_ = true;
    firstBall_ = false;
    retourPonton_ = false;
    verrouille_ = false;
    score_ = 0;
    lastCollect_ = std::chrono::steady_clock::now() - cfg_.antiRebond;
    for (auto& kv : counts_) kv.second = 0;
}

void ModeCourse::stop()
{
    running_ = false;
}

void ModeCourse::update(const std::vector<RobotBall>& visibleBalls,
                        float rearDistanceMm,
                        MotherboardLink& board)
{
    if (!running_ || verrouille_) {
        return;
    }

    bool closeBall = false;
    std::vector<RobotBall> collectedCandidates;
    for (const auto& b : visibleBalls) {
        if (b.distanceM < cfg_.distCollecteRapideM) closeBall = true;
        if (b.distanceM < cfg_.distBalleCollecteeM) collectedCandidates.push_back(b);
    }

    // La carte mère gère physiquement le convoyeur/catch ; ici on ne lui envoie qu'un ordre symbolique.
    if (closeBall) {
        board.sendCatch();
    }

    const auto now = std::chrono::steady_clock::now();
    if (!collectedCandidates.empty() && (now - lastCollect_) >= cfg_.antiRebond) {
        auto best = std::max_element(collectedCandidates.begin(), collectedCandidates.end(),
            [](const RobotBall& a, const RobotBall& b) { return a.confidence < b.confidence; });
        registerBall_(best->type);
        lastCollect_ = now;
        board.sendObjective(predictedScore_);
    }

    if (!retourPonton_ && rearDistanceMm > 0.0f && rearDistanceMm < 300.0f) {
        retourPonton_ = true;
        score_ += cfg_.scoreBonusPonton;
    }
}

void ModeCourse::registerBall_(BallType type)
{
    if (type == BallType::Unknown) return;
    if (!firstBall_) {
        firstBall_ = true;
        score_ += cfg_.scoreBaseCollecte;
    }
    score_ += BallLogic::scoreFor(type);
    counts_[type] += 1;
}

} // namespace CATJ_robot
