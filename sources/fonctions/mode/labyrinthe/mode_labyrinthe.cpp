#include "mode_labyrinthe.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace CATJ_robot {

    ModeLabyrinthe::ModeLabyrinthe() : cfg_{} {}

    ModeLabyrinthe::ModeLabyrinthe(const Config& cfg) : cfg_(cfg) {}

    void ModeLabyrinthe::start()
    {
        running_ = true;
        circuitComplete_ = false;
        contacts_ = 0;
        returnSeenCount_ = 0;
        state_ = State::WallFollow;
        startTs_ = std::chrono::steady_clock::now();
        lastContactTs_ = startTs_ - cfg_.contactDebounce;
    }

    void ModeLabyrinthe::stop()
    {
        running_ = false;
        if (!circuitComplete_) {
            state_ = State::Idle;
        }
    }

    double ModeLabyrinthe::elapsedSec() const
    {
        if (!running_ && startTs_ == std::chrono::steady_clock::time_point{}) return 0.0;
        const auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - startTs_).count() / 1000.0;
    }

    std::string ModeLabyrinthe::stateName() const
    {
        switch (state_) {
        case State::WallFollow: return "wall_follow";
        case State::Avoid: return "avoid";
        case State::Finish: return "finish";
        case State::Idle:
        default:
            return "idle";
        }
    }

    bool ModeLabyrinthe::isFinitePositive_(float v)
    {
        return std::isfinite(v) && v > 0.0f;
    }

    void ModeLabyrinthe::registerContact_(const LidarDirections& d)
    {
        const bool probableContact =
            (isFinitePositive_(d.frontMm) && d.frontMm <= cfg_.frontCriticalMm)
            || (isFinitePositive_(d.leftMm) && d.leftMm <= cfg_.sideContactMm)
            || (isFinitePositive_(d.rightMm) && d.rightMm <= cfg_.sideContactMm);

        if (!probableContact) return;

        const auto now = std::chrono::steady_clock::now();
        if ((now - lastContactTs_) >= cfg_.contactDebounce) {
            ++contacts_;
            lastContactTs_ = now;
        }
    }

    void ModeLabyrinthe::update(const LidarDirections& d, MotherboardLink& board)
    {
        if (!running_) return;

        registerContact_(d);

        if (elapsedSec() >= cfg_.minRunBeforeReturnSec
            && isFinitePositive_(d.rearMm)
            && d.rearMm < cfg_.rearReturnMm) {
            ++returnSeenCount_;
        }
        else {
            returnSeenCount_ = 0;
        }

        if (returnSeenCount_ >= std::max(1, cfg_.returnConfirmCycles)) {
            circuitComplete_ = true;
            state_ = State::Finish;
            board.sendStopAll();
            running_ = false;
            return;
        }

        const bool obstacleFront = isFinitePositive_(d.frontMm) && d.frontMm < cfg_.frontAvoidMm;
        const bool obstacleFrontLeft = isFinitePositive_(d.frontLeftMm) && d.frontLeftMm < cfg_.frontAvoidMm;
        const bool obstacleFrontRight = isFinitePositive_(d.frontRightMm) && d.frontRightMm < cfg_.frontAvoidMm;

        if (obstacleFront || obstacleFrontLeft || obstacleFrontRight) {
            state_ = State::Avoid;
            board.sendSpeedPct(std::clamp(cfg_.avoidSpeedPct, 0, 100));

            const float leftRef = isFinitePositive_(d.leftMm) ? d.leftMm :
                (isFinitePositive_(d.frontLeftMm) ? d.frontLeftMm : std::numeric_limits<float>::infinity());
            const float rightRef = isFinitePositive_(d.rightMm) ? d.rightMm :
                (isFinitePositive_(d.frontRightMm) ? d.frontRightMm : std::numeric_limits<float>::infinity());

            const bool goLeft = leftRef >= rightRef;
            board.sendTurnDeg(goLeft ? -cfg_.turnAngleDeg : +cfg_.turnAngleDeg);
            board.sendMove(MoveCommand::Forward);
            return;
        }

        state_ = State::WallFollow;
        wallFollow_(d, board);
    }

    void ModeLabyrinthe::wallFollow_(const LidarDirections& d, MotherboardLink& board)
    {
        board.sendSpeedPct(std::clamp(cfg_.speedPct, 0, 100));

        const float rightMm = d.rightMm;
        const float leftMm = d.leftMm;

        if (!isFinitePositive_(rightMm)) {
            if (isFinitePositive_(leftMm) && leftMm < (cfg_.wallFollowMm * 1.5f)) {
                board.sendTurnDeg(+cfg_.wallCorrectionDeg);
            }
            else {
                board.sendTurnDeg(+0.5f * cfg_.wallCorrectionDeg);
            }
            board.sendMove(MoveCommand::Forward);
            return;
        }

        const float error = cfg_.wallFollowMm - rightMm;
        if (std::fabs(error) <= cfg_.wallToleranceMm) {
            board.sendTurnDeg(0.0f);
            board.sendMove(MoveCommand::Forward);
            return;
        }

        if (error > 0.0f) {
            board.sendTurnDeg(-cfg_.wallCorrectionDeg);
        }
        else {
            board.sendTurnDeg(+cfg_.wallCorrectionDeg);
        }
        board.sendMove(MoveCommand::Forward);
    }

} // namespace CATJ_robot
