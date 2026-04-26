#include "mode_course.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace CATJ_robot {

    ModeCourse::ModeCourse() : cfg_{}
    {
        counts_[BallType::PingPongOrange] = 0;
        counts_[BallType::PiscineRouge] = 0;
        counts_[BallType::PiscineAutre] = 0;
    }

    ModeCourse::ModeCourse(const Config& cfg) : cfg_(cfg)
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
        allowPontoonBonus_ = false;
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

        if (closeBall) {
            board.sendCatch();
        }

        const auto now = std::chrono::steady_clock::now();
        if (!collectedCandidates.empty() && (now - lastCollect_) >= cfg_.antiRebond) {
            auto best = std::max_element(collectedCandidates.begin(), collectedCandidates.end(),
                [](const RobotBall& a, const RobotBall& b) { return a.confidence < b.confidence; });
            registerBall_(best->type);
            lastCollect_ = now;
            board.sendObjective(score_);
        }

        if (allowPontoonBonus_ && !retourPonton_ && rearDistanceMm > 0.0f && rearDistanceMm < 300.0f) {
            retourPonton_ = true;
            score_ += cfg_.scoreBonusPonton;
            board.sendObjective(score_);
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

    ModeCourseDemiTour::ModeCourseDemiTour() : cfg_{} {}

    ModeCourseDemiTour::ModeCourseDemiTour(const Config& cfg) : cfg_(cfg) {}

    bool ModeCourseDemiTour::isFinitePositive_(float v)
    {
        return std::isfinite(v) && v > 0.0f;
    }

    double ModeCourseDemiTour::elapsedSec() const
    {
        if (startTs_ == std::chrono::steady_clock::time_point{}) return 0.0;
        const auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - startTs_).count() / 1000.0;
    }

    std::string ModeCourseDemiTour::stateName() const
    {
        switch (state_) {
        case State::StraightOut:  return "straight";
        case State::BackupClear:  return "backup_clear";
        case State::ArcAround:    return "arc_around";
        case State::CounterSteer: return "counter_steer";
        case State::ReturnHome:   return "return_home";
        case State::Finish:       return "finish";
        case State::Idle:
        default:
            return "idle";
        }
    }

    void ModeCourseDemiTour::start()
    {
        running_ = true;
        courseComplete_ = false;
        markerSeen_ = false;
        homeSeen_ = false;
        contacts_ = 0;
        homeSeenCount_ = 0;
        searchTurnSign_ = 1;
        state_ = State::StraightOut;
        lastMove_ = MoveCommand::Stop;
        lastSpeedPct_ = -1;
        lastTurnDeg_ = 9999.0f;
        startTs_ = std::chrono::steady_clock::now();
        stateTs_ = startTs_;
        lastContactTs_ = startTs_ - cfg_.contactDebounce;
        lastSearchFlipTs_ = startTs_;
    }

    void ModeCourseDemiTour::stop()
    {
        running_ = false;
        if (!courseComplete_) {
            state_ = State::Idle;
        }
    }

    void ModeCourseDemiTour::registerContact_(const CourseLidarDirections& d)
    {
        const bool probableContact =
            (isFinitePositive_(d.frontMm) && d.frontMm <= cfg_.frontContactMm)
            || (isFinitePositive_(d.leftMm) && d.leftMm <= cfg_.sideContactMm)
            || (isFinitePositive_(d.rightMm) && d.rightMm <= cfg_.sideContactMm);

        if (!probableContact) return;

        const auto now = std::chrono::steady_clock::now();
        if ((now - lastContactTs_) >= cfg_.contactDebounce) {
            ++contacts_;
            lastContactTs_ = now;
        }
    }

    void ModeCourseDemiTour::setMotion_(MotherboardLink& board, MoveCommand move, int speedPct, float turnDeg)
    {
        speedPct = std::clamp(speedPct, 0, 100);
        turnDeg = std::clamp(turnDeg, -180.0f, 180.0f);

        if (lastSpeedPct_ != speedPct) {
            board.sendSpeedPct(speedPct);
            lastSpeedPct_ = speedPct;
        }
        if (!std::isfinite(lastTurnDeg_) || std::fabs(lastTurnDeg_ - turnDeg) >= 1.0f) {
            board.sendTurnDeg(turnDeg);
            lastTurnDeg_ = turnDeg;
        }
        if (lastMove_ != move) {
            board.sendMove(move);
            lastMove_ = move;
        }
    }

    void ModeCourseDemiTour::transitionTo_(State next, MotherboardLink& board)
    {
        if (state_ == next) return;
        state_ = next;
        stateTs_ = std::chrono::steady_clock::now();
        homeSeenCount_ = 0;

        if (next == State::Finish) {
            courseComplete_ = true;
            homeSeen_ = true;
            setMotion_(board, MoveCommand::Stop, 0, 0.0f);
            board.sendStopAll();
            running_ = false;
        }
    }

    void ModeCourseDemiTour::update(const CourseLidarDirections& d, MotherboardLink& board)
    {
        if (!running_) return;

        registerContact_(d);

        const auto now = std::chrono::steady_clock::now();
        const double elapsed = elapsedSec();
        const double stateElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - stateTs_).count() / 1000.0;

        const bool markerAhead = isFinitePositive_(d.frontMm) && d.frontMm <= cfg_.markerApproachMm;
        const bool markerVeryClose = isFinitePositive_(d.frontMm) && d.frontMm <= cfg_.markerTooCloseMm;

        switch (state_) {
        case State::StraightOut:
        {
            if (markerAhead) {
                markerSeen_ = true;
                if (markerVeryClose) {
                    transitionTo_(State::BackupClear, board);
                    break;
                }
                if (isFinitePositive_(d.frontMm) && d.frontMm <= cfg_.markerCaptureMm) {
                    transitionTo_(State::ArcAround, board);
                    break;
                }

                setMotion_(board, MoveCommand::Forward, cfg_.speedApproachPct, 0.0f);
                break;
            }

            if (elapsed >= cfg_.fallbackMarkerSec) {
                transitionTo_(State::ArcAround, board);
                break;
            }

            float turn = 0.0f;
            if (isFinitePositive_(d.leftMm) && isFinitePositive_(d.rightMm)) {
                const float diff = d.leftMm - d.rightMm;
                if (std::fabs(diff) >= cfg_.straightWallBalanceMm) {
                    turn = (diff > 0.0f) ? +cfg_.straightHeadingCorrectionDeg : -cfg_.straightHeadingCorrectionDeg;
                }
            }
            setMotion_(board, MoveCommand::Forward, cfg_.speedStraightPct, turn);
            break;
        }

        case State::BackupClear:
            if (now - stateTs_ >= cfg_.backupDuration) {
                transitionTo_(State::ArcAround, board);
                break;
            }
            setMotion_(board, MoveCommand::Backward, cfg_.speedBackupPct, 0.0f);
            break;

        case State::ArcAround:
        {
            const float baseTurn = cfg_.clockwiseTurn ? +cfg_.arcTurnDeg : -cfg_.arcTurnDeg;
            const float tightTurn = cfg_.clockwiseTurn ? +cfg_.arcTightTurnDeg : -cfg_.arcTightTurnDeg;
            const float commandedTurn = markerVeryClose ? tightTurn : baseTurn;
            setMotion_(board, MoveCommand::Forward, cfg_.speedArcPct, commandedTurn);

            if (now - stateTs_ >= cfg_.arcDuration) {
                transitionTo_(State::CounterSteer, board);
            }
            break;
        }

        case State::CounterSteer:
        {
            const float counterTurn = cfg_.clockwiseTurn ? -cfg_.counterTurnDeg : +cfg_.counterTurnDeg;
            setMotion_(board, MoveCommand::Forward, cfg_.speedCounterPct, counterTurn);
            if (now - stateTs_ >= cfg_.counterDuration) {
                transitionTo_(State::ReturnHome, board);
            }
            break;
        }

        case State::ReturnHome:
        {
            if (now - lastSearchFlipTs_ >= cfg_.searchFlipPeriod) {
                searchTurnSign_ = -searchTurnSign_;
                lastSearchFlipTs_ = now;
            }

            float turn = 0.0f;
            if (isFinitePositive_(d.leftMm) && isFinitePositive_(d.rightMm)) {
                const float diff = d.leftMm - d.rightMm;
                if (std::fabs(diff) >= cfg_.straightWallBalanceMm) {
                    turn = (diff > 0.0f) ? +cfg_.straightHeadingCorrectionDeg : -cfg_.straightHeadingCorrectionDeg;
                }
            }
            else if (isFinitePositive_(d.frontMm) && d.frontMm > cfg_.homeDetectMm) {
                turn = static_cast<float>(searchTurnSign_) * cfg_.searchTurnDeg;
            }

            setMotion_(board, MoveCommand::Forward, cfg_.speedReturnPct, turn);

            if (elapsed >= cfg_.minHomeDetectSec && stateElapsed >= cfg_.minReturnTravelSec
                && isFinitePositive_(d.frontMm) && d.frontMm <= cfg_.homeDetectMm) {
                ++homeSeenCount_;
            }
            else {
                homeSeenCount_ = 0;
            }

            if (homeSeenCount_ >= std::max(1, cfg_.homeConfirmCycles)) {
                transitionTo_(State::Finish, board);
            }
            break;
        }

        case State::Finish:
            running_ = false;
            break;

        case State::Idle:
        default:
            break;
        }
    }

} // namespace CATJ_robot
