#include "motherboard_protocol.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace CATJ_robot {

MotherboardLink::MotherboardLink(const MotherboardConfig& cfg)
{
    open(cfg);
}

bool MotherboardLink::open(const MotherboardConfig& cfg)
{
    std::scoped_lock lock(mutex_);
    cfg_ = cfg;
    lastStatus_ = {};
    return uart_.open(cfg_.uart);
}

void MotherboardLink::close()
{
    std::scoped_lock lock(mutex_);
    uart_.close();
}

bool MotherboardLink::isOpen() const
{
    return uart_.isOpen();
}

bool MotherboardLink::sendTurnDeg(float angleDeg)
{
    std::ostringstream oss;
    oss << "TURN " << std::lround(angleDeg);
    return sendLine_(oss.str());
}

bool MotherboardLink::sendMove(MoveCommand move)
{
    return sendLine_(std::string("MOVE ") + moveToString(move));
}

bool MotherboardLink::sendSpeedPct(int speedPct)
{
    speedPct = std::clamp(speedPct, 0, 100);
    std::ostringstream oss;
    oss << "SPEED " << speedPct;
    return sendLine_(oss.str());
}

bool MotherboardLink::sendCatch()
{
    return sendLine_("CATCH");
}

bool MotherboardLink::sendLockCollector()
{
    // Commande volontairement explicite pour l'epreuve de collecte :
    // la carte mere doit verrouiller mecaniquement le systeme de recuperation.
    return sendLine_("LOCK_COLLECTOR");
}

bool MotherboardLink::sendMode(RobotMode mode)
{
    return sendLine_(std::string("MODE ") + modeToString(mode));
}

bool MotherboardLink::sendObjective(int predictedScore)
{
    std::ostringstream oss;
    oss << "OBJECTIF " << predictedScore;
    return sendLine_(oss.str());
}

bool MotherboardLink::sendStopAll()
{
    return sendLine_("MOVE STOP") && sendLine_("SPEED 0");
}

bool MotherboardLink::sendHeartbeat()
{
    return sendLine_("CATJ");
}

bool MotherboardLink::requestStatus()
{
    return sendLine_("GET_STATUS");
}

std::optional<MotherboardStatus> MotherboardLink::readStatusOnce(int timeoutMs)
{
    std::scoped_lock lock(mutex_);
    if (!uart_.isOpen()) {
        return std::nullopt;
    }

    std::string line;
    const int effectiveTimeout = (timeoutMs >= 0) ? timeoutMs : cfg_.statusTimeoutMs;
    if (!uart_.readLine(line, '\n', effectiveTimeout, 256)) {
        return std::nullopt;
    }

    auto st = parseLine_(line);
    if (st) {
        lastStatus_ = *st;
    }
    return st;
}

bool MotherboardLink::sendLine_(const std::string& line)
{
    std::scoped_lock lock(mutex_);
    if (!uart_.isOpen()) {
        return false;
    }
    return uart_.writeString(line + "\n") > 0;
}

std::optional<MotherboardStatus> MotherboardLink::parseLine_(const std::string& line) const
{
    MotherboardStatus st;
    st.valid = true;
    st.rawLine = trim_(line);

    if (st.rawLine.empty()) return std::nullopt;

    std::istringstream iss(st.rawLine);
    std::string key;
    iss >> key;
    key = upper_(key);

    if (key == "ETAT") {
        int value = 0;
        iss >> value;
        st.etat = (value != 0);
        return st;
    }

    if (key == "ACK") {
        std::string sub;
        std::string result;
        iss >> sub >> result;
        sub = upper_(sub);
        result = upper_(result);
        const bool ok = (result == "OK" || result == "1" || result == "TRUE");
        if (sub == "CATCH") {
            st.catchAck = ok ? CatchAck::Ok : CatchAck::Failed;
            return st;
        }
        if (sub == "LOCK" || sub == "LOCK_COLLECTOR" || sub == "VERROU" || sub == "VERROUILLAGE") {
            st.lockAck = ok ? LockAck::Ok : LockAck::Failed;
            return st;
        }
    }

    if (key == "BALLE" || key == "BALL") {
        std::string kind;
        iss >> kind;
        st.balle = parseBallKind_(kind);

        // Formats acceptes :
        //   BALLE RED
        //   BALLE RED SCORE=35
        //   BALLE SCORE 35
        std::string token;
        while (iss >> token) {
            const auto pos = token.find('=');
            if (pos != std::string::npos) {
                const std::string lhs = upper_(token.substr(0, pos));
                const std::string rhs = token.substr(pos + 1);
                if (lhs == "SCORE") st.scoreValid = parseInt_(rhs, st.score);
                else if (lhs == "DELTA" || lhs == "POINTS") st.scoreDeltaValid = parseInt_(rhs, st.scoreDelta);
            }
            else {
                int v = 0;
                if (parseInt_(token, v)) {
                    st.scoreValid = true;
                    st.score = v;
                }
            }
        }
        return st;
    }

    if (key == "SCORE") {
        int value = 0;
        if (iss >> value) {
            st.scoreValid = true;
            st.score = value;
        }
        return st;
    }

    if (key == "DELTA" || key == "POINTS") {
        int value = 0;
        if (iss >> value) {
            st.scoreDeltaValid = true;
            st.scoreDelta = value;
        }
        return st;
    }

    if (key == "SPEEDVAL") {
        iss >> st.speedVal;
        return st;
    }

    if (key == "GYRO" || key == "HEADING" || key == "CAP" || key == "YAW") {
        std::string tok;
        if (iss >> tok) {
            float heading = 0.0f;
            if (parseFloat_(tok, heading)) {
                st.headingValid = true;
                st.headingDeg = heading;
            }
            else {
                const auto pos = tok.find('=');
                if (pos != std::string::npos) {
                    const std::string lhs = upper_(tok.substr(0, pos));
                    const std::string rhs = tok.substr(pos + 1);
                    if (lhs == "YAW" || lhs == "HEADING" || lhs == "CAP") {
                        st.headingValid = parseFloat_(rhs, st.headingDeg);
                    }
                }
            }
        }
        return st;
    }

    // Variante simple: STATUS etat=1 catch=ok balle=trash score=42 heading=123.4 speed=42
    if (key == "STATUS") {
        std::string token;
        while (iss >> token) {
            auto pos = token.find('=');
            if (pos == std::string::npos) continue;
            std::string lhs = upper_(token.substr(0, pos));
            std::string rhs = token.substr(pos + 1);
            std::string rhsUpper = upper_(rhs);
            if (lhs == "ETAT") st.etat = (rhsUpper == "1" || rhsUpper == "TRUE" || rhsUpper == "OK");
            else if (lhs == "CATCH") st.catchAck = (rhsUpper == "OK" || rhsUpper == "1" || rhsUpper == "TRUE") ? CatchAck::Ok : CatchAck::Failed;
            else if (lhs == "LOCK" || lhs == "VERROU") st.lockAck = (rhsUpper == "OK" || rhsUpper == "1" || rhsUpper == "TRUE") ? LockAck::Ok : LockAck::Failed;
            else if (lhs == "BALLE" || lhs == "BALL") st.balle = parseBallKind_(rhs);
            else if (lhs == "SPEED") st.speedVal = std::atoi(rhs.c_str());
            else if (lhs == "SCORE") st.scoreValid = parseInt_(rhs, st.score);
            else if (lhs == "DELTA" || lhs == "POINTS") st.scoreDeltaValid = parseInt_(rhs, st.scoreDelta);
            else if (lhs == "HEADING" || lhs == "YAW" || lhs == "CAP" || lhs == "GYRO") st.headingValid = parseFloat_(rhs, st.headingDeg);
        }
        return st;
    }

    return st; // on conserve la ligne brute meme si format partiel
}

std::string MotherboardLink::moveToString(MoveCommand move)
{
    switch (move) {
    case MoveCommand::Forward: return "FORWARD";
    case MoveCommand::Backward: return "BACKWARD";
    case MoveCommand::Stop:
    default: return "STOP";
    }
}

std::string MotherboardLink::modeToString(RobotMode mode)
{
    switch (mode) {
    case RobotMode::Labyrinthe: return "LABYRINTHE";
    case RobotMode::Ramassage:  return "RAMASSAGE";
    case RobotMode::Course:     return "COURSE";
    case RobotMode::Debug:      return "DEBUG";
    case RobotMode::Stop:
    default: return "STOP";
    }
}

std::string MotherboardLink::catchAckToString(CatchAck v)
{
    switch (v) {
    case CatchAck::Ok: return "OK";
    case CatchAck::Failed: return "FAILED";
    default: return "UNKNOWN";
    }
}

std::string MotherboardLink::lockAckToString(LockAck v)
{
    switch (v) {
    case LockAck::Ok: return "OK";
    case LockAck::Failed: return "FAILED";
    default: return "UNKNOWN";
    }
}

std::string MotherboardLink::ballKindToString(BallKind v)
{
    switch (v) {
    case BallKind::Score: return "SCORE";
    case BallKind::Trash: return "TRASH";
    case BallKind::PingPongOrange: return "PINGPONG_ORANGE";
    case BallKind::PingPongWhite: return "PINGPONG_WHITE";
    case BallKind::PiscineRed: return "PISCINE_RED";
    case BallKind::PiscineOther: return "PISCINE_OTHER";
    default: return "UNKNOWN";
    }
}

std::string MotherboardLink::trim_(std::string s)
{
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

std::string MotherboardLink::upper_(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return s;
}

bool MotherboardLink::startsWith_(const std::string& s, const std::string& prefix)
{
    return s.rfind(prefix, 0) == 0;
}

bool MotherboardLink::parseInt_(const std::string& s, int& out)
{
    try {
        size_t pos = 0;
        int v = std::stoi(s, &pos, 10);
        if (pos != s.size()) return false;
        out = v;
        return true;
    }
    catch (...) {
        return false;
    }
}

bool MotherboardLink::parseFloat_(const std::string& s, float& out)
{
    try {
        size_t pos = 0;
        float v = std::stof(s, &pos);
        if (pos != s.size()) return false;
        out = v;
        return true;
    }
    catch (...) {
        return false;
    }
}

BallKind MotherboardLink::parseBallKind_(const std::string& s)
{
    const std::string v = upper_(trim_(s));
    if (v == "SCORE" || v == "GOOD" || v == "BONNE" || v == "OK" || v == "RED" || v == "ROUGE") return BallKind::Score;
    if (v == "TRASH" || v == "BAD" || v == "MAUVAISE" || v == "OTHER" || v == "AUTRE") return BallKind::Trash;
    if (v == "ORANGE" || v == "PINGPONG_ORANGE" || v == "PING_PONG_ORANGE") return BallKind::PingPongOrange;
    if (v == "WHITE" || v == "BLANC" || v == "BLANCHE" || v == "PINGPONG_WHITE" || v == "PING_PONG_WHITE") return BallKind::PingPongWhite;
    if (v == "PISCINE_RED" || v == "PISCINE_ROUGE") return BallKind::PiscineRed;
    if (v == "PISCINE_OTHER" || v == "PISCINE_AUTRE") return BallKind::PiscineOther;
    return BallKind::Unknown;
}

} // namespace CATJ_robot
