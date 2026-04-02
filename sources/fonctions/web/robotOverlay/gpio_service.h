#pragma once

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if !defined(_WIN32)
  #include <filesystem>
#endif

namespace CATJ_gpio_web {

struct GpioEvent {
    uint64_t tsMs = 0;
    int pin = -1;                 // BCM numbering
    std::string kind;             // sample | info | err
    std::string message;          // human readable
    int value = -1;               // when sample/read
};

using EventCallback = std::function<void(const GpioEvent&)>;

class GpioService {
public:
    GpioService() = default;
    ~GpioService() { stopAll(); }

    GpioService(const GpioService&) = delete;
    GpioService& operator=(const GpioService&) = delete;

    void setEventCallback(EventCallback cb)
    {
        std::lock_guard<std::mutex> lk(m_);
        cb_ = std::move(cb);
    }

    bool configurePin(int pin, const std::string& mode, std::string* err = nullptr)
    {
        auto pr = getOrCreate_(pin);
        if (!pr) return fail_(err, pin, "invalid BCM pin");
#if defined(_WIN32)
        return fail_(err, pin, "GPIO only supported on Linux/Raspberry Pi");
#else
        stopPwm(pin, nullptr);
        stopSampling(pin, nullptr);

        const std::string m = lower_(trim_(mode));
        if (m != "input" && m != "output") return fail_(err, pin, "mode must be input or output");

        if (usePinctrl_()) {
            const std::string cmd = std::string("pinctrl set ") + std::to_string(pin) + (m == "input" ? " ip" : " op");
            std::string out;
            if (!execCmdOk_(cmd, &out)) {
                return fail_(err, pin, "pinctrl failed to set mode");
            }
        } else {
            if (!ensureExported_(pr, err)) return false;
            if (!writeFile_(gpioPath_(pin, "direction"), m == "input" ? "in" : "out")) {
                return fail_(err, pin, "failed to set direction");
            }
        }

        {
            std::lock_guard<std::mutex> lk(m_);
            pr->mode = m;
            pr->error.clear();
        }
        emit_({nowMs_(), pin, "info", std::string("mode=") + m, -1});
        return true;
#endif
    }

    bool writePin(int pin, int value, std::string* err = nullptr)
    {
        auto pr = getOrCreate_(pin);
        if (!pr) return fail_(err, pin, "invalid BCM pin");
#if defined(_WIN32)
        return fail_(err, pin, "GPIO only supported on Linux/Raspberry Pi");
#else
        if (pr->mode != "output") {
            if (!configurePin(pin, "output", err)) return false;
        }
        stopPwm(pin, nullptr);

        if (!writeValue_(pin, value ? 1 : 0)) {
            return fail_(err, pin, usePinctrl_() ? "pinctrl write failed" : "failed to write value");
        }
        {
            std::lock_guard<std::mutex> lk(m_);
            pr->value = value ? 1 : 0;
            pr->error.clear();
        }
        emit_({nowMs_(), pin, "info", std::string("write=") + (value ? "1" : "0"), value ? 1 : 0});
        return true;
#endif
    }

    bool readPin(int pin, int* outValue, std::string* err = nullptr)
    {
        auto pr = getOrCreate_(pin);
        if (!pr) return fail_(err, pin, "invalid BCM pin");
#if defined(_WIN32)
        return fail_(err, pin, "GPIO only supported on Linux/Raspberry Pi");
#else
        int v = -1;
        if (!readValue_(pin, v)) {
            return fail_(err, pin, usePinctrl_() ? "pinctrl read failed" : "failed to read value");
        }
        if (outValue) *outValue = v;
        {
            std::lock_guard<std::mutex> lk(m_);
            pr->value = v;
            pr->lastSampleTsMs = nowMs_();
            pr->error.clear();
        }
        emit_({nowMs_(), pin, "sample", "read", v});
        return true;
#endif
    }

    bool startPwm(int pin, double freqHz, double dutyPct, std::string* err = nullptr)
    {
        auto pr = getOrCreate_(pin);
        if (!pr) return fail_(err, pin, "invalid BCM pin");
#if defined(_WIN32)
        return fail_(err, pin, "GPIO only supported on Linux/Raspberry Pi");
#else
        if (!configurePin(pin, "output", err)) return false;

        freqHz = std::max(0.5, std::min(freqHz, 500.0));
        dutyPct = std::max(0.0, std::min(dutyPct, 100.0));

        stopPwm(pin, nullptr);
        {
            std::lock_guard<std::mutex> lk(m_);
            pr->pwmStop.store(false);
            pr->pwmRunning = true;
            pr->pwmFreqHz = freqHz;
            pr->pwmDutyPct = dutyPct;
            pr->mode = "output";
            pr->error.clear();
        }

        pr->pwmThread = std::thread([this, pr]() {
            const int pin = pr->pin;
            emit_({nowMs_(), pin, "info", usePinctrl_() ? "pwm started (soft+pinctrl)" : "pwm started", -1});
            while (!pr->pwmStop.load()) {
                double freq = 0.0, duty = 0.0;
                {
                    std::lock_guard<std::mutex> lk(m_);
                    freq = pr->pwmFreqHz;
                    duty = pr->pwmDutyPct;
                }
                const double periodUs = 1e6 / std::max(0.5, freq);
                const double highUs = periodUs * (std::max(0.0, std::min(100.0, duty)) / 100.0);
                const double lowUs  = std::max(0.0, periodUs - highUs);

                if (duty <= 0.0) {
                    writeValue_(pin, 0);
                    std::this_thread::sleep_for(std::chrono::microseconds((int)std::max(1000.0, periodUs)));
                    continue;
                }
                if (duty >= 100.0) {
                    writeValue_(pin, 1);
                    std::this_thread::sleep_for(std::chrono::microseconds((int)std::max(1000.0, periodUs)));
                    continue;
                }

                writeValue_(pin, 1);
                std::this_thread::sleep_for(std::chrono::microseconds((int)std::max(50.0, highUs)));
                writeValue_(pin, 0);
                std::this_thread::sleep_for(std::chrono::microseconds((int)std::max(50.0, lowUs)));
            }
            writeValue_(pin, 0);
            emit_({nowMs_(), pin, "info", "pwm stopped", 0});
        });
        return true;
#endif
    }

    bool updatePwm(int pin, double freqHz, double dutyPct, std::string* err = nullptr)
    {
        auto pr = getOrCreate_(pin);
        if (!pr) return fail_(err, pin, "invalid BCM pin");
        std::lock_guard<std::mutex> lk(m_);
        if (!pr->pwmRunning) return fail_(err, pin, "pwm not running");
        pr->pwmFreqHz = std::max(0.5, std::min(freqHz, 500.0));
        pr->pwmDutyPct = std::max(0.0, std::min(dutyPct, 100.0));
        pr->error.clear();
        return true;
    }

    bool stopPwm(int pin, std::string* err = nullptr)
    {
        auto pr = getOrCreate_(pin);
        if (!pr) return false;
        bool had = false;
        {
            std::lock_guard<std::mutex> lk(m_);
            had = pr->pwmRunning;
            pr->pwmStop.store(true);
        }
        if (pr->pwmThread.joinable()) pr->pwmThread.join();
        {
            std::lock_guard<std::mutex> lk(m_);
            pr->pwmRunning = false;
            pr->pwmStop.store(false);
        }
        (void)err;
        return had;
    }

    bool startSampling(int pin, double hz, std::string* err = nullptr)
    {
        auto pr = getOrCreate_(pin);
        if (!pr) return fail_(err, pin, "invalid BCM pin");
#if defined(_WIN32)
        return fail_(err, pin, "GPIO only supported on Linux/Raspberry Pi");
#else
        if (pr->mode != "input") {
            // Allow sampling output too, but default to input semantics.
            configurePin(pin, "input", nullptr);
        }
        hz = std::max(0.5, std::min(hz, 200.0));
        stopSampling(pin, nullptr);
        {
            std::lock_guard<std::mutex> lk(m_);
            pr->sampleStop.store(false);
            pr->sampleRunning = true;
            pr->sampleHz = hz;
            pr->error.clear();
        }
        pr->sampleThread = std::thread([this, pr]() {
            int last = -999;
            emit_({nowMs_(), pr->pin, "info", "sampling started", -1});
            while (!pr->sampleStop.load()) {
                double hz = 1.0;
                {
                    std::lock_guard<std::mutex> lk(m_);
                    hz = pr->sampleHz;
                }
                int v = -1;
                if (readValue_(pr->pin, v)) {
                    const auto ts = nowMs_();
                    bool changed = false;
                    {
                        std::lock_guard<std::mutex> lk(m_);
                        pr->value = v;
                        pr->lastSampleTsMs = ts;
                        pr->error.clear();
                        changed = (v != last);
                    }
                    if (changed || hz <= 5.0) {
                        emit_({ts, pr->pin, "sample", "sample", v});
                    }
                    last = v;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds((int)std::max(5.0, 1000.0 / std::max(0.5, hz))));
            }
            emit_({nowMs_(), pr->pin, "info", "sampling stopped", -1});
        });
        return true;
#endif
    }

    bool stopSampling(int pin, std::string* err = nullptr)
    {
        auto pr = getOrCreate_(pin);
        if (!pr) return false;
        bool had = false;
        {
            std::lock_guard<std::mutex> lk(m_);
            had = pr->sampleRunning;
            pr->sampleStop.store(true);
        }
        if (pr->sampleThread.joinable()) pr->sampleThread.join();
        {
            std::lock_guard<std::mutex> lk(m_);
            pr->sampleRunning = false;
            pr->sampleStop.store(false);
        }
        (void)err;
        return had;
    }

    void stopAll()
    {
        std::vector<std::shared_ptr<PinRuntime>> pins;
        {
            std::lock_guard<std::mutex> lk(m_);
            for (auto& kv : pins_) pins.push_back(kv.second);
        }
        for (auto& p : pins) {
            if (!p) continue;
            p->pwmStop.store(true);
            p->sampleStop.store(true);
        }
        for (auto& p : pins) {
            if (!p) continue;
            if (p->pwmThread.joinable()) p->pwmThread.join();
            if (p->sampleThread.joinable()) p->sampleThread.join();
        }
        std::lock_guard<std::mutex> lk(m_);
        for (auto& kv : pins_) {
            kv.second->pwmRunning = false;
            kv.second->sampleRunning = false;
        }
    }

    std::string statusJson() const
    {
        std::ostringstream oss;
        oss << "{";
#if defined(_WIN32)
        oss << "\"backend\":\"unsupported_windows\",";
#else
        oss << "\"backend\":\"" << (usePinctrl_() ? "pinctrl" : "sysfs") << "\",";
#endif
        oss << "\"pins\":[";
        bool first = true;
        std::lock_guard<std::mutex> lk(m_);
        for (const auto& kv : pins_) {
            const auto& p = kv.second;
            if (!p) continue;
            if (!first) oss << ',';
            first = false;
            oss << '{';
            oss << "\"pin\":" << p->pin << ',';
            oss << "\"mode\":\"" << jsonEsc_(p->mode) << "\",";
            oss << "\"value\":" << p->value << ',';
            oss << "\"pwm_running\":" << (p->pwmRunning ? "true" : "false") << ',';
            oss << "\"pwm_freq_hz\":" << p->pwmFreqHz << ',';
            oss << "\"pwm_duty_pct\":" << p->pwmDutyPct << ',';
            oss << "\"sample_running\":" << (p->sampleRunning ? "true" : "false") << ',';
            oss << "\"sample_hz\":" << p->sampleHz << ',';
            oss << "\"last_sample_ts_ms\":" << p->lastSampleTsMs << ',';
            oss << "\"error\":\"" << jsonEsc_(p->error) << "\"";
            oss << '}';
        }
        oss << "]}";
        return oss.str();
    }

private:
    struct PinRuntime {
        explicit PinRuntime(int bcmPin) : pin(bcmPin) {}
        int pin = -1;
        std::string mode = "unconfigured";
        int value = 0;
        bool exported = false;
        std::string error;

        std::atomic<bool> pwmStop{false};
        bool pwmRunning = false;
        double pwmFreqHz = 10.0;
        double pwmDutyPct = 50.0;
        std::thread pwmThread;

        std::atomic<bool> sampleStop{false};
        bool sampleRunning = false;
        double sampleHz = 5.0;
        uint64_t lastSampleTsMs = 0;
        std::thread sampleThread;
    };

    mutable std::mutex m_;
    std::map<int, std::shared_ptr<PinRuntime>> pins_;
    EventCallback cb_;

    static uint64_t nowMs_()
    {
        using namespace std::chrono;
        return (uint64_t)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }

    static std::string trim_(std::string s)
    {
        auto isWs = [](unsigned char c) { return std::isspace(c) != 0; };
        while (!s.empty() && isWs((unsigned char)s.front())) s.erase(s.begin());
        while (!s.empty() && isWs((unsigned char)s.back())) s.pop_back();
        return s;
    }

    static std::string lower_(std::string s)
    {
        for (auto& c : s) c = (char)std::tolower((unsigned char)c);
        return s;
    }

    static std::string jsonEsc_(const std::string& s)
    {
        std::ostringstream o;
        for (char c : s) { if (c == '"' || c == '\\') o << '\\'; o << c; }
        return o.str();
    }

    std::shared_ptr<PinRuntime> getOrCreate_(int pin)
    {
        if (pin < 0 || pin > 27) return nullptr;
        std::lock_guard<std::mutex> lk(m_);
        auto it = pins_.find(pin);
        if (it != pins_.end()) return it->second;
        auto pr = std::make_shared<PinRuntime>(pin);
        pins_[pin] = pr;
        return pr;
    }

    bool fail_(std::string* err, int pin, const std::string& msg)
    {
        if (err) *err = msg;
        if (pin >= 0) {
            auto pr = getOrCreate_(pin);
            if (pr) {
                std::lock_guard<std::mutex> lk(m_);
                pr->error = msg;
            }
        }
        emit_({nowMs_(), pin, "err", msg, -1});
        return false;
    }

    void emit_(GpioEvent ev)
    {
        EventCallback cb;
        {
            std::lock_guard<std::mutex> lk(m_);
            cb = cb_;
        }
        if (cb) cb(ev);
    }

#if !defined(_WIN32)
    static bool execCmdOk_(const std::string& cmd, std::string* out = nullptr)
    {
        FILE* fp = ::popen((cmd + " 2>/dev/null").c_str(), "r");
        if (!fp) return false;
        std::string acc;
        char buf[256];
        while (std::fgets(buf, sizeof(buf), fp)) acc += buf;
        const int rc = ::pclose(fp);
        if (out) *out = acc;
        return rc == 0;
    }

    static bool usePinctrl_()
    {
        static int cached = -1;
        if (cached >= 0) return cached == 1;
        std::string out;
        cached = execCmdOk_("command -v pinctrl", &out) ? 1 : 0;
        return cached == 1;
    }

    static bool parsePinctrlValue_(const std::string& s, int& out)
    {
        std::string l = lower_(s);
        if (l.find("| hi") != std::string::npos || l.find("level=1") != std::string::npos || l.find(" = hi") != std::string::npos) {
            out = 1; return true;
        }
        if (l.find("| lo") != std::string::npos || l.find("level=0") != std::string::npos || l.find(" = lo") != std::string::npos) {
            out = 0; return true;
        }
        return false;
    }

    static std::string gpioPath_(int pin, const std::string& leaf)
    {
        return std::string("/sys/class/gpio/gpio") + std::to_string(pin) + "/" + leaf;
    }

    static bool writeFile_(const std::string& path, const std::string& v)
    {
        std::ofstream ofs(path);
        if (!ofs) return false;
        ofs << v;
        return ofs.good();
    }

    static bool readFile_(const std::string& path, std::string& out)
    {
        std::ifstream ifs(path);
        if (!ifs) return false;
        std::ostringstream oss; oss << ifs.rdbuf();
        out = oss.str();
        return true;
    }

    bool ensureExported_(const std::shared_ptr<PinRuntime>& pr, std::string* err)
    {
        if (!pr) return fail_(err, -1, "null pin runtime");
        const auto dir = gpioPath_(pr->pin, "direction");
        if (!std::filesystem::exists(dir)) {
            (void)writeFile_("/sys/class/gpio/export", std::to_string(pr->pin));
            for (int i = 0; i < 200; ++i) {
                if (std::filesystem::exists(dir)) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        if (!std::filesystem::exists(dir)) {
            return fail_(err, pr->pin, "gpio path not ready (sysfs disabled? use sudo or install pinctrl)");
        }
        {
            std::lock_guard<std::mutex> lk(m_);
            pr->exported = true;
        }
        return true;
    }

    bool writeValue_(int pin, int v) const
    {
        if (usePinctrl_()) {
            const std::string cmd = std::string("pinctrl set ") + std::to_string(pin) + (v ? " dh" : " dl");
            return execCmdOk_(cmd, nullptr);
        }
        return writeFile_(gpioPath_(pin, "value"), v ? "1" : "0");
    }

    bool readValue_(int pin, int& out) const
    {
        if (usePinctrl_()) {
            std::string s;
            if (!execCmdOk_(std::string("pinctrl get ") + std::to_string(pin), &s)) return false;
            return parsePinctrlValue_(s, out);
        }
        std::string s;
        if (!readFile_(gpioPath_(pin, "value"), s)) return false;
        s = trim_(s);
        out = (!s.empty() && s[0] == '1') ? 1 : 0;
        return true;
    }
#endif
};

} // namespace CATJ_gpio_web
