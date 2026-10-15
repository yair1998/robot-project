// Deterministic tests of the local perception / navigation / safety core.
// No network, no sleeps: a ManualClock and direct calls to ControlCore::tick(). Exit code 1 on any failure.
// Scenario numbers follow B-STEP 9 of the task; extras follow its "Also test" list.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "core/clock.h"
#include "core/command_mailbox.h"
#include "core/control_core.h"
#include "core/local_perception.h"
#include "core/motion_backend.h"
#include "core/python_perception_adapter.h"
#include "core/robot_config.h"
#include "core/simulated_sensor_provider.h"
#include "core/stopping_model.h"
#include "core/telemetry_channel.h"

using namespace robot;

namespace {

int g_pass = 0;
int g_fail = 0;

void check(const std::string& description, bool condition) {
    if (condition) {
        ++g_pass;
        std::cout << "  PASS: " << description << std::endl;
    } else {
        ++g_fail;
        std::cout << "  FAIL: " << description << std::endl;
    }
}

void header(const std::string& title) {
    std::cout << std::endl << "=== " << title << " ===" << std::endl;
}

std::string num(double v, int decimals = 3) { return fmt(v, decimals); }

// LiveHarness.java fires retreat_held when the robot is within this distance of the B->D corridor line.
constexpr double kLiveHarnessTriggerToleranceM = 0.5;
constexpr double kJavaArrivalThresholdM = 0.5;   // MissionManager.ARRIVAL_THRESHOLD_METERS
constexpr double kJavaStopThresholdM = 1.0;      // RobotChannelService.evaluateSafety: STOP_PROXIMITY below this
constexpr double kBdLegLengthM = 9.0;            // B (10,0) -> D (10,9) in mission_map.json

SectorReading freeR(double range, double confidence = 1.0, double uncertainty = 0.0) {
    SectorReading r;
    r.state = SectorState::FREE_VERIFIED;
    r.free_range_m = range;
    r.range_uncertainty_m = uncertainty;
    r.confidence = confidence;
    return r;
}

SectorReading occR(double range, double confidence = 1.0, double uncertainty = 0.0, bool verifiesFree = true) {
    SectorReading r;
    r.state = SectorState::OCCUPIED;
    r.hazard_range_m = range;
    if (verifiesFree) {
        r.free_range_m = range;
    }
    r.range_uncertainty_m = uncertainty;
    r.confidence = confidence;
    return r;
}

SectorReading unknownR() { return SectorReading{}; }

SensorObservation obs3(SectorReading ahead, SectorReading left, SectorReading right) {
    SensorObservation o;
    o.validity = Validity::VALID;
    o.confidence = 1.0;
    o.sectors[index(Sector::AHEAD)] = ahead;
    o.sectors[index(Sector::LEFT)] = left;
    o.sectors[index(Sector::RIGHT)] = right;
    return o;
}

SensorObservation clearObs(double range = 8.0) { return obs3(freeR(range), freeR(range), freeR(range)); }

// Test double: returns whatever the test scripted, stamped with the snapshot time unless told otherwise.
class ScriptedSensor final : public ISensorProvider {
public:
    explicit ScriptedSensor(std::string id) : id_(std::move(id)) { set(clearObs()); }
    const std::string& id() const override { return id_; }
    SensorObservation snapshot(SteadyTime now) override {
        SensorObservation o = obs_;
        o.source_id = id_;
        if (stamp_now_) {
            o.acquired_at = now;
        }
        return o;
    }
    void set(const SensorObservation& o, bool stampNow = true) {
        obs_ = o;
        stamp_now_ = stampNow;
    }

private:
    std::string id_;
    SensorObservation obs_;
    bool stamp_now_ = true;
};

PerceptionMessage msg(const std::string& session, uint64_t seq) {
    PerceptionMessage m;
    m.schema_version = 1;
    m.robot_id = "robot-1";
    m.session_id = session;
    m.sequence = seq;
    m.sender_age_at_send_ms = 0.0;
    m.source_id = "py-test";
    m.frame = "ROBOT";
    return m;
}

PerceptionMessage withSummary(PerceptionMessage m, Sector s, SectorState state, double range, double confidence) {
    if (!m.sector_summary) {
        m.sector_summary.emplace();
    }
    (*m.sector_summary)[index(s)] = SectorSummaryEntry{state, range, 0.0, confidence};
    return m;
}

// The whole core around a ScriptedSensor ("direct", required) and the Python adapter (supplementary), or around the
// real SimulatedSensorProvider when useSim is true.
struct Bed {
    RobotConfig cfg;
    ManualClock clock;
    std::unique_ptr<SimulatedMotionBackend> motion;
    CommandMailbox mailbox;
    TelemetryChannel telemetry;
    ScriptedSensor direct{"direct"};
    std::unique_ptr<SimulatedSensorProvider> sim;
    std::unique_ptr<PythonPerceptionAdapter> python;
    std::unique_ptr<LocalPerception> perception;
    std::unique_ptr<ControlCore> core;
    std::vector<std::string> logs;
    int64_t nextId = 0;

    explicit Bed(RobotConfig c = RobotConfig{}, bool useSim = false) : cfg(std::move(c)) {
        LogSink sink = [this](const std::string& line) { logs.push_back(line); };
        motion = std::make_unique<SimulatedMotionBackend>(cfg.motion);
        python = std::make_unique<PythonPerceptionAdapter>(
                cfg.python, cfg.nav.robot_half_width_m + cfg.nav.clearance_margin_m, clock);
        std::vector<PerceptionSource> sources;
        if (useSim) {
            sim = std::make_unique<SimulatedSensorProvider>(cfg.sim, sink);
            sources.push_back({sim.get(), SourceRole::REQUIRED_FOR_SAFETY});
        } else {
            sources.push_back({&direct, SourceRole::REQUIRED_FOR_SAFETY});
        }
        sources.push_back({python.get(), SourceRole::SUPPLEMENTARY});
        perception = std::make_unique<LocalPerception>(cfg.perception, sources);
        core = std::make_unique<ControlCore>(cfg, clock, *motion, mailbox, *perception, sim.get(), telemetry, sink);
    }

    void step(int n = 1) {
        for (int i = 0; i < n; ++i) {
            clock.advance(cfg.control_period_s);
            core->tick();
        }
    }
    void java(JavaCommandType type, double speed = 1.0, std::optional<Point> waypoint = std::nullopt) {
        mailbox.post(JavaCommand{type, ++nextId, speed, waypoint});
    }
    void goTo(Point p) { java(JavaCommandType::REROUTE, 1.0, p); }
    TelemetrySnapshot tel() const { return *telemetry.latest(); }
    Point pos() const { return motion->pose().position; }
    double lateral() const { return signedLateralOffset(core->legStart(), core->target(), pos()); }
    SafetyStatus status() const { return core->lastSafety().status; }
    NavState nav() const { return core->lastNav().state; }
    bool logged(const std::string& needle) const {
        for (const std::string& l : logs) {
            if (l.find(needle) != std::string::npos) return true;
        }
        return false;
    }
};

// ---------------------------------------------------------------------------------------------------------------
void s01_clearRoute() {
    header("1: clear route -> normal waypoint tracking");
    Bed b;
    b.goTo({10, 0});
    b.step();
    check("TRACKING_WAYPOINT (got " + std::string(toString(b.nav())) + ")", b.nav() == NavState::TRACKING_WAYPOINT);
    check("moved 0.2 m toward the waypoint in one tick (x " + num(b.pos().x) + ")", std::fabs(b.pos().x - 0.2) < 1e-9);
    check("telemetry: measured speed 1.0, nearest 8.0, no emergency",
          std::fabs(b.tel().measured_speed_mps - 1.0) < 1e-9 && b.tel().nearest_obstacle_m
          && *b.tel().nearest_obstacle_m == 8.0 && !b.tel().emergency_stop);
    int ticks = 0;
    while (b.core->hasTarget() && ticks < 200) {
        b.step();
        ++ticks;
    }
    check("arrives (\"Arrived at target (10, 0)\") after " + std::to_string(ticks + 1) + " ticks",
          !b.core->hasTarget() && b.logged("Arrived at target (10, 0)"));
    check("arrival is within 0.3 m of the REAL waypoint (" + num(distance(b.pos(), {10, 0})) + " m)",
          distance(b.pos(), {10, 0}) < 0.3);
    b.step();
    check("then IDLE_NO_TARGET with speed 0 and no emergency",
          b.nav() == NavState::IDLE_NO_TARGET && b.tel().measured_speed_mps == 0.0 && !b.tel().emergency_stop);
}

void s02_obstacleAheadSlows() {
    header("2: obstacle ahead -> speed reduction");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.direct.set(obs3(occR(2.5), freeR(8.0), freeR(8.0)));
    double x0 = b.pos().x;
    b.step();
    check("SLOWING_DOWN (got " + std::string(toString(b.nav())) + ")", b.nav() == NavState::SLOWING_DOWN);
    check("speed 0.5 = 1.0 * (2.5 - 2.0) / (3.0 - 2.0) (measured " + num(b.tel().measured_speed_mps) + ")",
          std::fabs(b.tel().measured_speed_mps - 0.5) < 1e-9);
    check("still moving (" + num(b.pos().x - x0) + " m this tick) and no local stop",
          b.pos().x > x0 && !b.core->safety().latched() && !b.tel().emergency_stop);
}

void s03_04_criticalObstacleAndRelease() {
    header("3: critical obstacle -> immediate local stop, without waiting for Java");
    Bed b;
    b.goTo({10, 0});
    b.step();
    Point before = b.pos();
    uint64_t javaCommands = b.mailbox.posted();
    b.direct.set(obs3(occR(0.5), unknownR(), unknownR()));
    b.step();
    check("LOCAL_STOP_LATCHED in the same tick (got " + std::string(toString(b.status())) + ")",
          b.status() == SafetyStatus::LOCAL_STOP_LATCHED);
    check("no movement in that tick (" + num(distance(before, b.pos())) + " m)", distance(before, b.pos()) < 1e-12);
    check("telemetry: speed 0, emergency_stop true, nearest 0.5",
          b.tel().measured_speed_mps == 0.0 && b.tel().emergency_stop && b.tel().nearest_obstacle_m
          && *b.tel().nearest_obstacle_m == 0.5);
    check("no Java command was involved (" + std::to_string(b.mailbox.posted() - javaCommands) + " new)",
          b.mailbox.posted() == javaCommands);
    check("log: \"LOCAL_STOP: hazard 0.50 m <= d_stop 1.05 m at 1.00 m/s\"",
          b.logged("LOCAL_STOP: hazard 0.50 m <= d_stop 1.05 m at 1.00 m/s"));

    header("4: obstacle disappears -> controlled recovery, only after the release rule");
    b.direct.set(clearObs());
    b.step();
    check("1st clear observation: still latched, no movement", b.core->safety().latched() && b.tel().measured_speed_mps == 0.0);
    b.step();
    check("2nd clear observation: still latched, no movement", b.core->safety().latched() && b.tel().measured_speed_mps == 0.0);
    double x0 = b.pos().x;
    b.step();
    check("3rd clear observation (0.4 s >= 0.3 s hold): released and moving (" + num(b.pos().x - x0) + " m)",
          !b.core->safety().latched() && b.pos().x > x0 && b.status() == SafetyStatus::NORMAL);
    check("log: \"local stop released: 3 consecutive clear observations\"",
          b.logged("local stop released: 3 consecutive clear observations"));

    Bed f;
    f.goTo({10, 0});
    f.step();
    f.direct.set(obs3(occR(0.5), unknownR(), unknownR()));
    f.step();
    f.direct.set(clearObs());
    f.step(2);
    f.direct.set(obs3(occR(0.5), unknownR(), unknownR()));
    f.step();
    f.direct.set(clearObs());
    f.step(2);
    check("one hazard reading between clear ones resets the count: still latched after 2 more clear ticks",
          f.core->safety().latched());
    f.step();
    check("released only at the 3rd consecutive clear observation", !f.core->safety().latched());
}

void s05_javaStopWins() {
    header("5: Java STOP -> local navigation cannot override it");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.java(JavaCommandType::STOP, 0.0);
    b.step();
    check("status JAVA_STOP, speed 0 (got " + std::string(toString(b.status())) + ", "
          + num(b.tel().measured_speed_mps) + ")",
          b.status() == SafetyStatus::JAVA_STOP && b.tel().measured_speed_mps == 0.0);
    check("although navigation still proposes 1.0 m/s on a clear route",
          b.nav() == NavState::TRACKING_WAYPOINT && b.core->lastNav().speed_mps == 1.0);
    check("a Java STOP alone is not a local stop: emergency_stop false", !b.tel().emergency_stop);
    b.java(JavaCommandType::CONTINUE, 1.0);
    b.step();
    check("CONTINUE: moving again at 1.0", std::fabs(b.tel().measured_speed_mps - 1.0) < 1e-9);
}

void s06_resumeDoesNotClearLocalStop() {
    header("6: Java RESUME while local danger remains -> stays stopped");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.direct.set(obs3(occR(0.5), unknownR(), unknownR()));
    b.java(JavaCommandType::STOP, 0.0);
    b.step();
    b.java(JavaCommandType::RESUME, 1.0);
    b.step(3);
    check("RESUME cleared the Java flag", !b.core->javaStop());
    check("but the local stop holds: LOCAL_STOP_LATCHED, speed 0, emergency_stop true",
          b.status() == SafetyStatus::LOCAL_STOP_LATCHED && b.tel().measured_speed_mps == 0.0 && b.tel().emergency_stop);
}

void s07_rerouteDuringAvoidance() {
    header("7: Java REROUTE during local avoidance -> safe target transition, no stale deviation");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.direct.set(obs3(occR(1.5), freeR(8.0), unknownR()));
    b.step(4);
    check("in LOCAL_AVOIDANCE toward LEFT (lateral " + num(b.lateral()) + " m)",
          b.nav() == NavState::LOCAL_AVOIDANCE && b.lateral() > 0.0);
    Point p = b.pos();
    b.direct.set(clearObs());
    b.goTo({p.x, p.y + 10.0});
    b.step();
    check("the leg starts at the pose where the new waypoint was adopted",
          distance(b.core->legStart(), p) < 1e-12);
    check("avoidance state reset (log) and TRACKING the new waypoint (got " + std::string(toString(b.nav())) + ")",
          b.logged("new waypoint - avoidance state reset") && b.nav() == NavState::TRACKING_WAYPOINT);
    check("heading is the direct bearing to the new waypoint (" + num(radToDeg(b.core->lastNav().heading_rad), 1) + " deg)",
          std::fabs(b.core->lastNav().heading_rad - kPi / 2.0) < 1e-9);
    check("no deviation carried over: lateral offset from the new leg " + num(b.lateral(), 6) + " m",
          std::fabs(b.lateral()) < 1e-9);
}

void s08_perceptionStaleOrLost() {
    header("8: perception stale or disconnected -> conservative behaviour");
    {
        Bed b;
        b.goTo({10, 0});
        b.step();
        SensorObservation stale = clearObs();
        stale.validity = Validity::STALE;
        b.direct.set(stale);
        b.step();
        check("required source reports STALE -> LOCAL_STOP (\"required perception not valid (direct STALE\")",
              b.status() == SafetyStatus::LOCAL_STOP_LATCHED && b.logged("required perception not valid (direct STALE"));
        check("no valid reading left -> nearest_obstacle_distance absent, emergency_stop true",
              !b.tel().nearest_obstacle_m && b.tel().emergency_stop);
    }
    {
        Bed b;
        b.goTo({10, 0});
        b.step();
        SensorObservation old = clearObs();
        old.acquired_at = b.clock.now();
        b.direct.set(old, false);   // VALID per the provider, but its timestamp stops moving
        b.step(3);                  // 0.6 s > max_required_age_s 0.5
        check("required source VALID but 0.6 s old -> treated as STALE -> LOCAL_STOP",
              b.status() == SafetyStatus::LOCAL_STOP_LATCHED);
    }
    {
        Bed b;
        b.goTo({10, 0});
        b.step(2);
        check("Python never connected (UNAVAILABLE, supplementary): normal tracking",
              b.nav() == NavState::TRACKING_WAYPOINT && b.tel().measured_speed_mps == 1.0
              && b.core->lastPerception().sources[1].validity == Validity::UNAVAILABLE);
    }
    {
        Bed b;
        b.goTo({10, 0});
        b.step();
        b.direct.set(obs3(occR(1.5), unknownR(), unknownR()));
        b.python->offer(withSummary(msg("s", 1), Sector::LEFT, SectorState::FREE_VERIFIED, 8.0, 0.9));
        b.step();
        check("only Python verifies LEFT free -> LOCAL_AVOIDANCE toward LEFT",
              b.nav() == NavState::LOCAL_AVOIDANCE && b.core->lastNav().motion_sector == Sector::LEFT);
        b.step(2);   // Python's message is now 0.6 s old (> stale_after_s 0.5): no longer VALID
        Point p = b.pos();
        b.step(2);
        check("Python silent -> LEFT no longer verified -> BLOCKED, speed 0 (got " + std::string(toString(b.nav())) + ")",
              b.nav() == NavState::BLOCKED && b.tel().measured_speed_mps == 0.0);
        check("no blind lateral motion after the loss (" + num(distance(p, b.pos())) + " m)", distance(p, b.pos()) < 1e-12);
        check("reported as a temporary block: TEMPORARY_BLOCK and emergency_stop true",
              b.status() == SafetyStatus::TEMPORARY_BLOCK && b.tel().emergency_stop);
    }
}

void s09_directDangerWins() {
    header("9: direct sensor reports danger while Python reports clear -> danger wins");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.direct.set(obs3(occR(0.5), freeR(8.0), freeR(8.0)));
    b.python->offer(withSummary(msg("s", 1), Sector::AHEAD, SectorState::FREE_VERIFIED, 8.0, 1.0));
    b.step();
    const FusedSector& ahead = b.core->lastPerception().sectors[index(Sector::AHEAD)];
    check("fused AHEAD is OCCUPIED at 0.5 despite Python's confident FREE (got " + std::string(toString(ahead.state)) + ")",
          ahead.state == SectorState::OCCUPIED && ahead.hazard_m && *ahead.hazard_m == 0.5);
    check("-> LOCAL_STOP", b.status() == SafetyStatus::LOCAL_STOP_LATCHED);
}

void s10_noFreeDirection() {
    header("10: no verified-free direction -> stops; no blind lateral deviation");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.direct.set(obs3(occR(1.5), unknownR(), unknownR()));
    Point p = b.pos();
    b.step(3);
    check("BLOCKED (got " + std::string(toString(b.nav())) + "), speed 0", b.nav() == NavState::BLOCKED && b.tel().measured_speed_mps == 0.0);
    check("position unchanged, no lateral offset (y " + num(b.pos().y, 6) + ")", distance(p, b.pos()) < 1e-12);
    check("a temporary block, not a latch (1.5 m > d_stop): TEMPORARY_BLOCK, emergency_stop true",
          b.status() == SafetyStatus::TEMPORARY_BLOCK && !b.core->safety().latched() && b.tel().emergency_stop);
}

void s11_latestWins() {
    header("11: perception updates faster than control -> latest valid data used, no backlog");
    ManualClock clock;
    PythonAdapterConfig cfg;
    PythonPerceptionAdapter a(cfg, 0.45, clock);
    for (uint64_t seq = 1; seq <= 100; ++seq) {
        PerceptionMessage m = msg("s", seq);
        m.obstacles.push_back({0.0, 2.0 + seq / 1000.0, 0.0, std::nullopt, 0.9});
        a.offer(m);
    }
    AdapterStats st = a.stats();
    check("100 offered, 100 accepted, 99 replaced before any snapshot (one slot)",
          st.offered == 100 && st.accepted == 100 && st.replaced_unconsumed == 99);
    SensorObservation o = a.snapshot(clock.now());
    const SectorReading& ahead = o.sectors[index(Sector::AHEAD)];
    check("the snapshot carries the LAST message (AHEAD hazard " + num(ahead.hazard_range_m.value_or(-1)) + " m = 2.100)",
          o.validity == Validity::VALID && ahead.hazard_range_m && std::fabs(*ahead.hazard_range_m - 2.1) < 1e-9);
    check("an older sequence afterwards is rejected OUT_OF_ORDER", a.offer(msg("s", 50)) == RejectReason::OUT_OF_ORDER);

    Bed b;
    b.goTo({10, 0});
    b.step();
    for (uint64_t seq = 1; seq <= 50; ++seq) {
        b.python->offer(withSummary(msg("s", seq), Sector::LEFT, SectorState::FREE_VERIFIED, seq < 50 ? 1.0 : 7.0, 0.9));
    }
    b.direct.set(obs3(occR(1.5), unknownR(), unknownR()));
    b.step();
    check("between two ticks 50 messages arrived; the tick used only the last one (LEFT free to "
          + num(b.core->lastPerception().sectors[index(Sector::LEFT)].free_m.value_or(-1)) + " m)",
          b.core->lastPerception().sectors[index(Sector::LEFT)].free_m == 7.0);
}

void s12_concurrency() {
    header("12: concurrent commands and observations -> no corrupted state, no deadlock (watchdog 60 s)");
    Bed b;
    b.goTo({10, 0});
    const int kCommands = 20000;
    const int kMessages = 20000;
    const int kTicks = 5000;
    std::atomic<bool> controlDone{false};

    auto reader = std::async(std::launch::async, [&] {
        for (int i = 0; i < kCommands; ++i) {
            JavaCommandType t = static_cast<JavaCommandType>(1 + i % 5);
            std::optional<Point> wp;
            if (t == JavaCommandType::REROUTE) wp = Point{10.0 + (i % 7), (i % 3) * 1.0};
            b.mailbox.post(JavaCommand{t, i + 1, (i % 4) * 0.5, wp});
        }
        return true;
    });
    auto producer = std::async(std::launch::async, [&] {
        int rejected = 0;
        for (int i = 0; i < kMessages; ++i) {
            PerceptionMessage m = msg("session-" + std::to_string(i / 5000), static_cast<uint64_t>(i % 5000 + 1));
            m.obstacles.push_back({0.1 * (i % 10), 1.0 + (i % 5), 0.05, std::nullopt, 0.8});
            if (b.python->offer(m)) ++rejected;
        }
        return rejected;
    });
    auto consumer = std::async(std::launch::async, [&] {
        uint64_t last = 0;
        bool monotonic = true;
        while (!controlDone.load()) {
            std::optional<TelemetrySnapshot> s = b.telemetry.latest();
            if (s) {
                if (s->seq < last) monotonic = false;
                last = s->seq;
            }
            std::this_thread::yield();
        }
        return monotonic;
    });
    auto control = std::async(std::launch::async, [&] {
        for (int i = 0; i < kTicks; ++i) b.step();
        controlDone.store(true);
        return true;
    });

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
    bool finished = reader.wait_until(deadline) == std::future_status::ready
                    && producer.wait_until(deadline) == std::future_status::ready
                    && control.wait_until(deadline) == std::future_status::ready
                    && consumer.wait_until(deadline) == std::future_status::ready;
    check("watchdog: all four threads finished within 60 s (no deadlock)", finished);
    if (!finished) {
        std::cout << "  (aborting: a thread is stuck)" << std::endl;
        std::exit(1);
    }
    reader.get();
    int rejected = producer.get();
    control.get();
    bool monotonic = consumer.get();
    AdapterStats st = b.python->stats();
    check("telemetry seq reached " + std::to_string(kTicks) + " (got " + std::to_string(b.tel().seq) + ")",
          b.tel().seq == static_cast<uint64_t>(kTicks));
    check("the consumer never saw telemetry go backwards", monotonic);
    check("mailbox counted every command: the setup REROUTE + " + std::to_string(kCommands) + " concurrent ones (got "
          + std::to_string(b.mailbox.posted()) + ")", b.mailbox.posted() == static_cast<uint64_t>(kCommands) + 1);
    check("adapter: offered = accepted + rejected = " + std::to_string(kMessages) + " (rejected " + std::to_string(rejected) + ")",
          st.offered == static_cast<uint64_t>(kMessages) && st.accepted + st.rejectedTotal() == st.offered);
    check("the final state is consistent: speed finite and >= 0, position finite",
          std::isfinite(b.pos().x) && std::isfinite(b.pos().y) && b.tel().measured_speed_mps >= 0.0);
}

void s13_telemetry() {
    header("13: telemetry reports correct position, MEASURED speed and safety status");
    Bed b;
    b.goTo({10, 0});
    b.step();
    TelemetrySnapshot t = b.tel();
    check("position equals the motion backend's pose", distance(t.position, b.pos()) < 1e-12);
    check("measured speed 1.0 at cruise", std::fabs(t.measured_speed_mps - 1.0) < 1e-9);
    b.java(JavaCommandType::SLOW_DOWN, 1.0);
    Point p = b.pos();
    b.step();
    check("SLOW_DOWN visibly changes motion: measured 0.5 m/s, 0.1 m this tick (" + num(distance(p, b.pos())) + ")",
          std::fabs(b.tel().measured_speed_mps - 0.5) < 1e-9 && std::fabs(distance(p, b.pos()) - 0.1) < 1e-9);
    b.java(JavaCommandType::CONTINUE, 1.0);
    b.step();
    check("CONTINUE lifts the cap: measured 1.0 again", std::fabs(b.tel().measured_speed_mps - 1.0) < 1e-9);
    b.java(JavaCommandType::STOP, 0.0);
    b.step();
    check("Java STOP: measured 0.0, emergency_stop false", b.tel().measured_speed_mps == 0.0 && !b.tel().emergency_stop);
    b.java(JavaCommandType::CONTINUE, 1.0);
    b.direct.set(obs3(occR(1.5), unknownR(), unknownR()));
    b.step();
    check("blocked locally: emergency_stop true, nav state BLOCKED in the snapshot",
          b.tel().emergency_stop && b.tel().nav_state == NavState::BLOCKED);
    SensorObservation none;
    none.validity = Validity::UNAVAILABLE;
    b.direct.set(none);
    b.step();
    check("no valid reading -> nearest_obstacle_distance absent (not 0, not 8)", !b.tel().nearest_obstacle_m);
}

// ---------------------------------------------------------------------------------------------------------------
void e01_stoppingModelConservative() {
    header("extra: the stopping-distance model is conservative against a finite-deceleration backend");
    StoppingModel m;
    for (double v : {0.5, 1.0}) {
        for (double offset : {-0.15, 0.0, 0.05, 0.15, 0.25, 0.4}) {
            double appearAt = stoppingDistance(m, v) + offset;   // the obstacle pops up this far ahead
            RobotConfig c;
            c.cruise_speed_mps = v;
            c.motion.max_decel_mps2 = m.braking_decel_mps2;
            Bed b(c);
            b.java(JavaCommandType::REROUTE, v, Point{30, 0});
            b.step(3);
            double obstacleX = b.pos().x + appearAt;
            std::optional<double> latchedAt;
            double speedAtLatch = 0.0;
            for (int i = 0; i < 100; ++i) {
                double gap = obstacleX - b.pos().x;
                b.direct.set(obs3(occR(gap), unknownR(), unknownR()));
                double speedBefore = b.motion->currentSpeed();
                b.step();
                if (!latchedAt && b.core->safety().latched()) {
                    latchedAt = gap;
                    speedAtLatch = speedBefore;
                }
                if (b.motion->currentSpeed() == 0.0 && b.tel().measured_speed_mps == 0.0) break;
            }
            double finalGap = obstacleX - b.pos().x;
            std::string label = "v " + num(v, 1) + ", obstacle appears at d_stop" + (offset >= 0 ? "+" : "") + num(offset, 2)
                                + " = " + num(appearAt) + " m: final gap " + num(finalGap);
            if (offset >= 0.0) {
                check(label + " >= margin " + num(m.margin_m), finalGap >= m.margin_m - 1e-9);
                if (latchedAt) {
                    double braking = *latchedAt - finalGap;
                    check("  braking distance from the latch " + num(braking) + " <= d_stop(" + num(speedAtLatch) + ") "
                          + num(stoppingDistance(m, speedAtLatch)), braking <= stoppingDistance(m, speedAtLatch) + 1e-9);
                }
            } else {
                check(label + " (inside d_stop at first sight: latched immediately, still no contact)",
                      latchedAt.has_value() && finalGap > 0.0);
            }
        }
    }
    RobotConfig soft;
    soft.motion.max_decel_mps2 = 0.5;   // brakes softer than the model assumes (1.0 m/s^2)
    Bed b(soft);
    b.goTo({30, 0});
    b.step(3);
    double obstacleX = b.pos().x + stoppingDistance(m, 1.0);
    for (int i = 0; i < 100; ++i) {
        b.direct.set(obs3(occR(obstacleX - b.pos().x), unknownR(), unknownR()));
        b.step();
        if (b.motion->currentSpeed() == 0.0) break;
    }
    double gap = obstacleX - b.pos().x;
    check("negative control: a drive braking at 0.5 m/s^2 < the assumed 1.0 ends " + num(gap) + " m from the obstacle,"
          " inside the 0.35 m margin - the guarantee holds only if braking_decel_mps2 is truly achievable",
          gap < m.margin_m);
}

void e02_nonPositiveSpeedGuard() {
    header("extra: a non-STOP command with target_speed <= 0 never sets the speed limit to zero");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.java(JavaCommandType::CONTINUE, 0.0);
    b.step();
    check("CONTINUE 0.0: requested speed still 1.0, robot still moving at 1.0",
          b.core->requestedSpeed() == 1.0 && std::fabs(b.tel().measured_speed_mps - 1.0) < 1e-9);
    check("and it was counted as ignored (1)", b.core->ignoredNonPositiveSpeeds() == 1);
    b.java(JavaCommandType::SLOW_DOWN, 0.0);
    b.step();
    check("SLOW_DOWN 0.0: the cap is the configured 0.5, not 0 (measured " + num(b.tel().measured_speed_mps) + ")",
          std::fabs(b.tel().measured_speed_mps - 0.5) < 1e-9);
    b.java(JavaCommandType::REROUTE, -1.0, Point{0, 10});
    b.java(JavaCommandType::CONTINUE, 1.0);
    b.step();
    check("REROUTE -1.0 then CONTINUE 1.0: moving at 1.0 toward the new waypoint",
          std::fabs(b.tel().measured_speed_mps - 1.0) < 1e-9 && b.core->target().y == 10.0);
}

void e03_sessionReset() {
    header("extra: a new sender session_id resets sequence tracking");
    ManualClock clock;
    PythonPerceptionAdapter a(PythonAdapterConfig{}, 0.45, clock);
    check("session A seq 10 accepted", !a.offer(msg("A", 10)));
    check("session A seq 5 rejected OUT_OF_ORDER", a.offer(msg("A", 5)) == RejectReason::OUT_OF_ORDER);
    check("session A seq 10 again rejected OUT_OF_ORDER", a.offer(msg("A", 10)) == RejectReason::OUT_OF_ORDER);
    check("restarted sender, session B seq 1 accepted (not locked out)", !a.offer(msg("B", 1)));
    check("session B seq 2 accepted", !a.offer(msg("B", 2)));
}

void e04_ageFormula() {
    header("extra: age = (now - local receive time) + sender_age_at_send; never sender clock vs local clock");
    ManualClock clock;
    PythonPerceptionAdapter a(PythonAdapterConfig{}, 0.45, clock);
    clock.advance(100.0);
    SteadyTime received = clock.now();
    PerceptionMessage m = msg("s", 1);
    m.sender_age_at_send_ms = 150.0;
    a.offer(m);
    clock.advance(0.2);
    SensorObservation o = a.snapshot(clock.now());
    check("0.2 s after receipt, sender age 0.15 s: age 0.35 s -> VALID (<= 0.5)", o.validity == Validity::VALID);
    check("acquisition time = receipt - sender age", std::fabs(secondsBetween(o.acquired_at, received) - 0.15) < 1e-9);
    clock.advance(0.16);
    check("age 0.51 s -> STALE", a.snapshot(clock.now()).validity == Validity::STALE);
    clock.advance(1.5);
    check("age 2.01 s -> UNAVAILABLE", a.snapshot(clock.now()).validity == Validity::UNAVAILABLE);
}

void e05_harnessToleranceAndRelationships() {
    header("extra: configuration relationships that must not drift");
    RobotConfig c;
    StoppingModel m = c.safety.stopping;
    double dCruise = stoppingDistance(m, c.cruise_speed_mps);
    check("live harness trigger tolerance " + num(kLiveHarnessTriggerToleranceM, 2) + " m >= max_lateral_deviation_m "
          + num(c.nav.max_lateral_deviation_m, 2), kLiveHarnessTriggerToleranceM >= c.nav.max_lateral_deviation_m);
    double holdBound = c.nav.max_lateral_deviation_m
                       + kJavaArrivalThresholdM * (1.0 - c.sim.escalation_start_m / kBdLegLengthM);
    check("worst-case B->D hold distance from the corridor line (max deviation + leg-start offset "
          + num(kJavaArrivalThresholdM, 1) + " * (1 - 6/9)) = " + num(holdBound) + " m <= harness tolerance",
          holdBound <= kLiveHarnessTriggerToleranceM);
    check("(a) escalation hazard " + num(c.sim.escalation_hazard_m, 2) + " m <= d_stop(cruise) " + num(dCruise, 2)
          + " m -> LOCAL_STOP at cruise", c.sim.escalation_hazard_m <= dCruise);
    check("(b) avoidance hazard " + num(c.sim.avoidance_hazard_m, 2) + " m > d_stop(cruise) -> not a stop at cruise",
          c.sim.avoidance_hazard_m > dCruise);
    check("(c) clear range " + num(c.sim.clear_range_m, 1) + " m allows " + num(maxSpeedForDistance(m, c.sim.clear_range_m))
          + " m/s >= cruise: no limiting", maxSpeedForDistance(m, c.sim.clear_range_m) >= c.cruise_speed_mps);
    check("(d) d_stop(cruise) " + num(dCruise, 2) + " m >= Java's STOP threshold " + num(kJavaStopThresholdM, 1) + " m",
          dCruise >= kJavaStopThresholdM);
    check("maxSpeedForDistance inverts stoppingDistance (1.5 m -> " + num(maxSpeedForDistance(m, 1.5)) + " m/s -> "
          + num(stoppingDistance(m, maxSpeedForDistance(m, 1.5))) + " m)",
          std::fabs(stoppingDistance(m, maxSpeedForDistance(m, 1.5)) - 1.5) < 1e-9);
    check("reaction time " + num(m.reaction_time_s, 2) + " s >= control period " + num(c.control_period_s, 2) + " s",
          m.reaction_time_s >= c.control_period_s);
    check("C++ arrival threshold " + num(c.nav.arrival_threshold_m, 2) + " m <= Java's " + num(kJavaArrivalThresholdM, 2) + " m",
          c.nav.arrival_threshold_m <= kJavaArrivalThresholdM);
    check("release threshold d_stop(cruise) + 0.5 = " + num(dCruise + c.safety.release_extra_m, 2)
          + " m is above every stop threshold up to cruise (hysteresis)", dCruise + c.safety.release_extra_m > dCruise);
}

void e06_simulatedSensorScript() {
    header("extra: SimulatedSensorProvider - today's zones, per-sector readings, time-based clear while stopped");
    RobotConfig c;
    ManualClock clock;
    std::vector<std::string> logs;
    SimulatedSensorProvider sim(c.sim, [&](const std::string& l) { logs.push_back(l); });
    auto at = [&](double progress, uint64_t leg = 1) {
        sim.setContext({true, {progress, 0.0}, {0.0, 0.0}, leg});
        return sim.snapshot(clock.now());
    };
    sim.setContext({false, {0, 0}, {0, 0}, 0});
    SensorObservation o = sim.snapshot(clock.now());
    check("no target: still sensing - VALID, all sectors FREE_VERIFIED to 8.0",
          o.validity == Validity::VALID && o.sectors[0].state == SectorState::FREE_VERIFIED && o.sectors[0].free_range_m == 8.0);
    o = at(2.99);
    check("progress 2.99: clear", o.sectors[index(Sector::AHEAD)].state == SectorState::FREE_VERIFIED);
    o = at(3.0);
    check("progress 3.0: AHEAD OCCUPIED at 1.5, LEFT FREE_VERIFIED to 8.0, RIGHT UNKNOWN",
          o.sectors[0].state == SectorState::OCCUPIED && o.sectors[0].hazard_range_m == 1.5
          && o.sectors[1].state == SectorState::FREE_VERIFIED && o.sectors[1].free_range_m == 8.0
          && o.sectors[2].state == SectorState::UNKNOWN);
    o = at(5.0);
    check("progress 5.0: clear again", o.sectors[0].state == SectorState::FREE_VERIFIED);
    o = at(6.0);
    check("progress 6.0: AHEAD OCCUPIED at 0.5, no side FREE_VERIFIED",
          o.sectors[0].state == SectorState::OCCUPIED && o.sectors[0].hazard_range_m == 0.5
          && o.sectors[1].state != SectorState::FREE_VERIFIED && o.sectors[2].state != SectorState::FREE_VERIFIED);
    clock.advance(2.9);
    o = at(6.0);
    check("robot stationary, 2.9 s later: still 0.5 (no progress needed, none made)", o.sectors[0].hazard_range_m == 0.5);
    clock.advance(0.1);
    o = at(6.0);
    check("3.0 s later, still stationary: cleared by TIME -> clear 8.0, logged",
          o.sectors[0].state == SectorState::FREE_VERIFIED && !logs.empty()
          && logs.back().find("cleared after 3.000s") != std::string::npos);
    o = at(6.5);
    check("later in the same zone: stays clear (fires once per leg)", o.sectors[0].state == SectorState::FREE_VERIFIED);
    o = at(6.5, 2);
    check("a new leg re-arms it: 0.5 again", o.sectors[0].hazard_range_m == 0.5);
}

void e07_liveLikeLeg() {
    header("extra: a live-like A->B leg with the real simulated sensor (avoid, reconverge, escalation, release, arrive)");
    Bed b(RobotConfig{}, true);
    b.goTo({10, 0});
    double maxLateral = 0.0;
    bool sawAvoidance = false;
    bool avoidanceOnlyInZone = true;
    std::optional<Point> holdAt;
    int stoppedTicks = 0;
    int ticks = 0;
    while ((b.core->hasTarget() || ticks == 0) && ticks < 400) {
        Point before = b.pos();
        b.step();
        ++ticks;
        maxLateral = std::max(maxLateral, std::fabs(b.lateral()));
        if (b.nav() == NavState::LOCAL_AVOIDANCE) {
            sawAvoidance = true;
            double progress = distance({0, 0}, before);
            if (progress < 3.0 - 1e-9 || progress > 5.0 + 4 * 0.2) avoidanceOnlyInZone = false;
        }
        if (b.status() == SafetyStatus::LOCAL_STOP_LATCHED) {
            if (!holdAt) holdAt = b.pos();
            ++stoppedTicks;
        }
    }
    check("arrived at (10, 0) after " + std::to_string(ticks) + " ticks", !b.core->hasTarget() && b.logged("Arrived at target (10, 0)"));
    check("LOCAL_AVOIDANCE happened, only in or just after the 3.0-5.0 m zone", sawAvoidance && avoidanceOnlyInZone);
    check("the lateral offset from the leg line never exceeded 0.3 m (max " + num(maxLateral) + ")", maxLateral <= 0.3 + 1e-9);
    check("the escalation latched a local stop at the first sensing in the zone (hold at progress "
          + num(holdAt ? distance({0, 0}, *holdAt) : -1) + ", in [6.0, 6.2))",
          holdAt && distance({0, 0}, *holdAt) >= 6.0 && distance({0, 0}, *holdAt) < 6.2);
    check("stopped " + std::to_string(stoppedTicks) + " ticks = 3.0 s until the time-based clear + the release rule (17)",
          stoppedTicks == 17);
    check("log has the avoidance, the escalation stop, the time-based clear and the release",
          b.logged("-> LOCAL_AVOIDANCE") && b.logged("LOCAL_STOP: hazard 0.50 m") && b.logged("cleared after 3.000s")
          && b.logged("local stop released"));
}

void e08_newWaypointWhileStopped() {
    header("extra: a new waypoint arriving while stopped is stored and acted on only when every stop has cleared");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.direct.set(obs3(occR(0.5), unknownR(), unknownR()));
    b.step();
    Point p = b.pos();
    b.goTo({0, 10});
    b.step(2);
    check("target replaced (0, 10), leg start = the stop position", b.core->target().y == 10.0 && distance(b.core->legStart(), p) < 1e-12);
    check("but no motion while the local stop holds", distance(p, b.pos()) < 1e-12 && b.status() == SafetyStatus::LOCAL_STOP_LATCHED);
    b.direct.set(clearObs());
    b.step(3);
    check("after the release it moves toward the NEW waypoint (heading " + num(radToDeg(b.core->lastNav().heading_rad), 1) + " deg)",
          b.pos().y > p.y && std::fabs(radToDeg(b.core->lastNav().heading_rad) - radToDeg(bearing(b.pos(), {0, 10}))) < 2.0);
}

void e09_emergencyAndStreamLoss() {
    header("extra: hardware emergency input and Java stream loss");
    Bed b;
    b.goTo({10, 0});
    b.step();
    SensorObservation e = clearObs();
    e.emergency_input = true;
    b.direct.set(e);
    b.java(JavaCommandType::CONTINUE, 1.0);
    b.step();
    check("emergency input on a clear route, with a Java CONTINUE: LOCAL_STOP (\"hardware emergency input\")",
          b.status() == SafetyStatus::LOCAL_STOP_LATCHED && b.logged("LOCAL_STOP: hardware emergency input"));
    b.step(2);
    check("held while the input stays active", b.core->safety().latched());
    b.direct.set(clearObs());
    b.step(3);
    check("released only after the input is gone and the release rule passed", !b.core->safety().latched());

    Bed s;
    s.goTo({10, 0});
    s.step();
    s.mailbox.markStreamBroken("Read() returned false");
    s.step();
    check("stream broken -> controlled local stop, logged with \"no automatic reconnect\"",
          s.status() == SafetyStatus::LOCAL_STOP_LATCHED && s.logged("no automatic reconnect"));
    s.java(JavaCommandType::RESUME, 1.0);
    s.step(10);
    check("stays stopped for good: 10 clear ticks and a RESUME later, speed 0",
          s.core->safety().latched() && s.tel().measured_speed_mps == 0.0);
}

void e10_fusionRules() {
    header("extra: fusion rules");
    ManualClock clock;
    clock.advance(10.0);
    ScriptedSensor a("a");
    ScriptedSensor bS("b");
    PerceptionConfig pc;
    LocalPerception fusion(pc, {{&a, SourceRole::REQUIRED_FOR_SAFETY}, {&bS, SourceRole::SUPPLEMENTARY}});

    a.set(obs3(unknownR(), unknownR(), unknownR()));
    bS.set(obs3(freeR(8.0, 0.5), unknownR(), unknownR()));
    LocalPerceptionState s = fusion.update(clock.now());
    check("a low-confidence (0.5 < 0.7) FREE report cannot verify free space: AHEAD UNKNOWN",
          s.sectors[0].state == SectorState::UNKNOWN);
    check("missing measurements are not a clear path: no nearest value at all", !s.nearest_m);

    bS.set(obs3(occR(3.0, 0.1), unknownR(), unknownR()));
    s = fusion.update(clock.now());
    check("a low-confidence (0.1) hazard still marks the sector OCCUPIED", s.sectors[0].state == SectorState::OCCUPIED);

    a.set(obs3(occR(2.0, 1.0, 0.2), unknownR(), unknownR()));
    bS.set(obs3(occR(1.9, 1.0, 0.0), unknownR(), unknownR()));
    s = fusion.update(clock.now());
    check("conservative range = minimum of (range - uncertainty): min(2.0-0.2, 1.9-0.0) = 1.8 (got "
          + num(s.sectors[0].hazard_m.value_or(-1)) + ")", s.sectors[0].hazard_m && std::fabs(*s.sectors[0].hazard_m - 1.8) < 1e-9);

    a.set(clearObs());
    SensorObservation gone;
    gone.validity = Validity::UNAVAILABLE;
    bS.set(gone);
    s = fusion.update(clock.now());
    check("a supplementary source UNAVAILABLE does not invalidate perception", s.required_valid);
    SensorObservation stale = clearObs();
    stale.validity = Validity::STALE;
    a.set(stale);
    s = fusion.update(clock.now());
    check("a required source STALE does", !s.required_valid);

    LocalPerception none(pc, {{&bS, SourceRole::SUPPLEMENTARY}});
    bS.set(clearObs());
    check("with no required source configured nothing is ever valid for safety", !none.update(clock.now()).required_valid);
}

void e11_adapterValidation() {
    header("extra: perception message validation, counted by reason");
    ManualClock clock;
    PythonPerceptionAdapter a(PythonAdapterConfig{}, 0.45, clock);
    PerceptionMessage m = msg("s", 1);
    m.schema_version = 2;
    check("schema_version 2 -> UNKNOWN_SCHEMA_VERSION", a.offer(m) == RejectReason::UNKNOWN_SCHEMA_VERSION);
    m = msg("s", 1);
    m.robot_id = "robot-2";
    check("robot_id robot-2 -> WRONG_ROBOT_ID", a.offer(m) == RejectReason::WRONG_ROBOT_ID);
    m = msg("s", 1);
    m.frame = "MAP";
    check("frame MAP -> UNSUPPORTED_FRAME", a.offer(m) == RejectReason::UNSUPPORTED_FRAME);
    m = msg("s", 1);
    m.obstacles.push_back({0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, std::nullopt, 0.5});
    check("range NaN -> NON_FINITE_VALUE", a.offer(m) == RejectReason::NON_FINITE_VALUE);
    m = msg("s", 1);
    m.obstacles.push_back({std::numeric_limits<double>::infinity(), 1.0, 0.0, std::nullopt, 0.5});
    check("bearing inf -> NON_FINITE_VALUE", a.offer(m) == RejectReason::NON_FINITE_VALUE);
    m = msg("s", 1);
    m.obstacles.push_back({0.0, -1.0, 0.0, std::nullopt, 0.5});
    check("range -1 -> NEGATIVE_VALUE", a.offer(m) == RejectReason::NEGATIVE_VALUE);
    m = msg("s", 1);
    m.obstacles.push_back({0.0, 1.0, 0.0, std::nullopt, 1.5});
    check("confidence 1.5 -> CONFIDENCE_OUT_OF_RANGE", a.offer(m) == RejectReason::CONFIDENCE_OUT_OF_RANGE);
    AdapterStats st = a.stats();
    check("7 rejected, counted by reason (2 NON_FINITE_VALUE), 0 accepted, slot still empty",
          st.rejectedTotal() == 7 && st.rejected[static_cast<int>(RejectReason::NON_FINITE_VALUE)] == 2
          && st.accepted == 0 && a.snapshot(clock.now()).validity == Validity::UNAVAILABLE);
    m = msg("s", 2);
    m.obstacles.push_back({0.3, 1.0, 0.0, std::nullopt, 0.5});
    m.obstacles.push_back({2.0, 1.0, 0.0, std::nullopt, 0.5});
    a.offer(m);
    SensorObservation o = a.snapshot(clock.now());
    check("an obstacle 0.3 rad off-axis at 1.0 m (lateral 0.30 <= 0.45) counts as AHEAD; one at 2.0 rad is behind",
          o.sectors[0].state == SectorState::OCCUPIED && o.sectors[1].state == SectorState::UNKNOWN);
    check("a detection list never verifies free space (AHEAD free range absent)", !o.sectors[0].free_range_m);
}

void e12_avoidanceDurationLimit() {
    header("extra: max_avoidance_duration_s ends the manoeuvre - BLOCKED until AHEAD clears or a new waypoint arrives");
    Bed b;
    b.goTo({30, 0});
    b.step();
    // A hazard that keeps its distance (it moves with the robot, or a reading that never changes) with BOTH sides
    // verified free: the lateral limit alone never ends this avoidance - the robot can weave inside the band.
    SensorObservation pursued = obs3(occR(1.5), freeR(8.0), freeR(8.0));
    b.direct.set(pursued);
    b.step();
    check("LOCAL_AVOIDANCE at the first tick with the hazard (got " + std::string(toString(b.nav())) + ")",
          b.nav() == NavState::LOCAL_AVOIDANCE);
    SteadyTime started = b.clock.now();
    double maxLateral = std::fabs(b.lateral());
    int ticks = 0;
    while (b.nav() == NavState::LOCAL_AVOIDANCE && ticks < 100) {
        b.step();
        ++ticks;
        maxLateral = std::max(maxLateral, std::fabs(b.lateral()));
    }
    double elapsed = secondsBetween(started, b.clock.now());
    check("BLOCKED at the first tick after " + num(b.cfg.nav.max_avoidance_duration_s, 1) + " s of avoidance ("
          + num(elapsed, 1) + " s), logged \"avoidance exceeded 8.0 s\"",
          b.nav() == NavState::BLOCKED && elapsed > b.cfg.nav.max_avoidance_duration_s
          && elapsed <= b.cfg.nav.max_avoidance_duration_s + b.cfg.control_period_s + 1e-9
          && b.logged("avoidance exceeded 8.0 s"));
    check("the weave stayed inside max_lateral_deviation_m (max " + num(maxLateral) + " m)",
          maxLateral <= b.cfg.nav.max_lateral_deviation_m + 1e-9);
    check("blocked: speed 0, TEMPORARY_BLOCK, emergency_stop true",
          b.tel().measured_speed_mps == 0.0 && b.status() == SafetyStatus::TEMPORARY_BLOCK && b.tel().emergency_stop);
    Point p = b.pos();
    int stillBlocked = 0;
    for (int i = 0; i < 10; ++i) {
        b.step();
        if (b.nav() == NavState::BLOCKED) ++stillBlocked;
    }
    check("not re-armed while AHEAD stays blocked: BLOCKED in " + std::to_string(stillBlocked) + " of the next 10 ticks, "
          "moved " + num(distance(p, b.pos())) + " m, reason \"" + b.core->lastNav().reason + "\"",
          stillBlocked == 10 && distance(p, b.pos()) < 1e-12
          && b.core->lastNav().reason.find("used up") != std::string::npos);
    b.direct.set(clearObs());
    b.step();
    check("AHEAD verified free again -> TRACKING_WAYPOINT and moving (got " + std::string(toString(b.nav())) + ", "
          + num(b.tel().measured_speed_mps) + " m/s)",
          b.nav() == NavState::TRACKING_WAYPOINT && b.tel().measured_speed_mps > 0.0);
    b.direct.set(pursued);
    b.step();
    check("a hazard later on the same leg may be avoided again, the direct path having been clear (got "
          + std::string(toString(b.nav())) + ")", b.nav() == NavState::LOCAL_AVOIDANCE);
    ticks = 0;
    while (b.nav() == NavState::LOCAL_AVOIDANCE && ticks < 100) {
        b.step();
        ++ticks;
    }
    bool usedUpAgain = b.nav() == NavState::BLOCKED;
    b.goTo({b.pos().x, b.pos().y + 10.0});
    b.step();
    check("used up again (" + std::string(usedUpAgain ? "BLOCKED" : "not BLOCKED") + "), then a new waypoint re-arms "
          "avoidance on the new leg (got " + std::string(toString(b.nav())) + ")",
          usedUpAgain && b.nav() == NavState::LOCAL_AVOIDANCE);
}

void e13_batchedCommandsLikeOnePerTick() {
    header("extra: commands arriving within one tick have the same net effect as one per tick");
    Bed one;
    one.goTo({10, 0});
    one.step();
    one.java(JavaCommandType::SLOW_DOWN, 0.2);
    one.step();
    check("SLOW_DOWN 0.2 caps motion at 0.2 m/s (measured " + num(one.tel().measured_speed_mps) + ")",
          std::fabs(one.tel().measured_speed_mps - 0.2) < 1e-9);
    one.java(JavaCommandType::CONTINUE, 1.0);
    one.step();
    one.java(JavaCommandType::SLOW_DOWN, 0.0);
    one.step();

    Bed batched;
    batched.goTo({10, 0});
    batched.step();
    batched.java(JavaCommandType::SLOW_DOWN, 0.2);
    batched.java(JavaCommandType::CONTINUE, 1.0);
    batched.step();
    check("SLOW_DOWN 0.2 and CONTINUE in the same tick: cap lifted, measured 1.0 ("
          + num(batched.tel().measured_speed_mps) + ")", std::fabs(batched.tel().measured_speed_mps - 1.0) < 1e-9);
    batched.java(JavaCommandType::SLOW_DOWN, 0.0);
    batched.step();
    check("then SLOW_DOWN 0.0 gets the configured 0.5 cap: one per tick " + num(one.tel().measured_speed_mps)
          + ", batched " + num(batched.tel().measured_speed_mps) + " (the lifted SLOW_DOWN's 0.2 must not survive)",
          std::fabs(one.tel().measured_speed_mps - 0.5) < 1e-9
          && std::fabs(batched.tel().measured_speed_mps - 0.5) < 1e-9);

    Bed again;
    again.goTo({10, 0});
    again.step();
    again.java(JavaCommandType::SLOW_DOWN, 0.2);
    again.step();
    again.java(JavaCommandType::CONTINUE, 1.0);
    again.java(JavaCommandType::SLOW_DOWN, 0.0);
    again.step();
    check("SLOW_DOWN 0.2, then CONTINUE + SLOW_DOWN 0.0 in one tick: 0.5 as well (measured "
          + num(again.tel().measured_speed_mps) + ")", std::fabs(again.tel().measured_speed_mps - 0.5) < 1e-9);
}

void e14_unknownAheadDuringAvoidance() {
    header("extra: AHEAD turning UNKNOWN during avoidance stops the robot - the swept path still runs through AHEAD");
    Bed b;
    b.goTo({10, 0});
    b.step();
    b.direct.set(obs3(occR(1.5), freeR(8.0), unknownR()));
    b.step(2);
    check("LOCAL_AVOIDANCE into the verified-free LEFT (got " + std::string(toString(b.nav())) + ")",
          b.nav() == NavState::LOCAL_AVOIDANCE && b.core->lastNav().motion_sector == Sector::LEFT);
    b.direct.set(obs3(unknownR(), freeR(8.0), unknownR()));
    Point p = b.pos();
    b.step();
    check("AHEAD UNKNOWN, LEFT still verified free: no motion (moved " + num(distance(p, b.pos())) + " m), "
          + toString(b.status()) + ", emergency_stop " + (b.tel().emergency_stop ? "true" : "false"),
          distance(p, b.pos()) < 1e-12 && b.status() == SafetyStatus::TEMPORARY_BLOCK && b.tel().emergency_stop);
    b.direct.set(obs3(occR(1.5), freeR(8.0), unknownR()));
    p = b.pos();
    b.step();
    check("AHEAD measured again (OCCUPIED at 1.5 m, verified free up to it): the avoidance moves on (moved "
          + num(distance(p, b.pos())) + " m)",
          distance(p, b.pos()) > 0.0 && b.nav() == NavState::LOCAL_AVOIDANCE && !b.tel().emergency_stop);
}

}  // namespace

int main() {
    s01_clearRoute();
    s02_obstacleAheadSlows();
    s03_04_criticalObstacleAndRelease();
    s05_javaStopWins();
    s06_resumeDoesNotClearLocalStop();
    s07_rerouteDuringAvoidance();
    s08_perceptionStaleOrLost();
    s09_directDangerWins();
    s10_noFreeDirection();
    s11_latestWins();
    s12_concurrency();
    s13_telemetry();
    e01_stoppingModelConservative();
    e02_nonPositiveSpeedGuard();
    e03_sessionReset();
    e04_ageFormula();
    e05_harnessToleranceAndRelationships();
    e06_simulatedSensorScript();
    e07_liveLikeLeg();
    e08_newWaypointWhileStopped();
    e09_emergencyAndStreamLoss();
    e10_fusionRules();
    e11_adapterValidation();
    e12_avoidanceDurationLimit();
    e13_batchedCommandsLikeOnePerTick();
    e14_unknownAheadDuringAvoidance();

    std::cout << std::endl << "=== FINAL SUMMARY ===" << std::endl;
    std::cout << g_pass << " passed, " << g_fail << " failed" << std::endl;
    return g_fail == 0 ? 0 : 1;
}
