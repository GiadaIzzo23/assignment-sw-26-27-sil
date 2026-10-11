#include "sil.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

namespace {

using raceup::sil::Ecu;
using raceup::sil::Frame;
using raceup::sil::OutputId;
using raceup::sil::SensorId;
using raceup::sil::SilSystem;
namespace can_id = raceup::sil::can_id;
namespace ecu_io = raceup::sil::ecu_io;

constexpr auto kWait = std::chrono::milliseconds(300);

struct TestContext {
  int failures = 0;

  void expect(const bool condition, const char* message) {
    if (condition) {
      std::printf("[PASS] %s\n", message);
    } else {
      std::printf("[FAIL] %s\n", message);
      ++failures;
    }
  }
};

template <typename Predicate>
bool wait_for_matching_frame(SilSystem& system, const std::uint32_t id,
                             Predicate predicate, Frame& result) {
  const auto deadline = std::chrono::steady_clock::now() + kWait;
  while (std::chrono::steady_clock::now() < deadline) {
    Frame frame{};
    if (system.receive_frame(frame, std::chrono::milliseconds(20)) &&
        frame.id == id && predicate(frame)) {
      result = frame;
      return true;
    }
  }
  return false;
}

bool wait_for_frame(SilSystem& system, const std::uint32_t id, Frame& result) {
  return wait_for_matching_frame(
      system, id, [](const Frame&) { return true; }, result);
}

Frame vehicle_status(const bool manual, const bool ready_to_drive,
                     const std::uint8_t speed) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::VehicleStatus, 3U);
  frame.data[0] = static_cast<std::uint8_t>(manual);
  frame.data[1] = static_cast<std::uint8_t>(ready_to_drive);
  frame.data[2] = speed;
  return frame;
}

Frame brake_pressure(const std::uint8_t front, const std::uint8_t rear,
                     const std::uint8_t pneumatic_left,
                     const std::uint8_t pneumatic_right) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::BrakePressure, 4U);
  frame.data[0] = front;
  frame.data[1] = rear;
  frame.data[2] = pneumatic_left;
  frame.data[3] = pneumatic_right;
  return frame;
}

Frame ebs_command(const bool brake_request, const bool mission_finished) {
  Frame frame = raceup::sil::make_frame(raceup::sil::can_id::EbsCommand, 2U);
  frame.data[0] = static_cast<std::uint8_t>(brake_request);
  frame.data[1] = static_cast<std::uint8_t>(mission_finished);
  return frame;
}

void test_lifecycle(TestContext& context) {
  SilSystem system;
  context.expect(system.start(), "SIL starts");
  context.expect(system.running(), "SIL reports running");
  system.stop();
  context.expect(!system.running(), "SIL stops and joins its threads");
}

void configure_nominal_inputs(SilSystem& system) {
  system.set_sensor(SensorId::SmuApps1Raw, 3500U);
  system.set_sensor(SensorId::SmuApps2Raw, 3000U);
  system.set_sensor(SensorId::SmuBrakeRaw, 2500U);
  system.set_sensor(SensorId::EbsValvesPowered, 1U);
  system.set_sensor(SensorId::EbsAsmsActive, 1U);
  system.set_sensor(SensorId::EbsWatchdogHealthy, 1U);
}

void test_smu_nominal(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  Frame frame{};
  const bool received = wait_for_frame(
      system, raceup::sil::can_id::SmuSense, frame);
  context.expect(received, "SMU publishes a nominal sense frame");
  if (received) {
    context.expect(frame.dlc == 3U, "SMU sense frame has the expected DLC");
    context.expect(frame.data[0] > 40U && frame.data[0] < 60U,
                   "SMU computes the expected throttle percentage");
    context.expect(frame.data[2] == 0U, "SMU reports no pedal fault");
  }

  system.stop();
}

void test_smu_fault(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  Frame nominal{};
  const bool nominal_received = wait_for_frame(
      system, raceup::sil::can_id::SmuSense, nominal);
  context.expect(nominal_received,
                 "SMU publishes a frame before a live sensor change");

  system.set_sensor(SensorId::SmuApps2Raw, 3490U);

  Frame frame{};
  const bool received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.data[0] == 0U && candidate.data[2] != 0U;
      },
      frame);
  context.expect(received, "SMU publishes a fault frame after a live change");
  if (received) {
    context.expect(frame.data[0] == 0U,
                   "SMU inhibits throttle on APP disagreement");
    context.expect(frame.data[2] != 0U, "SMU reports the APP fault");
  }

  system.set_sensor(SensorId::SmuApps2Raw, 3000U);
  Frame recovered{};
  const bool recovered_received = wait_for_matching_frame(
      system, raceup::sil::can_id::SmuSense,
      [](const Frame& candidate) {
        return candidate.data[0] != 0U && candidate.data[2] == 0U;
      },
      recovered);
  context.expect(recovered_received,
                 "SMU publishes a frame after recovering the live input");
  if (recovered_received) {
    context.expect(recovered.data[0] != 0U && recovered.data[2] == 0U,
                   "SMU returns to the nominal state after recovery");
  }

  system.stop();
}

void test_ebs_nominal_and_stale_input(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  system.send_frame(vehicle_status(false, true, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS closes ASNode with valid fresh inputs");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 0U &&
                     system.output(OutputId::EbsValve2Engaged) == 0U,
                 "EBS releases both valves in the nominal state");

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode after critical inputs become stale");
  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports a stale-input fault");

  system.stop();
}

void test_ebs_requires_ready_to_drive(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  system.send_frame(vehicle_status(true, false, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS stays safe when ready-to-drive is inactive");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves without ready-to-drive");
  context.expect(system.output(OutputId::EbsFaultCode) != 0U,
                 "EBS reports a readiness fault");

  system.send_frame(vehicle_status(true, true, 0U));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS closes ASNode after ready-to-drive becomes active");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 0U &&
                     system.output(OutputId::EbsValve2Engaged) == 0U,
                 "EBS releases both valves after readiness is restored");

  system.stop();
}

void test_ebs_brake_request(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  system.send_frame(vehicle_status(false, true, 0U));
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 1U,
                 "EBS is nominal before a live brake request");

  system.send_frame(ebs_command(true, false));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  context.expect(system.output(OutputId::EbsAsNodeClosed) == 0U,
                 "EBS opens ASNode after a live brake request");
  context.expect(system.output(OutputId::EbsValve1Engaged) == 1U &&
                     system.output(OutputId::EbsValve2Engaged) == 1U,
                 "EBS engages both valves after a live brake request");

  system.stop();
}

// ===========================================================================
// Additional tests (candidate)
//
// Design notes:
//  * Everything is black-box: only SilSystem / ecu_io / the bus are used.
//  * No test sleeps for a fixed time and then hopes. Each one waits for a
//    condition under a deadline (drive_until / wait_for_matching_frame).
//  * Expected values sit exactly on the thresholds (49/50 counts, 10/11
//    points, 7/10 bar, 15/99 bar, 5/6 dm/s, 250 ms) so that a flipped
//    comparison in the ECU logic is detected.
//  * EBS "safe" is checked as a full output set (ASNode open AND both valves
//    engaged AND the expected fault code), never on a single output, so that
//    the not-yet-run initial outputs (all zero) can never look like a pass.
// ===========================================================================

constexpr auto kSettle = std::chrono::milliseconds(600);
constexpr auto kTick = std::chrono::milliseconds(10);

// Frames the (simulated) rest of the vehicle sends to the EBS.
struct EbsInputs {
  bool manual = false;
  bool ready_to_drive = true;
  std::uint8_t speed = 0U;
  std::uint8_t front = 40U;
  std::uint8_t rear = 40U;
  std::uint8_t left = 8U;
  std::uint8_t right = 8U;
  bool brake_request = false;
  bool mission_finished = false;
  // Clearing one of these simulates a dead sender.
  bool send_vehicle = true;
  bool send_pressure = true;
  bool send_command = true;

  template <typename Send>
  void publish_to(Send send) const {
    if (send_vehicle) {
      send(vehicle_status(manual, ready_to_drive, speed));
    }
    if (send_pressure) {
      send(brake_pressure(front, rear, left, right));
    }
    if (send_command) {
      send(ebs_command(brake_request, mission_finished));
    }
  }

  void publish(SilSystem& system) const {
    publish_to([&system](const Frame& frame) { system.send_frame(frame); });
  }
};

// Re-publishes the inputs every tick (so they never go stale) until done()
// holds or the timeout expires.
template <typename Done>
bool drive_until(SilSystem& system, const EbsInputs& inputs, Done done,
                 const std::chrono::milliseconds timeout = kSettle) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    inputs.publish(system);
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(kTick);
  }
  return false;
}

void drive_for(SilSystem& system, const EbsInputs& inputs,
               const std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    inputs.publish(system);
    std::this_thread::sleep_for(kTick);
  }
}

// Like drive_until, but waits for a matching frame on the monitor queue.
template <typename Predicate>
bool drive_until_frame(SilSystem& system, const EbsInputs& inputs,
                       const std::uint32_t id, Predicate predicate,
                       Frame& result,
                       const std::chrono::milliseconds timeout = kSettle) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto next_publish = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::chrono::steady_clock::now() >= next_publish) {
      inputs.publish(system);
      next_publish += kTick;
    }
    Frame frame{};
    if (system.receive_frame(frame, kTick) && frame.id == id &&
        predicate(frame)) {
      result = frame;
      return true;
    }
  }
  return false;
}

bool ebs_nominal(const SilSystem& system) {
  return system.output(OutputId::EbsAsNodeClosed) == 1U &&
         system.output(OutputId::EbsValve1Engaged) == 0U &&
         system.output(OutputId::EbsValve2Engaged) == 0U &&
         system.output(OutputId::EbsFaultCode) == 0U;
}

bool ebs_safe(const SilSystem& system, const std::uint16_t fault) {
  return system.output(OutputId::EbsAsNodeClosed) == 0U &&
         system.output(OutputId::EbsValve1Engaged) == 1U &&
         system.output(OutputId::EbsValve2Engaged) == 1U &&
         system.output(OutputId::EbsFaultCode) == fault;
}

void print_ebs_outputs(const SilSystem& system) {
  std::printf("       outputs: asnode=%u valve1=%u valve2=%u fault=%u\n",
              static_cast<unsigned>(system.output(OutputId::EbsAsNodeClosed)),
              static_cast<unsigned>(system.output(OutputId::EbsValve1Engaged)),
              static_cast<unsigned>(system.output(OutputId::EbsValve2Engaged)),
              static_cast<unsigned>(system.output(OutputId::EbsFaultCode)));
}

// ---------------------------------------------------------------- SMU -----

struct SmuCase {
  const char* name;
  std::uint16_t apps1;
  std::uint16_t apps2;
  std::uint16_t brake_raw;
  std::uint8_t throttle;
  std::uint8_t brake;
  std::uint8_t fault;
};

// Fault codes: 0 none, 1 APP tracks shorted (<50 counts), 2 implausible
// (>10 points). For two APP readings 50 counts apart the percentages differ
// by about 45-55 points, so exactly 50 counts must give fault 2, not 1.
const SmuCase kSmuCases[] = {
    {"SMU: 50% pedals, 50% throttle, no fault", 3500, 3000, 4000, 50, 0, 0},
    {"SMU: released pedals give 0%", 4000, 3500, 4000, 0, 0, 0},
    {"SMU: raw values beyond zero are clamped to 0%", 4095, 3600, 4095, 0, 0,
     0},
    {"SMU: fully pressed pedals give 100%", 3000, 2500, 1000, 100, 100, 0},
    {"SMU: raw values beyond full are clamped to 100%", 2900, 2400, 500, 100,
     100, 0},
    {"SMU: 25% throttle and 75% brake", 3750, 3250, 1750, 25, 75, 0},
    {"SMU: brake percentage is truncated (33%)", 3500, 3000, 3000, 50, 33, 0},
    {"SMU: APP raw difference 49 is a track short", 3500, 3451, 4000, 0, 0,
     1},
    {"SMU: APP raw difference 50 is not a track short (but implausible)", 3500,
     3450, 4000, 0, 0, 2},
    {"SMU: identical APP readings are a track short", 3500, 3500, 4000, 0, 0,
     1},
    {"SMU: 10 point APP difference is plausible, APP1 drives throttle", 3500,
     3100, 4000, 50, 0, 0},
    {"SMU: 11 point APP difference (APP2 low) is implausible", 3500, 3110,
     4000, 0, 0, 2},
    {"SMU: 10 point APP difference (APP2 high) is plausible", 3500, 2900, 4000,
     50, 0, 0},
    {"SMU: 11 point APP difference (APP2 high) is implausible", 3500, 2890,
     4000, 0, 0, 2},
    {"SMU: brake is still published while an APP fault inhibits throttle",
     3500, 3490, 1000, 0, 100, 1},
};

void test_smu_cases(TestContext& context) {
  for (const SmuCase& test_case : kSmuCases) {
    SilSystem system;
    system.set_sensor(SensorId::SmuApps1Raw, test_case.apps1);
    system.set_sensor(SensorId::SmuApps2Raw, test_case.apps2);
    system.set_sensor(SensorId::SmuBrakeRaw, test_case.brake_raw);
    system.start();

    Frame frame{};
    const bool received = wait_for_frame(system, can_id::SmuSense, frame);
    bool ok = received && frame.dlc == 3U &&
              frame.data[0] == test_case.throttle &&
              frame.data[1] == test_case.brake &&
              frame.data[2] == test_case.fault &&
              system.output(OutputId::SmuThrottlePercent) ==
                  test_case.throttle &&
              system.output(OutputId::SmuBrakePercent) == test_case.brake;
    if (!ok && received) {
      std::printf("       got throttle=%u brake=%u fault=%u dlc=%u\n",
                  static_cast<unsigned>(frame.data[0]),
                  static_cast<unsigned>(frame.data[1]),
                  static_cast<unsigned>(frame.data[2]),
                  static_cast<unsigned>(frame.dlc));
    }
    context.expect(ok, test_case.name);
    system.stop();
  }
}

void test_smu_follows_live_changes(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  Frame frame{};
  context.expect(
      wait_for_matching_frame(system, can_id::SmuSense,
                              [](const Frame& f) {
                                return f.data[0] == 50U && f.data[2] == 0U;
                              },
                              frame),
      "SMU live: starts at 50% throttle");

  system.set_sensor(SensorId::SmuApps1Raw, 3250U);
  system.set_sensor(SensorId::SmuApps2Raw, 2750U);
  context.expect(
      wait_for_matching_frame(system, can_id::SmuSense,
                              [](const Frame& f) {
                                return f.data[0] == 75U && f.data[2] == 0U;
                              },
                              frame),
      "SMU live: throttle follows a live pedal change to 75%");

  system.set_sensor(SensorId::SmuApps2Raw, 2890U);  // 61% vs 75%: 14 points
  context.expect(
      wait_for_matching_frame(system, can_id::SmuSense,
                              [](const Frame& f) {
                                return f.data[0] == 0U && f.data[2] == 2U;
                              },
                              frame),
      "SMU live: plausibility fault inhibits throttle after a live change");

  system.set_sensor(SensorId::SmuApps2Raw, 2750U);
  context.expect(
      wait_for_matching_frame(system, can_id::SmuSense,
                              [](const Frame& f) {
                                return f.data[0] == 75U && f.data[2] == 0U;
                              },
                              frame),
      "SMU live: throttle is restored after the plausibility fault clears");

  system.set_sensor(SensorId::SmuBrakeRaw, 1000U);
  context.expect(
      wait_for_matching_frame(system, can_id::SmuSense,
                              [](const Frame& f) { return f.data[1] == 100U; },
                              frame),
      "SMU live: brake percentage follows a live brake change");
  system.stop();
}

// ---------------------------------------------------------------- EBS -----

struct EbsCase {
  const char* name = "";
  EbsInputs inputs{};
  std::uint16_t asms = 1U;
  std::uint16_t watchdog = 1U;
  std::uint16_t valves = 1U;
  bool expect_nominal = true;
  std::uint16_t expect_fault = 0U;  // used when !expect_nominal
};

// Fault codes: 1 communication/readiness, 2 ASMS, 3 pneumatic, 4 hydraulic or
// coherency, 5 watchdog/valve power, 6 brake request or finished mission.
std::vector<EbsCase> ebs_cases() {
  std::vector<EbsCase> cases;
  auto add = [&cases](const char* name, const bool nominal,
                      const std::uint16_t fault, auto tweak) {
    EbsCase test_case;
    test_case.name = name;
    test_case.expect_nominal = nominal;
    test_case.expect_fault = fault;
    tweak(test_case);
    cases.push_back(test_case);
  };

  // --- Nominal states, including the values just inside every limit. ---
  add("EBS nominal: valid fresh inputs", true, 0U, [](EbsCase&) {});
  add("EBS nominal: manual mission with ready-to-drive", true, 0U,
      [](EbsCase& c) { c.inputs.manual = true; });
  add("EBS nominal: speed 5 dm/s is still stationary", true, 0U,
      [](EbsCase& c) { c.inputs.speed = 5U; });
  add("EBS nominal: pneumatic 9 bar on both sides", true, 0U, [](EbsCase& c) {
    c.inputs.left = 9U;
    c.inputs.right = 9U;
  });
  add("EBS nominal: hydraulic 98 bar is inside the range", true, 0U,
      [](EbsCase& c) {
        c.inputs.front = 98U;
        c.inputs.rear = 98U;
      });
  add("EBS nominal: hydraulic 17 bar is coherent with 9 bar pneumatic", true,
      0U, [](EbsCase& c) {
        c.inputs.front = 17U;
        c.inputs.rear = 17U;
        c.inputs.left = 9U;
        c.inputs.right = 9U;
      });
  add("EBS nominal: front is paired with the left pneumatic only", true, 0U,
      [](EbsCase& c) {  // 16 vs 8 coherent; 16 vs 9 would not be
        c.inputs.front = 16U;
        c.inputs.left = 8U;
        c.inputs.right = 9U;
      });
  add("EBS nominal: rear is paired with the right pneumatic only", true, 0U,
      [](EbsCase& c) {
        c.inputs.rear = 16U;
        c.inputs.right = 8U;
        c.inputs.left = 9U;
      });

  // --- Unsafe: every boundary on the excluded side. ---
  add("EBS safe: ASMS inactive (fault 2)", false, 2U,
      [](EbsCase& c) { c.asms = 0U; });
  add("EBS safe: unhealthy watchdog (fault 5)", false, 5U,
      [](EbsCase& c) { c.watchdog = 0U; });
  add("EBS safe: valves not powered (fault 5)", false, 5U,
      [](EbsCase& c) { c.valves = 0U; });
  add("EBS safe: left pneumatic exactly 7 bar (fault 3)", false, 3U,
      [](EbsCase& c) { c.inputs.left = 7U; });
  add("EBS safe: left pneumatic exactly 10 bar (fault 3)", false, 3U,
      [](EbsCase& c) { c.inputs.left = 10U; });
  add("EBS safe: right pneumatic exactly 7 bar (fault 3)", false, 3U,
      [](EbsCase& c) { c.inputs.right = 7U; });
  add("EBS safe: right pneumatic exactly 10 bar (fault 3)", false, 3U,
      [](EbsCase& c) { c.inputs.right = 10U; });
  add("EBS safe: front hydraulic exactly 15 bar (fault 4)", false, 4U,
      [](EbsCase& c) { c.inputs.front = 15U; });
  add("EBS safe: front hydraulic exactly 99 bar (fault 4)", false, 4U,
      [](EbsCase& c) { c.inputs.front = 99U; });
  add("EBS safe: rear hydraulic exactly 15 bar (fault 4)", false, 4U,
      [](EbsCase& c) { c.inputs.rear = 15U; });
  add("EBS safe: rear hydraulic exactly 99 bar (fault 4)", false, 4U,
      [](EbsCase& c) { c.inputs.rear = 99U; });
  add("EBS safe: front 16 bar is incoherent with left 9 bar (fault 4)", false,
      4U, [](EbsCase& c) {
        c.inputs.front = 16U;
        c.inputs.left = 9U;
      });
  add("EBS safe: rear 16 bar is incoherent with right 9 bar (fault 4)", false,
      4U, [](EbsCase& c) {
        c.inputs.rear = 16U;
        c.inputs.right = 9U;
      });
  add("EBS safe: brake request (fault 6)", false, 6U,
      [](EbsCase& c) { c.inputs.brake_request = true; });
  add("EBS safe: mission finished (fault 6)", false, 6U,
      [](EbsCase& c) { c.inputs.mission_finished = true; });
  add("EBS safe: not ready to drive (fault 1)", false, 1U,
      [](EbsCase& c) { c.inputs.ready_to_drive = false; });
  add("EBS safe: manual mission does not bypass ready-to-drive", false, 1U,
      [](EbsCase& c) {
        c.inputs.manual = true;
        c.inputs.ready_to_drive = false;
      });
  add("EBS safe: speed 6 dm/s keeps the safe outputs (no fault code)", false,
      0U, [](EbsCase& c) { c.inputs.speed = 6U; });
  add("EBS safe: speed 255 dm/s keeps the safe outputs (no fault code)", false,
      0U, [](EbsCase& c) { c.inputs.speed = 255U; });
  return cases;
}

void test_ebs_cases(TestContext& context) {
  for (const EbsCase& test_case : ebs_cases()) {
    SilSystem system;
    configure_nominal_inputs(system);
    system.set_sensor(SensorId::EbsAsmsActive, test_case.asms);
    system.set_sensor(SensorId::EbsWatchdogHealthy, test_case.watchdog);
    system.set_sensor(SensorId::EbsValvesPowered, test_case.valves);
    system.start();

    const bool ok = drive_until(system, test_case.inputs, [&] {
      return test_case.expect_nominal
                 ? ebs_nominal(system)
                 : ebs_safe(system, test_case.expect_fault);
    });
    if (!ok) {
      print_ebs_outputs(system);
    }
    context.expect(ok, test_case.name);
    system.stop();
  }
}

void test_ebs_stays_nominal_with_refreshed_inputs(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  EbsInputs inputs;
  const bool reached =
      drive_until(system, inputs, [&] { return ebs_nominal(system); });
  context.expect(reached, "EBS refresh: reaches the nominal state");

  // Run for well over the 250 ms staleness limit while refreshing.
  bool always_nominal = true;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
  while (std::chrono::steady_clock::now() < deadline) {
    inputs.publish(system);
    always_nominal = always_nominal && ebs_nominal(system);
    std::this_thread::sleep_for(kTick);
  }
  context.expect(always_nominal,
                 "EBS refresh: periodically refreshed inputs never go stale");
  system.stop();
}

void test_ebs_each_input_goes_stale(TestContext& context) {
  struct StaleCase {
    const char* name;
    bool EbsInputs::*sender;
  };
  const StaleCase stale_cases[] = {
      {"EBS stale: VEHICLE_STATUS stops arriving", &EbsInputs::send_vehicle},
      {"EBS stale: BRAKE_PRESSURE stops arriving", &EbsInputs::send_pressure},
      {"EBS stale: EBS_COMMAND stops arriving", &EbsInputs::send_command},
  };

  for (const StaleCase& stale_case : stale_cases) {
    SilSystem system;
    configure_nominal_inputs(system);
    system.start();

    EbsInputs inputs;
    bool ok = drive_until(system, inputs, [&] { return ebs_nominal(system); });

    // One sender dies; the other two keep refreshing.
    inputs.*(stale_case.sender) = false;
    drive_for(system, inputs, std::chrono::milliseconds(120));
    const bool still_fresh = ebs_nominal(system);  // limit is 250 ms

    const bool went_safe = drive_until(
        system, inputs, [&] { return ebs_safe(system, 1U); },
        std::chrono::milliseconds(500));

    // The sender comes back: the EBS recovers.
    inputs.*(stale_case.sender) = true;
    const bool recovered =
        drive_until(system, inputs, [&] { return ebs_nominal(system); });

    ok = ok && still_fresh && went_safe && recovered;
    if (!ok) {
      std::printf("       fresh_at_120ms=%d safe_after_stale=%d recovered=%d\n",
                  still_fresh, went_safe, recovered);
    }
    context.expect(ok, stale_case.name);
    system.stop();
  }
}

void test_ebs_missing_inputs_from_start(TestContext& context) {
  struct MissingCase {
    const char* name;
    bool vehicle;
    bool pressure;
    bool command;
  };
  const MissingCase missing_cases[] = {
      {"EBS missing: no input frame ever arrives", false, false, false},
      {"EBS missing: VEHICLE_STATUS never arrives", false, true, true},
      {"EBS missing: BRAKE_PRESSURE never arrives", true, false, true},
      {"EBS missing: EBS_COMMAND never arrives", true, true, false},
  };

  for (const MissingCase& missing_case : missing_cases) {
    SilSystem system;
    configure_nominal_inputs(system);  // sensors fine: only the bus is lacking
    system.start();

    EbsInputs inputs;
    inputs.send_vehicle = missing_case.vehicle;
    inputs.send_pressure = missing_case.pressure;
    inputs.send_command = missing_case.command;

    bool ok = drive_until(system, inputs, [&] { return ebs_safe(system, 1U); });

    // It must not drift into the nominal state afterwards.
    bool stays_safe = true;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    while (std::chrono::steady_clock::now() < deadline) {
      inputs.publish(system);
      stays_safe = stays_safe && ebs_safe(system, 1U);
      std::this_thread::sleep_for(kTick);
    }
    ok = ok && stays_safe;
    if (!ok) {
      print_ebs_outputs(system);
    }
    context.expect(ok, missing_case.name);
    system.stop();
  }
}

void test_ebs_live_sensor_and_frame_changes(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  EbsInputs inputs;
  context.expect(drive_until(system, inputs, [&] { return ebs_nominal(system); }),
                 "EBS live: starts nominal");

  system.set_sensor(SensorId::EbsAsmsActive, 0U);
  context.expect(
      drive_until(system, inputs, [&] { return ebs_safe(system, 2U); }),
      "EBS live: dropping ASMS makes the EBS safe");
  system.set_sensor(SensorId::EbsAsmsActive, 1U);
  context.expect(drive_until(system, inputs, [&] { return ebs_nominal(system); }),
                 "EBS live: restoring ASMS returns to nominal");

  system.set_sensor(SensorId::EbsWatchdogHealthy, 0U);
  context.expect(
      drive_until(system, inputs, [&] { return ebs_safe(system, 5U); }),
      "EBS live: watchdog loss makes the EBS safe");
  system.set_sensor(SensorId::EbsWatchdogHealthy, 1U);
  context.expect(drive_until(system, inputs, [&] { return ebs_nominal(system); }),
                 "EBS live: watchdog recovery returns to nominal");

  system.set_sensor(SensorId::EbsValvesPowered, 0U);
  context.expect(
      drive_until(system, inputs, [&] { return ebs_safe(system, 5U); }),
      "EBS live: losing valve power makes the EBS safe");
  system.set_sensor(SensorId::EbsValvesPowered, 1U);
  context.expect(drive_until(system, inputs, [&] { return ebs_nominal(system); }),
                 "EBS live: valve power recovery returns to nominal");

  inputs.left = 7U;
  context.expect(
      drive_until(system, inputs, [&] { return ebs_safe(system, 3U); }),
      "EBS live: an out-of-range pressure frame makes the EBS safe");
  inputs.left = 8U;
  context.expect(drive_until(system, inputs, [&] { return ebs_nominal(system); }),
                 "EBS live: a valid pressure frame returns to nominal");

  inputs.mission_finished = true;
  context.expect(
      drive_until(system, inputs, [&] { return ebs_safe(system, 6U); }),
      "EBS live: mission finished makes the EBS safe");
  inputs.mission_finished = false;
  context.expect(drive_until(system, inputs, [&] { return ebs_nominal(system); }),
                 "EBS live: clearing mission finished returns to nominal");

  inputs.speed = 6U;
  context.expect(
      drive_until(system, inputs, [&] { return ebs_safe(system, 0U); }),
      "EBS live: moving above 5 dm/s makes the EBS safe");
  inputs.speed = 0U;
  context.expect(drive_until(system, inputs, [&] { return ebs_nominal(system); }),
                 "EBS live: stopping returns to nominal");
  system.stop();
}

void test_ebs_status_frame(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  EbsInputs inputs;
  inputs.speed = 3U;
  Frame frame{};
  const bool nominal = drive_until_frame(
      system, inputs, can_id::EbsStatus,
      [](const Frame& f) { return f.data[0] == 1U && f.data[3] == 0U; },
      frame);
  context.expect(nominal, "EBS_STATUS: a nominal frame is emitted");
  if (nominal) {
    context.expect(frame.dlc == 5U, "EBS_STATUS: DLC is 5");
    context.expect(frame.data[0] == 1U && frame.data[1] == 0U &&
                       frame.data[2] == 0U && frame.data[3] == 0U,
                   "EBS_STATUS: ASNode closed, both valves released, no fault");
    context.expect(frame.data[4] == 3U, "EBS_STATUS: reports the vehicle speed");
  }

  inputs.brake_request = true;
  const bool safe = drive_until_frame(
      system, inputs, can_id::EbsStatus,
      [](const Frame& f) { return f.data[0] == 0U && f.data[3] == 6U; },
      frame);
  context.expect(safe, "EBS_STATUS: a frame is emitted after a live brake request");
  if (safe) {
    context.expect(frame.data[0] == 0U && frame.data[1] == 1U &&
                       frame.data[2] == 1U && frame.data[3] == 6U,
                   "EBS_STATUS: ASNode open, both valves engaged, fault 6");
  }
  system.stop();
}

// -------------------------------------------------- bus / integration -----

void test_both_ecus_publish_periodically(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  int smu_frames = 0;
  int ebs_frames = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline) {
    Frame frame{};
    if (system.receive_frame(frame, std::chrono::milliseconds(20))) {
      smu_frames += frame.id == can_id::SmuSense ? 1 : 0;
      ebs_frames += frame.id == can_id::EbsStatus ? 1 : 0;
    }
  }
  // Both ECUs run every 10 ms: about 50 frames each in 500 ms.
  context.expect(smu_frames >= 15,
                 "bus: SMU_SENSE frames keep being emitted periodically");
  context.expect(ebs_frames >= 15,
                 "bus: EBS_STATUS frames keep being emitted periodically");
  system.stop();
}

void test_receive_frame_times_out(TestContext& context) {
  SilSystem system;  // never started: nothing is ever emitted
  Frame frame{};
  const auto begin = std::chrono::steady_clock::now();
  const bool received = system.receive_frame(frame, std::chrono::milliseconds(50));
  const auto elapsed = std::chrono::steady_clock::now() - begin;
  context.expect(!received, "bus: receive_frame returns false on timeout");
  context.expect(elapsed >= std::chrono::milliseconds(40) &&
                     elapsed < std::chrono::milliseconds(500),
                 "bus: receive_frame waits for the timeout without hanging");
}

void test_smu_written_frames_reach_ebs(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  // Frames written by the SMU must be delivered to the EBS input queue and
  // be observable on the monitor queue.
  const EbsInputs inputs;
  const auto deadline =
      std::chrono::steady_clock::now() + kSettle;
  bool nominal = false;
  while (!nominal && std::chrono::steady_clock::now() < deadline) {
    inputs.publish_to(
        [](const Frame& frame) { ecu_io::write(Ecu::Smu, frame); });
    nominal = ebs_nominal(system);
    std::this_thread::sleep_for(kTick);
  }
  context.expect(nominal, "bus: frames written by the SMU are delivered to the EBS");

  Frame frame{};
  context.expect(
      wait_for_matching_frame(system, can_id::VehicleStatus,
                              [](const Frame&) { return true; }, frame),
      "bus: frames written by the SMU are observable on the monitor queue");
  system.stop();
}

void test_ebs_written_frames_are_not_routed_to_ebs(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);
  system.start();

  EbsInputs inputs;
  context.expect(drive_until(system, inputs, [&] { return ebs_nominal(system); }),
                 "bus: EBS reaches the nominal state before the loopback check");

  // A brake request written *by the EBS* must reach the monitor only; if it
  // were looped back to the EBS input the EBS would go safe.
  bool stayed_nominal = true;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  while (std::chrono::steady_clock::now() < deadline) {
    inputs.publish(system);
    ecu_io::write(Ecu::Ebs, ebs_command(true, false));
    stayed_nominal = stayed_nominal && ebs_nominal(system);
    std::this_thread::sleep_for(kTick);
  }
  context.expect(stayed_nominal,
                 "bus: frames written by the EBS are not delivered to the EBS");

  Frame frame{};
  context.expect(
      wait_for_matching_frame(
          system, can_id::EbsCommand,
          [](const Frame& f) { return f.data[0] == 1U; }, frame),
      "bus: frames written by the EBS are observable on the monitor queue");
  system.stop();
}

void test_ebs_input_queue_discards_oldest(TestContext& context) {
  SilSystem system;
  configure_nominal_inputs(system);

  // 100 VEHICLE_STATUS frames before start: speed counts 1..100, far more
  // than the queue capacity. The newest frame must survive.
  for (std::uint8_t speed = 1U; speed <= 100U; ++speed) {
    system.send_frame(vehicle_status(false, true, speed));
  }
  system.send_frame(brake_pressure(40U, 40U, 8U, 8U));
  system.send_frame(ebs_command(false, false));
  system.start();

  Frame frame{};
  context.expect(
      wait_for_matching_frame(
          system, can_id::EbsStatus,
          [](const Frame& f) { return f.data[4] == 100U; }, frame),
      "bus: a full EBS input queue discards the oldest frames, keeps the newest");
  system.stop();
}

void test_lifecycle_edge_cases(TestContext& context) {
  {
    SilSystem never_started;
    never_started.stop();  // must be harmless
    context.expect(!never_started.running(),
                   "lifecycle: stop() on a system that never started is harmless");
  }
  {
    SilSystem system;
    configure_nominal_inputs(system);
    context.expect(system.start() && system.start(),
                   "lifecycle: calling start() twice is harmless");
    system.stop();
    system.stop();
    context.expect(!system.running(), "lifecycle: calling stop() twice is harmless");

    Frame leftover{};
    while (system.receive_frame(leftover, std::chrono::milliseconds(0))) {
    }
    context.expect(!ecu_io::try_read(Ecu::Ebs, leftover) &&
                       ecu_io::read_sensor(SensorId::SmuApps1Raw) == 0U,
                   "lifecycle: ECU callbacks are inert while no SIL is active");

    context.expect(system.start(), "lifecycle: the SIL can be restarted");
    Frame frame{};
    context.expect(wait_for_frame(system, can_id::SmuSense, frame),
                   "lifecycle: ECUs run again after a restart");
    system.stop();
  }
  {
    // Destroying a running SIL must stop and join its threads.
    {
      SilSystem system;
      configure_nominal_inputs(system);
      system.start();
    }
    SilSystem next;
    configure_nominal_inputs(next);
    next.start();
    Frame frame{};
    context.expect(wait_for_frame(next, can_id::SmuSense, frame),
                   "lifecycle: a new SIL works after destroying a running one");
    next.stop();
  }
  {
    SilSystem system;
    system.set_sensor(SensorId::Count, 1234U);  // invalid id must be ignored
    context.expect(system.output(OutputId::Count) == 0U,
                   "lifecycle: invalid sensor/output ids are ignored safely");
  }
}

}  // namespace

int main() {
  TestContext context;
  test_lifecycle(context);
  test_smu_nominal(context);
  test_smu_fault(context);
  test_ebs_nominal_and_stale_input(context);
  test_ebs_requires_ready_to_drive(context);
  test_ebs_brake_request(context);

  test_smu_cases(context);
  test_smu_follows_live_changes(context);
  test_ebs_cases(context);
  test_ebs_stays_nominal_with_refreshed_inputs(context);
  test_ebs_each_input_goes_stale(context);
  test_ebs_missing_inputs_from_start(context);
  test_ebs_live_sensor_and_frame_changes(context);
  test_ebs_status_frame(context);
  test_both_ecus_publish_periodically(context);
  test_receive_frame_times_out(context);
  test_smu_written_frames_reach_ebs(context);
  test_ebs_written_frames_are_not_routed_to_ebs(context);
  test_ebs_input_queue_discards_oldest(context);
  test_lifecycle_edge_cases(context);

  if (context.failures != 0) {
    std::printf("%d test assertion(s) failed\n", context.failures);
    return 1;
  }

  std::printf("all SIL tests passed\n");
  return 0;
}
