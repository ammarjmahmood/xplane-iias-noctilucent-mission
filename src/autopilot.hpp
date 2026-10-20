#pragma once
// Mission sequencer and guidance ("autoflight") for the air-launch profile:
// takeoff, mated climb, release at 50,000 ft, boost, probe deployment at
// 150,000 ft, cloud-layer crossing at ~270,000 ft, apogee, feathered reentry,
// heading-alignment-circle approach and landing.
#include "flight.hpp"

#include <string>
#include <vector>

namespace autopilot {

struct Field {
  double runway_heading_deg = 0;  // landing direction, true
  double elevation_m = 0;
  double runway_length_m = 0;
};

enum class Step {
  Hold, TakeoffRoll, ClimbOut, ClimbCircle, RunIn, Drop, Boost, Coast,
  Reentry, Defeather, ToHac, Hac, Final, Flare, Rollout, Done
};
const char* step_name(Step s);

struct Event {
  double time_s;
  std::string text;
};

struct State {
  bool automatic = true;   // false: pilot flies SpaceShipTwo after release
  Step step = Step::Hold;
  double step_time_s = 0;
  double target_apogee_m = 88500.0;
  int hac_loops = 0;
  bool runin_captured = false;
  double aligned_s = 0;
  double last_vu = 0;
  double last_alt = 0;
  double hac_prev_angle = 0;
  double hac_swept_deg = 0;
  double warp = 1.0;
  double touchdown_sink_mps = 0;
  double touchdown_keas = 0;
  std::vector<Event> events;
};

struct Output {
  flight::Controls controls;
  flight::CarrierCommand carrier;
  double warp = 1.0;
};

// Geometry of the approach, exported for the navigation display.
struct ApproachGeometry {
  flight::Vec3 hac_center;
  double hac_radius_m;
  flight::Vec3 final_entry;   // tangent point on the extended centreline
  flight::Vec3 release_point;
  flight::Vec3 climb_center;
};
ApproachGeometry geometry(const Field& field);

void start(State& st);
Output update(State& st, flight::World& w, const Field& field, const flight::Controls& pilot,
              double dt_s);
void log_event(State& st, const flight::World& w, const std::string& text);

}  // namespace autopilot
