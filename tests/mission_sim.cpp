// Full autoflight mission without X-Plane: takeoff to wheels stop.
#include "autopilot.hpp"
#include "flight.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

int main(int argc, char** argv) {
  const bool verbose = argc > 1 && std::strcmp(argv[1], "-v") == 0;
  autopilot::Field field{339.2, 166.0, 4400.0};
  auto w = flight::initial(field.runway_heading_deg, field.elevation_m);
  autopilot::State st;
  autopilot::start(st);
  size_t printed = 0;
  double sim = 0, wall = 0, next_report = 0;
  double max_q = 0;
  bool hit_150k_probe = false;
  while (sim < 6.0 * 3600.0 && st.step != autopilot::Step::Done) {
    const double frame = 1.0 / 30.0;
    auto out = autopilot::update(st, w, field, {}, frame);
    const double dt = frame * out.warp;
    flight::advance(w, out.controls, out.carrier, dt);
    sim += dt;
    wall += frame;
    max_q = std::max(max_q, w.ship.q_pa);
    if (w.probe.deployed && !hit_150k_probe) {
      hit_150k_probe = true;
      assert(w.probe.pos.u > flight::kProbeAltM - 200.0);
    }
    for (; printed < st.events.size(); ++printed)
      std::printf("[sim %7.1fs wall %6.1fs] %s\n", sim, wall, st.events[printed].text.c_str());
    if (verbose && sim >= next_report && w.ship.released) {
      const auto& s = w.ship;
      std::printf("   %-16s h=%7.0f m  vu=%6.0f  tas=%5.0f  M=%4.2f q=%6.0f a=%5.1f p=%5.1f r=%5.1f g=%4.1f "
                  "E=%7.0f N=%7.0f fe=%.2f\n",
                  autopilot::step_name(st.step), s.pos.u, s.vel.u, s.tas, s.mach, s.q_pa, s.alpha_deg,
                  s.pitch_deg, s.roll_deg, s.g_load, s.pos.e, s.pos.n, s.feather);
      next_report = sim + (st.step == autopilot::Step::Flare ? 0.25 : st.step == autopilot::Step::Final ? 2 : 8);
    }
    assert(std::isfinite(w.ship.pos.u));
  }
  const auto& s = w.ship;
  const double along = s.pos.e * std::sin(339.2 * M_PI / 180) + s.pos.n * std::cos(339.2 * M_PI / 180);
  const double cross = s.pos.e * std::cos(339.2 * M_PI / 180) - s.pos.n * std::sin(339.2 * M_PI / 180);
  std::printf("apogee %.0f m (%.0f ft)  max Mach %.2f  max g %.1f  max q %.0f Pa  probe apogee %.0f m\n",
              s.apogee_m, s.apogee_m / 0.3048, s.max_mach, s.max_g, max_q, w.probe.apogee_m);
  std::printf("stopped %.0f m past threshold, %.0f m off centreline; wall clock %.1f min\n", along, cross,
              wall / 60.0);
  assert(st.step == autopilot::Step::Done);
  assert(s.apogee_m > flight::kCloudAltM + 2000.0 && s.apogee_m < 95000.0);
  assert(w.probe.deployed && w.probe.apogee_m > flight::kCloudAltM);
  assert(along > -100.0 && along < 4400.0 && std::abs(cross) < 30.0);
  std::printf("PASS\n");
}
