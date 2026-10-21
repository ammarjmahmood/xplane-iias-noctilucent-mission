// Manual-mode stick response without X-Plane. X-Plane's
// sim/joystick/yoke_*_ratio datarefs (filled by any configured joystick)
// are passed straight into flight::Controls: pitch +1 = stick fully aft
// (nose up), roll +1 = right, yaw +1 = right pedal.
#include "flight.hpp"

#include "check.hpp"
#include <cmath>
#include <cstdio>

namespace {
// flight::advance caps a single call at 2 s, so step longer waits.
void wait(flight::World& w, const flight::Controls& c, const flight::CarrierCommand& cc, double seconds) {
  for (double t = 0; t < seconds; t += 0.05) flight::advance(w, c, cc, 0.05);
}

flight::World released_and_burning() {
  auto w = flight::initial(339.2, 166.0);
  flight::CarrierCommand cc;
  cc.takeoff = true;
  cc.heading_deg = 339.2;
  for (int i = 0; i < 20000 && !w.carrier.airborne; ++i) flight::advance(w, {}, cc, 0.05);
  w.carrier.pos.u = flight::kReleaseAltM;  // skip the climb
  w.carrier.tas_mps = 200;
  flight::advance(w, {}, cc, 0.1);
  CHECK(flight::release(w));
  wait(w, {}, cc, 4.5);
  CHECK(flight::ignite(w));
  wait(w, {}, cc, 2.0);
  return w;
}

double run(flight::World w, flight::Controls c, double seconds, double flight::Ship::*field) {
  flight::CarrierCommand cc;
  wait(w, c, cc, seconds);
  return w.ship.*field;
}
}  // namespace

int main() {
  const auto w = released_and_burning();
  flight::Controls neutral, aft, fwd, right, left, rudder;
  aft.pitch = 1.0;
  fwd.pitch = -1.0;
  right.roll = 0.5;
  left.roll = -0.5;
  rudder.yaw = 1.0;

  const double p0 = run(w, neutral, 4.0, &flight::Ship::pitch_deg);
  const double p_aft = run(w, aft, 4.0, &flight::Ship::pitch_deg);
  const double p_fwd = run(w, fwd, 4.0, &flight::Ship::pitch_deg);
  const double r_right = run(w, right, 1.5, &flight::Ship::roll_deg);
  const double r_left = run(w, left, 1.5, &flight::Ship::roll_deg);
  const double h0 = run(w, neutral, 3.0, &flight::Ship::heading_deg);
  const double h_rud = run(w, rudder, 3.0, &flight::Ship::heading_deg);
  std::printf("pitch after 4 s: neutral %.1f, stick aft %.1f, stick forward %.1f deg\n", p0, p_aft, p_fwd);
  std::printf("roll after 1.5 s: right %.1f, left %.1f deg\n", r_right, r_left);
  std::printf("heading after 3 s: neutral %.1f, right rudder %.1f deg\n", h0, h_rud);
  CHECK(p_aft > p0 + 10.0);   // pulling back raises the nose
  CHECK(p_fwd < p0 - 10.0);   // pushing lowers it
  CHECK(r_right > 10.0);      // right stick rolls right
  CHECK(r_left < -10.0);
  const double dh = std::fmod(h_rud - h0 + 540.0, 360.0) - 180.0;
  CHECK(dh > 1.0);            // right pedal yaws right
  std::printf("PASS\n");
}
