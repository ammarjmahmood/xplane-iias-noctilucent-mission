#include "autopilot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace autopilot {
namespace {
using flight::Vec3;
constexpr double kPi = 3.14159265358979323846;
constexpr double kG0 = 9.80665;
constexpr double kHacRadius = 4500.0;
constexpr double kFinalEntry = 8000.0;   // m before threshold
constexpr double kAimPoint = 1000.0;     // steep-glide aim point before threshold
constexpr double kSteepDeg = 12.0;
constexpr double kShallowDeg = 2.0;
constexpr double kHacGlideDeg = 9.0;
double rad(double d) { return d * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }
double clamp(double v, double lo, double hi) { return std::clamp(v, lo, hi); }
double wrap180(double a) { return std::fmod(a + 540.0, 360.0) - 180.0; }
Vec3 dir(double heading) { return {std::sin(rad(heading)), std::cos(rad(heading)), 0}; }
Vec3 left_of(double heading) { return dir(heading - 90.0); }
Vec3 add(Vec3 a, Vec3 b) { return {a.e + b.e, a.n + b.n, a.u + b.u}; }
Vec3 mul(Vec3 a, double s) { return {a.e * s, a.n * s, a.u * s}; }
double dot2(Vec3 a, Vec3 b) { return a.e * b.e + a.n * b.n; }
double bearing(Vec3 from, Vec3 to) {
  return std::fmod(deg(std::atan2(to.e - from.e, to.n - from.n)) + 360.0, 360.0);
}
double dist2d(Vec3 a, Vec3 b) { return std::hypot(a.e - b.e, a.n - b.n); }

// Course that converges onto a circle (counter-clockwise if ccw).
double circle_course(Vec3 pos, Vec3 center, double radius, bool ccw) {
  const double dx = pos.e - center.e, dy = pos.n - center.n;
  const double r = std::max(1.0, std::hypot(dx, dy));
  const double ue = dx / r, un = dy / r;
  double te = ccw ? -un : un, tn = ccw ? ue : -ue;
  const double k = clamp(1.6 * (radius - r) / radius, -1.6, 1.6);
  te += k * ue;
  tn += k * un;
  return std::fmod(deg(std::atan2(te, tn)) + 360.0, 360.0);
}

// Angle of pos around center, measured counter-clockwise from east (deg).
double polar_angle(Vec3 pos, Vec3 center) {
  return deg(std::atan2(pos.n - center.n, pos.e - center.e));
}

double flight_path_deg(const flight::Ship& s) {
  return deg(std::atan2(s.vel.u, std::max(1.0, std::hypot(s.vel.e, s.vel.n))));
}
double track_deg(const flight::Ship& s) {
  return std::fmod(deg(std::atan2(s.vel.e, s.vel.n)) + 360.0, 360.0);
}

double stick_for_alpha(const flight::Ship& s, double alpha_cmd) {
  const double want_rate = 2.2 * (alpha_cmd - s.alpha_deg);
  return clamp((want_rate - flight::stability_pitch_rate(s)) / flight::pitch_authority(s), -1, 1);
}

double stick_for_pitch(const flight::Ship& s, double pitch_cmd, double max_rate) {
  const double want_rate = clamp(1.6 * (pitch_cmd - s.pitch_deg), -max_rate, max_rate);
  return clamp((want_rate - flight::stability_pitch_rate(s)) / flight::pitch_authority(s), -1, 1);
}

double stick_for_bank(const flight::Ship& s, double bank_cmd) {
  const double want_rate = clamp(1.8 * (bank_cmd - s.roll_deg), -25.0, 25.0);
  return clamp(want_rate / flight::roll_authority(s), -1, 1);
}

// Flight-path-angle controller: computes the lift needed for the commanded
// curvature, converts it to angle of attack, then to stick.
double stick_for_gamma(const flight::Ship& s, double gamma_cmd, double alpha_max, double gain = 0.55) {
  const double v = std::max(30.0, s.tas);
  const double gamma = flight_path_deg(s);
  const double bank = rad(clamp(s.roll_deg, -70.0, 70.0));
  const double an = v * gain * rad(gamma_cmd - gamma) + kG0 * std::cos(rad(gamma)) / std::cos(bank);
  const double cl = s.mass_kg * an / std::max(50.0, s.q_pa * flight::kWingArea);
  const double alpha = deg(cl / flight::lift_slope_per_rad(s.mach));
  return stick_for_alpha(s, clamp(alpha, -3.0, alpha_max));
}

double bank_for_course(const flight::Ship& s, double course, double limit) {
  return clamp(2.0 * wrap180(course - track_deg(s)), -limit, limit);
}

void set_step(State& st, Step next) {
  st.step = next;
  st.step_time_s = 0;
}

std::string feet(double meters) {
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.0f ft", meters / flight::kFeetToMeters);
  return buf;
}
}  // namespace

const char* step_name(Step s) {
  switch (s) {
    case Step::Hold: return "HOLD";
    case Step::TakeoffRoll: return "TAKEOFF ROLL";
    case Step::ClimbOut: return "CLIMB OUT";
    case Step::ClimbCircle: return "CLIMB ORBIT";
    case Step::RunIn: return "RELEASE RUN-IN";
    case Step::Drop: return "DROP";
    case Step::Boost: return "BOOST";
    case Step::Coast: return "COAST";
    case Step::Reentry: return "FEATHERED REENTRY";
    case Step::Defeather: return "DEFEATHER";
    case Step::ToHac: return "GLIDE TO HAC";
    case Step::Hac: return "HAC";
    case Step::Final: return "FINAL";
    case Step::Flare: return "FLARE";
    case Step::Rollout: return "ROLLOUT";
    case Step::Done: return "MISSION COMPLETE";
  }
  return "?";
}

ApproachGeometry geometry(const Field& f) {
  ApproachGeometry g{};
  const Vec3 d = dir(f.runway_heading_deg);
  const Vec3 left = left_of(f.runway_heading_deg);
  g.hac_radius_m = kHacRadius;
  g.final_entry = mul(d, -kFinalEntry);
  g.hac_center = add(g.final_entry, mul(left, kHacRadius));
  g.release_point = mul(d, -30000.0);
  g.climb_center = add(mul(d, -50000.0), mul(left, -9000.0));
  return g;
}

void log_event(State& st, const flight::World& w, const std::string& text) {
  st.events.push_back({w.ship.time_s, text});
  if (st.events.size() > 40) st.events.erase(st.events.begin());
}

void start(State& st) {
  const bool automatic = st.automatic;
  st = State{};
  st.automatic = automatic;
  set_step(st, Step::TakeoffRoll);
}

Output update(State& st, flight::World& w, const Field& field, const flight::Controls& pilot,
              double dt) {
  Output out;
  auto& s = w.ship;
  const auto g = geometry(field);
  const double hdg = field.runway_heading_deg;
  const Vec3 d = dir(hdg);
  const double agl = s.pos.u - s.ground_elev_m - flight::kGearHeightM;
  st.step_time_s += dt;
  out.carrier.heading_deg = w.carrier.heading_deg;
  out.carrier.target_alt_m = flight::kReleaseAltM;

  // ---------------------------------------------------------------- carrier
  switch (st.step) {
    case Step::Hold:
      break;
    case Step::TakeoffRoll:
      out.carrier.takeoff = true;
      out.carrier.heading_deg = hdg;
      if (w.carrier.airborne) {
        log_event(st, w, "WhiteKnightTwo airborne, Eielson AFB runway 32");
        set_step(st, Step::ClimbOut);
      }
      break;
    case Step::ClimbOut:
      out.carrier.heading_deg = hdg;
      if (w.carrier.pos.u - field.elevation_m > 1500.0) set_step(st, Step::ClimbCircle);
      break;
    case Step::ClimbCircle:
      out.carrier.heading_deg = circle_course(w.carrier.pos, g.climb_center, 9000.0, false);
      if (w.carrier.pos.u > 14950.0) {
        log_event(st, w, "Climb orbit complete, " + feet(w.carrier.pos.u) + ", turning to run-in");
        set_step(st, Step::RunIn);
      }
      break;
    case Step::RunIn: {
      // Straight run-in along the extended runway centreline toward the
      // field, so SpaceShipTwo can glide home after the flight.
      const Vec3 start_pt = mul(d, -62000.0);
      const double along = dot2(w.carrier.pos, d);
      const double cross = w.carrier.pos.e * (-d.n) + w.carrier.pos.n * d.e;  // + left
      if (!st.runin_captured && dist2d(w.carrier.pos, start_pt) < 3000.0) st.runin_captured = true;
      out.carrier.heading_deg = st.runin_captured
          ? std::fmod(hdg + clamp(cross * 0.01, -40.0, 40.0) + 360.0, 360.0)
          : bearing(w.carrier.pos, start_pt);
      const bool aligned = st.runin_captured && std::abs(wrap180(w.carrier.heading_deg - hdg)) < 4.0 &&
                           std::abs(cross) < 1500.0 && along < -20000.0;
      st.aligned_s = aligned && w.carrier.pos.u >= flight::kReleaseAltM - 25.0 ? st.aligned_s + dt : 0.0;
      if (!s.released && st.aligned_s > 14.0 && (st.automatic || st.step_time_s > 900.0)) {
        flight::release(w);
        log_event(st, w, "RELEASE at " + feet(s.pos.u) + " -- SpaceShipTwo free of WhiteKnightTwo");
        set_step(st, Step::Drop);
      }
      break;
    }
    default:
      // Carrier peels off left and descends toward the field after release.
      out.carrier.heading_deg = std::fmod(hdg - 70.0 + 360.0, 360.0);
      out.carrier.target_alt_m = 9000.0;
      break;
  }
  if (s.released && st.step == Step::RunIn) set_step(st, Step::Drop);

  // ---------------------------------------------------------------- warp
  double warp = 1.0;
  if (st.step == Step::ClimbOut && st.step_time_s > 25.0) warp = 16.0;
  if (st.step == Step::ClimbCircle) warp = 16.0;
  if (st.step == Step::RunIn) warp = st.aligned_s > 0.0 ? 1.0 : 8.0;
  if (st.step == Step::Reentry && s.pos.u < 72000.0 && s.pos.u > 24000.0) warp = 2.0;
  if (st.step == Step::ToHac || st.step == Step::Hac) warp = 3.0;
  if (st.step == Step::Final && agl > 2500.0) warp = 2.0;
  // The carrier climb is compressed in both modes; after release a pilot
  // flies in real time.
  out.warp = (st.automatic || !s.released) ? warp : 1.0;

  if (!s.released) {
    out.controls = {};
    return out;
  }

  // ---------------------------------------------------------------- events
  double& last_vu = st.last_vu;
  if (last_vu > 0 && s.vel.u <= 0 && s.pos.u > 30000.0)
    log_event(st, w, "APOGEE " + feet(s.pos.u) + " -- " + std::to_string(int(s.max_mach * 10) / 10) +
                         "." + std::to_string(int(s.max_mach * 10) % 10) + " max Mach");
  if (st.last_alt < flight::kCloudAltM && s.pos.u >= flight::kCloudAltM)
    log_event(st, w, "Crossing noctilucent cloud layer, " + feet(flight::kCloudAltM) + " (ascending)");
  if (st.last_alt > flight::kCloudAltM && s.pos.u <= flight::kCloudAltM && st.last_alt > 0)
    log_event(st, w, "Crossing noctilucent cloud layer, " + feet(flight::kCloudAltM) + " (descending)");
  st.last_alt = s.pos.u;
  last_vu = s.vel.u;

  if (!st.automatic && st.step != Step::Done) {
    // Pilot flies; the sequencer only tracks phase changes for captions.
    out.controls = pilot;
    if (s.on_ground && st.step != Step::Rollout) {
      log_event(st, w, "TOUCHDOWN");
      set_step(st, Step::Rollout);
    }
    return out;
  }

  // ---------------------------------------------------------------- ship
  flight::Controls c{};
  switch (st.step) {
    case Step::Drop:
      c.pitch = stick_for_pitch(s, 4.0, 3.0);
      c.roll = stick_for_bank(s, 0.0);
      if (w.carrier_separation_s > 4.0 && flight::ignite(w)) {
        log_event(st, w, "IGNITION -- RocketMotorTwo lit at " + feet(s.pos.u));
        set_step(st, Step::Boost);
      }
      break;
    case Step::Boost: {
      // Pull to near-vertical: ~3.5 g gamma turn, then hold 85 deg. Shut down
      // when the vacuum-ballistic apogee prediction reaches the target.
      const double pull_rate = s.g_load > 3.8 ? 1.0 : 3.2;
      const double pitch_cmd = std::min(85.0, 8.0 + 3.2 * st.step_time_s);
      c.pitch = stick_for_pitch(s, pitch_cmd, pull_rate);
      c.roll = stick_for_bank(s, 0.0);
      const double drag_margin = s.pos.u < 35000.0 ? 2500.0 : 900.0;
      if (s.rocket_on && flight::predicted_apogee(s) - drag_margin >= st.target_apogee_m) {
        flight::cutoff(w);
        log_event(st, w, "SHUTDOWN at " + feet(s.pos.u) + ", Mach " +
                             std::to_string(int(s.mach * 10) / 10) + "." +
                             std::to_string(int(s.mach * 10) % 10));
      }
      if (!s.rocket_on) {
        if (!s.burned_out) flight::cutoff(w);
        set_step(st, Step::Coast);
      }
      break;
    }
    case Step::Coast:
      // RCS: hold the climb attitude briefly, then raise the feather and
      // orient belly-forward for reentry.
      if (st.step_time_s < 6.0) c.pitch = stick_for_pitch(s, s.pitch_deg, 1.0);
      else c.pitch = stick_for_alpha(s, 60.0);
      c.roll = stick_for_bank(s, 0.0);
      if (!w.probe.deployed && s.pos.u >= flight::kProbeAltM) {
        flight::deploy_probe(w);
        log_event(st, w, "PROBE DEPLOYED at " + feet(s.pos.u));
      }
      if (!s.feather_cmd && st.step_time_s > 10.0) {
        flight::set_feather(w, true);
        log_event(st, w, "FEATHER UNLOCKED AND RAISED");
      }
      if (s.vel.u < 0 && st.step_time_s > 5.0) set_step(st, Step::Reentry);
      break;
    case Step::Reentry:
      c.pitch = stick_for_alpha(s, 62.0);
      c.roll = stick_for_bank(s, 0.0);
      if (!w.probe.deployed && s.pos.u >= flight::kProbeAltM) {
        flight::deploy_probe(w);
        log_event(st, w, "PROBE DEPLOYED at " + feet(s.pos.u));
      }
      if (s.pos.u < flight::kDefeatherAltM && s.vel.u < 0) {
        flight::set_feather(w, false);
        log_event(st, w, "DEFEATHER at " + feet(s.pos.u) + " -- peak load " +
                             std::to_string(int(s.max_g * 10) / 10) + "." +
                             std::to_string(int(s.max_g * 10) % 10) + " g");
        set_step(st, Step::Defeather);
      }
      break;
    case Step::Defeather:
      c.pitch = stick_for_alpha(s, s.feather > 0.3 ? 30.0 : 6.0);
      c.roll = stick_for_bank(s, 0.0);
      if (s.feather < 0.02) {
        log_event(st, w, "Glider configuration -- turning for Eielson");
        set_step(st, Step::ToHac);
      }
      break;
    case Step::ToHac: {
      const double crs = bearing(s.pos, g.hac_center);
      c.roll = stick_for_bank(s, bank_for_course(s, crs, 45.0));
      // Best glide (L/D max near alpha 8.5 deg) unless already low.
      c.pitch = stick_for_alpha(s, flight_path_deg(s) < -25 ? 12.0 : 8.5);
      if (dist2d(s.pos, g.hac_center) < kHacRadius * 1.25) {
        st.hac_prev_angle = polar_angle(s.pos, g.hac_center);
        st.hac_swept_deg = 0;
        log_event(st, w, "On heading-alignment circle, " + feet(s.pos.u));
        set_step(st, Step::Hac);
      }
      break;
    }
    case Step::Hac: {
      const double crs = circle_course(s.pos, g.hac_center, kHacRadius, true);
      const double ff = -deg(std::atan(s.tas * s.tas / (kG0 * kHacRadius)));
      const double bank = clamp(ff + 2.0 * wrap180(crs - track_deg(s)), -55.0, 15.0);
      c.roll = stick_for_bank(s, bank);
      // Remaining CCW arc to the tangent point on the extended centreline.
      const double ang = polar_angle(s.pos, g.hac_center);
      const double exit_ang = polar_angle(g.final_entry, g.hac_center);
      double arc = std::fmod(exit_ang - ang + 720.0, 360.0);
      st.hac_swept_deg += std::fmod(ang - st.hac_prev_angle + 540.0, 360.0) - 180.0;
      st.hac_prev_angle = ang;
      const double loop_m = 2 * kPi * kHacRadius;
      const double entry_h = field.elevation_m + (kFinalEntry - kAimPoint) * std::tan(rad(kSteepDeg));
      const double ref_no_loop = entry_h + rad(arc) * kHacRadius * std::tan(rad(kHacGlideDeg));
      const int extra = std::max(0, int(std::floor((s.pos.u - ref_no_loop) /
                                                   (loop_m * std::tan(rad(kHacGlideDeg))) + 0.25)));
      const double ref = ref_no_loop + extra * loop_m * std::tan(rad(kHacGlideDeg));
      const double gamma_cmd = -kHacGlideDeg - clamp((s.pos.u - ref) / 700.0 * 4.0, -4.0, 7.0);
      c.pitch = stick_for_gamma(s, gamma_cmd, 14.0);
      if (arc < 6.0 && extra == 0 && st.hac_swept_deg > 90.0) {
        log_event(st, w, "Rolling out on final, runway 32, " + feet(s.pos.u));
        set_step(st, Step::Final);
      }
      if (arc > 354.0 && st.hac_swept_deg > 90.0 && extra > 0 && st.step_time_s > 5.0) {
        // Passed the exit point with energy to spare: one more turn.
        st.hac_swept_deg = 0;
        ++st.hac_loops;
      }
      break;
    }
    case Step::Final: {
      const double along = dot2(s.pos, d);  // negative before the threshold
      const double cross = s.pos.e * d.n - s.pos.n * d.e;  // + right of centreline
      const double crs = std::fmod(hdg - clamp(cross * 0.04, -30.0, 30.0) + 360.0, 360.0);
      c.roll = stick_for_bank(s, bank_for_course(s, crs, agl < 60.0 ? 8.0 : 30.0));
      // Steep glide to an aim point short of the runway, then a progressive
      // preflare onto a shallow 2 deg path below 180 m.
      const double steep_h = (-kAimPoint - along) * std::tan(rad(kSteepDeg));
      double gamma_cmd = -kSteepDeg + clamp((steep_h - agl) * 0.04, -5.0, 5.0);
      if (agl < 180.0) gamma_cmd = -(kShallowDeg + (kSteepDeg - kShallowDeg) * clamp((agl - 40.0) / 140.0, 0.0, 1.0));
      c.pitch = stick_for_gamma(s, gamma_cmd, 15.0);
      if (!s.gear_cmd && agl < 800.0) {
        flight::set_gear(w, true);
        log_event(st, w, "GEAR DOWN");
      }
      if (agl < 30.0) set_step(st, Step::Flare);
      break;
    }
    case Step::Flare: {
      const double cross = s.pos.e * d.n - s.pos.n * d.e;
      const double crs = std::fmod(hdg - clamp(cross * 0.04, -10.0, 10.0) + 360.0, 360.0);
      c.roll = stick_for_bank(s, bank_for_course(s, crs, 4.0));
      // Hold off a metre or two above the runway until the speed bleeds back.
      const double sink = s.keas > 150.0 ? std::max(0.0, 0.25 * (agl - 1.5))
                                          : 0.45 + 0.08 * std::max(0.0, agl);
      c.pitch = stick_for_gamma(s, -deg(std::asin(clamp(sink / std::max(40.0, s.tas), 0.0, 0.3))), 15.5, 1.6);
      if (s.on_ground) {
        st.touchdown_keas = s.keas;
        char buf[96];
        std::snprintf(buf, sizeof buf, "TOUCHDOWN runway 32 -- %.0f KEAS", s.keas);
        log_event(st, w, buf);
        set_step(st, Step::Rollout);
      }
      break;
    }
    case Step::Rollout:
      c.brake = st.step_time_s > 3.0 ? 0.8 : 0.0;
      if (s.tas < 0.3) {
        log_event(st, w, "WHEELS STOP -- mission complete");
        set_step(st, Step::Done);
      }
      break;
    case Step::Done:
      c.brake = 1.0;
      break;
    default:
      break;
  }
  if (s.on_ground && st.step != Step::Rollout && st.step != Step::Done) {
    st.touchdown_keas = s.keas;
    log_event(st, w, "TOUCHDOWN");
    set_step(st, Step::Rollout);
  }
  out.controls = c;
  return out;
}

}  // namespace autopilot
