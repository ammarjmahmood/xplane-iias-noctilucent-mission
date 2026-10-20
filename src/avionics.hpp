#pragma once
// Three-display glass cockpit drawn in X-Plane plugin windows:
//   PFD   - attitude, speed/Mach and altitude tapes, VSI, g, heading
//   NAV   - ascent/descent profile against the plan, plan-view map with the
//           runway, heading-alignment circle, carrier and probe
//   SYS   - rocket motor, feather, gear, probe telemetry and the event log
// The displays can also be popped out to separate monitors.
#include <string>
#include <vector>

namespace avionics {

struct Point2 {
  float x, y;
};

struct Snapshot {
  double met_s = 0;
  std::string phase, step, camera;
  double warp = 1;
  bool automatic = true;
  // attitude / air data
  double pitch = 0, roll = 0, heading = 0, track = 0, alpha = 0, beta = 0;
  double keas = 0, mach = 0, max_mach = 0, alt_ft = 0, agl_ft = 0, vs_fpm = 0, g = 1, gmax = 1;
  double apogee_ft = 0, predicted_apogee_ft = 0, q_kpa = 0;
  // propulsion and configuration
  double thrust_kn = 0, chamber_bar = 0, ox_kg = 0, ox_max = 4800, burn_s = 0, mass_kg = 0;
  bool rocket_on = false, burned_out = false, released = false, on_ground = true;
  double feather = 0, gear = 0;
  bool feather_cmd = false, gear_cmd = false;
  // probe and cloud layer
  bool probe_deployed = false, probe_chute = false, probe_landed = false;
  double probe_alt_ft = 0, probe_apogee_ft = 0, probe_density = 0, probe_max_density = 0, probe_iwc = 0;
  bool probe_in_layer = false;
  double ship_density = 0;
  // plan-view positions (km, east/north from the runway threshold)
  Point2 ship{0, 0}, carrier{0, 0}, probe{0, 0}, hac{0, 0}, final_entry{0, 0}, release{0, 0};
  double hac_radius_km = 4.5, runway_heading = 0, runway_length_km = 4.4;
  double carrier_alt_ft = 0;
  std::string field_name;
  // histories (downrange km, altitude km)
  std::vector<Point2> profile, probe_profile, plan, ground_track;
  std::vector<float> density_history;
  std::vector<std::string> events;
  std::string caption;
  double caption_age_s = 99;
};

void draw_pfd(const Snapshot& s, int l, int t, int r, int b);
void draw_nav(const Snapshot& s, int l, int t, int r, int b);
void draw_sys(const Snapshot& s, int l, int t, int r, int b);
void draw_caption(const Snapshot& s, int l, int t, int r, int b, bool compact_telemetry);

// Stroke font, cap height `size` pixels.
void text(float x, float y, float size, const std::string& s, float r, float g, float b, float a = 1.f,
          float weight = 1.6f);
float text_width(float size, const std::string& s);

}  // namespace avionics
