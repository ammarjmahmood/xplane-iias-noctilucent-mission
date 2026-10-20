// IIAS Noctilucent Mission for X-Plane 12.
//
// WhiteKnightTwo carries SpaceShipTwo from Eielson AFB to 50,000 ft and
// releases it; the ship boosts to ~290,000 ft, deploys a probe at 150,000 ft,
// crosses a synthetic noctilucent cloud layer at ~270,000 ft, reenters
// feathered and glides back to the runway. The flight model, autoflight,
// three-display avionics and camera director are in this plugin; the user
// aircraft is the "IIAS SpaceShipTwo" package, positioned by the plugin.
#include "XPLMCamera.h"
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMMenus.h"
#include "XPLMPlanes.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"

#include "autopilot.hpp"
#include "avionics.hpp"
#include "flight.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <string>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetersPerDegLat = 111132.0;
double rad(double d) { return d * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }

// ---------------------------------------------------------------- fields
struct FieldDef {
  const char* name;
  const char* label;
  double thr_lat, thr_lon;  // landing threshold
  double end_lat, end_lon;  // far end of the runway
  double elev_m;
};
const FieldDef kFields[] = {
    {"EIELSON AFB RWY 32", "Eielson AFB (PAEI) runway 32", 64.6471515, -147.0850382, 64.6842042, -147.1179305, 166.4},
    {"FAIRBANKS INTL RWY 2L", "Fairbanks Intl (PAFA) runway 2L", 64.8026010, -147.8859047, 64.8280431, -147.8391932,
     132.3},
    {"JUNEAU INTL RWY 8", "Juneau Intl (PAJN) runway 8 (demo scenery)", 58.3578488, -134.5969749, 58.3516292,
     -134.5523594, 7.0},
};
int field_index = 0;

struct Geo {
  double lat0 = 0, lon0 = 0;
  void enu_to_ll(double e, double n, double& lat, double& lon) const {
    lat = lat0 + n / kMetersPerDegLat;
    lon = lon0 + e / (111320.0 * std::cos(rad(lat0)));
  }
  void ll_to_enu(double lat, double lon, double& e, double& n) const {
    n = (lat - lat0) * kMetersPerDegLat;
    e = (lon - lon0) * 111320.0 * std::cos(rad(lat0));
  }
} geo;

autopilot::Field field_geometry(const FieldDef& f) {
  Geo g{f.thr_lat, f.thr_lon};
  double e, n;
  g.ll_to_enu(f.end_lat, f.end_lon, e, n);
  autopilot::Field out;
  out.runway_heading_deg = std::fmod(deg(std::atan2(e, n)) + 360.0, 360.0);
  out.runway_length_m = std::hypot(e, n);
  out.elevation_m = f.elev_m;
  return out;
}

// ---------------------------------------------------------------- state
std::filesystem::path plugin_root;
flight::World world;
autopilot::State ap;
autopilot::Field fld;
bool active = false;
bool pending_start = false;
double pending_timer = 0;
bool wants_auto = true;
double met = 0;               // mission elapsed (sim seconds since start)
double wall = 0;
std::vector<avionics::Point2> plan_profile, ship_profile, probe_profile;
std::vector<float> density_hist;
double hist_timer = 0, log_timer = 0;
double caption_time = -100;
std::string caption;
size_t events_seen = 0;
std::mt19937 rng(42);
double probe_density = 0, ship_density = 0;
// Real NASA AIM/CIPS cloud layer (see tools/generate_assets.py).
struct CloudData {
  int n = 0;
  double macro_m = 0, detail_m = 12000, off_e = 0, off_n = 0;
  std::vector<float> albedo, iwc;        // n*n, rows south->north
  std::vector<unsigned char> detail;     // 1024*1024, rows north->south
} cloud;
double probe_iwc = 0;
// Fallback Eielson scenery (tools/generate_terrain.py), used only when
// X-Plane has no terrain there (the free demo).
struct HeightGrid {
  int n = 0;
  double half = 0, step = 0;
  std::vector<short> h;  // rows north-increasing, cols east-increasing
  bool sample(double e, double nn, double& out) const {
    if (n < 2 || std::abs(e) > half - step || std::abs(nn) > half - step) return false;
    const double x = (e + half) / step, y = (nn + half) / step;
    const int i = static_cast<int>(y), j = static_cast<int>(x);
    const double fx = x - j, fy = y - i;
    auto at = [&](int r, int c) { return static_cast<double>(h[r * n + c]); };
    out = (at(i, j) * (1 - fx) + at(i, j + 1) * fx) * (1 - fy) + (at(i + 1, j) * (1 - fx) + at(i + 1, j + 1) * fx) * fy;
    return true;
  }
};
std::array<HeightGrid, 3> scenery_hgt;
constexpr double kScnLat = 64.6471515, kScnLon = -147.0850382;  // mesh origin (RWY 32 threshold)
bool scenery_active = false;
std::ofstream probe_log;

// datarefs
XPLMDataRef dr_override = nullptr, dr_x = nullptr, dr_y = nullptr, dr_z = nullptr, dr_vx = nullptr, dr_vy = nullptr,
            dr_vz = nullptr, dr_psi = nullptr, dr_theta = nullptr, dr_phi = nullptr, dr_q = nullptr,
            dr_lat = nullptr, dr_lon = nullptr, dr_elev = nullptr, dr_paused = nullptr, dr_sun = nullptr,
            dr_yoke_p = nullptr, dr_yoke_r = nullptr, dr_yoke_h = nullptr, dr_date = nullptr, dr_zulu = nullptr,
            dr_wx_preset = nullptr, dr_wx_now = nullptr, dr_use_sys_time = nullptr, dr_ev = nullptr;
double ev_base = 7.8;
std::vector<XPLMDataRef> owned_refs;
float v_one = 1.0f, v_nlc_visible = 0.0f, v_ev_offset = 0.0f, v_ev_space = 13.5f;
float v_feather = 0, v_gear = 0, v_plume = 0, v_flicker = 0, v_glow = 0.8f, v_density = 0;
int v_in_layer = 0;

// objects
XPLMObjectRef obj_wk2 = nullptr, obj_ss2 = nullptr, obj_probe = nullptr, obj_trail = nullptr,
              obj_plume = nullptr;
XPLMInstanceRef inst_wk2 = nullptr, inst_ss2 = nullptr, inst_probe = nullptr, inst_plume = nullptr;
XPLMObjectRef obj_macro = nullptr, obj_detail = nullptr;
XPLMInstanceRef inst_macro = nullptr;
std::array<XPLMInstanceRef, 9> inst_detail{};
std::array<XPLMObjectRef, 4> obj_scenery{};      // l0, l1, l2, runway
std::array<XPLMInstanceRef, 4> inst_scenery{};
struct TrailSeg {
  XPLMInstanceRef inst = nullptr;
  double lat = 0, lon = 0, alt = 0, heading = 0, pitch = 0, born = -1;
};
std::vector<TrailSeg> trail;
size_t trail_next = 0;
flight::Vec3 last_trail_pos;
bool user_is_ss2 = false;
bool debug_ss2_instance = true;  // SpaceShipTwo is always drawn as an instance
XPLMProbeRef terrain_probe = nullptr;

// ---------------------------------------------------------------- helpers
template <typename T>
T clampv(T v, T lo, T hi) { return std::max(lo, std::min(hi, v)); }

void local_of(const flight::Vec3& enu, double& x, double& y, double& z) {
  double lat, lon;
  geo.enu_to_ll(enu.e, enu.n, lat, lon);
  XPLMWorldToLocal(lat, lon, enu.u, &x, &y, &z);
}

struct V3 {
  double x, y, z;
};
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
V3 norm(V3 a) {
  const double m = std::sqrt(dot(a, a));
  return m > 1e-9 ? a * (1.0 / m) : V3{0, 0, -1};
}
// ENU direction -> local OpenGL direction.
V3 gl_dir(const flight::Vec3& v) { return {v.e, v.u, -v.n}; }

V3 ship_local() {
  double x, y, z;
  local_of(world.ship.pos, x, y, z);
  return {x, y, z};
}
V3 carrier_local() {
  double x, y, z;
  local_of(world.carrier.pos, x, y, z);
  return {x, y, z};
}
// OBJ-space point on a body (x right, y up, z aft) -> local OpenGL.
V3 body_point(V3 origin, const flight::Quat& q, V3 obj) {
  const V3 f = gl_dir(flight::body_forward(q)), l = gl_dir(flight::body_left(q)), u = gl_dir(flight::body_up(q));
  return origin + f * (-obj.z) + l * (-obj.x) + u * obj.y;
}

void set_quaternion(double psi, double theta, double phi) {
  const double p = kPi / 360.0 * psi, t = kPi / 360.0 * theta, r = kPi / 360.0 * phi;
  float q[4];
  q[0] = static_cast<float>(std::cos(p) * std::cos(t) * std::cos(r) + std::sin(p) * std::sin(t) * std::sin(r));
  q[1] = static_cast<float>(std::cos(p) * std::cos(t) * std::sin(r) - std::sin(p) * std::sin(t) * std::cos(r));
  q[2] = static_cast<float>(std::cos(p) * std::sin(t) * std::cos(r) + std::sin(p) * std::cos(t) * std::sin(r));
  q[3] = static_cast<float>(-std::cos(p) * std::sin(t) * std::sin(r) + std::sin(p) * std::cos(t) * std::cos(r));
  if (dr_q) XPLMSetDatavf(dr_q, q, 0, 4);
}

void place_instance(XPLMInstanceRef inst, V3 p, double heading, double pitch, double roll, const float* data) {
  if (!inst) return;
  XPLMDrawInfo_t d{};
  d.structSize = sizeof(d);
  d.x = static_cast<float>(p.x);
  d.y = static_cast<float>(p.y);
  d.z = static_cast<float>(p.z);
  d.heading = static_cast<float>(heading);
  d.pitch = static_cast<float>(pitch);
  d.roll = static_cast<float>(roll);
  static const float zeros[8] = {};
  XPLMInstanceSetPosition(inst, &d, data ? data : zeros);
}

double terrain_msl(V3 p, bool& ok) {
  ok = false;
  if (!terrain_probe) return 0;
  XPLMProbeInfo_t info{};
  info.structSize = sizeof(info);
  if (XPLMProbeTerrainXYZ(terrain_probe, static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
                          &info) != xplm_ProbeHitTerrain)
    return 0;
  double lat, lon, alt;
  XPLMLocalToWorld(info.locationX, info.locationY, info.locationZ, &lat, &lon, &alt);
  ok = true;
  return alt;
}

// ------------------------------------------------------- cloud density
// Same tiling as the rendered cloud field: 120 km tiles anchored to a fixed
// lat/lon lattice, middle sheet UV offset (0.37, 0.61).
bool scenery_ground(double lat, double lon, double& h) {
  if (!scenery_active) return false;
  const double n = (lat - kScnLat) * kMetersPerDegLat;
  const double e = (lon - kScnLon) * 111320.0 * std::cos(rad(kScnLat));
  for (const auto& g : scenery_hgt)
    if (g.sample(e, n, h)) return true;
  return false;
}

// Cloud fields at a point east/north (m) of the mission origin.
double grid_sample(const std::vector<float>& g, double e, double n) {
  if (cloud.n < 2 || g.size() != static_cast<size_t>(cloud.n * cloud.n)) return 0;
  const double M = cloud.macro_m;
  const double x = ((e - cloud.off_e) + M / 2) / M * cloud.n - 0.5;
  const double y = ((n - cloud.off_n) + M / 2) / M * cloud.n - 0.5;
  if (x < 0 || y < 0 || x > cloud.n - 1.001 || y > cloud.n - 1.001) return 0;
  const int i = static_cast<int>(y), j = static_cast<int>(x);
  const double fx = x - j, fy = y - i;
  auto at = [&](int r, int c) { return static_cast<double>(g[r * cloud.n + c]); };
  return (at(i, j) * (1 - fx) + at(i, j + 1) * fx) * (1 - fy) + (at(i + 1, j) * (1 - fx) + at(i + 1, j + 1) * fx) * fy;
}
double cloud_detail(double e, double n) {
  if (cloud.detail.size() != 1024 * 1024) return 0.5;
  const double D = cloud.detail_m;
  double u = (e - cloud.off_e) / D + 0.5, v = (n - cloud.off_n) / D + 0.5;
  u -= std::floor(u);
  v -= std::floor(v);
  const int px = clampv(static_cast<int>(u * 1024), 0, 1023);
  const int py = clampv(static_cast<int>((1.0 - v) * 1024), 0, 1023);
  return cloud.detail[py * 1024 + px] / 255.0;
}
double vertical_envelope(double alt) {
  const double dz = (alt - flight::kCloudAltM) / 450.0;
  return std::abs(dz) > 3.5 ? 0.0 : std::exp(-dz * dz);
}
// Normalised 0..1 "NLC density" for the probe: real albedo x fine structure.
double cloud_density_enu(double e, double n, double alt) {
  const double env = vertical_envelope(alt);
  if (env <= 0) return 0;
  const double alb = clampv(grid_sample(cloud.albedo, e, n) / 45.0, 0.0, 1.0);
  return alb * (0.55 + 0.45 * cloud_detail(e, n)) * env;
}
double cloud_iwc_enu(double e, double n, double alt) {
  return grid_sample(cloud.iwc, e, n) * vertical_envelope(alt);
}

// Orientation that aligns an object's axes with east/north/up at a point.
// X-Plane's local frame is a tangent plane at a reference point that moves
// as the user flies, so large objects far from it must be re-oriented.
void place_geo(XPLMInstanceRef inst, double lat, double lon, double alt, const float* data) {
  double x0, y0, z0, xn, yn, zn, xu, yu, zu;
  XPLMWorldToLocal(lat, lon, alt, &x0, &y0, &z0);
  XPLMWorldToLocal(lat + 0.01, lon, alt, &xn, &yn, &zn);
  XPLMWorldToLocal(lat, lon, alt + 1000.0, &xu, &yu, &zu);
  const V3 f = norm({xn - x0, yn - y0, zn - z0});
  const V3 u = norm({xu - x0, yu - y0, zu - z0});
  const V3 r0 = norm(cross(f, {0, 1, 0}));
  const V3 u0 = cross(r0, f);
  place_instance(inst, {x0, y0, z0}, deg(std::atan2(f.x, -f.z)), deg(std::asin(clampv(f.y, -1.0, 1.0))),
                 deg(std::atan2(dot(u, r0), dot(u, u0))), data);
}

void place_scenery() {
  for (auto& inst : inst_scenery) place_geo(inst, kScnLat, kScnLon, 0.0, nullptr);
}

void place_clouds(double ship_e, double ship_n) {
  double clat, clon;
  geo.enu_to_ll(cloud.off_e, cloud.off_n, clat, clon);
  place_geo(inst_macro, clat, clon, flight::kCloudAltM, nullptr);
  // 3x3 high-resolution tiles around the ship on a lattice anchored to the sheet.
  const double D = cloud.detail_m;
  const double ce = std::round((ship_e - cloud.off_e) / D) * D + cloud.off_e;
  const double cn = std::round((ship_n - cloud.off_n) / D) * D + cloud.off_n;
  int k = 0;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc, ++k) {
      const double e = ce + dc * D, n = cn + dr * D;
      const double alb = grid_sample(cloud.albedo, e, n);
      float level = alb > 16 ? 2.0f : alb > 5 ? 1.0f : 0.0f;
      if (v_nlc_visible < 0.05f) level = 0.0f;
      geo.enu_to_ll(e, n, clat, clon);
      place_geo(inst_detail[k], clat, clon, flight::kCloudAltM, &level);
    }
  }
}

// ---------------------------------------------------------------- camera
enum class Cam { Cockpit, Chase, Side, Carrier, Ground, Probe, Flyby, Orbit, Tower, Count };
const char* cam_name(Cam c) {
  switch (c) {
    case Cam::Cockpit: return "COCKPIT";
    case Cam::Chase: return "CHASE CAM";
    case Cam::Side: return "WINGMAN CAM";
    case Cam::Carrier: return "WHITEKNIGHTTWO CAM";
    case Cam::Ground: return "GROUND TRACKER";
    case Cam::Probe: return "PROBE CAM";
    case Cam::Flyby: return "FLYBY CAM";
    case Cam::Orbit: return "ORBIT CAM";
    case Cam::Tower: return "RUNWAY CAM";
    default: return "";
  }
}
Cam cam = Cam::Chase;
bool cam_auto = true;
bool camera_owned = false;
double cam_switch_wall = 0;
double orbit_angle = 0;
V3 flyby_point{0, 0, 0};
bool flyby_valid = false;
V3 chase_dir{0, 0, 1};
bool displays_docked = true;
double base_fov = 60.0, wanted_fov = 60.0, applied_fov = -1;
XPLMDataRef dr_fov = nullptr;
bool popout = false;

void set_cam(Cam c) {
  if (c != cam) {
    cam = c;
    cam_switch_wall = wall;
    flyby_valid = false;
    char buf[96];
    std::snprintf(buf, sizeof buf, "NLC: camera %s at wall %.1f\n", cam_name(c), wall);
    XPLMDebugString(buf);
  }
}

void look_at(XPLMCameraPosition_t* out, V3 eye, V3 target, double roll, double zoom) {
  const V3 d = norm(target - eye);
  out->x = static_cast<float>(eye.x);
  out->y = static_cast<float>(eye.y);
  out->z = static_cast<float>(eye.z);
  out->heading = static_cast<float>(deg(std::atan2(d.x, -d.z)));
  out->pitch = static_cast<float>(deg(std::atan2(d.y, std::hypot(d.x, d.z))));
  out->roll = static_cast<float>(roll);
  // X-Plane 12 ignores the plugin camera's zoom; narrow the field of view.
  out->zoom = 1.0f;
  wanted_fov = clampv(base_fov / std::max(0.5, zoom), 0.6, 100.0);
}

// Choose camera from the mission state (the "director").
void direct_camera() {
  using autopilot::Step;
  const double t = ap.step_time_s;
  const double since = wall - cam_switch_wall;
  const auto& s = world.ship;
  auto cycle = [&](std::initializer_list<Cam> cams, double every) {
    const int idx = static_cast<int>(wall / every) % static_cast<int>(cams.size());
    set_cam(*(cams.begin() + idx));
  };
  switch (ap.step) {
    case Step::Hold:
    case Step::TakeoffRoll:
      set_cam(t < 18 ? Cam::Tower : Cam::Chase);
      break;
    case Step::ClimbOut:
      set_cam(t < 20 ? Cam::Chase : Cam::Orbit);
      break;
    case Step::ClimbCircle:
      cycle({Cam::Orbit, Cam::Carrier, Cam::Side, Cam::Cockpit, Cam::Chase}, 9.0);
      break;
    case Step::RunIn:
      set_cam(ap.aligned_s > 0.0 ? Cam::Carrier : Cam::Cockpit);
      break;
    case Step::Drop:
      set_cam(Cam::Carrier);
      break;
    case Step::Boost:
      if (t < 7) set_cam(Cam::Carrier);
      else if (t < 16) set_cam(Cam::Chase);
      else if (t < 27) set_cam(Cam::Cockpit);
      else if (t < 38) set_cam(Cam::Side);
      else set_cam(Cam::Ground);
      break;
    case Step::Coast: {
      const double since_probe = s.time_s - world.probe.deploy_time_s;
      const bool probe_recent = world.probe.deployed && since_probe > 3.0 && since_probe < 15.0;
      const bool near_cloud = std::abs(s.pos.u - flight::kCloudAltM) < 3500.0;
      if (probe_recent) set_cam(Cam::Probe);
      else if (near_cloud) set_cam(since > 8 && cam == Cam::Side ? Cam::Cockpit : (cam == Cam::Cockpit ? Cam::Cockpit : Cam::Side));
      else if (s.vel.u < 250) set_cam(Cam::Orbit);
      else set_cam(Cam::Cockpit);
      break;
    }
    case Step::Reentry:
      if (std::abs(s.pos.u - flight::kCloudAltM) < 3000.0) set_cam(Cam::Side);
      else cycle({Cam::Chase, Cam::Cockpit, Cam::Orbit}, 10.0);
      break;
    case Step::Defeather:
      set_cam(Cam::Chase);
      break;
    case Step::ToHac:
    case Step::Hac:
      cycle({Cam::Orbit, Cam::Cockpit, Cam::Flyby, Cam::Chase}, 11.0);
      break;
    case Step::Final: {
      const double agl = s.pos.u - s.ground_elev_m;
      set_cam(agl > 900 ? Cam::Cockpit : agl > 250 ? Cam::Chase : Cam::Tower);
      break;
    }
    case Step::Flare:
      set_cam(Cam::Tower);
      break;
    case Step::Rollout:
    case Step::Done:
      set_cam(t < 6 ? Cam::Tower : Cam::Orbit);
      break;
  }
}

int camera_func(XPLMCameraPosition_t* out, int losing, void*) {
  if (losing) {
    camera_owned = false;
    return 0;
  }
  if (!active || !out) return 0;
  const auto& s = world.ship;
  const V3 ship = ship_local();
  const V3 f = gl_dir(flight::body_forward(s.att)), u = gl_dir(flight::body_up(s.att));
  V3 vel = gl_dir(s.vel);
  if (dot(vel, vel) < 4) vel = f;
  const V3 vdir = norm(vel);
  switch (cam) {
    case Cam::Cockpit: {
      const V3 eye = body_point(ship, s.att, {-0.40, 0.18, -6.15});
      const double up_tilt = rad(17.0);
      const V3 fwd = norm(f * std::cos(up_tilt) + u * std::sin(up_tilt));
      const V3 cup = norm(u * std::cos(up_tilt) - f * std::sin(up_tilt));
      const V3 r0 = norm(cross(fwd, {0, 1, 0}));
      const V3 u0 = cross(r0, fwd);
      look_at(out, eye, eye + fwd, deg(std::atan2(dot(cup, r0), dot(cup, u0))), 0.85);
      break;
    }
    case Cam::Chase: {
      // Smoothly lag the chase direction toward the velocity vector.
      chase_dir = norm(chase_dir * 0.96 + vdir * 0.04);
      const double dist = s.rocket_on ? 70.0 : 42.0;
      const V3 eye = ship - chase_dir * dist + V3{0, 9.0, 0};
      look_at(out, eye, ship + vdir * 6.0, 0, 1.0);
      break;
    }
    case Cam::Side: {
      const V3 right = norm(cross(vdir, {0, 1, 0}));
      const V3 eye = ship + right * 55.0 + vdir * 18.0 + V3{0, -6.0, 0};
      look_at(out, eye, ship, 0, 1.0);
      break;
    }
    case Cam::Carrier: {
      const V3 c = carrier_local();
      const auto cq = flight::from_euler(world.carrier.heading_deg, world.carrier.pitch_deg, world.carrier.roll_deg);
      const V3 eye = body_point(c, cq, {4.2, 2.6, 7.5});
      const V3 target = s.released ? ship : body_point(c, cq, {0, -0.6, -1.5});
      const double dist = std::sqrt(dot(target - eye, target - eye));
      look_at(out, eye, target, 0, clampv(dist / 60.0, 1.0, 12.0));
      break;
    }
    case Cam::Ground: {
      // Tracking camera 6 km beside the runway, telephoto on the ship.
      flight::Vec3 g{0, 0, 0};
      const double h = rad(fld.runway_heading_deg + 90.0);
      g.e = 6000.0 * std::sin(h) + 1500.0 * std::sin(rad(fld.runway_heading_deg));
      g.n = 6000.0 * std::cos(h) + 1500.0 * std::cos(rad(fld.runway_heading_deg));
      g.u = fld.elevation_m + 3.0;
      double x, y, z;
      local_of(g, x, y, z);
      const V3 eye{x, y, z};
      const double dist = std::sqrt(dot(ship - eye, ship - eye));
      look_at(out, eye, ship, 0, clampv(dist / 150.0, 1.0, 90.0));
      break;
    }
    case Cam::Tower: {
      flight::Vec3 g{0, 0, 0};
      const double rh = rad(fld.runway_heading_deg);
      const double along = ap.step == autopilot::Step::TakeoffRoll || ap.step == autopilot::Step::Hold ? 900.0 : 1300.0;
      g.e = along * std::sin(rh) + 260.0 * std::cos(rh);
      g.n = along * std::cos(rh) - 260.0 * std::sin(rh);
      g.u = fld.elevation_m + 6.0;
      double x, y, z;
      local_of(g, x, y, z);
      const V3 eye{x, y, z};
      const V3 tgt = s.released ? ship : carrier_local();
      const double dist = std::sqrt(dot(tgt - eye, tgt - eye));
      look_at(out, eye, tgt, 0, clampv(dist / 70.0, 1.0, 40.0));
      break;
    }
    case Cam::Probe: {
      if (!world.probe.deployed) {
        set_cam(Cam::Chase);
        break;
      }
      double x, y, z;
      local_of(world.probe.pos, x, y, z);
      const V3 p{x, y, z};
      // Camera on the probe's flank: the probe in the foreground, the ship
      // climbing away beyond it.
      const V3 pv = norm(gl_dir(world.probe.vel));
      const V3 side = norm(cross(pv, {0, 1, 0}));
      const V3 eye = p + side * 1.4 - pv * 1.6;
      const double dist = std::sqrt(dot(ship - p, ship - p));
      if (dist < 6000.0) look_at(out, eye, ship * 0.35 + p * 0.65, 0, 1.0);
      else look_at(out, eye, eye + pv, 0, 1.0);
      break;
    }
    case Cam::Flyby: {
      const double dist = std::sqrt(dot(ship - flyby_point, ship - flyby_point));
      if (!flyby_valid || dist > 1500.0) {
        const V3 right = norm(cross(vdir, {0, 1, 0}));
        flyby_point = ship + vdir * std::min(500.0, 12.0 * std::sqrt(dot(vel, vel))) + right * 35.0 + V3{0, -8, 0};
        flyby_valid = true;
      }
      look_at(out, flyby_point, ship, 0, clampv(dist / 90.0, 1.0, 8.0));
      break;
    }
    case Cam::Orbit:
    default: {
      orbit_angle += 0.15;
      const double a = rad(orbit_angle);
      const V3 eye = ship + V3{55.0 * std::sin(a), 14.0 + 6 * std::sin(a * 0.7), 55.0 * std::cos(a)};
      look_at(out, eye, ship, 0, 1.0);
      break;
    }
  }
  return 1;
}

void take_camera() {
  if (!camera_owned) {
    XPLMControlCamera(xplm_ControlCameraForever, camera_func, nullptr);
    camera_owned = true;
  }
}
void release_camera() {
  if (camera_owned) XPLMDontControlCamera();
  camera_owned = false;
}

// ---------------------------------------------------------- avionics glue
avionics::Snapshot snapshot() {
  avionics::Snapshot sn;
  const auto& s = world.ship;
  sn.met_s = met;
  sn.phase = flight::phase_name(s.phase);
  sn.step = autopilot::step_name(ap.step);
  sn.camera = cam_name(cam) + std::string(cam_auto ? "" : " (MANUAL)");
  sn.warp = ap.automatic ? ap.warp : 1.0;
  sn.automatic = ap.automatic;
  sn.pitch = s.pitch_deg;
  sn.roll = s.roll_deg;
  sn.heading = s.heading_deg;
  sn.alpha = s.alpha_deg;
  sn.beta = s.beta_deg;
  sn.keas = s.keas;
  sn.mach = s.mach;
  sn.max_mach = s.max_mach;
  sn.alt_ft = s.pos.u / flight::kFeetToMeters;
  sn.agl_ft = (s.pos.u - s.ground_elev_m) / flight::kFeetToMeters;
  sn.vs_fpm = s.vel.u / flight::kFeetToMeters * 60.0;
  sn.g = s.g_load;
  sn.gmax = s.max_g;
  sn.apogee_ft = s.apogee_m / flight::kFeetToMeters;
  sn.predicted_apogee_ft = flight::predicted_apogee(s) / flight::kFeetToMeters;
  sn.q_kpa = s.q_pa / 1000.0;
  sn.thrust_kn = s.thrust_n / 1000.0;
  sn.chamber_bar = s.chamber_bar;
  sn.ox_kg = s.oxidizer_kg;
  sn.burn_s = s.burn_s;
  sn.mass_kg = s.mass_kg;
  sn.rocket_on = s.rocket_on;
  sn.burned_out = s.burned_out;
  sn.released = s.released;
  sn.on_ground = s.on_ground;
  sn.feather = s.feather;
  sn.gear = s.gear;
  sn.feather_cmd = s.feather_cmd;
  sn.gear_cmd = s.gear_cmd;
  const auto& p = world.probe;
  sn.probe_deployed = p.deployed;
  sn.probe_chute = p.chute;
  sn.probe_landed = p.landed;
  sn.probe_alt_ft = p.pos.u / flight::kFeetToMeters;
  sn.probe_apogee_ft = p.apogee_m / flight::kFeetToMeters;
  sn.probe_density = probe_density;
  sn.probe_iwc = probe_iwc;
  sn.probe_max_density = p.max_density;
  sn.probe_in_layer = p.deployed && std::abs(p.pos.u - flight::kCloudAltM) < 600 && probe_density > 0.05;
  sn.ship_density = ship_density;
  sn.ship = {static_cast<float>(s.pos.e / 1000), static_cast<float>(s.pos.n / 1000)};
  sn.carrier = {static_cast<float>(world.carrier.pos.e / 1000), static_cast<float>(world.carrier.pos.n / 1000)};
  sn.probe = {static_cast<float>(p.pos.e / 1000), static_cast<float>(p.pos.n / 1000)};
  const auto g = autopilot::geometry(fld);
  sn.hac = {static_cast<float>(g.hac_center.e / 1000), static_cast<float>(g.hac_center.n / 1000)};
  sn.final_entry = {static_cast<float>(g.final_entry.e / 1000), static_cast<float>(g.final_entry.n / 1000)};
  sn.release = {static_cast<float>(g.release_point.e / 1000), static_cast<float>(g.release_point.n / 1000)};
  sn.hac_radius_km = g.hac_radius_m / 1000;
  sn.runway_heading = fld.runway_heading_deg;
  sn.runway_length_km = fld.runway_length_m / 1000;
  sn.carrier_alt_ft = world.carrier.pos.u / flight::kFeetToMeters;
  sn.field_name = kFields[field_index].name;
  sn.profile = ship_profile;
  sn.probe_profile = probe_profile;
  sn.plan = plan_profile;
  sn.density_history = density_hist;
  for (const auto& e : ap.events) sn.events.push_back(e.text);
  sn.caption = caption;
  sn.caption_age_s = wall - caption_time;
  return sn;
}

bool show_panel() { return active && displays_docked && cam == Cam::Cockpit; }

int draw_overlay(XPLMDrawingPhase, int, void*) {
  if (!active) return 1;
  int sw = 0, sh = 0;
  XPLMGetScreenSize(&sw, &sh);
  const auto sn = snapshot();
  const bool panel = show_panel() && !popout;
  if (panel) {
    const int ph = static_cast<int>(sh * 0.40);
    const int dw = std::min(sw / 3 - 10, static_cast<int>(ph * 1.5));
    const int total = dw * 3 + 20;
    const int x0 = (sw - total) / 2;
    // Glareshield/panel backdrop.
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
    avionics::draw_pfd(sn, x0, ph + 4, x0 + dw, 4);
    avionics::draw_nav(sn, x0 + dw + 10, ph + 4, x0 + 2 * dw + 10, 4);
    avionics::draw_sys(sn, x0 + 2 * dw + 20, ph + 4, x0 + 3 * dw + 20, 4);
  }
  avionics::draw_caption(sn, 0, sh, sw, panel ? static_cast<int>(sh * 0.40) + 8 : 0, !panel);
  return 1;
}

// Pop-out windows so the three displays can sit on three monitors.
std::array<XPLMWindowID, 3> windows{};
void draw_popout(XPLMWindowID id, void* ref) {
  if (!active) return;
  int l, t, r, b;
  XPLMGetWindowGeometry(id, &l, &t, &r, &b);
  const auto sn = snapshot();
  const auto which = reinterpret_cast<intptr_t>(ref);
  if (which == 0) avionics::draw_pfd(sn, l, t, r, b);
  else if (which == 1) avionics::draw_nav(sn, l, t, r, b);
  else avionics::draw_sys(sn, l, t, r, b);
}
int win_mouse(XPLMWindowID, int, int, XPLMMouseStatus, void*) { return 1; }
void win_key(XPLMWindowID, char, XPLMKeyFlags, char, void*, int) {}
XPLMCursorStatus win_cursor(XPLMWindowID, int, int, void*) { return xplm_CursorDefault; }
int win_wheel(XPLMWindowID, int, int, int, int, void*) { return 1; }

void toggle_popout() {
  popout = !popout;
  if (popout) {
    int l, t, r, b;
    XPLMGetScreenBoundsGlobal(&l, &t, &r, &b);
    for (int k = 0; k < 3; ++k) {
      if (!windows[k]) {
        XPLMCreateWindow_t p{};
        p.structSize = sizeof(p);
        p.left = l + 40 + k * 620;
        p.right = p.left + 600;
        p.top = t - 80;
        p.bottom = p.top - 420;
        p.visible = 1;
        p.drawWindowFunc = draw_popout;
        p.handleMouseClickFunc = win_mouse;
        p.handleRightClickFunc = win_mouse;
        p.handleKeyFunc = win_key;
        p.handleCursorFunc = win_cursor;
        p.handleMouseWheelFunc = win_wheel;
        p.refcon = reinterpret_cast<void*>(static_cast<intptr_t>(k));
        p.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
        p.layer = xplm_WindowLayerFloatingWindows;
        windows[k] = XPLMCreateWindowEx(&p);
        static const char* titles[] = {"PFD", "TRAJ / NAV", "SYS / PROBE"};
        XPLMSetWindowTitle(windows[k], titles[k]);
      }
      XPLMSetWindowIsVisible(windows[k], 1);
      XPLMSetWindowPositioningMode(windows[k], xplm_WindowPopOut, -1);
    }
  } else {
    for (auto& w : windows)
      if (w) XPLMSetWindowIsVisible(w, 0);
  }
}

// ---------------------------------------------------------------- mission
void compute_plan() {
  plan_profile.clear();
  auto w = flight::initial(fld.runway_heading_deg, fld.elevation_m);
  autopilot::State st;
  st.automatic = true;
  autopilot::start(st);
  const double hd = rad(fld.runway_heading_deg);
  double next = 0;
  for (int i = 0; i < 400000 && st.step != autopilot::Step::Done; ++i) {
    const auto out = autopilot::update(st, w, fld, {}, 0.05);
    flight::advance(w, out.controls, out.carrier, 0.05 * out.warp);
    if (w.ship.released && w.ship.time_s >= next) {
      const double along = w.ship.pos.e * std::sin(hd) + w.ship.pos.n * std::cos(hd);
      plan_profile.push_back({static_cast<float>(along / 1000), static_cast<float>(w.ship.pos.u / 1000)});
      next = w.ship.time_s + 2.0;
    }
  }
}

void set_environment() {
  // Late-summer Alaskan twilight (8 August, 22:15 AKDT): the ground is in
  // dusk while the mesosphere is still sunlit, the classic NLC geometry.
  if (dr_use_sys_time) XPLMSetDatai(dr_use_sys_time, 0);
  if (dr_date) XPLMSetDatai(dr_date, 219);
  if (dr_zulu) XPLMSetDataf(dr_zulu, 6.25f * 3600.0f);
  if (dr_wx_preset) XPLMSetDatai(dr_wx_preset, 0);
  if (dr_wx_now) XPLMSetDatai(dr_wx_now, 1);
}

// Build the invisible "IIAS SpaceShipTwo" container aircraft from the stock
// ASK 21 glider if it is missing (same result as tools/make_aircraft.py), so
// installing the plugin folder is all a user has to do on any platform.
bool ensure_shell_aircraft(const std::filesystem::path& xp_root, std::filesystem::path& acf_out) {
  namespace fs = std::filesystem;
  const fs::path dst = xp_root / "Aircraft" / "IIAS Noctilucent" / "SpaceShipTwo";
  acf_out = dst / "SpaceShipTwo.acf";
  if (fs::exists(acf_out)) return true;
  const fs::path src = xp_root / "Aircraft" / "Laminar Research" / "Schleicher ASK 21";
  std::ifstream in(src / "ASK21.acf");
  if (!in) return false;
  std::error_code ec;
  fs::create_directories(dst / "objects", ec);
  fs::copy(src / "Airfoils", dst / "Airfoils", fs::copy_options::recursive | fs::copy_options::skip_existing, ec);
  std::ofstream out(acf_out);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.rfind("P _obja/", 0) == 0) continue;  // no 3-D objects: invisible shell
    if (line.rfind("P acf/_name ", 0) == 0) line = "P acf/_name IIAS SpaceShipTwo (mission plugin)";
    else if (line.rfind("P acf/_ICAO ", 0) == 0) line = "P acf/_ICAO SS2";
    out << line << '\n';
  }
  XPLMDebugString("NLC: created the IIAS SpaceShipTwo container aircraft\n");
  return static_cast<bool>(out);
}

void begin_mission(bool automatic) {
  const FieldDef& f = kFields[field_index];
  fld = field_geometry(f);
  geo = Geo{f.thr_lat, f.thr_lon};
  wants_auto = automatic;
  // Load our aircraft so X-Plane draws SpaceShipTwo as the user vehicle.
  char file[512] = {}, path[2048] = {};
  XPLMGetNthAircraftModel(0, file, path);
  user_is_ss2 = std::string(file).find("SpaceShipTwo") != std::string::npos;
  if (!user_is_ss2) {
    char sys[1024] = {};
    XPLMGetSystemPath(sys);
    std::filesystem::path acf;
    if (ensure_shell_aircraft(std::filesystem::path(sys), acf)) XPLMSetUsersAircraft(acf.string().c_str());
  }
  XPLMPlaceUserAtLocation(f.thr_lat, f.thr_lon, static_cast<float>(f.elev_m + 3.0),
                          static_cast<float>(fld.runway_heading_deg), 0.0f);
  set_environment();
  pending_start = true;
  pending_timer = 0;
  XPLMDebugString("NLC: mission requested; waiting for scenery\n");
}

void finish_start() {
  pending_start = false;
  bool ok = false;
  double x, y, z;
  XPLMWorldToLocal(geo.lat0, geo.lon0, fld.elevation_m, &x, &y, &z);
  const double ground = terrain_msl({x, y + 500, z}, ok);
  // Prefer X-Plane's own scenery. Only when it has no terrain here (the free
  // demo shows water around Fairbanks) draw the bundled Eielson scenery.
  const bool builtin = ok && std::abs(ground - kFields[field_index].elev_m) < 80.0;
  scenery_active = !builtin && field_index <= 1 && obj_scenery[0] && scenery_hgt[0].n > 0;
  if (scenery_active) {
    place_scenery();
    double h;
    if (scenery_ground(geo.lat0, geo.lon0, h)) fld.elevation_m = h;
    XPLMDebugString("NLC: built-in terrain missing here; drawing bundled Eielson scenery\n");
  } else if (ok) {
    fld.elevation_m = ground;
  }
  world = flight::initial(fld.runway_heading_deg, fld.elevation_m);
  world.ship.gear = 0;  // mated: SpaceShipTwo gear stowed
  world.ship.gear_cmd = false;
  ap = autopilot::State{};
  ap.automatic = wants_auto;
  autopilot::start(ap);
  if (!wants_auto) ap.automatic = false;
  compute_plan();
  ship_profile.clear();
  probe_profile.clear();
  density_hist.clear();
  events_seen = 0;
  met = 0;
  caption = "WHITEKNIGHTTWO / SPACESHIPTWO -- " + std::string(kFields[field_index].name);
  caption_time = wall;
  for (auto& seg : trail) {
    seg.born = -1;
    place_instance(seg.inst, {0, -100000, 0}, 0, 0, 0, nullptr);
  }
  if (!dr_fov) dr_fov = XPLMFindDataRef("sim/graphics/view/field_of_view_deg");
  if (dr_fov) {
    const double f = XPLMGetDataf(dr_fov);
    if (f > 20 && f < 120) base_fov = f;
    applied_fov = -1;
  }
  // Art controls are registered after plugins start; look this up late.
  if (!dr_ev) dr_ev = XPLMFindDataRef("sim/private/controls/photometric/ev100");
  if (dr_ev) {
    const double e = XPLMGetDataf(dr_ev);
    if (e > 4.0 && e < 13.0) ev_base = e;
  }
  int on = 1;
  XPLMSetDatavi(dr_override, &on, 0, 1);
  // External view (no 2-D panel) under our camera; in autoflight the mouse is
  // not a yoke, which also hides X-Plane's mouse-flight box.
  if (XPLMCommandRef v = XPLMFindCommand("sim/view/chase")) XPLMCommandOnce(v);
  if (XPLMDataRef j = XPLMFindDataRef("sim/operation/override/override_joystick"))
    XPLMSetDatai(j, wants_auto ? 1 : 0);
  // The container aircraft's radios stay off: no ATC chatter windows.
  for (const char* r : {"sim/cockpit2/radios/actuators/com1_power", "sim/cockpit2/radios/actuators/com2_power",
                        "sim/cockpit2/switches/avionics_power_on", "sim/atc/atis_enabled"})
    if (XPLMDataRef d = XPLMFindDataRef(r)) XPLMSetDatai(d, 0);
  active = true;
  take_camera();
  const auto log_path = plugin_root / "probe_samples.csv";
  probe_log.open(log_path, std::ios::trunc);
  if (probe_log)
    probe_log << "met_s,source,latitude_deg,longitude_deg,altitude_m_msl,nlc_density,ice_water_ug_m2,in_layer,phase\n";
  char buf[256];
  std::snprintf(buf, sizeof buf, "NLC: mission started at %s, field elevation %.1f m (%s), %s\n",
                kFields[field_index].name, fld.elevation_m, ok ? "terrain" : "database",
                wants_auto ? "autoflight" : "manual");
  XPLMDebugString(buf);
}

void end_mission() {
  if (!active && !pending_start) return;
  active = false;
  pending_start = false;
  int off = 0;
  if (dr_override) XPLMSetDatavi(dr_override, &off, 0, 1);
  if (XPLMDataRef j = XPLMFindDataRef("sim/operation/override/override_joystick")) XPLMSetDatai(j, 0);
  if (dr_ev) XPLMSetDataf(dr_ev, static_cast<float>(ev_base));
  if (dr_fov) XPLMSetDataf(dr_fov, static_cast<float>(base_fov));
  release_camera();
  if (probe_log.is_open()) probe_log.close();
  place_instance(inst_wk2, {0, -100000, 0}, 0, 0, 0, nullptr);
  place_instance(inst_probe, {0, -100000, 0}, 0, 0, 0, nullptr);
  place_instance(inst_ss2, {0, -100000, 0}, 0, 0, 0, nullptr);
  place_instance(inst_macro, {0, -100000, 0}, 0, 0, 0, nullptr);
  for (auto& c : inst_detail) place_instance(c, {0, -100000, 0}, 0, 0, 0, nullptr);
  for (auto& c : inst_scenery) place_instance(c, {0, -100000, 0}, 0, 0, 0, nullptr);
  scenery_active = false;
  XPLMDebugString("NLC: mission stopped\n");
}

void update_trail(double now) {
  const auto& s = world.ship;
  const bool smoke = s.rocket_on && s.pos.u < 42000.0;
  const double dx = s.pos.e - last_trail_pos.e, dy = s.pos.n - last_trail_pos.n, dz = s.pos.u - last_trail_pos.u;
  if (smoke && std::sqrt(dx * dx + dy * dy + dz * dz) > 55.0 && !trail.empty()) {
    TrailSeg& seg = trail[trail_next];
    trail_next = (trail_next + 1) % trail.size();
    const flight::Vec3 mid{(s.pos.e + last_trail_pos.e) / 2, (s.pos.n + last_trail_pos.n) / 2,
                           (s.pos.u + last_trail_pos.u) / 2};
    geo.enu_to_ll(mid.e, mid.n, seg.lat, seg.lon);
    seg.alt = mid.u;
    seg.heading = deg(std::atan2(dx, dy));
    seg.pitch = deg(std::atan2(dz, std::hypot(dx, dy)));
    seg.born = now;
    last_trail_pos = s.pos;
  } else if (!smoke) {
    last_trail_pos = s.pos;
  }
  for (auto& seg : trail) {
    if (seg.born < 0) continue;
    const float age = static_cast<float>(now - seg.born);
    double x, y, z;
    XPLMWorldToLocal(seg.lat, seg.lon, seg.alt, &x, &y, &z);
    // Smoke spreads faster in thin air.
    const float scaled_age = age * static_cast<float>(1.0 + seg.alt / 12000.0);
    place_instance(seg.inst, {x, y, z}, seg.heading, seg.pitch, 0, &scaled_age);
    if (age > 400) seg.born = -1;
  }
}

void update_visuals(double dt) {
  const auto& s = world.ship;
  // SpaceShipTwo as the user aircraft.
  const V3 p = ship_local();
  XPLMSetDatad(dr_x, p.x);
  XPLMSetDatad(dr_y, p.y);
  XPLMSetDatad(dr_z, p.z);
  XPLMSetDataf(dr_vx, static_cast<float>(s.vel.e));
  XPLMSetDataf(dr_vy, static_cast<float>(s.vel.u));
  XPLMSetDataf(dr_vz, static_cast<float>(-s.vel.n));
  XPLMSetDataf(dr_psi, static_cast<float>(s.heading_deg));
  XPLMSetDataf(dr_theta, static_cast<float>(s.pitch_deg));
  XPLMSetDataf(dr_phi, static_cast<float>(s.roll_deg));
  set_quaternion(s.heading_deg, s.pitch_deg, s.roll_deg);
  v_feather = static_cast<float>(s.feather);
  v_gear = static_cast<float>(s.gear);
  v_plume = s.rocket_on ? (s.pos.u < 28000.0 ? 1.0f : 2.0f) : 0.0f;
  v_flicker = std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
  const float ss2_data[4] = {v_feather, v_gear, v_plume, v_flicker};
  if (!user_is_ss2 || debug_ss2_instance) place_instance(inst_ss2, p, s.heading_deg, s.pitch_deg, s.roll_deg, ss2_data);
  const float plume_data[2] = {v_plume, v_flicker};
  place_instance(inst_plume, v_plume > 0 ? p : V3{0, -100000, 0}, s.heading_deg, s.pitch_deg, s.roll_deg, plume_data);

  // WhiteKnightTwo.
  const auto& c = world.carrier;
  const float wk2_gear = c.pos.u - fld.elevation_m < 300.0 ? 1.0f : 0.0f;
  place_instance(inst_wk2, carrier_local(), c.heading_deg, c.pitch_deg, c.roll_deg, &wk2_gear);

  // Probe.
  if (world.probe.deployed) {
    double x, y, z;
    local_of(world.probe.pos, x, y, z);
    const float pdata[2] = {std::fmod(static_cast<float>(wall), 1.2f) < 0.12f ? 0.25f : 1.0f,
                            world.probe.chute ? 1.0f : 0.0f};
    const auto& pv = world.probe.vel;
    place_instance(inst_probe, {x, y, z}, deg(std::atan2(pv.e, pv.n)),
                   deg(std::atan2(pv.u, std::hypot(pv.e, pv.n))), 0, pdata);
  }
  update_trail(world.ship.time_s);
  (void)dt;
}

void update_science(double dt) {
  const auto& s = world.ship;
  double lat, lon;
  geo.enu_to_ll(s.pos.e, s.pos.n, lat, lon);
  ship_density = cloud_density_enu(s.pos.e, s.pos.n, s.pos.u);
  v_density = static_cast<float>(ship_density);
  // Optically thin: the layer reads only near it (or edge-on from close by).
  {
    double h = s.pos.u;
    if (cam == Cam::Probe && world.probe.deployed) h = std::max(h, world.probe.pos.u);
    v_nlc_visible = static_cast<float>(clampv((h - 52000.0) / 18000.0, 0.0, 1.0));
  }
  v_in_layer = std::abs(s.pos.u - flight::kCloudAltM) < 600 ? 1 : 0;
  auto& pr = world.probe;
  double plat = lat, plon = lon;
  if (pr.deployed) {
    geo.enu_to_ll(pr.pos.e, pr.pos.n, plat, plon);
    probe_density = cloud_density_enu(pr.pos.e, pr.pos.n, pr.pos.u);
    probe_iwc = cloud_iwc_enu(pr.pos.e, pr.pos.n, pr.pos.u);
    pr.max_density = std::max(pr.max_density, probe_density);
  }
  hist_timer += dt;
  log_timer += dt;
  const double hd = rad(fld.runway_heading_deg);
  if (hist_timer >= 1.0) {
    hist_timer = 0;
    if (s.released) {
      const double along = s.pos.e * std::sin(hd) + s.pos.n * std::cos(hd);
      ship_profile.push_back({static_cast<float>(along / 1000), static_cast<float>(s.pos.u / 1000)});
      if (pr.deployed && !pr.landed) {
        const double pa = pr.pos.e * std::sin(hd) + pr.pos.n * std::cos(hd);
        probe_profile.push_back({static_cast<float>(pa / 1000), static_cast<float>(pr.pos.u / 1000)});
      }
    }
  }
  if (pr.deployed && !pr.landed) {
    density_hist.push_back(static_cast<float>(probe_density));
    if (density_hist.size() > 400) density_hist.erase(density_hist.begin());
  }
  if (probe_log.is_open() && log_timer >= 0.5) {
    log_timer = 0;
    probe_log << std::fixed << std::setprecision(6) << met << ",ship," << lat << ',' << lon << ','
              << std::setprecision(1) << s.pos.u << ',' << std::setprecision(4) << ship_density << ','
              << std::setprecision(1) << cloud_iwc_enu(s.pos.e, s.pos.n, s.pos.u) << ',' << v_in_layer << ',' << flight::phase_name(s.phase) << '\n';
    if (pr.deployed && !pr.landed)
      probe_log << std::fixed << std::setprecision(6) << met << ",probe," << plat << ',' << plon << ','
                << std::setprecision(1) << pr.pos.u << ',' << std::setprecision(4) << probe_density << ','
                << std::setprecision(1) << probe_iwc << ',' << (std::abs(pr.pos.u - flight::kCloudAltM) < 600 ? 1 : 0) << ",PROBE\n";
  }
}

flight::Controls read_pilot() {
  flight::Controls c;
  if (dr_yoke_p) c.pitch = XPLMGetDataf(dr_yoke_p);
  if (dr_yoke_r) c.roll = XPLMGetDataf(dr_yoke_r);
  if (dr_yoke_h) c.yaw = XPLMGetDataf(dr_yoke_h);
  return c;
}

float flight_loop(float elapsed, float, int, void*) {
  const double frame = clampv(static_cast<double>(elapsed), 0.0, 0.25);
  wall += frame;
  // Glow of the mesospheric layer: strongest in twilight, still visible by day.
  if (dr_sun) {
    const double sun = XPLMGetDataf(dr_sun);
    v_glow = static_cast<float>(clampv(0.45 + 0.55 * (8.0 - sun) / 14.0, 0.45, 1.0));
  }
  if (pending_start) {
    pending_timer += frame;
    if (pending_timer > 3.0) finish_start();
    return -1.0f;
  }
  if (!active) return -1.0f;
  place_clouds(world.ship.pos.e, world.ship.pos.n);
  if (dr_paused && XPLMGetDatai(dr_paused)) return -1.0f;

  // Terrain under the ship (and the carrier while on the runway).
  bool ok = false;
  double g = 0;
  if (scenery_active) {
    double slat, slon;
    geo.enu_to_ll(world.ship.pos.e, world.ship.pos.n, slat, slon);
    ok = scenery_ground(slat, slon, g);
  } else {
    g = terrain_msl(ship_local(), ok);
  }
  if (ok && world.ship.released) world.ship.ground_elev_m = g;

  const auto out = autopilot::update(ap, world, fld, read_pilot(), frame);
  ap.warp = out.warp;
  const double dt = frame * out.warp;
  met += dt;
  flight::advance(world, out.controls, out.carrier, dt);
  for (; events_seen < ap.events.size(); ++events_seen) {
    caption = ap.events[events_seen].text;
    caption_time = wall;
    char stamp[32];
    std::snprintf(stamp, sizeof stamp, " [wall %.1f]", wall);
    XPLMDebugString(("NLC: " + caption + stamp + "\n").c_str());
  }
  if (ap.events.size() < events_seen) events_seen = ap.events.size();
  update_science(dt);
  update_visuals(dt);
  if (cam_auto) direct_camera();
  if (dr_fov && std::abs(wanted_fov - applied_fov) > 0.01) {
    XPLMSetDataf(dr_fov, static_cast<float>(wanted_fov));
    applied_fov = wanted_fov;
  }
  if (scenery_active) place_scenery();
  // Exposure: X-Plane's auto-exposure brightens toward the black sky above
  // the atmosphere and blows out the sunlit hull, so the mission sets a
  // photographic exposure: dusk at low altitude, full sunlight in space.
  if (dr_ev) {
    double h = world.ship.pos.u;
    if (cam == Cam::Probe && world.probe.deployed) h = world.probe.pos.u;
    const double k = clampv((h - 15000.0) / 27000.0, 0.0, 1.0);
    const double smooth = k * k * (3 - 2 * k);
    XPLMSetDataf(dr_ev, static_cast<float>(ev_base + (v_ev_space - ev_base) * smooth + v_ev_offset));
  }
  take_camera();
  return -1.0f;
}

// ---------------------------------------------------------------- commands
enum Cmd {
  kStartAuto, kStartManual, kStop, kRelease, kIgnite, kCutoff, kFeather, kGear, kProbe, kCamNext, kCamAuto,
  kDisplays, kPopout, kBase0, kBase1, kBase2, kJump, kCmdCount
};
std::array<XPLMCommandRef, kCmdCount> cmds{};

int command_handler(XPLMCommandRef, XPLMCommandPhase phase, void* ref) {
  if (phase != xplm_CommandBegin) return 0;
  const auto which = static_cast<int>(reinterpret_cast<intptr_t>(ref));
  switch (which) {
    case kStartAuto: end_mission(); begin_mission(true); break;
    case kStartManual: end_mission(); begin_mission(false); break;
    case kStop: end_mission(); break;
    case kRelease:
      if (active && flight::release(world)) autopilot::log_event(ap, world, "RELEASE (pilot)");
      break;
    case kIgnite:
      if (active && flight::ignite(world)) autopilot::log_event(ap, world, "IGNITION (pilot)");
      break;
    case kCutoff:
      if (active) flight::cutoff(world);
      break;
    case kFeather:
      if (active) flight::set_feather(world, !world.ship.feather_cmd);
      break;
    case kGear:
      if (active) flight::set_gear(world, !world.ship.gear_cmd);
      break;
    case kProbe:
      if (active && flight::deploy_probe(world)) autopilot::log_event(ap, world, "PROBE DEPLOYED (pilot)");
      break;
    case kCamNext:
      cam_auto = false;
      set_cam(static_cast<Cam>((static_cast<int>(cam) + 1) % static_cast<int>(Cam::Count)));
      break;
    case kCamAuto: cam_auto = true; break;
    case kDisplays: displays_docked = !displays_docked; break;
    case kPopout: toggle_popout(); break;
    case kBase0: case kBase1: case kBase2:
      field_index = which - kBase0;
      XPLMDebugString((std::string("NLC: base set to ") + kFields[field_index].label + "\n").c_str());
      break;
    case kJump:
      if (active && !world.ship.released) {
        // Put the mated pair on the aligned run-in, 50,000 ft, 45 km out.
        const double h = rad(fld.runway_heading_deg);
        world.carrier.pos = {-45000.0 * std::sin(h), -45000.0 * std::cos(h), flight::kReleaseAltM};
        world.carrier.heading_deg = fld.runway_heading_deg;
        world.carrier.airborne = true;
        world.carrier.tas_mps = 200;
        world.carrier.climb_mps = 0;
        world.carrier.roll_deg = 0;
        ap.step = autopilot::Step::RunIn;
        ap.step_time_s = 0;
        ap.runin_captured = true;
        met = 2900;
        autopilot::log_event(ap, world, "TEST JUMP to release run-in");
      }
      break;
    default: break;
  }
  return 0;
}

XPLMMenuID menu = nullptr;

float rd_f(void* ref) { return *static_cast<float*>(ref); }
void wr_f(void* ref, float v) { *static_cast<float*>(ref) = v; }
int rd_i(void* ref) { return *static_cast<int*>(ref); }
float rd_layer(void*) { return static_cast<float>(flight::kCloudAltM); }
float rd_alt(void*) { return static_cast<float>(world.ship.pos.u); }
int rd_phase(void*) { return active ? static_cast<int>(world.ship.phase) : -1; }

void reg_rw(const char* name, float* v) {
  owned_refs.push_back(XPLMRegisterDataAccessor(name, xplmType_Float, 1, nullptr, nullptr, rd_f, wr_f, nullptr,
                                                nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, v, v));
}

void reg_f(const char* name, float* v) {
  owned_refs.push_back(XPLMRegisterDataAccessor(name, xplmType_Float, 0, nullptr, nullptr, rd_f, nullptr, nullptr,
                                                nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, v,
                                                nullptr));
}

}  // namespace

PLUGIN_API int XPluginStart(char* name, char* sig, char* desc) {
  std::strcpy(name, "IIAS Noctilucent Mission");
  std::strcpy(sig, "org.xplaneiias.noctilucentclouds");
  std::strcpy(desc, "WhiteKnightTwo / SpaceShipTwo air-launch to the 270,000 ft noctilucent cloud layer");
  XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
  char path[2048] = {};
  XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
  plugin_root = std::filesystem::path(path).parent_path().parent_path();
  {
    std::ifstream in(plugin_root / "assets" / "nlc_real.bin", std::ios::binary);
    int n = 0;
    float hdr[4] = {};
    if (in.read(reinterpret_cast<char*>(&n), 4) && in.read(reinterpret_cast<char*>(hdr), 16) && n > 1 && n < 2000) {
      cloud.n = n;
      cloud.macro_m = hdr[0];
      cloud.detail_m = hdr[1];
      cloud.off_e = hdr[2];
      cloud.off_n = hdr[3];
      cloud.albedo.resize(n * n);
      cloud.iwc.resize(n * n);
      cloud.detail.resize(1024 * 1024);
      in.read(reinterpret_cast<char*>(cloud.albedo.data()), n * n * 4);
      in.read(reinterpret_cast<char*>(cloud.iwc.data()), n * n * 4);
      in.read(reinterpret_cast<char*>(cloud.detail.data()), 1024 * 1024);
      if (!in) cloud = CloudData{};
    }
    static const char* names[3] = {"eielson_l0.hgt", "eielson_l1.hgt", "eielson_l2.hgt"};
    for (int k = 0; k < 3; ++k) {
      std::ifstream h(plugin_root / "assets" / "scenery" / names[k], std::ios::binary);
      int gn = 0;
      float gh[2] = {};
      if (h.read(reinterpret_cast<char*>(&gn), 4) && h.read(reinterpret_cast<char*>(gh), 8) && gn > 1 && gn < 5000) {
        scenery_hgt[k].n = gn;
        scenery_hgt[k].half = gh[0];
        scenery_hgt[k].step = gh[1];
        scenery_hgt[k].h.resize(static_cast<size_t>(gn) * gn);
        h.read(reinterpret_cast<char*>(scenery_hgt[k].h.data()), static_cast<std::streamsize>(gn) * gn * 2);
        if (!h) scenery_hgt[k] = HeightGrid{};
      }
    }
  }
  auto F = [](const char* n) { return XPLMFindDataRef(n); };
  dr_override = F("sim/operation/override/override_planepath");
  dr_x = F("sim/flightmodel/position/local_x");
  dr_y = F("sim/flightmodel/position/local_y");
  dr_z = F("sim/flightmodel/position/local_z");
  dr_vx = F("sim/flightmodel/position/local_vx");
  dr_vy = F("sim/flightmodel/position/local_vy");
  dr_vz = F("sim/flightmodel/position/local_vz");
  dr_psi = F("sim/flightmodel/position/psi");
  dr_theta = F("sim/flightmodel/position/theta");
  dr_phi = F("sim/flightmodel/position/phi");
  dr_q = F("sim/flightmodel/position/q");
  dr_lat = F("sim/flightmodel/position/latitude");
  dr_lon = F("sim/flightmodel/position/longitude");
  dr_elev = F("sim/flightmodel/position/elevation");
  dr_paused = F("sim/time/paused");
  dr_sun = F("sim/graphics/scenery/sun_pitch_degrees");
  dr_yoke_p = F("sim/joystick/yoke_pitch_ratio");
  dr_yoke_r = F("sim/joystick/yoke_roll_ratio");
  dr_yoke_h = F("sim/joystick/yoke_heading_ratio");
  dr_date = F("sim/time/local_date_days");
  dr_zulu = F("sim/time/zulu_time_sec");
  dr_use_sys_time = F("sim/time/use_system_time");
  dr_wx_preset = F("sim/weather/region/weather_preset");
  dr_wx_now = F("sim/weather/region/update_immediately");
  reg_f("iias/ss2/feather", &v_feather);
  reg_f("iias/ss2/gear", &v_gear);
  reg_f("iias/ss2/plume_mode", &v_plume);
  reg_f("iias/ss2/plume_flicker", &v_flicker);
  reg_f("iias/nlc/glow", &v_glow);
  reg_f("iias/const/one", &v_one);
  reg_f("iias/nlc/visible", &v_nlc_visible);
  reg_rw("iias/tune/ev_offset", &v_ev_offset);
  reg_rw("iias/tune/ev_space", &v_ev_space);
  reg_f("iias/nlc/density", &v_density);
  owned_refs.push_back(XPLMRegisterDataAccessor("iias/nlc/in_layer", xplmType_Int, 0, rd_i, nullptr, nullptr, nullptr,
                                                nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                                                nullptr, &v_in_layer, nullptr));
  owned_refs.push_back(XPLMRegisterDataAccessor("iias/nlc/layer_altitude_m", xplmType_Float, 0, nullptr, nullptr,
                                                rd_layer, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                                                nullptr, nullptr, nullptr, nullptr, nullptr));
  owned_refs.push_back(XPLMRegisterDataAccessor("iias/mission/altitude_m", xplmType_Float, 0, nullptr, nullptr,
                                                rd_alt, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                                                nullptr, nullptr, nullptr, nullptr, nullptr));
  owned_refs.push_back(XPLMRegisterDataAccessor("iias/mission/phase", xplmType_Int, 0, rd_phase, nullptr, nullptr,
                                                nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                                                nullptr, nullptr, nullptr, nullptr));

  struct CmdDef { const char* id; const char* label; };
  static const CmdDef defs[kCmdCount] = {
      {"iias/mission/start_autoflight", "Start mission (autoflight)"},
      {"iias/mission/start_manual", "Start mission (pilot flies after release)"},
      {"iias/mission/stop", "Stop mission"},
      {"iias/mission/release", "Release SpaceShipTwo"},
      {"iias/mission/ignite", "Ignite rocket motor"},
      {"iias/mission/cutoff", "Rocket motor shutdown"},
      {"iias/mission/feather", "Toggle feather"},
      {"iias/mission/gear", "Toggle landing gear"},
      {"iias/mission/deploy_probe", "Deploy probe"},
      {"iias/mission/camera_next", "Next camera"},
      {"iias/mission/camera_auto", "Automatic camera director"},
      {"iias/mission/toggle_displays", "Show/hide cockpit displays"},
      {"iias/mission/popout_displays", "Pop out the three displays (multi-monitor)"},
      {"iias/mission/base_eielson", "Base: Eielson AFB"},
      {"iias/mission/base_fairbanks", "Base: Fairbanks Intl"},
      {"iias/mission/base_juneau", "Base: Juneau (demo scenery)"},
      {"iias/mission/jump_to_release", "Test: jump to the 50,000 ft release run-in"},
  };
  XPLMMenuID plugins = XPLMFindPluginsMenu();
  const int item = XPLMAppendMenuItem(plugins, "IIAS Noctilucent Mission", nullptr, 1);
  menu = XPLMCreateMenu("IIAS Noctilucent Mission", plugins, item, nullptr, nullptr);
  for (int k = 0; k < kCmdCount; ++k) {
    cmds[k] = XPLMCreateCommand(defs[k].id, defs[k].label);
    XPLMRegisterCommandHandler(cmds[k], command_handler, 1, reinterpret_cast<void*>(static_cast<intptr_t>(k)));
    if (menu) XPLMAppendMenuItemWithCommand(menu, defs[k].label, cmds[k]);
  }
  XPLMRegisterDrawCallback(draw_overlay, xplm_Phase_Window, 0, nullptr);
  terrain_probe = XPLMCreateProbe(xplm_ProbeY);
  XPLMDebugString("NLC: plugin started\n");
  return 1;
}

PLUGIN_API void XPluginStop() {
  end_mission();
  XPLMUnregisterDrawCallback(draw_overlay, xplm_Phase_Window, 0, nullptr);
  for (int k = 0; k < kCmdCount; ++k)
    if (cmds[k])
      XPLMUnregisterCommandHandler(cmds[k], command_handler, 1, reinterpret_cast<void*>(static_cast<intptr_t>(k)));
  for (auto r : owned_refs)
    if (r) XPLMUnregisterDataAccessor(r);
  for (auto& w : windows)
    if (w) XPLMDestroyWindow(w);
  if (menu) XPLMDestroyMenu(menu);
  if (terrain_probe) XPLMDestroyProbe(terrain_probe);
}

namespace {
void destroy_objects() {
  auto kill = [](XPLMInstanceRef& i) {
    if (i) XPLMDestroyInstance(i);
    i = nullptr;
  };
  kill(inst_wk2);
  kill(inst_ss2);
  kill(inst_probe);
  kill(inst_plume);
  kill(inst_macro);
  for (auto& c : inst_detail) kill(c);
  for (auto& c : inst_scenery) kill(c);
  for (auto& t : trail) kill(t.inst);
  trail.clear();
  for (auto& o : obj_scenery) {
    if (o) XPLMUnloadObject(o);
    o = nullptr;
  }
  for (XPLMObjectRef* o : {&obj_macro, &obj_detail, &obj_wk2, &obj_ss2, &obj_probe, &obj_trail, &obj_plume}) {
    if (*o) XPLMUnloadObject(*o);
    *o = nullptr;
  }
}
}  // namespace

PLUGIN_API int XPluginEnable() {
  auto load = [](const char* file) {
    return XPLMLoadObject((plugin_root / "assets" / file).string().c_str());
  };
  obj_macro = load("nlc_macro.obj");
  obj_detail = load("nlc_detail.obj");
  static const char* scn[4] = {"scenery/eielson_l0.obj", "scenery/eielson_l1.obj", "scenery/eielson_l2.obj",
                               "scenery/runway.obj"};
  for (int k = 0; k < 4; ++k) obj_scenery[k] = load(scn[k]);  // optional
  obj_wk2 = load("whiteknight2.obj");
  obj_ss2 = load("spaceship2.obj");
  obj_probe = load("probe.obj");
  obj_trail = load("trail.obj");
  obj_plume = load("plume.obj");
  if (!obj_macro || !obj_detail || !obj_wk2 || !obj_ss2 || !obj_probe || !obj_trail || !obj_plume) {
    XPLMDebugString("NLC: could not load assets\n");
    destroy_objects();
    return 0;
  }
  const char* none[] = {nullptr};
  const char* wk2_refs[] = {"iias/wk2/gear", nullptr};
  const char* ss2_refs[] = {"iias/ss2/feather", "iias/ss2/gear", "iias/ss2/plume_mode", "iias/ss2/plume_flicker",
                            nullptr};
  const char* probe_refs[] = {"iias/probe/strobe", "iias/probe/chute", nullptr};
  const char* trail_refs[] = {"iias/trail/age", nullptr};
  inst_macro = XPLMCreateInstance(obj_macro, none);
  const char* detail_refs[] = {"iias/nlc/detail_level", nullptr};
  for (auto& c : inst_detail) c = XPLMCreateInstance(obj_detail, detail_refs);
  for (int k = 0; k < 4; ++k)
    if (obj_scenery[k]) inst_scenery[k] = XPLMCreateInstance(obj_scenery[k], none);
  inst_wk2 = XPLMCreateInstance(obj_wk2, wk2_refs);
  inst_ss2 = XPLMCreateInstance(obj_ss2, ss2_refs);
  inst_probe = XPLMCreateInstance(obj_probe, probe_refs);
  const char* plume_refs[] = {"iias/ss2/plume_mode", "iias/ss2/plume_flicker", nullptr};
  inst_plume = XPLMCreateInstance(obj_plume, plume_refs);
  trail.resize(900);
  for (auto& t : trail) t.inst = XPLMCreateInstance(obj_trail, trail_refs);
  const V3 hidden{0, -100000, 0};
  place_instance(inst_wk2, hidden, 0, 0, 0, nullptr);
  place_instance(inst_ss2, hidden, 0, 0, 0, nullptr);
  place_instance(inst_probe, hidden, 0, 0, 0, nullptr);
  place_instance(inst_plume, hidden, 0, 0, 0, nullptr);
  place_instance(inst_macro, hidden, 0, 0, 0, nullptr);
  for (auto& c : inst_detail) place_instance(c, hidden, 0, 0, 0, nullptr);
  for (auto& c : inst_scenery) place_instance(c, hidden, 0, 0, 0, nullptr);
  for (auto& t : trail) place_instance(t.inst, hidden, 0, 0, 0, nullptr);
  XPLMRegisterFlightLoopCallback(flight_loop, -1.0f, nullptr);
  XPLMDebugString("NLC: assets loaded; cloud layer at 82.3 km enabled\n");
  return 1;
}

PLUGIN_API void XPluginDisable() {
  end_mission();
  XPLMUnregisterFlightLoopCallback(flight_loop, nullptr);
  destroy_objects();
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID, int msg, void* param) {
  if (msg == XPLM_MSG_PLANE_LOADED && reinterpret_cast<intptr_t>(param) == 0) {
    char file[512] = {}, path[2048] = {};
    XPLMGetNthAircraftModel(0, file, path);
    user_is_ss2 = std::string(file).find("SpaceShipTwo") != std::string::npos;
  }
}
