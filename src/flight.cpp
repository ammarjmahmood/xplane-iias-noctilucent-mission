#include "flight.hpp"

#include <algorithm>
#include <cmath>

namespace flight {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kG0 = 9.80665;
constexpr double kEarthR = 6371000.0;
double rad(double d) { return d * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }
double clamp(double v, double lo, double hi) { return std::clamp(v, lo, hi); }

// SpaceShipTwo-class figures from public material: ~9,740 kg loaded,
// ~60 s hybrid burn, Mach ~3, apogee ~85-90 km. Dimensions: 18.3 m long,
// 8.3 m span.
constexpr double kShipDryKg = 4940.0;    // structure, crew, payload
constexpr double kOxidizerKg = 4800.0;   // N2O + fuel grain consumed
constexpr double kThrustVacN = 205000.0;
constexpr double kNozzleExitM2 = 0.9;
constexpr double kIspVacS = 255.0;
constexpr double kWingAreaM2 = 27.0;
constexpr double kAspect = 8.3 * 8.3 / kWingAreaM2;

Vec3 add(Vec3 a, Vec3 b) { return {a.e + b.e, a.n + b.n, a.u + b.u}; }
Vec3 sub(Vec3 a, Vec3 b) { return {a.e - b.e, a.n - b.n, a.u - b.u}; }
Vec3 mul(Vec3 a, double s) { return {a.e * s, a.n * s, a.u * s}; }
double dot(Vec3 a, Vec3 b) { return a.e * b.e + a.n * b.n + a.u * b.u; }
double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
Vec3 unit(Vec3 a) {
  const double m = norm(a);
  return m > 1e-9 ? mul(a, 1.0 / m) : Vec3{0, 0, 0};
}

Quat qmul(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
Quat qnorm(Quat q) {
  const double m = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return {q.w / m, q.x / m, q.y / m, q.z / m};
}
Quat axis_angle(double x, double y, double z, double angle_rad) {
  const double s = std::sin(angle_rad / 2);
  return {std::cos(angle_rad / 2), x * s, y * s, z * s};
}
Vec3 rotate(const Quat& q, double x, double y, double z) {
  // v' = q v q*
  const Quat v{0, x, y, z};
  const Quat c{q.w, -q.x, -q.y, -q.z};
  const Quat r = qmul(qmul(q, v), c);
  return {r.x, r.y, r.z};
}

// Integrate body rates (deg/s) in the pilot's sense.
void rotate_body(Quat& q, double pitch_up, double roll_right, double yaw_right, double dt) {
  const double wx = rad(roll_right), wy = rad(-pitch_up), wz = rad(-yaw_right);
  const Quat w{0, wx, wy, wz};
  const Quat d = qmul(q, w);
  q.w += 0.5 * d.w * dt;
  q.x += 0.5 * d.x * dt;
  q.y += 0.5 * d.y * dt;
  q.z += 0.5 * d.z * dt;
  q = qnorm(q);
}

double cd0_for_mach(double m) {
  if (m < 0.85) return 0.032;
  if (m < 1.1) return 0.032 + (0.090 - 0.032) * (m - 0.85) / 0.25;
  if (m < 3.5) return 0.090 - (0.090 - 0.048) * (m - 1.1) / 2.4;
  return 0.048;
}

double lift_slope(double m) {
  if (m < 0.9) return 3.3 / std::sqrt(std::max(0.2, 1.0 - m * m * 0.5));
  if (m < 1.2) return 3.6;
  return clamp(4.0 / std::sqrt(m * m - 1.0), 1.1, 3.6);
}

double clean_cl(double alpha_rad, double mach) {
  const double cla = lift_slope(mach);
  const double a_stall = rad(19.0);
  const double a = std::abs(alpha_rad);
  double cl = cla * a;
  if (a > a_stall) cl = cla * a_stall * std::max(0.55, 1.0 - 1.6 * (a - a_stall));
  return std::copysign(std::min(cl, 1.15), alpha_rad);
}

void update_air_data(Ship& s) {
  const auto atm = atmosphere(s.pos.u);
  s.tas = norm(s.vel);
  s.q_pa = 0.5 * atm.rho * s.tas * s.tas;
  s.mach = s.tas / atm.sound_mps;
  s.keas = s.tas * std::sqrt(atm.rho / 1.225) / 0.514444;
  s.max_mach = std::max(s.max_mach, s.mach);
}

void derive_euler(Ship& s) { to_euler(s.att, s.heading_deg, s.pitch_deg, s.roll_deg); }

void carrier_step(World& w, const CarrierCommand& cc, double dt) {
  auto& c = w.carrier;
  const double ground = w.ship.ground_elev_m;
  const auto atm = atmosphere(c.pos.u);
  if (!c.airborne) {
    if (cc.takeoff) c.tas_mps = std::min(80.0, c.tas_mps + 2.3 * dt);
    c.pos.u = ground + 2.6;
    c.climb_mps = 0;
    c.pitch_deg = c.tas_mps > 66.0 ? std::min(8.0, c.pitch_deg + 3.0 * dt) : 0.0;
    if (c.tas_mps >= 75.0) c.airborne = true;
  } else {
    // WhiteKnightTwo climbs at roughly constant equivalent airspeed; the
    // climb rate tapers with altitude so 50,000 ft takes about 50 minutes.
    const double eas = 88.0;
    const double tas_target = std::min(205.0, eas * std::sqrt(1.225 / atm.rho));
    c.tas_mps += clamp(tas_target - c.tas_mps, -1.5 * dt, 1.5 * dt);
    const double alt_err = cc.target_alt_m - c.pos.u;
    const double max_climb = alt_err > 0
        ? clamp(9.5 - 6.5 * (c.pos.u / 15000.0), 2.5, 9.5)
        : 12.0;
    const double want = clamp(alt_err * 0.05, -max_climb, max_climb);
    c.climb_mps += clamp(want - c.climb_mps, -0.6 * dt, 0.6 * dt);
    double err = std::fmod(cc.heading_deg - c.heading_deg + 540.0, 360.0) - 180.0;
    const double bank = clamp(err * 1.2, -25.0, 25.0);
    c.roll_deg += clamp(bank - c.roll_deg, -5.0 * dt, 5.0 * dt);
    const double turn = deg(kG0 * std::tan(rad(c.roll_deg)) / std::max(c.tas_mps, 30.0));
    c.heading_deg = std::fmod(c.heading_deg + turn * dt + 360.0, 360.0);
    const double gamma = deg(std::asin(clamp(c.climb_mps / std::max(c.tas_mps, 1.0), -1.0, 1.0)));
    c.pitch_deg += clamp(gamma + 3.0 - c.pitch_deg, -2.0 * dt, 2.0 * dt);
    c.pos.u += c.climb_mps * dt;
  }
  const double h = std::sqrt(std::max(0.0, c.tas_mps * c.tas_mps - c.climb_mps * c.climb_mps));
  c.pos.e += h * std::sin(rad(c.heading_deg)) * dt;
  c.pos.n += h * std::cos(rad(c.heading_deg)) * dt;
}

void mated_ship(World& w) {
  auto& s = w.ship;
  const auto& c = w.carrier;
  s.att = from_euler(c.heading_deg, c.pitch_deg, c.roll_deg);
  // SpaceShipTwo hangs beneath the centre wing between the twin fuselages.
  const Vec3 up = body_up(s.att);
  s.pos = add(c.pos, mul(up, -0.6));
  const double h = std::sqrt(std::max(0.0, c.tas_mps * c.tas_mps - c.climb_mps * c.climb_mps));
  s.vel = {h * std::sin(rad(c.heading_deg)), h * std::cos(rad(c.heading_deg)), c.climb_mps};
  s.alpha_deg = 3.0;
  s.beta_deg = 0;
  s.g_load = 1.0;
  s.on_ground = !c.airborne;
  s.phase = c.airborne ? Phase::MatedClimb : Phase::Ground;
}

void probe_step(World& w, double dt) {
  auto& p = w.probe;
  if (!p.deployed || p.landed) return;
  const auto atm = atmosphere(p.pos.u);
  const double v = norm(p.vel);
  if (!p.chute && p.vel.u < 0 && p.pos.u < 6000.0) p.chute = true;
  const double cda = p.chute ? 9.0 : 0.018;
  const double mass = 14.0;
  const Vec3 drag = mul(unit(p.vel), -0.5 * atm.rho * v * v * cda / mass);
  Vec3 acc = drag;
  acc.u -= gravity(p.pos.u);
  p.vel = add(p.vel, mul(acc, dt));
  p.pos = add(p.pos, mul(p.vel, dt));
  p.apogee_m = std::max(p.apogee_m, p.pos.u);
  if (p.pos.u <= w.ship.ground_elev_m) {
    p.pos.u = w.ship.ground_elev_m;
    p.vel = {0, 0, 0};
    p.landed = true;
  }
}

void ship_step(World& w, const Controls& in, double dt) {
  auto& s = w.ship;
  const auto atm = atmosphere(s.pos.u);
  update_air_data(s);

  // Feather and gear actuators.
  s.feather += clamp((s.feather_cmd ? 1.0 : 0.0) - s.feather, -dt / 12.0, dt / 12.0);
  s.gear += clamp((s.gear_cmd ? 1.0 : 0.0) - s.gear, -dt / 4.0, dt / 4.0);

  // Propulsion: hybrid motor with a short start transient.
  s.thrust_n = 0;
  if (s.rocket_on) {
    if (s.oxidizer_kg <= 0) {
      s.rocket_on = false;
      s.burned_out = true;
    } else {
      const double start = clamp(s.burn_s / 1.5, 0.0, 1.0);
      const double vac = kThrustVacN * start * (1.0 - 0.06 * s.burn_s / 60.0);
      s.thrust_n = std::max(0.0, vac - atm.pressure_pa * kNozzleExitM2);
      const double mdot = vac / (kIspVacS * kG0);
      s.oxidizer_kg = std::max(0.0, s.oxidizer_kg - mdot * dt);
      s.burn_s += dt;
      s.chamber_bar = 48.0 * start * (1.0 - 0.12 * s.burn_s / 60.0);
    }
  }
  if (!s.rocket_on) s.chamber_bar += (0.0 - s.chamber_bar) * std::min(1.0, 4.0 * dt);
  s.mass_kg = kShipDryKg + s.oxidizer_kg;

  const Vec3 f = body_forward(s.att), l = body_left(s.att), u = body_up(s.att);
  const Vec3 vhat = unit(s.vel);
  double alpha = 0, beta = 0;
  if (s.tas > 1.0) {
    alpha = std::atan2(-dot(vhat, u), dot(vhat, f));
    beta = std::atan2(dot(vhat, l), dot(vhat, f));
  }
  s.alpha_deg = deg(alpha);
  s.beta_deg = deg(beta);

  // Aerodynamics, blended between the clean glider and the feathered
  // "shuttlecock" configuration (booms raised ~60 deg; high-drag, self-
  // righting belly-first descent).
  const double fe = s.feather;
  const double cl_clean = clean_cl(alpha, s.mach);
  const double k = 1.0 / (kPi * kAspect * 0.78);
  const double cd_clean = cd0_for_mach(s.mach) + k * cl_clean * cl_clean + 0.018 * s.gear +
                          (s.rocket_on ? -0.006 : 0.0);
  const double cl = (1 - fe) * cl_clean + fe * 0.18 * std::sin(2 * alpha);
  const double cd = (1 - fe) * cd_clean + fe * 1.75;
  Vec3 accel{0, 0, 0};
  if (s.tas > 1.0) {
    Vec3 lift_dir = sub(u, mul(vhat, dot(u, vhat)));
    lift_dir = unit(lift_dir);
    accel = add(accel, mul(lift_dir, s.q_pa * kWingAreaM2 * cl / s.mass_kg));
    accel = add(accel, mul(vhat, -s.q_pa * kWingAreaM2 * cd / s.mass_kg));
  }
  accel = add(accel, mul(f, s.thrust_n / s.mass_kg));

  // Ground reaction for the rollout.
  const double wheel_alt = s.pos.u - kGearHeightM;
  if (s.on_ground) {
    const double speed = norm(s.vel);
    const double decel = kG0 * (0.03 + 0.32 * clamp(in.brake, 0.0, 1.0));
    if (speed > 0.2) accel = add(accel, mul(unit(s.vel), -decel));
    else s.vel = {0, 0, 0};
  }
  const Vec3 nongrav = accel;
  s.g_load = dot(nongrav, u) / kG0 + (s.on_ground ? 1.0 : 0.0);
  if (!s.on_ground) s.max_g = std::max(s.max_g, std::abs(s.g_load));

  const double h = std::hypot(s.vel.e, s.vel.n);
  accel.u -= gravity(s.pos.u) - h * h / (kEarthR + s.pos.u);
  s.vel = add(s.vel, mul(accel, dt));
  if (s.on_ground) s.vel.u = 0;
  s.pos = add(s.pos, mul(s.vel, dt));

  // Attitude: rate command blended with natural aerodynamic stability. Above
  // the sensible atmosphere the reaction-control system gives slow rates.
  const double aero = clamp(s.q_pa / 3000.0, 0.0, 1.0);
  const double stiff = std::min(1.5, 0.6 * std::sqrt(s.q_pa / 1000.0)) * (1.0 + 1.5 * fe);
  const double yaw_max = 3.0 + 6.0 * aero;
  double pitch_rate = pitch_authority(s) * clamp(in.pitch, -1.0, 1.0) + stability_pitch_rate(s);
  double roll_rate = roll_authority(s) * clamp(in.roll, -1.0, 1.0);
  double yaw_rate = yaw_max * clamp(in.yaw, -1.0, 1.0) - stiff * 1.2 * deg(beta);
  if (s.on_ground) {
    // Nose settles onto the runway, heading tracks the ground track.
    pitch_rate = clamp(-2.0 * s.pitch_deg, -4.0, 4.0);
    roll_rate = clamp(-3.0 * s.roll_deg, -10.0, 10.0);
    yaw_rate = 0;
  }
  rotate_body(s.att, pitch_rate, roll_rate, yaw_rate, dt);
  derive_euler(s);

  if (!s.on_ground && wheel_alt <= s.ground_elev_m && s.vel.u < 0) {
    s.on_ground = true;
    s.pos.u = s.ground_elev_m + kGearHeightM;
    s.vel.u = 0;
  }
  if (s.on_ground) s.pos.u = s.ground_elev_m + kGearHeightM;

  s.apogee_m = std::max(s.apogee_m, s.pos.u);
  if (!s.crossed_cloud && s.pos.u >= kCloudAltM) {
    s.crossed_cloud = true;
    s.cloud_cross_time_s = s.time_s;
  }

  Phase next = s.phase;
  if (s.on_ground) next = Phase::Landed;
  else if (s.rocket_on) next = Phase::Boost;
  else if (s.burned_out && s.vel.u > 0) next = Phase::Coast;
  else if (s.burned_out && s.feather > 0.5) next = Phase::Reentry;
  else if (!s.burned_out && s.phase_time_s < 20.0 && s.phase == Phase::Released) next = Phase::Released;
  else next = Phase::Glide;
  if (next != s.phase) {
    s.phase = next;
    s.phase_time_s = 0;
  }
}
}  // namespace

Atmosphere atmosphere(double h) {
  // US Standard Atmosphere 1976 (geopotential layers to 86 km), then an
  // exponential tail for the lower thermosphere.
  struct Layer { double base, temp, lapse, pressure; };
  static const Layer layers[] = {
      {0, 288.15, -0.0065, 101325.0},     {11000, 216.65, 0.0, 22632.06},
      {20000, 216.65, 0.001, 5474.889},   {32000, 228.65, 0.0028, 868.0187},
      {47000, 270.65, 0.0, 110.9063},     {51000, 270.65, -0.0028, 66.93887},
      {71000, 214.65, -0.002, 3.956420},  {84852, 186.946, 0.0, 0.3734},
  };
  constexpr double R = 287.053, g = 9.80665;
  const double geo = 6356766.0 * std::max(h, -500.0) / (6356766.0 + std::max(h, -500.0));
  int i = 0;
  while (i < 7 && geo >= layers[i + 1].base) ++i;
  const Layer& L = layers[i];
  const double dh = geo - L.base;
  double t, p;
  if (L.lapse == 0.0) {
    t = L.temp;
    p = L.pressure * std::exp(-g * dh / (R * t));
  } else {
    t = L.temp + L.lapse * dh;
    p = L.pressure * std::pow(t / L.temp, -g / (R * L.lapse));
  }
  if (i == 7) {
    t = 186.87;
    p = L.pressure * std::exp(-dh / 5800.0);
  }
  return {p / (R * t), t, p, std::sqrt(1.4 * R * t)};
}

double gravity(double h) { return kG0 * std::pow(kEarthR / (kEarthR + h), 2); }

double pitch_authority(const Ship& s) {
  const double aero = clamp(s.q_pa / 3000.0, 0.0, 1.0);
  return (3.0 + 12.0 * aero) * (1.0 - 0.7 * s.feather);
}
double roll_authority(const Ship& s) {
  return 6.0 + 50.0 * clamp(s.q_pa / 3000.0, 0.0, 1.0);
}
double stability_pitch_rate(const Ship& s) {
  const double stiff = std::min(1.5, 0.6 * std::sqrt(s.q_pa / 1000.0)) * (1.0 + 1.5 * s.feather);
  const double alpha0 = 3.5 + 58.0 * s.feather;
  return -stiff * (s.alpha_deg - alpha0);
}
double lift_slope_per_rad(double mach) { return lift_slope(mach); }

const char* phase_name(Phase p) {
  switch (p) {
    case Phase::Ground: return "ON RUNWAY";
    case Phase::MatedClimb: return "MATED CLIMB";
    case Phase::Released: return "RELEASED";
    case Phase::Boost: return "BOOST";
    case Phase::Coast: return "COAST";
    case Phase::Reentry: return "FEATHERED REENTRY";
    case Phase::Glide: return "GLIDE";
    case Phase::Landed: return "ROLLOUT";
  }
  return "?";
}

Vec3 body_forward(const Quat& q) { return rotate(q, 1, 0, 0); }
Vec3 body_left(const Quat& q) { return rotate(q, 0, 1, 0); }
Vec3 body_up(const Quat& q) { return rotate(q, 0, 0, 1); }

Quat from_euler(double heading, double pitch, double roll) {
  const Quat yaw = axis_angle(0, 0, 1, rad(90.0 - heading));
  const Quat pit = axis_angle(0, 1, 0, rad(-pitch));
  const Quat rol = axis_angle(1, 0, 0, rad(roll));
  return qnorm(qmul(qmul(yaw, pit), rol));
}

void to_euler(const Quat& q, double& heading, double& pitch, double& roll) {
  const Vec3 f = body_forward(q), l = body_left(q);
  pitch = deg(std::asin(clamp(f.u, -1.0, 1.0)));
  const double horiz = std::hypot(f.e, f.n);
  double psi;
  if (horiz > 1e-6) {
    psi = std::atan2(f.e, f.n);
  } else {
    // Straight up/down: take heading from the body "up" axis instead.
    const Vec3 u = body_up(q);
    psi = std::atan2(-u.e * (f.u > 0 ? 1 : -1), -u.n * (f.u > 0 ? 1 : -1));
  }
  heading = std::fmod(deg(psi) + 360.0, 360.0);
  const double th = rad(pitch);
  const Vec3 r0{std::cos(psi), -std::sin(psi), 0};
  const Vec3 u0{-std::sin(psi) * std::sin(th), -std::cos(psi) * std::sin(th), std::cos(th)};
  roll = deg(std::atan2(dot(l, u0), -dot(l, r0)));
}

double predicted_apogee(const Ship& s) {
  if (s.vel.u <= 0) return s.pos.u;
  return s.pos.u + s.vel.u * s.vel.u / (2.0 * gravity(s.pos.u));
}

World initial(double runway_heading_deg, double field_elev_m) {
  World w;
  w.ship.ground_elev_m = field_elev_m;
  w.carrier.heading_deg = runway_heading_deg;
  w.carrier.pos = {0, 0, field_elev_m + 2.6};
  w.ship.oxidizer_kg = kOxidizerKg;
  w.ship.mass_kg = kShipDryKg + kOxidizerKg;
  w.ship.gear = 1.0;
  w.ship.gear_cmd = true;
  mated_ship(w);
  derive_euler(w.ship);
  w.ship.apogee_m = w.ship.pos.u;
  return w;
}

bool release(World& w) {
  auto& s = w.ship;
  if (s.released || !w.carrier.airborne) return false;
  s.released = true;
  s.on_ground = false;
  s.gear_cmd = false;
  s.vel.u -= 1.0;  // hooks open; the ship drops away from the carrier
  s.phase = Phase::Released;
  s.phase_time_s = 0;
  w.carrier_separation_s = 0;
  return true;
}

bool ignite(World& w) {
  auto& s = w.ship;
  if (!s.released || s.rocket_on || s.burned_out || s.oxidizer_kg <= 0) return false;
  if (w.carrier_separation_s < 3.0) return false;
  s.rocket_on = true;
  s.burn_s = 0;
  return true;
}

void cutoff(World& w) {
  if (!w.ship.rocket_on) return;
  w.ship.rocket_on = false;
  w.ship.burned_out = true;  // hybrid motor is not restartable here
}

bool deploy_probe(World& w) {
  auto& p = w.probe;
  const auto& s = w.ship;
  if (p.deployed || !s.released) return false;
  p.deployed = true;
  p.deploy_time_s = s.time_s;
  const Vec3 u = body_up(s.att), f = body_forward(s.att);
  // Spring ejection from the aft payload bay: 3 m/s down and slightly aft.
  p.pos = add(add(s.pos, mul(u, -1.4)), mul(f, -4.0));
  p.vel = add(add(s.vel, mul(u, -3.0)), mul(f, -0.6));
  p.apogee_m = p.pos.u;
  return true;
}

void set_feather(World& w, bool raised) { w.ship.feather_cmd = raised; }
void set_gear(World& w, bool down) { w.ship.gear_cmd = down; }

void advance(World& w, const Controls& c, const CarrierCommand& cc, double dt_s) {
  double remaining = clamp(dt_s, 0.0, 2.0);
  while (remaining > 1e-9) {
    const double dt = std::min(remaining, 0.01);
    remaining -= dt;
    w.ship.time_s += dt;
    w.ship.phase_time_s += dt;
    carrier_step(w, cc, dt);
    if (!w.ship.released) {
      mated_ship(w);
      derive_euler(w.ship);
      update_air_data(w.ship);
      w.ship.apogee_m = std::max(w.ship.apogee_m, w.ship.pos.u);
    } else {
      w.carrier_separation_s += dt;
      ship_step(w, c, dt);
    }
    probe_step(w, dt);
  }
}

}  // namespace flight
