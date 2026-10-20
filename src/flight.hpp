#pragma once
// Point-mass + quaternion-attitude flight model for a WhiteKnightTwo /
// SpaceShipTwo style air-launched suborbital mission. Original training
// approximation built from public figures (MTOW, thrust class, burn time,
// apogee); it is not a certified or manufacturer vehicle model.

namespace flight {

constexpr double kFeetToMeters = 0.3048;
constexpr double kReleaseAltM = 50000.0 * kFeetToMeters;    // 15,240 m
constexpr double kProbeAltM = 150000.0 * kFeetToMeters;     // 45,720 m
constexpr double kCloudAltM = 82300.0;                      // ~270,000 ft
constexpr double kDefeatherAltM = 55000.0 * kFeetToMeters;  // 16,764 m
constexpr double kGearHeightM = 1.6;                        // CG above wheels

struct Atmosphere {
  double rho;  // kg/m^3
  double temp_k;
  double pressure_pa;
  double sound_mps;
};
Atmosphere atmosphere(double altitude_m);

struct Vec3 {
  double e = 0, n = 0, u = 0;
};

struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
};

enum class Phase {
  Ground = 0,     // mated pair on the runway
  MatedClimb,     // WhiteKnightTwo carrying SpaceShipTwo
  Released,       // free fall before ignition
  Boost,          // rocket motor burning
  Coast,          // ballistic climb to apogee
  Reentry,        // feathered descent
  Glide,          // unpowered glide back to the field
  Landed,         // rollout / stopped
};
const char* phase_name(Phase p);

// Normalized pilot inputs: -1..1 (pitch +1 = nose up, roll +1 = right,
// yaw +1 = nose right). These are rate commands blended with aerodynamic
// stability, so a steady stick deflection holds an angle-of-attack offset.
struct Controls {
  double pitch = 0, roll = 0, yaw = 0;
  double brake = 0;
};

struct Carrier {
  Vec3 pos;           // ENU meters from the mission origin; u = MSL
  double heading_deg = 0;
  double pitch_deg = 0;
  double roll_deg = 0;
  double tas_mps = 0;
  double climb_mps = 0;
  bool airborne = false;
};

struct Probe {
  bool deployed = false;
  bool chute = false;
  bool landed = false;
  Vec3 pos, vel;
  double deploy_time_s = 0;
  double apogee_m = 0;
  double max_density = 0;
};

struct Ship {
  Phase phase = Phase::Ground;
  double time_s = 0;
  double phase_time_s = 0;
  Vec3 pos, vel;   // ENU, u = altitude MSL
  Quat att;        // body (forward, left, up) -> ENU
  double pitch_deg = 0, roll_deg = 0, heading_deg = 0;  // derived Euler
  double alpha_deg = 0, beta_deg = 0;
  double mass_kg = 0, oxidizer_kg = 0, burn_s = 0;
  double thrust_n = 0, chamber_bar = 0;
  double mach = 0, q_pa = 0, keas = 0, tas = 0;
  double g_load = 1, max_g = 1, max_mach = 0, apogee_m = 0;
  double feather = 0;          // 0 = stowed, 1 = fully raised (60 deg)
  double gear = 0;             // 0 = up, 1 = down
  double ground_elev_m = 0;    // terrain under the ship (set by the host)
  bool released = false, rocket_on = false, burned_out = false;
  bool feather_cmd = false, gear_cmd = false, on_ground = true;
  bool crossed_cloud = false;
  double cloud_cross_time_s = -1;
};

struct World {
  Ship ship;
  Carrier carrier;
  Probe probe;
  double carrier_separation_s = 0;
};

// Mission origin is the runway threshold; the runway heading is used for the
// ground roll and initial placement.
World initial(double runway_heading_deg, double field_elev_m);

// Carrier guidance input: desired heading and target altitude.
struct CarrierCommand {
  double heading_deg = 0;
  double target_alt_m = kReleaseAltM;
  bool takeoff = false;
};

bool release(World& w);
bool ignite(World& w);
void cutoff(World& w);
bool deploy_probe(World& w);
void set_feather(World& w, bool raised);
void set_gear(World& w, bool down);

void advance(World& w, const Controls& c, const CarrierCommand& cc, double dt_s);

// Helpers shared with guidance and rendering.
Vec3 body_forward(const Quat& q);
Vec3 body_left(const Quat& q);
Vec3 body_up(const Quat& q);
Quat from_euler(double heading_deg, double pitch_deg, double roll_deg);
void to_euler(const Quat& q, double& heading_deg, double& pitch_deg, double& roll_deg);
double predicted_apogee(const Ship& s);
double gravity(double altitude_m);
// Attitude-loop characteristics, so guidance can cancel natural stability.
double pitch_authority(const Ship& s);   // deg/s at full stick
double roll_authority(const Ship& s);
double stability_pitch_rate(const Ship& s);  // deg/s the airframe adds
double lift_slope_per_rad(double mach);
constexpr double kWingArea = 27.0;

}  // namespace flight
