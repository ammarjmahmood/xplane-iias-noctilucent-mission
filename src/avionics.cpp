#include "avionics.hpp"

#include "XPLMGraphics.h"

#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>

namespace avionics {
namespace {
constexpr float kPi = 3.14159265f;

struct Color {
  float r, g, b, a;
};
const Color kWhite{0.95f, 0.97f, 1.0f, 1};
const Color kCyan{0.35f, 0.88f, 1.0f, 1};
const Color kGreen{0.35f, 1.0f, 0.55f, 1};
const Color kAmber{1.0f, 0.74f, 0.18f, 1};
const Color kMagenta{1.0f, 0.45f, 1.0f, 1};
const Color kRed{1.0f, 0.3f, 0.25f, 1};
const Color kDim{0.55f, 0.6f, 0.68f, 1};
const Color kBezel{0.06f, 0.065f, 0.075f, 0.96f};
const Color kSky{0.18f, 0.45f, 0.85f, 1};
const Color kGround{0.48f, 0.31f, 0.16f, 1};

void color(const Color& c, float a = -1) { glColor4f(c.r, c.g, c.b, a < 0 ? c.a : a); }

void rect(float l, float b, float r, float t, const Color& c, float a = -1) {
  color(c, a);
  glBegin(GL_TRIANGLES);
  glVertex2f(l, b);
  glVertex2f(r, b);
  glVertex2f(r, t);
  glVertex2f(l, b);
  glVertex2f(r, t);
  glVertex2f(l, t);
  glEnd();
}

void frame(float l, float b, float r, float t, const Color& c, float w = 1.5f) {
  glLineWidth(w);
  color(c);
  glBegin(GL_LINE_LOOP);
  glVertex2f(l, b);
  glVertex2f(r, b);
  glVertex2f(r, t);
  glVertex2f(l, t);
  glEnd();
}

void line(float x0, float y0, float x1, float y1, const Color& c, float w = 1.5f, float a = -1) {
  glLineWidth(w);
  color(c, a);
  glBegin(GL_LINES);
  glVertex2f(x0, y0);
  glVertex2f(x1, y1);
  glEnd();
}

// Sutherland-Hodgman clip of a polygon against an axis-aligned rectangle.
std::vector<Point2> clip_poly(std::vector<Point2> poly, float l, float b, float r, float t) {
  auto clip = [&](auto inside, auto intersect) {
    std::vector<Point2> out;
    for (size_t i = 0; i < poly.size(); ++i) {
      const Point2 cur = poly[i], prev = poly[(i + poly.size() - 1) % poly.size()];
      const bool ci = inside(cur), pi = inside(prev);
      if (ci) {
        if (!pi) out.push_back(intersect(prev, cur));
        out.push_back(cur);
      } else if (pi) {
        out.push_back(intersect(prev, cur));
      }
    }
    poly = out;
  };
  auto ix = [](Point2 a, Point2 c, float x) {
    const float k = (x - a.x) / (c.x - a.x);
    return Point2{x, a.y + k * (c.y - a.y)};
  };
  auto iy = [](Point2 a, Point2 c, float y) {
    const float k = (y - a.y) / (c.y - a.y);
    return Point2{a.x + k * (c.x - a.x), y};
  };
  clip([&](Point2 p) { return p.x >= l; }, [&](Point2 a, Point2 c) { return ix(a, c, l); });
  if (!poly.empty()) clip([&](Point2 p) { return p.x <= r; }, [&](Point2 a, Point2 c) { return ix(a, c, r); });
  if (!poly.empty()) clip([&](Point2 p) { return p.y >= b; }, [&](Point2 a, Point2 c) { return iy(a, c, b); });
  if (!poly.empty()) clip([&](Point2 p) { return p.y <= t; }, [&](Point2 a, Point2 c) { return iy(a, c, t); });
  return poly;
}

bool clip_line(float& x0, float& y0, float& x1, float& y1, float l, float b, float r, float t) {
  float t0 = 0, t1 = 1;
  const float dx = x1 - x0, dy = y1 - y0;
  const float p[4] = {-dx, dx, -dy, dy};
  const float q[4] = {x0 - l, r - x0, y0 - b, t - y0};
  for (int i = 0; i < 4; ++i) {
    if (p[i] == 0) {
      if (q[i] < 0) return false;
    } else {
      const float u = q[i] / p[i];
      if (p[i] < 0) t0 = std::max(t0, u);
      else t1 = std::min(t1, u);
    }
  }
  if (t0 > t1) return false;
  const float nx0 = x0 + t0 * dx, ny0 = y0 + t0 * dy;
  x1 = x0 + t1 * dx;
  y1 = y0 + t1 * dy;
  x0 = nx0;
  y0 = ny0;
  return true;
}

void fill_poly(const std::vector<Point2>& p, const Color& c) {
  if (p.size() < 3) return;
  color(c);
  glBegin(GL_TRIANGLES);
  for (size_t i = 1; i + 1 < p.size(); ++i) {
    glVertex2f(p[0].x, p[0].y);
    glVertex2f(p[i].x, p[i].y);
    glVertex2f(p[i + 1].x, p[i + 1].y);
  }
  glEnd();
}

// --------------------------------------------------------------- font
const std::map<char, const char*>& glyphs() {
  static const std::map<char, const char*> g = {
      {'A', "0,0 0,4 2,6 4,4 4,0|0,3 4,3"},
      {'B', "0,0 0,6 3,6 4,5 4,4 3,3 0,3|3,3 4,2 4,1 3,0 0,0"},
      {'C', "4,1 3,0 1,0 0,1 0,5 1,6 3,6 4,5"},
      {'D', "0,0 0,6 3,6 4,5 4,1 3,0 0,0"},
      {'E', "4,0 0,0 0,6 4,6|0,3 3,3"},
      {'F', "0,0 0,6 4,6|0,3 3,3"},
      {'G', "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,3 2,3"},
      {'H', "0,0 0,6|4,0 4,6|0,3 4,3"},
      {'I', "1,0 3,0|2,0 2,6|1,6 3,6"},
      {'J', "0,1 1,0 3,0 4,1 4,6"},
      {'K', "0,0 0,6|4,6 0,2|1,3 4,0"},
      {'L', "0,6 0,0 4,0"},
      {'M', "0,0 0,6 2,3 4,6 4,0"},
      {'N', "0,0 0,6 4,0 4,6"},
      {'O', "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0"},
      {'P', "0,0 0,6 3,6 4,5 4,4 3,3 0,3"},
      {'Q', "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0|2,2 4,0"},
      {'R', "0,0 0,6 3,6 4,5 4,4 3,3 0,3|2,3 4,0"},
      {'S', "4,5 3,6 1,6 0,5 0,4 1,3 3,3 4,2 4,1 3,0 1,0 0,1"},
      {'T', "0,6 4,6|2,6 2,0"},
      {'U', "0,6 0,1 1,0 3,0 4,1 4,6"},
      {'V', "0,6 2,0 4,6"},
      {'W', "0,6 1,0 2,3 3,0 4,6"},
      {'X', "0,0 4,6|0,6 4,0"},
      {'Y', "0,6 2,3 4,6|2,3 2,0"},
      {'Z', "0,6 4,6 0,0 4,0"},
      {'0', "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0"},
      {'1', "1,5 2,6 2,0|1,0 3,0"},
      {'2', "0,5 1,6 3,6 4,5 4,4 0,0 4,0"},
      {'3', "0,5 1,6 3,6 4,5 4,4 3,3 4,2 4,1 3,0 1,0 0,1|1,3 3,3"},
      {'4', "3,0 3,6 0,2 4,2"},
      {'5', "4,6 0,6 0,3 3,3 4,2 4,1 3,0 0,0"},
      {'6', "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,2 3,3 0,3"},
      {'7', "0,6 4,6 1,0"},
      {'8', "1,3 0,4 0,5 1,6 3,6 4,5 4,4 3,3 1,3 0,2 0,1 1,0 3,0 4,1 4,2 3,3"},
      {'9', "0,1 1,0 3,0 4,1 4,5 3,6 1,6 0,5 0,4 1,3 4,3"},
      {'.', "2,0 2,1"},
      {',', "2,1 1,-1"},
      {':', "2,1 2,2|2,4 2,5"},
      {'-', "1,3 3,3"},
      {'+', "0,3 4,3|2,1 2,5"},
      {'/', "0,0 4,6"},
      {'(', "3,0 2,1 2,5 3,6"},
      {')', "1,0 2,1 2,5 1,6"},
      {'%', "0,0 4,6|0,6 1,6 1,5 0,5 0,6|3,1 4,1 4,0 3,0 3,1"},
      {'\'', "2,6 2,5"},
      {'>', "0,5 4,3 0,1"},
      {'<', "4,5 0,3 4,1"},
      {'=', "0,2 4,2|0,4 4,4"},
      {'^', "1,5 2,6 3,5 2,4 1,5"},
      {'*', "2,1 2,5|0,2 4,4|0,4 4,2"},
      {'|', "2,-1 2,7"},
      {'!', "2,2 2,6|2,0 2,1"},
      {'?', "0,5 1,6 3,6 4,5 4,4 2,3 2,2|2,0 2,1"},
  };
  return g;
}
}  // namespace

float text_width(float size, const std::string& s) {
  return static_cast<float>(s.size()) * size * (4.0f / 6.0f + 0.38f);
}

void text(float x, float y, float size, const std::string& s, float r, float g, float b, float a,
          float weight) {
  const float k = size / 6.0f;
  const float adv = size * (4.0f / 6.0f + 0.38f);
  glLineWidth(weight);
  glColor4f(r, g, b, a);
  for (char ch : s) {
    const char up = (ch >= 'a' && ch <= 'z') ? static_cast<char>(ch - 32) : ch;
    auto it = glyphs().find(up);
    if (it != glyphs().end()) {
      const char* p = it->second;
      glBegin(GL_LINE_STRIP);
      while (*p) {
        if (*p == '|') {
          glEnd();
          glBegin(GL_LINE_STRIP);
          ++p;
          continue;
        }
        if (*p == ' ') {
          ++p;
          continue;
        }
        char* end = nullptr;
        const long gx = std::strtol(p, &end, 10);
        p = end + 1;  // skip comma
        const long gy = std::strtol(p, &end, 10);
        p = end;
        glVertex2f(x + gx * k, y + gy * k);
      }
      glEnd();
    }
    x += adv;
  }
}

namespace {
void txt(float x, float y, float size, const std::string& s, const Color& c, float w = 1.5f, float a = -1) {
  text(x, y, size, s, c.r, c.g, c.b, a < 0 ? c.a : a, w);
}
void txt_c(float x, float y, float size, const std::string& s, const Color& c, float w = 1.5f) {
  txt(x - text_width(size, s) / 2, y, size, s, c, w);
}
void txt_r(float x, float y, float size, const std::string& s, const Color& c, float w = 1.5f) {
  txt(x - text_width(size, s), y, size, s, c, w);
}
std::string fmt(const char* f, double v) {
  char buf[64];
  std::snprintf(buf, sizeof buf, f, v);
  return buf;
}
std::string thousands(double v) {
  char buf[32];
  const long long n = std::llround(v);
  std::snprintf(buf, sizeof buf, "%lld", std::llabs(n));
  std::string s = buf, out;
  int c = 0;
  for (int i = static_cast<int>(s.size()) - 1; i >= 0; --i) {
    out.insert(out.begin(), s[i]);
    if (++c % 3 == 0 && i > 0) out.insert(out.begin(), ',');
  }
  return (n < 0 ? "-" : "") + out;
}
std::string clock(double s) {
  const int t = static_cast<int>(std::max(0.0, s));
  char buf[32];
  std::snprintf(buf, sizeof buf, "%02d:%02d:%02d", t / 3600, (t / 60) % 60, t % 60);
  return buf;
}

void begin_display(int l, int t, int r, int b, const char* title) {
  XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
  rect(l, b, r, t, kBezel);
  frame(l + 2, b + 2, r - 2, t - 2, Color{0.25f, 0.27f, 0.3f, 1}, 2.0f);
  const float h = static_cast<float>(t - b);
  txt(l + 10, t - h * 0.075f, h * 0.035f, title, kDim, 1.3f);
}
}  // namespace

// ------------------------------------------------------------------ PFD
void draw_pfd(const Snapshot& s, int l, int t, int r, int b) {
  begin_display(l, t, r, b, "PFD");
  const float w = static_cast<float>(r - l), h = static_cast<float>(t - b);
  const float fs = h * 0.045f;
  // Annunciator strip.
  txt_c(l + w * 0.5f, t - h * 0.075f, fs, s.phase, s.rocket_on ? kAmber : kGreen, 1.8f);
  txt_r(r - 10, t - h * 0.075f, h * 0.035f, "MET " + clock(s.met_s), kWhite, 1.3f);
  // ADI.
  const float al = l + w * 0.2f, ar = r - w * 0.2f, ab = b + h * 0.2f, at = t - h * 0.13f;
  const float cx = (al + ar) / 2, cy = (ab + at) / 2;
  const float ppd = (at - ab) / 50.0f;
  const float rr = static_cast<float>(-s.roll) * kPi / 180.0f;
  const float cr = std::cos(rr), sr = std::sin(rr);
  auto xf = [&](float x, float y) {
    const float yy = y - static_cast<float>(s.pitch) * ppd;
    return Point2{cx + x * cr - yy * sr, cy + x * sr + yy * cr};
  };
  const float big = 4000;
  fill_poly(clip_poly({xf(-big, 0), xf(big, 0), xf(big, big), xf(-big, big)}, al, ab, ar, at), kSky);
  fill_poly(clip_poly({xf(-big, -big), xf(big, -big), xf(big, 0), xf(-big, 0)}, al, ab, ar, at), kGround);
  // Horizon and pitch ladder.
  for (int p = -90; p <= 90; p += 5) {
    const float half = (p == 0) ? 3000.0f : (p % 10 == 0 ? w * 0.09f : w * 0.045f);
    Point2 a = xf(-half, p * ppd), c = xf(half, p * ppd);
    float x0 = a.x, y0 = a.y, x1 = c.x, y1 = c.y;
    if (clip_line(x0, y0, x1, y1, al, ab, ar, at)) line(x0, y0, x1, y1, kWhite, p == 0 ? 2.0f : 1.4f);
    if (p != 0 && p % 10 == 0) {
      const Point2 lab = xf(half + w * 0.012f, p * ppd - fs * 0.35f);
      if (lab.x > al && lab.x < ar - fs * 2 && lab.y > ab && lab.y < at - fs)
        txt(lab.x, lab.y, fs * 0.7f, fmt("%.0f", std::abs(p)), kWhite, 1.2f);
    }
  }
  // Flight-path marker.
  {
    const float dx = static_cast<float>(-s.beta) * ppd;
    const float dy = static_cast<float>(s.pitch - s.alpha) * ppd;
    const Point2 m = xf(dx, dy);
    if (m.x > al + 10 && m.x < ar - 10 && m.y > ab + 10 && m.y < at - 10) {
      glLineWidth(2.0f);
      color(kGreen);
      glBegin(GL_LINE_LOOP);
      for (int i = 0; i < 16; ++i)
        glVertex2f(m.x + 7 * std::cos(i * kPi / 8), m.y + 7 * std::sin(i * kPi / 8));
      glEnd();
      line(m.x - 18, m.y, m.x - 7, m.y, kGreen, 2);
      line(m.x + 7, m.y, m.x + 18, m.y, kGreen, 2);
      line(m.x, m.y + 7, m.x, m.y + 14, kGreen, 2);
    }
  }
  // Fixed aircraft symbol and bank scale.
  line(cx - w * 0.13f, cy, cx - w * 0.05f, cy, kAmber, 4);
  line(cx + w * 0.05f, cy, cx + w * 0.13f, cy, kAmber, 4);
  line(cx - w * 0.05f, cy, cx - w * 0.05f, cy - 8, kAmber, 4);
  line(cx + w * 0.05f, cy, cx + w * 0.05f, cy - 8, kAmber, 4);
  rect(cx - 3, cy - 3, cx + 3, cy + 3, kAmber);
  const float arc_r = (at - ab) * 0.44f;
  for (int a : {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60}) {
    const float ang = (90.0f + a) * kPi / 180.0f;
    const float len = (a % 30 == 0) ? 12.0f : 7.0f;
    line(cx + arc_r * std::cos(ang), cy + arc_r * std::sin(ang), cx + (arc_r + len) * std::cos(ang),
         cy + (arc_r + len) * std::sin(ang), kWhite, 1.5f);
  }
  {
    const float ang = (90.0f - static_cast<float>(s.roll)) * kPi / 180.0f;
    const float px = cx + arc_r * std::cos(ang), py = cy + arc_r * std::sin(ang);
    color(kAmber);
    glBegin(GL_TRIANGLES);
    glVertex2f(px, py);
    glVertex2f(px - 7 * std::cos(ang) + 6 * std::sin(ang), py - 7 * std::sin(ang) - 6 * std::cos(ang));
    glVertex2f(px - 7 * std::cos(ang) - 6 * std::sin(ang), py - 7 * std::sin(ang) + 6 * std::cos(ang));
    glEnd();
  }
  frame(al, ab, ar, at, kDim, 1.5f);

  // Speed tape (KEAS) on the left with Mach underneath.
  const float sl = l + w * 0.03f, sr2 = al - w * 0.015f;
  rect(sl, ab, sr2, at, Color{0.1f, 0.11f, 0.13f, 1});
  const float kpp = (at - ab) / 160.0f;  // px per knot
  for (int k = static_cast<int>(s.keas / 10) * 10 - 90; k <= s.keas + 90; k += 10) {
    if (k < 0) continue;
    const float y = cy + static_cast<float>(k - s.keas) * kpp;
    if (y < ab + 4 || y > at - 4) continue;
    line(sr2 - 10, y, sr2, y, kWhite, 1.5f);
    if (k % 20 == 0) txt_r(sr2 - 14, y - fs * 0.35f, fs * 0.65f, fmt("%.0f", k), kWhite, 1.2f);
  }
  rect(sl, cy - fs * 0.8f, sr2, cy + fs * 0.8f, Color{0, 0, 0, 1});
  frame(sl, cy - fs * 0.8f, sr2, cy + fs * 0.8f, kWhite);
  txt_c((sl + sr2) / 2, cy - fs * 0.45f, fs * 0.9f, fmt("%.0f", s.keas), kWhite, 1.8f);
  txt_c((sl + sr2) / 2, at + 6, fs * 0.6f, "KEAS", kDim, 1.2f);
  txt_c((sl + sr2) / 2, ab - fs * 1.2f, fs * 0.85f, fmt("M %.2f", s.mach), s.mach > 1 ? kAmber : kWhite, 1.6f);

  // Altitude tape on the right.
  const float tl = ar + w * 0.015f, tr = r - w * 0.03f;
  rect(tl, ab, tr, at, Color{0.1f, 0.11f, 0.13f, 1});
  const double step = s.alt_ft > 60000 ? 5000.0 : 500.0;
  const float fpp = static_cast<float>((at - ab) / (step * 12.0));
  for (double a = std::floor(s.alt_ft / step) * step - step * 6; a <= s.alt_ft + step * 6; a += step) {
    const float y = cy + static_cast<float>(a - s.alt_ft) * fpp;
    if (y < ab + 4 || y > at - 4) continue;
    line(tl, y, tl + 10, y, kWhite, 1.5f);
    if (std::fmod(std::abs(a), step * 2) < 1) txt(tl + 14, y - fs * 0.35f, fs * 0.6f, thousands(a), kWhite, 1.2f);
  }
  // NLC layer band on the tape.
  {
    const float y0 = cy + static_cast<float>(266000.0 - s.alt_ft) * fpp;
    const float y1 = cy + static_cast<float>(274200.0 - s.alt_ft) * fpp;
    if (y1 > ab && y0 < at) rect(tl + 2, std::max(ab, y0), tl + 8, std::min(at, y1), kCyan, 0.8f);
  }
  rect(tl, cy - fs * 0.8f, tr, cy + fs * 0.8f, Color{0, 0, 0, 1});
  frame(tl, cy - fs * 0.8f, tr, cy + fs * 0.8f, kWhite);
  txt_c((tl + tr) / 2, cy - fs * 0.45f, fs * 0.78f, thousands(s.alt_ft), kWhite, 1.7f);
  txt_c((tl + tr) / 2, at + 6, fs * 0.6f, "FT", kDim, 1.2f);
  txt_c((tl + tr) / 2, ab - fs * 1.2f, fs * 0.7f, "VS " + thousands(s.vs_fpm), kWhite, 1.4f);

  // Heading tape and readouts.
  const float hb = b + h * 0.03f, ht = b + h * 0.1f;
  rect(al, hb, ar, ht, Color{0.1f, 0.11f, 0.13f, 1});
  const float dpp = (ar - al) / 60.0f;
  for (int hd = static_cast<int>(s.heading) - 35; hd <= s.heading + 35; ++hd) {
    if (hd % 5) continue;
    const float x = cx + static_cast<float>(hd - s.heading) * dpp;
    if (x < al + 2 || x > ar - 2) continue;
    line(x, ht, x, ht - (hd % 10 ? 5 : 10), kWhite, 1.4f);
    if (hd % 30 == 0) {
      const int v = ((hd % 360) + 360) % 360;
      txt_c(x, hb + 3, fs * 0.55f, fmt("%03.0f", v), kWhite, 1.2f);
    }
  }
  txt_c(cx, ht + 3, fs * 0.6f, fmt("%03.0f", std::fmod(s.heading + 360.0, 360.0)), kAmber, 1.5f);
  txt(l + 10, b + h * 0.12f, fs * 0.62f, fmt("G %.1f", s.g), std::abs(s.g) > 3.5 ? kAmber : kWhite, 1.4f);
  txt(l + 10, b + h * 0.05f, fs * 0.55f, fmt("MAX %.1f", s.gmax), kDim, 1.2f);
  txt_r(r - 10, b + h * 0.12f, fs * 0.62f, fmt("AOA %.1f", s.alpha), kWhite, 1.4f);
  txt_r(r - 10, b + h * 0.05f, fs * 0.55f, fmt("Q %.1f KPA", s.q_kpa), kDim, 1.2f);
}

// ------------------------------------------------------------------ NAV
void draw_nav(const Snapshot& s, int l, int t, int r, int b) {
  begin_display(l, t, r, b, "TRAJ / NAV");
  const float w = static_cast<float>(r - l), h = static_cast<float>(t - b);
  const float fs = h * 0.04f;
  txt_r(r - 10, t - h * 0.075f, fs * 0.85f, s.step, kGreen, 1.5f);
  // Profile: altitude (km) vs distance along the runway axis (km).
  const float pl = l + w * 0.1f, pr = r - w * 0.04f, pb = b + h * 0.53f, pt = t - h * 0.12f;
  rect(pl, pb, pr, pt, Color{0.03f, 0.05f, 0.09f, 1});
  float xmin = -70, xmax = 10;
  for (auto& p : s.plan) { xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x); }
  for (auto& p : s.profile) { xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x); }
  const float ymax = 100.0f;
  auto X = [&](float x) { return pl + (x - xmin) / (xmax - xmin) * (pr - pl); };
  auto Y = [&](float y) { return pb + std::clamp(y, 0.0f, ymax) / ymax * (pt - pb); };
  // Altitude reference lines.
  struct Ref { float km; const char* label; Color c; };
  for (const Ref& ref : {Ref{15.24f, "50K REL", kDim}, Ref{45.72f, "150K PROBE", kAmber},
                         Ref{82.3f, "270K NLC", kCyan}}) {
    line(pl, Y(ref.km), pr, Y(ref.km), ref.c, 1.0f, 0.55f);
    txt(pl + 4, Y(ref.km) + 3, fs * 0.55f, ref.label, ref.c, 1.1f);
  }
  rect(pl, Y(81.05f), pr, Y(83.55f), kCyan, 0.18f);
  for (int k = 0; k <= 100; k += 20) txt_r(pl - 4, Y(k) - fs * 0.3f, fs * 0.55f, fmt("%.0f", k), kDim, 1.1f);
  txt(pl - w * 0.085f, pt + 4, fs * 0.55f, "KM", kDim, 1.1f);
  auto polyline = [&](const std::vector<Point2>& pts, const Color& c, float wd, float a) {
    if (pts.size() < 2) return;
    glLineWidth(wd);
    color(c, a);
    glBegin(GL_LINE_STRIP);
    for (auto& p : pts) glVertex2f(X(p.x), Y(p.y));
    glEnd();
  };
  polyline(s.plan, kMagenta, 1.5f, 0.6f);
  polyline(s.probe_profile, kAmber, 2.0f, 0.95f);
  polyline(s.profile, kWhite, 2.2f, 1.0f);
  if (!s.profile.empty()) {
    const auto& p = s.profile.back();
    rect(X(p.x) - 4, Y(p.y) - 4, X(p.x) + 4, Y(p.y) + 4, kGreen);
  }
  // Field marker.
  line(X(0), pb, X(0), pb + 8, kGreen, 3);
  txt_c(X(0), pb + 10, fs * 0.5f, "RWY", kGreen, 1.1f);
  frame(pl, pb, pr, pt, kDim, 1.2f);
  txt(pl, b + h * 0.47f, fs * 0.65f, "APOGEE " + thousands(s.apogee_ft) + " FT", kWhite, 1.3f);
  if (s.burned_out && s.predicted_apogee_ft > s.alt_ft + 100)
    txt_r(pr, b + h * 0.47f, fs * 0.65f, "PRED " + thousands(s.predicted_apogee_ft), kMagenta, 1.3f);

  // Plan-view map.
  const float ml = l + w * 0.04f, mr = r - w * 0.04f, mb = b + h * 0.04f, mt = b + h * 0.43f;
  rect(ml, mb, mr, mt, Color{0.03f, 0.05f, 0.09f, 1});
  float cxk = (s.ship.x + 0.0f) / 2, cyk = (s.ship.y + 0.0f) / 2;
  float span = std::max(24.0f, 1.25f * std::max(std::abs(s.ship.x), std::abs(s.ship.y)) + 8.0f);
  span = std::min(span, 160.0f);
  const float aspect = (mr - ml) / (mt - mb);
  auto MX = [&](float x) { return (ml + mr) / 2 + (x - cxk) / (span * aspect) * (mr - ml); };
  auto MY = [&](float y) { return (mb + mt) / 2 + (y - cyk) / span * (mt - mb); };
  auto inside = [&](float x, float y) { return x > ml && x < mr && y > mb && y < mt; };
  auto mline = [&](Point2 a, Point2 c2, const Color& c, float wd, float alpha = -1) {
    float x0 = MX(a.x), y0 = MY(a.y), x1 = MX(c2.x), y1 = MY(c2.y);
    if (clip_line(x0, y0, x1, y1, ml, mb, mr, mt)) line(x0, y0, x1, y1, c, wd, alpha);
  };
  // Range rings around the field.
  for (int rk : {10, 25, 50, 100}) {
    glLineWidth(1.0f);
    color(kDim, 0.35f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= 72; ++i) {
      float x = MX(rk * std::cos(i * kPi / 36)), y = MY(rk * std::sin(i * kPi / 36));
      x = std::clamp(x, ml, mr);
      y = std::clamp(y, mb, mt);
      glVertex2f(x, y);
    }
    glEnd();
  }
  // HAC circle and final approach.
  glLineWidth(1.5f);
  color(kMagenta, 0.8f);
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i <= 72; ++i) {
    const float x = MX(s.hac.x + static_cast<float>(s.hac_radius_km) * std::cos(i * kPi / 36));
    const float y = MY(s.hac.y + static_cast<float>(s.hac_radius_km) * std::sin(i * kPi / 36));
    glVertex2f(std::clamp(x, ml, mr), std::clamp(y, mb, mt));
  }
  glEnd();
  mline(s.final_entry, {0, 0}, kMagenta, 1.5f, 0.8f);
  const float hr = static_cast<float>(s.runway_heading) * kPi / 180.0f;
  const Point2 rwy_end{std::sin(hr) * static_cast<float>(s.runway_length_km),
                       std::cos(hr) * static_cast<float>(s.runway_length_km)};
  mline({0, 0}, rwy_end, kWhite, 5.0f);
  if (inside(MX(s.release.x), MY(s.release.y))) {
    rect(MX(s.release.x) - 3, MY(s.release.y) - 3, MX(s.release.x) + 3, MY(s.release.y) + 3, kDim);
    txt(MX(s.release.x) + 6, MY(s.release.y) - 4, fs * 0.5f, "REL", kDim, 1.1f);
  }
  txt(MX(rwy_end.x) + 6, MY(rwy_end.y), fs * 0.55f, s.field_name, kWhite, 1.2f);
  if (s.released && inside(MX(s.carrier.x), MY(s.carrier.y))) {
    rect(MX(s.carrier.x) - 3, MY(s.carrier.y) - 3, MX(s.carrier.x) + 3, MY(s.carrier.y) + 3, kCyan);
    txt(MX(s.carrier.x) + 6, MY(s.carrier.y) - 4, fs * 0.5f, "WK2", kCyan, 1.1f);
  }
  if (s.probe_deployed && inside(MX(s.probe.x), MY(s.probe.y))) {
    rect(MX(s.probe.x) - 3, MY(s.probe.y) - 3, MX(s.probe.x) + 3, MY(s.probe.y) + 3, kAmber);
    txt(MX(s.probe.x) + 6, MY(s.probe.y) + 4, fs * 0.5f, "PROBE", kAmber, 1.1f);
  }
  {
    const float sx = MX(s.ship.x), sy = MY(s.ship.y);
    const float a = static_cast<float>(s.heading) * kPi / 180.0f;
    if (inside(sx, sy)) {
      color(kGreen);
      glBegin(GL_TRIANGLES);
      glVertex2f(sx + 12 * std::sin(a), sy + 12 * std::cos(a));
      glVertex2f(sx + 7 * std::sin(a + 2.5f), sy + 7 * std::cos(a + 2.5f));
      glVertex2f(sx + 7 * std::sin(a - 2.5f), sy + 7 * std::cos(a - 2.5f));
      glEnd();
    }
  }
  frame(ml, mb, mr, mt, kDim, 1.2f);
  txt(ml + 4, mt - fs * 0.9f, fs * 0.5f, fmt("SPAN %.0f KM", span), kDim, 1.1f);
  const float dist = std::hypot(s.ship.x, s.ship.y);
  txt_r(mr - 4, mt - fs * 0.9f, fs * 0.55f, fmt("RWY %.1f KM", dist), kGreen, 1.2f);
}

// ------------------------------------------------------------------ SYS
namespace {
void bar(float l, float b, float r, float t, double frac, const Color& c, const std::string& label,
         const std::string& value, float fs) {
  rect(l, b, r, t, Color{0.12f, 0.13f, 0.15f, 1});
  rect(l, b, l + (r - l) * static_cast<float>(std::clamp(frac, 0.0, 1.0)), t, c, 0.85f);
  frame(l, b, r, t, kDim, 1.0f);
  txt(l, t + 4, fs, label, kDim, 1.1f);
  txt_r(r, t + 4, fs, value, kWhite, 1.2f);
}
}  // namespace

void draw_sys(const Snapshot& s, int l, int t, int r, int b) {
  begin_display(l, t, r, b, "SYS / PROBE");
  const float w = static_cast<float>(r - l), h = static_cast<float>(t - b);
  const float fs = h * 0.036f;
  const float x0 = l + w * 0.05f, x1 = l + w * 0.47f, x2 = l + w * 0.53f, x3 = r - w * 0.05f;
  // Motor.
  std::string motor = s.rocket_on ? "BURN" : s.burned_out ? "SHUTDOWN" : s.released ? "ARMED" : "SAFE";
  txt(x0, t - h * 0.15f, fs * 1.1f, "MOTOR " + motor, s.rocket_on ? kAmber : s.burned_out ? kDim : kGreen, 1.6f);
  bar(x0, t - h * 0.27f, x1, t - h * 0.23f, s.thrust_kn / 220.0, kAmber, "THRUST", fmt("%.0f KN", s.thrust_kn), fs * 0.8f);
  bar(x0, t - h * 0.37f, x1, t - h * 0.33f, s.chamber_bar / 55.0, kAmber, "CHAMBER", fmt("%.1f BAR", s.chamber_bar),
      fs * 0.8f);
  bar(x0, t - h * 0.47f, x1, t - h * 0.43f, s.ox_kg / s.ox_max, kCyan, "N2O / GRAIN", fmt("%.0f KG", s.ox_kg),
      fs * 0.8f);
  txt(x0, t - h * 0.55f, fs * 0.85f, fmt("BURN %.1f S", s.burn_s), kWhite, 1.3f);
  txt(x0, t - h * 0.61f, fs * 0.85f, fmt("MASS %.0f KG", s.mass_kg), kWhite, 1.3f);
  // Feather schematic: side view, booms raised to the feather angle.
  const float fx = x2 + (x3 - x2) * 0.3f, fy = t - h * 0.3f;
  std::string fst = s.feather > 0.97 ? "RAISED 60^" : s.feather > 0.02 ? "MOVING" : s.feather_cmd ? "UNLOCKED"
                                                                                                   : "LOCKED";
  txt(x2, t - h * 0.15f, fs * 1.1f, "FEATHER " + fst, s.feather > 0.02 ? kCyan : kGreen, 1.6f);
  line(fx - 60, fy, fx + 10, fy, kWhite, 4);
  const float fa = static_cast<float>(s.feather) * 60.0f * kPi / 180.0f;
  line(fx + 10, fy, fx + 10 + 55 * std::cos(fa), fy + 55 * std::sin(fa), kCyan, 4);
  line(fx + 10 + 55 * std::cos(fa), fy + 55 * std::sin(fa), fx + 10 + 55 * std::cos(fa) - 8 * std::sin(fa),
       fy + 55 * std::sin(fa) + 22 * std::cos(fa), kCyan, 3);
  std::string gst = s.gear > 0.98 ? "DOWN" : s.gear > 0.02 ? "TRANSIT" : "UP";
  txt(x2, t - h * 0.45f, fs, "GEAR " + gst, s.gear > 0.98 ? kGreen : s.gear > 0.02 ? kAmber : kWhite, 1.4f);
  txt(x2, t - h * 0.52f, fs * 0.85f, "MAX M " + fmt("%.2f", s.max_mach) + fmt("  G %.1f", s.gmax), kDim, 1.2f);
  txt(x2, t - h * 0.58f, fs * 0.85f, "APOGEE " + thousands(s.apogee_ft), kWhite, 1.3f);

  // Probe.
  line(l + 8, b + h * 0.36f, r - 8, b + h * 0.36f, kDim, 1);
  std::string pst = !s.probe_deployed ? "STOWED" : s.probe_landed ? "LANDED" : s.probe_chute ? "CHUTE" : "FREE FLIGHT";
  txt(x0, b + h * 0.30f, fs * 1.05f, "PROBE " + pst, s.probe_deployed ? kAmber : kDim, 1.6f);
  if (s.probe_deployed) {
    txt(x0, b + h * 0.24f, fs * 0.85f, "ALT " + thousands(s.probe_alt_ft) + " FT", kWhite, 1.3f);
    txt(x0, b + h * 0.19f, fs * 0.85f, "APO " + thousands(s.probe_apogee_ft) + " FT", kWhite, 1.3f);
  }
  bar(x0, b + h * 0.08f, x1, b + h * 0.12f, s.probe_density, kCyan, "NLC DENSITY",
      fmt("%.2f", s.probe_density), fs * 0.8f);
  if (s.probe_in_layer)
    txt(x0, b + h * 0.025f, fs * 0.8f, fmt("IN LAYER  IWC %.0f G/KM2", s.probe_iwc), kCyan, 1.5f);
  // Density history sparkline.
  const float sl = x2, sr = x3, sb = b + h * 0.08f, st = b + h * 0.3f;
  rect(sl, sb, sr, st, Color{0.03f, 0.05f, 0.09f, 1});
  if (s.density_history.size() > 1) {
    glLineWidth(1.6f);
    color(kCyan);
    glBegin(GL_LINE_STRIP);
    const size_t n = s.density_history.size();
    for (size_t i = 0; i < n; ++i)
      glVertex2f(sl + (sr - sl) * static_cast<float>(i) / (n - 1), sb + (st - sb) * s.density_history[i]);
    glEnd();
  }
  frame(sl, sb, sr, st, kDim, 1.0f);
  txt(sl, st + 4, fs * 0.7f, "PROBE NLC HISTORY", kDim, 1.1f);
  txt(sl, b + h * 0.025f, fs * 0.7f, fmt("PEAK %.2f", s.probe_max_density), kWhite, 1.2f);
  // Event log.
  const float ly0 = b + h * 0.36f + fs * 0.6f;
  const size_t first = s.events.size() > 4 ? s.events.size() - 4 : 0;
  float y = ly0 + fs * 4.6f;
  for (size_t i = first; i < s.events.size(); ++i) {
    std::string e = s.events[i];
    if (e.size() > 46) e = e.substr(0, 46);
    txt(x0, y, fs * 0.62f, e, i + 1 == s.events.size() ? kGreen : kDim, 1.1f);
    y -= fs * 1.15f;
  }
}

// ------------------------------------------------------------- caption
void draw_caption(const Snapshot& s, int l, int t, int r, int b, bool compact) {
  XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
  const float w = static_cast<float>(r - l);
  const float big = std::max(16.0f, w * 0.0125f);
  rect(l, t - big * 2.4f, r, t, Color{0, 0, 0, 1}, 0.45f);
  txt(l + 16, t - big * 1.55f, big * 0.62f, "IIAS NOCTILUCENT MISSION  " + s.field_name, kWhite, 1.6f);
  std::string right = s.camera + "   MET " + clock(s.met_s);
  if (s.warp > 1.01) right += fmt("   TIME X%.0f", s.warp);
  txt_r(r - 16, t - big * 1.55f, big * 0.62f, right, s.warp > 1.01 ? kAmber : kWhite, 1.6f);
  if (!s.caption.empty() && s.caption_age_s < 9.0) {
    const float a = static_cast<float>(std::clamp(9.0 - s.caption_age_s, 0.0, 1.0));
    const float cw = text_width(big, s.caption);
    rect((l + r) / 2 - cw / 2 - 18, t - big * 5.0f, (l + r) / 2 + cw / 2 + 18, t - big * 3.0f,
         Color{0, 0, 0, 1}, 0.55f * a);
    txt_c((l + r) / 2, t - big * 4.35f, big, s.caption, kWhite, 2.4f);
  }
  if (compact) {
    const float bw = big * 15.0f, bh = big * 6.2f;
    const float bl = l + 16, bb = b + 16;
    rect(bl, bb, bl + bw, bb + bh, Color{0, 0, 0, 1}, 0.5f);
    const float fs = big * 0.7f;
    txt(bl + 12, bb + bh - fs * 1.6f, fs, s.phase, s.rocket_on ? kAmber : kGreen, 1.8f);
    txt(bl + 12, bb + bh - fs * 3.1f, fs, "ALT " + thousands(s.alt_ft) + " FT", kWhite, 1.7f);
    txt(bl + 12, bb + bh - fs * 4.6f, fs, fmt("MACH %.2f", s.mach) + fmt("  %.0f KEAS", s.keas), kWhite, 1.7f);
    txt(bl + 12, bb + bh - fs * 6.1f, fs, "VS " + thousands(s.vs_fpm) + fmt("  G %.1f", s.g), kWhite, 1.7f);
    if (s.ship_density > 0.02)
      txt(bl + bw + 16, bb + 12, fs, fmt("NLC DENSITY %.2f", s.ship_density), kCyan, 1.8f);
  }
}

}  // namespace avionics
