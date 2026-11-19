// =====================================================================
//  Tutoriel DEM 2D -- Étape 2 : rebond sur le sol (contact normal)
//
//  Compilation : g++ -O2 -std=c++17 -Wall etape2_rebond.cpp -o etape2
// =====================================================================
#include <cmath>
#include <cstdio>
#include <iostream>





// ===================== STRUCTURES ====================================

struct Vec2 {
  double x = 0.0, y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(double s, Vec2 a) { return {s * a.x, s * a.y}; }
inline Vec2& operator+=(Vec2& a, Vec2 b) {
  a.x += b.x;
  a.y += b.y;
  return a;
}
inline Vec2 operator/(Vec2 a, double b) { return {a.x / b, a.y / b}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

struct Particle {
  Vec2 pos;    // position du centre (m)
  Vec2 vel;    // vitesse (m/s)
  Vec2 force;  // force résultante
  double radius = 0.0;
  double mass = 0.0;
};


// ===================== PARAMÈTRES ====================================
const double radius = 0.01;           // rayon de la particule (m)
const double rho = 2500.0;            // masse volumique (kg/m^3)
const double h0 = 0.1;                // hauteur initiale du centre (m)
const double v0 = 0.5;                // vitesse initiale de la particule (m/s)
const double kn = 1.0e7;              // raideur normale
const double re = 0.5;                // coefficient de restitution visé
const double t_end = 0.5;             // durée simulée (s)
const double dt_factor = 50.0;        // nombre de pas de temps par contact

const Vec2 gravity{0.0, -9.81};       // gravité (m/s^2)
const Vec2 floor_point{0.0, 0.0};     // point du sol (m)
const Vec2 floor_normal{0.0, 1.0};    // normale du sol (m)



//==================== FONCTIONS ====================================

// Compute the damping ratio from the restitution coefficient
double damping_ratio_from_restitution(double e) {
  const double le = std::log(e);
  return -le / std::sqrt(M_PI * M_PI + le * le);
}

//compute the force on a particle due to gravity and contact with the floor
Vec2 compute_force(const Particle& p, double cn) {
  Vec2 f = p.mass * gravity;
  const double overlap = p.radius - dot(p.pos - floor_point, floor_normal);
  if (overlap > 0.0) {
    const double vn = dot(p.vel, floor_normal);  // < 0 quand la particule descend
    double fn = kn * overlap - cn * vn;
    if (fn < 0.0) fn = 0.0;                      // le contact ne tire pas
    f += fn * floor_normal;
  }
  return f;
}


// ===================== PROGRAMME PRINCIPAL ====================================
int main() {
 
  // Particule
  Particle p;
  p.radius = radius;
  p.mass = rho * M_PI * p.radius * p.radius;
  p.pos = {0.0, h0};
  p.vel = {v0, 0.0};

  // amortissement
  const double cn = 2.0 * std::sqrt(kn * p.mass) * damping_ratio_from_restitution(re);

  // time step
  double dtcritical = 2.0* std::sqrt(p.mass/ kn);
  const double dt = dtcritical/dt_factor;

  // force intiale
  p.force = compute_force(p, cn);
  bool in_contact = false;
  double v_in =0.0, max_overlap = 0.0;

  //nbre de pas de temps
  const int nsteps = static_cast<int>(t_end / dt);

  std::printf("# m = %.4f kg/m, tc = %.3e s, dt = %.3e s, %d pas\n", p.mass, dtcritical, dt,
              nsteps);
  for (int step = 1; step < nsteps; ++step) {

    //--- velocity Verlet ---

    p.vel += (0.5 * dt / p.mass) * p.force;  // kick
    p.pos += dt * p.vel;                     // drift
    p.force = compute_force(p, cn);              // forces au temps t + dt (avec contact)
    p.vel += (0.5 * dt / p.mass) * p.force;  // kick

   // --- mesures : début et fin de chaque contact ---
    const double t = step * dt;
    const double overlap = p.radius - dot(p.pos - floor_point, floor_normal);
    if (overlap > max_overlap) max_overlap = overlap;
    if (!in_contact && overlap > 0.0) {  // le contact commence
      in_contact = true;
      v_in = p.vel.y;
    } else if (in_contact && overlap <= 0.0) {  // le contact se termine
      in_contact = false;
      std::printf("t = %.4f s : v_in = %+.4f, v_out = %+.4f, e = %.4f\n", t, v_in, p.vel.y,
                  -p.vel.y / v_in);
    }
  }
  std::printf("e visé = %.3f, interpénétration max / rayon = %.2e\n", re,
              max_overlap / radius);
  return 0;
}
