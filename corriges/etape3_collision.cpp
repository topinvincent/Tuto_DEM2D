// =====================================================================
//  Tutoriel DEM 2D -- Étape 3 : collision oblique de deux disques
//
//  Compilation : g++ -O2 -std=c++17 -Wall -I../viz etape3_collision.cpp -o etape3
//  Animation   : ouvrir sortie_etape3/viewer.html dans un navigateur
// =====================================================================
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include "./../viz/dem_viz.hpp"





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
inline Vec2& operator-=(Vec2& a, Vec2 b) {
  a.x -= b.x;
  a.y -= b.y;
  return a;
}
inline Vec2 operator/(Vec2 a, double b) { return {a.x / b, a.y / b}; }

inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double norm(Vec2 a) { return std::sqrt(dot(a, a)); }

struct Particle {
  Vec2 pos;    // position du centre (m)
  Vec2 vel;    // vitesse (m/s)
  Vec2 force;  // force résultante
  double radius = 0.0;
  double mass = 0.0;
};

struct Params {
  Vec2 gravity{0.0, -9.81};
  double kn = 1.0e7;       // raideur normale
  double damp_rate = 0.0;  // taux d'amortissement (nom d'exaDEM)
  double dt = 1.0e-5;      // pas de temps
};

struct System {
  std::vector<Particle> particles;
  Params prm;
};

// ===================== PARAMÈTRES ====================================
const double rho = 2500.0;            // masse volumique (kg/m^3)
const double re = 0.8;                // coefficient de restitution visé
const double t_end = 0.5;             // durée simulée (s)
const double dt_factor = 50.0;        // nombre de pas de temps par contact
const int n_save = 40;          // une image tous les n_save pas


//==================== FONCTIONS ====================================

// Compute the damping ratio from the restitution coefficient
double damping_ratio_from_restitution(double e) {
  const double le = std::log(e);
  return -le / std::sqrt(M_PI * M_PI + le * le);
}

// n : normale unitaire dirigée de l'objet j vers la particule i
// vn = (v_i - v_j).n : négative lorsque les objets se rapprochent
double normal_force(const Params& prm, double overlap, double vn, double meff) {
  const double cn = 2.0 * prm.damp_rate * std::sqrt(meff * prm.kn);
  const double fn = prm.kn * overlap - cn * vn;
  return fn > 0.0 ? fn : 0.0;  // pas de force attractive
}

//Calcul de la force de contact normale entre deux particules
void particle_particle(const Params& prm, Particle& pi, Particle& pj) {
  const Vec2 branch = pi.pos - pj.pos;  // vecteur j -> i
  const double dist = norm(branch);
  const double overlap = pi.radius + pj.radius - dist;
  if (overlap <= 0.0) return;  // pas de contact

  const Vec2 n = (1.0 / dist) * branch;
  const double vn = dot(pi.vel - pj.vel, n);
  const double meff = pi.mass * pj.mass / (pi.mass + pj.mass);
  const double fn = normal_force(prm, overlap, vn, meff);
  pi.force += fn * n;  // action ...
  pj.force -= fn * n;  // ... et réaction
}


void compute_forces(System& sys) {
  auto& P = sys.particles;
  for (auto& p : P) p.force = p.mass * sys.prm.gravity;
  //Test de toutes les paires
  for (size_t i = 0; i < P.size(); ++i)
    for (size_t j = i + 1; j < P.size(); ++j) particle_particle(sys.prm, P[i], P[j]);
}

// Calcul d'une itération de la simulation
void exec_one_iter(System& sys) {
  const double dt = sys.prm.dt;
  for (auto& p : sys.particles) {
    p.vel += (0.5 * dt / p.mass) * p.force;
    p.pos += dt * p.vel;
  }
  compute_forces(sys);
  for (auto& p : sys.particles) p.vel += (0.5 * dt / p.mass) * p.force;
}

// Crée un disque de masse volumique rho
Particle make_disk(Vec2 pos, Vec2 vel, double radius, double rho) {
  Particle p;
  p.pos = pos;
  p.vel = vel;
  p.radius = radius;
  p.mass = rho * M_PI * radius * radius;
  return p;
}
// Pas de temps : une fraction de la durée de contact pour la plus petite masse
double critical_time(double mmin, double kn) { return 2.0 * std::sqrt(mmin / kn); }

// ===================== PROGRAMME PRINCIPAL ====================================
// COLLISION 2 DISQUES OBLIQUES

// Quantité de mouvement totale du système
Vec2 total_momentum(const System& sys) {
  Vec2 q;
  for (const auto& p : sys.particles) q += p.mass * p.vel;
  return q;
}

// ===================== PROGRAMME PRINCIPAL ===========================
int main() {
 
  // definition du système
  System sys;
  sys.prm.gravity = {0.0, 0.0};
  sys.prm.damp_rate = damping_ratio_from_restitution(re);
  sys.particles.push_back(make_disk({0.0,0.0},{1.0,0.0},0.010, rho));
  sys.particles.push_back(make_disk({0.05, 0.008}, {-0.5, 0.0}, 0.005, rho));
  // références vers les deux disques (pour alléger l'écriture)
  const Particle& a = sys.particles[0];
  const Particle& b = sys.particles[1];
  const double meff = a.mass * b.mass / (a.mass + b.mass);
  sys.prm.dt = critical_time(meff, sys.prm.kn) / dt_factor;
  const int nsteps = static_cast<int>(t_end / sys.prm.dt);

  // enregistreur d'images (dossier de sortie)
  viz::Recorder rec("sortie_etape3");

  const Vec2 q0 = total_momentum(sys);
  compute_forces(sys);
  bool was_in_contact = false;
  double vn_in = 0.0, vt_in = 0.0;
  for (int step = 1; step < nsteps; ++step) {
    exec_one_iter(sys);
    if (step % n_save == 0) {  // une image tous les n_save pas
      viz::Scene scene;
      scene.time = step * sys.prm.dt;
      for (const auto& p : sys.particles)
        scene.disks.push_back({p.pos.x, p.pos.y, p.radius, 0.0, p.vel.x, p.vel.y});
      scene.xmin = -0.02;
      scene.xmax = 0.07;
      scene.ymin = -0.03;
      scene.ymax = 0.04;
      rec.save(scene);
    }

    // Vitesse relative décomposée dans le repère du contact (n, t)
    const Vec2 n = (1.0 / norm(a.pos - b.pos)) * (a.pos - b.pos);
    const Vec2 t{-n.y, n.x};
    const double vn = dot(a.vel - b.vel, n);
    const double vt = dot(a.vel - b.vel, t);

    // --- début et fin du choc ---
    const bool in_contact = norm(a.pos - b.pos) < a.radius + b.radius;
    if (in_contact && !was_in_contact) {  // début du choc
      vn_in = vn;
      vt_in = vt;
    }
    if (!in_contact && was_in_contact) {  // fin du choc
      const Vec2 q1 = total_momentum(sys);
      std::printf("vn avant = %+.4f, vn après = %+.4f -> e = %.4f\n", vn_in, vn,
                  -vn / vn_in);
      std::printf("vt avant = %+.4f, vt après = %+.4f (inchangée : pas de frottement)\n",
                  vt_in, vt);

      std::printf("quantité de mouvement : avant (%.6e, %.6e), après (%.6e, %.6e)\n",
                  q0.x, q0.y, q1.x, q1.y);
      break;
    }
    was_in_contact = in_contact;
  }
  std::printf("%d images : ouvrir sortie_etape3/viewer.html\n", rec.frames());
  return 0;
}
