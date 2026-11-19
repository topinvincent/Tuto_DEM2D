// =====================================================================
//  Tutoriel DEM 2D -- Étape 4 : un ensemble de grains dans une boîte
//
//  Compilation : g++ -O2 -std=c++17 -Wall -I../viz etape4_boite.cpp -o etape4
//  Animation   : ouvrir sortie_etape4/viewer.html dans un navigateur
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

struct Wall {
  Vec2 point;
  Vec2 normal;
};

struct Params {
  Vec2 gravity{0.0, -9.81};
  double kn = 1.0e7;       // raideur normale
  double damp_rate = 0.0;  // taux d'amortissement (nom d'exaDEM)
  double dt = 1.0e-5;      // pas de temps
};

struct System {
  std::vector<Particle> particles;
  std::vector<Wall> walls;
  Params prm;
};

// ===================== PARAMÈTRES ====================================
const double rho = 2500.0;            // masse volumique (kg/m^3)
const double re = 0.5;                // coefficient de restitution visé
const double radius = 0.004;          // rayon des disques (m)
const double width = 0.1;             // largeur de la boîte (m)
const int n_rows = 4;                // nombre de rangées de disques
const int n_cols = 5;                // nombre de colonnes de disques
const double t_end = 0.5;             // durée simulée (s)
const double dt_factor = 50.0;        // nombre de pas de temps par contact
const double t_save = 0.005;          // une image tous les t_save secondes


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

void particle_wall(const Params& prm, Particle& p, const Wall& w) {
  const double overlap = p.radius - dot(p.pos - w.point, w.normal);
  if (overlap <= 0.0) return;
  const double vn = dot(p.vel, w.normal);
  const double fn = normal_force(prm, overlap, vn, p.mass);
  p.force += fn * w.normal;
}

void compute_forces(System& sys) {
  auto& P = sys.particles;
  for (auto& p : P) p.force = p.mass * sys.prm.gravity;
  //Test de toutes les paires
  for (size_t i = 0; i < P.size(); ++i)
    for (size_t j = i + 1; j < P.size(); ++j) particle_particle(sys.prm, P[i], P[j]);
  //Test de toutes les particules et murs
  for (auto& p : P)
    for (const auto& w : sys.walls) particle_wall(sys.prm, p, w);
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


// -- Analyses 
// Quantité de mouvement totale du système
Vec2 total_momentum(const System& sys) {
  Vec2 q;
  for (const auto& p : sys.particles) q += p.mass * p.vel;
  return q;
}

double kinetic_energy(const System& sys) {
  double ek = 0.0;
  for (const auto& p : sys.particles) ek += 0.5 * p.mass * dot(p.vel, p.vel);
  return ek;
}

double potential_energy(const System& sys) {  // référence : y = 0
  double ep = 0.0;
  for (const auto& p : sys.particles) ep -= p.mass * dot(sys.prm.gravity, p.pos);
  return ep;
}

// visualisation
viz::Scene make_scene(const System& sys, double t) {
  viz::Scene scene;
  scene.time = t;
  for (const auto& p : sys.particles)  // x, y, rayon, angle, vx, vy
    scene.disks.push_back({p.pos.x, p.pos.y, p.radius, 0.0, p.vel.x, p.vel.y});
  for (const auto& w : sys.walls)      // un point et la normale
    scene.walls.push_back({w.point.x, w.point.y, w.normal.x, w.normal.y});
  return scene;
}
// ===================== PROGRAMME PRINCIPAL ===========================
int main() {
 
  // definition du système
  System sys;
  //sys.prm.gravity = {0.0, -9.81}; deja définie par defaut dans la structure Params
  sys.prm.damp_rate = damping_ratio_from_restitution(re);

  // La boîte : un sol et deux murs verticaux
  sys.walls.push_back({{0.0, 0.0}, {0.0, 1.0}});     // sol
  sys.walls.push_back({{0.0, 0.0}, {1.0, 0.0}});     // mur gauche
  sys.walls.push_back({{width, 0.0}, {-1.0, 0.0}});  // mur droit

  // Les disques, rangée par rangée
  for (int row = 0; row < n_rows; ++row) {
    for (int col = 0; col < n_cols; ++col) {
      const double x = 0.01 + 0.02 * col + 0.002 * row;
      const double y = 0.02 + 0.012 * row;
      sys.particles.push_back(make_disk({x, y}, {0.0, 0.0}, radius, rho));
    }
  }

  // Pas de temps : contact entre deux disques identiques (m_eff = m / 2)
  const double meff = 0.5 * sys.particles[0].mass;
  sys.prm.dt = critical_time(meff, sys.prm.kn) / dt_factor;

  const int nsteps = static_cast<int>(t_end / sys.prm.dt);
  const int n_save = static_cast<int>(t_save / sys.prm.dt);  // pas entre deux images
  const int n_print = nsteps / 10;                            // pas entre deux bilans

  // enregistreur d'images (dossier de sortie)
  viz::Recorder rec("sortie_etape4");

  compute_forces(sys);
  rec.save(make_scene(sys, 0.0));  // état initial
  std::printf("# t (s)   Ec (J/m)    Ep (J/m)\n");

  for (int step = 1; step < nsteps; ++step) {
    exec_one_iter(sys);
    const double t = step * sys.prm.dt;
    if (step % n_save == 0) {
      rec.save(make_scene(sys, t));
    }
    if (step % n_print == 0) {
      const double ek = kinetic_energy(sys);
      const double ep = potential_energy(sys);
      std::cout << t << " " << ek << " " << ep << std::endl;
    }

  }
  std::printf("%d images : ouvrir sortie_etape4/viewer.html\n", rec.frames());
  return 0;
}
