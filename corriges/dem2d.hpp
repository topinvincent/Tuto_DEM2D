// dem2d.hpp : le moteur DEM 2D construit aux étapes 1 à 7
// (+ résistance au roulement, section 5.5)
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <tuple>
#include <vector>

#include "./../viz/dem_viz.hpp"

// ===================== VECTEURS ====================================

struct Vec2 {
  double x = 0.0, y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(double s, Vec2 a) { return {s * a.x, s * a.y}; }
inline Vec2 operator-(Vec2 a) { return {-a.x, -a.y}; }
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

// a x b, pour deux vecteurs du plan : un scalaire (la composante z)
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
// w ez x r, pour une rotation w autour de z et un vecteur du plan : un vecteur du plan
inline Vec2 cross(double w, Vec2 r) { return {-w * r.y, w * r.x}; }



inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double norm(Vec2 a) { return std::sqrt(dot(a, a)); }

// ===================== DONNEES ====================================

struct Particle {
  Vec2 pos, vel, force;
  double angle = 0.0;   // rotation (rad)
  double omega = 0.0;   // vitesse angulaire (rad/s)
  double torque = 0.0;  // moment résultant
  double radius = 0.0, mass = 0.0, inertia = 0.0;
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
  double kt = 0.8e7;  // raideur tangentielle
  double mu = 0.5;    // coefficient de frottement
  double DVerlet = 1.0e-4;  // distance de Verlet (marge de la liste de voisins)
  double kr = 0.0;          // raideur en rotation (0 : pas de résistance au roulement)
  double mur = 0.0;         // seuil du moment = mur * fn (mur est une longueur)
};

// Une interaction entre la particule i et un objet j
// (une autre particule, ou une paroi si wall == true).
struct Interaction {
  int i = 0;
  int j = 0;
  bool wall = false;
  Vec2 ft;           // force tangentielle : c'est la MÉMOIRE du contact
  double mom = 0.0;  // moment de résistance au roulement (mémoire aussi)
  double fn = 0.0;   // force normale (gardée pour la visualisation)
  Vec2 contact;      // point de contact (pour la visualisation)
};

struct System {
  std::vector<Particle> particles;
  std::vector<Wall> walls;
  std::vector<Interaction> interactions;
  Params prm;
  double time = 0.0;
  std::vector<Vec2> pos_at_last_build;  // positions lors de la dernière construction
  int nb_rebuilds = 0;

};

// ==================== LOIS DE CONTACT ====================================

// Compute the damping ratio from the restitution coefficient
inline double damping_ratio_from_restitution(double e) {
  const double le = std::log(e);
  return -le / std::sqrt(M_PI * M_PI + le * le);
}

inline Vec2 contact_force(const Params& prm, double dn, Vec2 n, Vec2 vrel, double meff, Vec2& ft,
                   double& fn) {
  // --- partie normale : ressort + amortisseur, sans attraction
  const double damp = prm.damp_rate * 2.0 * std::sqrt(prm.kn * meff);
  const double vn = dot(vrel, n);
  fn = prm.kn * dn - damp * vn;
  if (fn < 0.0) fn = 0.0;

  // --- partie tangentielle : ressort incrémental
  const Vec2 t{-n.y, n.x};
  ft = dot(ft, t) * t;                 // on ramène ft dans le plan tangent actuel
  const Vec2 vt = vrel - vn * n;
  ft -= (prm.kt * prm.dt) * vt;        // incrément élastique

  // --- seuil de Coulomb
  const double threshold = prm.mu * prm.kn * dn;
  const double ft_norm = norm(ft);
  if (ft_norm > threshold) ft = (threshold / ft_norm) * ft;

  return fn * n + ft;                  // force exercée par j sur i
}

// Résistance au roulement : ressort de torsion incrémental, plafonné à mur * fn.
//   dw : vitesse de rotation de i par rapport à j
inline double rolling_moment(const Params& prm, double dw, double fn, double& mom) {
  if (prm.kr <= 0.0) return 0.0;  // résistance désactivée
  mom -= prm.kr * dw * prm.dt;    // incrément élastique
  const double threshold = prm.mur * fn;
  if (std::fabs(mom) > threshold) mom = std::copysign(threshold, mom);
  return mom;
}

inline bool particle_particle(const Params& prm, Interaction& I, Particle& pi, Particle& pj) {
  const Vec2 branch = pi.pos - pj.pos;
  const double dist = norm(branch);
  const double dn = pi.radius + pj.radius - dist;
  if (dn <= 0.0) return false;

  const Vec2 n = (1.0 / dist) * branch;
  const Vec2 c = pi.pos - (pi.radius - 0.5 * dn) * n;
  const Vec2 vi = pi.vel + cross(pi.omega, c - pi.pos);
  const Vec2 vj = pj.vel + cross(pj.omega, c - pj.pos);
  const double meff = pi.mass * pj.mass / (pi.mass + pj.mass);

  const Vec2 f = contact_force(prm, dn, n, vi - vj, meff, I.ft, I.fn);
  const double m = rolling_moment(prm, pi.omega - pj.omega, I.fn, I.mom);
  pi.force += f;
  pi.torque += cross(c - pi.pos, f) + m;
  pj.force -= f;
  pj.torque += cross(c - pj.pos, -f) - m;
  I.contact = c;
  return true;
}


inline bool particle_wall(const Params& prm, Interaction& I, Particle& p, const Wall& w) {
  const double overlap = p.radius - dot(p.pos - w.point, w.normal);
  if (overlap <= 0.0) return false;
  const Vec2 n = w.normal;// Normale de la paroi vers la particule
  const Vec2 c = p.pos - (p.radius - 0.5 * overlap) * n; // point de contact
  const Vec2 vrel = p.vel + cross(p.omega, c - p.pos);// vitesse relative de la particule par rapport à la paroi
  const double meff = p.mass; // masse effective pour le contact avec la paroi ici mass de la particule

  const Vec2 f = contact_force(prm, overlap, n, vrel, meff, I.ft, I.fn);
  const double m = rolling_moment(prm, p.omega, I.fn, I.mom);  // la paroi ne tourne pas
  p.force += f;
  p.torque += cross(c - p.pos, f) + m;  // moment de la force + résistance au roulement
  I.contact = c;
  return true;
}


//===================== OBB (Oriented Bounding Box) ====================================


struct OBB {
  Vec2 center;
  Vec2 axis[2];                  // deux axes unitaires orthogonaux
  double extent[2] = {0.0, 0.0}; // demi-longueurs selon chaque axe
};

inline OBB particle_obb(const Particle& p) {
  OBB b;
  b.center = p.pos;
  b.axis[0] = {std::cos(p.angle), std::sin(p.angle)};
  b.axis[1] = {-std::sin(p.angle), std::cos(p.angle)};
  b.extent[0] = b.extent[1] = p.radius;
  return b;
}

inline void enlarge(OBB& b, double d) {  // comme OBB::enlarge dans Rockable
  b.extent[0] += d;
  b.extent[1] += d;
}



// "Rayon" de l'OBB projetée sur une direction unitaire u
inline double projected_radius(const OBB& b, Vec2 u) {
  return b.extent[0] * std::fabs(dot(b.axis[0], u)) +
         b.extent[1] * std::fabs(dot(b.axis[1], u));
}

inline bool intersect(const OBB& a, const OBB& b) {
  const Vec2 d = b.center - a.center;
  const Vec2 axes[4] = {a.axis[0], a.axis[1], b.axis[0], b.axis[1]};
  for (const Vec2& u : axes)
    if (std::fabs(dot(d, u)) > projected_radius(a, u) + projected_radius(b, u)) return false;
  return true;
}

// Une OBB touche le demi-plan situé derrière une paroi
inline bool intersect(const OBB& b, const Wall& w) {
  return dot(b.center - w.point, w.normal) <= projected_radius(b, w.normal);
}

//===================== LISTE DES INTERACTIONS ====================================

// Recopie la mémoire (ft, mom) des interactions qui existaient déjà.
inline void transfer_history(const std::vector<Interaction>& old_list,
                      std::vector<Interaction>& new_list) {
  std::map<std::tuple<int, int, bool>, Interaction> memory;
  for (const auto& I : old_list) memory[{I.i, I.j, I.wall}] = I;
  for (auto& I : new_list) {
    auto it = memory.find({I.i, I.j, I.wall});
    if (it != memory.end()) {
      I.ft = it->second.ft;
      I.mom = it->second.mom;
    }
  }
}
inline Interaction new_interaction(int i, int j, bool wall) {
  Interaction I;
  I.i = i;
  I.j = j;
  I.wall = wall;
  return I;
}

inline bool need_update(const System& sys) {
  if (sys.pos_at_last_build.size() != sys.particles.size()) return true;
  const double limit2 = 0.25 * sys.prm.DVerlet * sys.prm.DVerlet;
  for (size_t i = 0; i < sys.particles.size(); ++i) {
    const Vec2 d = sys.particles[i].pos - sys.pos_at_last_build[i];
    if (dot(d, d) > limit2) return true;
  }
  return false;
}


inline void build_verlet_list_brute_force(const System& sys, std::vector<Interaction>& list) {
  const auto& P = sys.particles;
  const int n = (int)P.size();
  std::vector<OBB> obb(n);
  for (int i = 0; i < n; ++i) {
    obb[i] = particle_obb(P[i]);
    enlarge(obb[i], 0.5 * sys.prm.DVerlet);
  }
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j)
      if (intersect(obb[i], obb[j])) list.push_back(new_interaction(i, j, false));
    for (int w = 0; w < (int)sys.walls.size(); ++w)
      if (intersect(obb[i], sys.walls[w])) list.push_back(new_interaction(i, w, true));
  }
}

inline void build_verlet_list_cells(const System& sys, std::vector<Interaction>& list) {
  const auto& P = sys.particles;
  const int n = (int)P.size();
  if (n == 0) return;

  // OBB élargies, plus grand rayon et boîte [lo, hi] contenant tous les centres
  std::vector<OBB> obb(n);
  double rmax = 0.0;
  Vec2 lo = P[0].pos, hi = P[0].pos;
  for (int i = 0; i < n; ++i) {
    obb[i] = particle_obb(P[i]);
    enlarge(obb[i], 0.5 * sys.prm.DVerlet);
    rmax = std::max(rmax, P[i].radius);
    lo = {std::min(lo.x, P[i].pos.x), std::min(lo.y, P[i].pos.y)};
    hi = {std::max(hi.x, P[i].pos.x), std::max(hi.y, P[i].pos.y)};
  }
  const double cell_size = 2.0 * std::sqrt(2.0) * (rmax + 0.5 * sys.prm.DVerlet);
  const int nx = (int)((hi.x - lo.x) / cell_size) + 1;
  const int ny = (int)((hi.y - lo.y) / cell_size) + 1;

  // Rangement des particules dans les cellules
  std::vector<std::vector<int>> cells(nx * ny);
  std::vector<int> cx(n), cy(n);
  for (int i = 0; i < n; ++i) {
    cx[i] = (int)((P[i].pos.x - lo.x) / cell_size);
    cy[i] = (int)((P[i].pos.y - lo.y) / cell_size);
    cells[cy[i] * nx + cx[i]].push_back(i);
  }

  // Pour chaque particule, on ne teste que les 9 cellules autour d'elle
  for (int i = 0; i < n; ++i) {
    for (int ky = cy[i] - 1; ky <= cy[i] + 1; ++ky)
      for (int kx = cx[i] - 1; kx <= cx[i] + 1; ++kx) {
        if (kx < 0 || ky < 0 || kx >= nx || ky >= ny) continue;
        for (int j : cells[ky * nx + kx])
          if (j > i && intersect(obb[i], obb[j])) list.push_back(new_interaction(i, j, false));
      }
    for (int w = 0; w < (int)sys.walls.size(); ++w)
      if (intersect(obb[i], sys.walls[w])) list.push_back(new_interaction(i, w, true));
  } 
}



inline void update_interactions(System& sys, bool force_rebuild = false) {
  if (!force_rebuild && !need_update(sys)) return;  // la liste est encore valable

  std::vector<Interaction> list;
  build_verlet_list_cells(sys, list);        // grille de cellules liées (étape 7)
  transfer_history(sys.interactions, list);  // on garde la mémoire des contacts
  sys.interactions.swap(list);

  sys.pos_at_last_build.resize(sys.particles.size());
  for (size_t i = 0; i < sys.particles.size(); ++i)
    sys.pos_at_last_build[i] = sys.particles[i].pos;
  ++sys.nb_rebuilds;
}


//==================== BOUCLE DEM ====================================

inline void compute_forces(System& sys) {
  for (auto& p : sys.particles) {
    p.force = p.mass * sys.prm.gravity;
    p.torque = 0.0;
  }
  for (auto& I : sys.interactions) {
    Particle& pi = sys.particles[I.i];
    const bool active = I.wall ? particle_wall(sys.prm, I, pi, sys.walls[I.j])
                               : particle_particle(sys.prm, I, pi, sys.particles[I.j]);
    if (!active) {  // plus de contact : on oublie la mémoire
      I.ft = {0.0, 0.0};
      I.mom = 0.0;
      I.fn = 0.0;
    }
  }
}


// Calcul d'une itération de la simulation
inline void exec_one_iter(System& sys) {
  const double dt = sys.prm.dt;
  for (auto& p : sys.particles) {
    p.vel += (0.5 * dt / p.mass) * p.force;
    p.pos += dt * p.vel;
    p.omega += (0.5 * dt / p.inertia) * p.torque;
    p.angle += dt * p.omega;
  }
  update_interactions(sys);
  compute_forces(sys);
  for (auto& p : sys.particles){ 
    p.vel += (0.5 * dt / p.mass) * p.force;
    p.omega += (0.5 * dt / p.inertia) * p.torque;
  }
  sys.time += dt;  // le système connaît son temps physique
}
//==================== UTILITAIRES ====================================


// Crée un disque de masse volumique rho
inline Particle make_disk(Vec2 pos, Vec2 vel, double radius, double rho) {
  Particle p;
  p.pos = pos;
  p.vel = vel;
  p.radius = radius;
  p.mass = rho * M_PI * radius * radius;
  p.inertia = 0.5 * p.mass * radius * radius;
  return p;
}

// Pas de temps : une fraction de la durée de contact pour la plus petite masse
inline double critical_time(double mmin, double kn) { return 2.0 * std::sqrt(mmin / kn); }

// Énergie cinétique de translation et de rotation
inline double kinetic_energy(const System& sys) {
  double ek = 0.0;
  for (const auto& p : sys.particles)
    ek += 0.5 * p.mass * dot(p.vel, p.vel) + 0.5 * p.inertia * p.omega * p.omega;
  return ek;
}


// Quantité de mouvement totale du système
inline Vec2 total_momentum(const System& sys) {
  Vec2 q;
  for (const auto& p : sys.particles) q += p.mass * p.vel;
  return q;
}

inline double potential_energy(const System& sys) {  // référence : y = 0
  double ep = 0.0;
  for (const auto& p : sys.particles) ep -= p.mass * dot(sys.prm.gravity, p.pos);
  return ep;
}

// visualisation
inline viz::Scene make_scene(const System& sys, double t) {
  viz::Scene scene;
  scene.time = t;
  for (const auto& p : sys.particles)  // x, y, rayon, angle, vx, vy
    scene.disks.push_back({p.pos.x, p.pos.y, p.radius, p.angle, p.vel.x, p.vel.y});
  for (const auto& w : sys.walls)      // un point et la normale
    scene.walls.push_back({w.point.x, w.point.y, w.normal.x, w.normal.y});
  for (const auto& I : sys.interactions) {  // contacts actifs : segment + force normale
    if (I.fn <= 0.0) continue;
    const Vec2 a = sys.particles[I.i].pos;
    const Vec2 b = I.wall ? I.contact : sys.particles[I.j].pos;
    scene.contacts.push_back({a.x, a.y, b.x, b.y, I.fn});
  }
  return scene;
}
