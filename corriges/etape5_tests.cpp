// =====================================================================
//  Tutoriel DEM 2D -- Étape 5 (validation) : cas tests analytiques
//
//  Nouveautés :
//    - degré de liberté de rotation (angle, vitesse angulaire, couple) ;
//    - vitesse relative au point de contact ;
//    - force tangentielle par ressort incrémental + seuil de Coulomb,
//      comme la loi "Hooke" d'exaDEM ou la loi "Default" de Rockable ;
//    - une liste d'interactions (Interaction) qui MÉMORISE la force
//      tangentielle d'un pas de temps à l'autre (historique du contact) ;
//    - une résistance au roulement (moment de contact), comme kr / mur
//      dans Rockable et exaDEM ;
//    - tests analytiques : glissement -> roulement, plan incliné, freinage
//      par la résistance au roulement.
//
//  Compilation : g++ -O2 -std=c++17 etape5_tests.cpp -o etape5_tests
// =====================================================================
#include <cmath>
#include <cstdio>
#include <map>
#include <tuple>
#include <vector>


// ------------------------------ Vec2 ---------------------------------
struct Vec2 {
  double x = 0.0, y = 0.0;
};
inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator-(Vec2 a) { return {-a.x, -a.y}; }
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
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double norm(Vec2 a) { return std::sqrt(dot(a, a)); }
// En 2D, le produit vectoriel de deux vecteurs du plan est porté par z :
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
// ... et celui d'une rotation w (portée par z) avec un vecteur du plan est dans le plan :
inline Vec2 cross(double w, Vec2 r) { return {-w * r.y, w * r.x}; }

// --------------------------- Données ---------------------------------
struct Particle {
  Vec2 pos, vel, force;
  double angle = 0.0;   // rotation (rad)
  double omega = 0.0;   // vitesse angulaire (rad/s)
  double torque = 0.0;  // moment résultant
  double radius = 0.0, mass = 0.0, inertia = 0.0;
};

struct Wall {
  Vec2 point, normal;
};

// Une interaction potentielle entre la particule i et un objet j
// (une autre particule, ou une paroi si wall == true).
struct Interaction {
  int i = 0;
  int j = 0;
  bool wall = false;
  Vec2 ft;           // force tangentielle : c'est la MÉMOIRE du contact
  double mom = 0.0;  // moment de résistance au roulement (mémoire aussi)
  double fn = 0.0;   // force normale (stockée pour la visualisation)
  Vec2 contact;      // point de contact (pour la visualisation)
};

struct Params {
  Vec2 gravity{0.0, -9.81};
  double kn = 1.0e7;       // raideur normale
  double kt = 0.8e7;       // raideur tangentielle
  double mu = 0.5;         // coefficient de frottement
  double kr = 0.0;         // raideur en rotation (résistance au roulement)
  double mur = 0.0;        // seuil du moment = mur * fn (mur est une longueur)
  double damp_rate = 0.0;  // taux d'amortissement normal
  double dt = 1.0e-5;
};

struct System {
  std::vector<Particle> particles;
  std::vector<Wall> walls;
  std::vector<Interaction> interactions;
  Params prm;
  double time = 0.0;
};

double damping_ratio_from_restitution(double e) {
  const double le = std::log(e);
  return -le / std::sqrt(M_PI * M_PI + le * le);
}

// ------------------------- Loi de contact ----------------------------
// Loi ressort-amortisseur normale + ressort tangentiel avec seuil de Coulomb.
//   dn   : interpénétration (> 0 si contact)
//   n    : normale unitaire, dirigée de l'objet j vers la particule i
//   vrel : vitesse de i par rapport à j, AU POINT DE CONTACT
//   meff : masse effective
//   ft   : force tangentielle du pas précédent, mise à jour ici
// Retourne la force exercée par j sur i ; fn reçoit la norme de la force normale.
Vec2 contact_force(const Params& prm, double dn, Vec2 n, Vec2 vrel, double meff, Vec2& ft,
                   double& fn) {
  // --- partie normale : ressort + amortisseur, sans attraction
  const double damp = prm.damp_rate * 2.0 * std::sqrt(prm.kn * meff);
  const double vn = dot(vrel, n);
  fn = prm.kn * dn - damp * vn;
  if (fn < 0.0) fn = 0.0;

  // --- partie tangentielle : ressort incrémental
  // Le repère du contact a pu tourner depuis le pas précédent : on ramène
  // l'ancienne force dans le plan tangent actuel.
  const Vec2 t{-n.y, n.x};
  ft = dot(ft, t) * t;
  // Incrément élastique : -kt * (déplacement tangentiel pendant dt)
  const Vec2 vt = vrel - vn * n;
  ft -= (prm.kt * prm.dt) * vt;

  // --- seuil de Coulomb (sur la partie élastique de la force normale,
  //     comme dans exaDEM et Rockable) : glissement si |ft| > mu * kn * dn
  const double threshold = prm.mu * prm.kn * dn;
  const double ft_norm = norm(ft);
  if (ft_norm > threshold) ft = (threshold / ft_norm) * ft;

  return fn * n + ft;
}

// Résistance au roulement (même principe que la force tangentielle) :
// un "ressort de torsion" incrémental, plafonné à mur * fn.
//   dw : vitesse de rotation de i par rapport à j
double rolling_moment(const Params& prm, double dw, double fn, double& mom) {
  if (prm.kr <= 0.0) return 0.0;
  mom -= prm.kr * dw * prm.dt;
  const double threshold = prm.mur * fn;
  if (std::fabs(mom) > threshold) mom = std::copysign(threshold, mom);
  return mom;
}

// Contact entre deux particules : calcule et applique les efforts.
// Retourne false si les particules ne se touchent pas.
bool particle_particle(const Params& prm, Interaction& I, Particle& pi, Particle& pj) {
  const Vec2 branch = pi.pos - pj.pos;
  const double dist = norm(branch);
  const double dn = pi.radius + pj.radius - dist;
  if (dn <= 0.0) return false;

  const Vec2 n = (1.0 / dist) * branch;
  // Point de contact : au milieu de la zone d'interpénétration
  const Vec2 c = pi.pos - (pi.radius - 0.5 * dn) * n;
  // Vitesse de chaque particule au point de contact : v + w x (c - x)
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

bool particle_wall(const Params& prm, Interaction& I, Particle& p, const Wall& w) {
  const double dn = p.radius - dot(p.pos - w.point, w.normal);
  if (dn <= 0.0) return false;
  const Vec2 n = w.normal;
  const Vec2 c = p.pos - (p.radius - 0.5 * dn) * n;
  const Vec2 v = p.vel + cross(p.omega, c - p.pos);  // la paroi est immobile
  const Vec2 f = contact_force(prm, dn, n, v, p.mass, I.ft, I.fn);
  const double m = rolling_moment(prm, p.omega, I.fn, I.mom);
  p.force += f;
  p.torque += cross(c - p.pos, f) + m;
  I.contact = c;
  return true;
}

// ------------------- Gestion de la liste d'interactions --------------------
Interaction new_interaction(int i, int j, bool wall) {
  Interaction I;
  I.i = i;
  I.j = j;
  I.wall = wall;
  return I;  // ft = 0 : pas encore de mémoire
}

// Recopie la mémoire (ft, mom) des interactions qui existaient déjà dans l'ancienne
// liste.
void transfer_history(const std::vector<Interaction>& old_list,
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

// Étape 5 : on reconstruit la liste à chaque pas, par force brute, en ne
// gardant que les objets qui se recouvrent. (L'étape 6 présente une méthode plus efficace.)
void update_interactions(System& sys) {
  const auto& P = sys.particles;
  std::vector<Interaction> list;
  for (int i = 0; i < static_cast<int>(P.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(P.size()); ++j)
      if (norm(P[i].pos - P[j].pos) < P[i].radius + P[j].radius)
        list.push_back(new_interaction(i, j, false));
    for (int w = 0; w < static_cast<int>(sys.walls.size()); ++w) {
      const Wall& W = sys.walls[w];
      if (dot(P[i].pos - W.point, W.normal) < P[i].radius)
        list.push_back(new_interaction(i, w, true));
    }
  }
  transfer_history(sys.interactions, list);
  sys.interactions.swap(list);
}

// ------------------------- Boucle DEM --------------------------------
void compute_forces(System& sys) {
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

void time_step(System& sys) {
  const double dt = sys.prm.dt;
  for (auto& p : sys.particles) {  // demi-pas sur les vitesses + pas sur les positions
    p.vel += (0.5 * dt / p.mass) * p.force;
    p.omega += (0.5 * dt / p.inertia) * p.torque;
    p.pos += dt * p.vel;
    p.angle += dt * p.omega;
  }
  update_interactions(sys);
  compute_forces(sys);
  for (auto& p : sys.particles) {  // second demi-pas sur les vitesses
    p.vel += (0.5 * dt / p.mass) * p.force;
    p.omega += (0.5 * dt / p.inertia) * p.torque;
  }
  sys.time += dt;
}

Particle make_disk(Vec2 pos, Vec2 vel, double radius, double rho) {
  Particle p;
  p.pos = pos;
  p.vel = vel;
  p.radius = radius;
  p.mass = rho * M_PI * radius * radius;
  p.inertia = 0.5 * p.mass * radius * radius;  // disque plein
  return p;
}

double critical_time(double mmin, double kn) { return M_PI * std::sqrt(mmin / kn); }

// ------------------------------ Tests --------------------------------
// Un disque lancé en glissant (sans rotation) sur un sol finit par rouler.
// Théorie : vitesse finale v0 / (1 + I/(m r^2)) = 2 v0 / 3, atteinte à t = v0 / (3 mu g).
void test_slide_to_roll() {
  std::printf("--- Test 1 : glissement -> roulement ---\n");
  System sys;
  sys.prm.damp_rate = damping_ratio_from_restitution(0.5);
  sys.walls.push_back({{0.0, 0.0}, {0.0, 1.0}});
  const double r = 0.01, v0 = 1.0;
  sys.particles.push_back(make_disk({0.0, r - 0.785 * 9.81 / 1e7}, {v0, 0.0}, r, 2500.0));
  sys.prm.dt = critical_time(sys.particles[0].mass, sys.prm.kn) / 50.0;
  update_interactions(sys);
  compute_forces(sys);
  const double t_roll = v0 / (3.0 * sys.prm.mu * 9.81);
  while (sys.time < 2.0 * t_roll) time_step(sys);
  const Particle& p = sys.particles[0];
  std::printf("v = %.5f (théorie %.5f), w*r = %.5f, glissement v + w r = %.2e\n", p.vel.x,
              2.0 * v0 / 3.0, p.omega * r, p.vel.x + p.omega * r);
}

// Un disque roule sans glisser sur un plan incliné : a = (2/3) g sin(theta).
// Pour simplifier, on incline la gravité plutôt que le sol.
void test_incline() {
  std::printf("--- Test 2 : roulement sur un plan incliné à 20 degrés ---\n");
  System sys;
  const double theta = 20.0 * M_PI / 180.0, g = 9.81, r = 0.01;
  sys.prm.gravity = {g * std::sin(theta), -g * std::cos(theta)};
  sys.prm.damp_rate = damping_ratio_from_restitution(0.5);
  sys.walls.push_back({{0.0, 0.0}, {0.0, 1.0}});
  sys.particles.push_back(make_disk({0.0, r}, {0.0, 0.0}, r, 2500.0));
  sys.prm.dt = critical_time(sys.particles[0].mass, sys.prm.kn) / 50.0;
  update_interactions(sys);
  compute_forces(sys);
  // On laisse la particule se poser, puis on mesure l'accélération entre 0.1 et 0.3 s
  double v1 = 0.0;
  while (sys.time < 0.3) {
    time_step(sys);
    if (v1 == 0.0 && sys.time >= 0.1) v1 = sys.particles[0].vel.x;
  }
  const double a = (sys.particles[0].vel.x - v1) / (sys.time - 0.1);
  std::printf("a = %.4f m/s^2 (théorie : 2/3 g sin(theta) = %.4f)\n", a,
              2.0 / 3.0 * g * std::sin(theta));
}

// Un disque qui roule sur le sol est freiné par la résistance au roulement.
// Théorie (roulement sans glissement, moment mur * m g) :
//   décélération a = (2/3) mur g / r, distance d'arrêt v0^2 / (2 a).
void test_rolling_resistance() {
  std::printf("--- Test 3 : freinage par la résistance au roulement ---\n");
  System sys;
  sys.prm.damp_rate = damping_ratio_from_restitution(0.5);
  const double r = 0.01, v0 = 0.5, g = 9.81;
  sys.prm.mur = 0.1 * r;
  sys.prm.kr = 0.25 * sys.prm.kn * r * r;
  sys.walls.push_back({{0.0, 0.0}, {0.0, 1.0}});
  Particle p = make_disk({0.0, r}, {v0, 0.0}, r, 2500.0);
  p.pos.y -= p.mass * g / sys.prm.kn;  // posé à l'équilibre
  p.omega = -v0 / r;                   // il roule déjà sans glisser
  sys.particles.push_back(p);
  sys.prm.dt = critical_time(p.mass, sys.prm.kn) / 50.0;
  update_interactions(sys);
  compute_forces(sys);
  while (sys.particles[0].vel.x > 0.0) time_step(sys);
  const double a = 2.0 / 3.0 * sys.prm.mur * g / r;
  std::printf("arrêt à t = %.4f s (théorie %.4f), distance %.4f m (théorie %.4f)\n",
              sys.time, v0 / a, sys.particles[0].pos.x, v0 * v0 / (2.0 * a));
}

int main() {
  test_slide_to_roll();
  test_incline();
  test_rolling_resistance();
  return 0;
}
