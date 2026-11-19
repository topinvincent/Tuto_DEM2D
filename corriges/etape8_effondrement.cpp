// =====================================================================
//  Tutoriel DEM 2D -- Étape 8 : effondrement d'une colonne granulaire
//
//  Le moteur est rangé dans dem2d.hpp ; ce fichier ne contient que la
//  mise en données, la boucle en temps et les mesures.
//
//  1) on remplit par gravité une colonne de largeur L0 fermée par une
//     "porte" (paroi verticale) ;
//  2) on retire la porte : la colonne s'effondre et s'étale sur le sol ;
//  3) on mesure l'étalement final L et la hauteur finale H.
//
//  Compilation : g++ -O2 -std=c++17 etape8_effondrement.cpp -o etape8
//  Exécution   : ./etape8         (rapport d'aspect a = H0/L0 = 2)
//                ./etape8 4       (rapport d'aspect au choix)
// =====================================================================
#include <algorithm>
#include <cstdlib>
#include <random>

#include "dem2d.hpp"

// ===================== PARAMÈTRES ====================================
const double rho = 2500.0;                 // masse volumique (kg/m^3)
const double re = 0.5;                     // coefficient de restitution visé
const double rmin = 1.5e-3, rmax = 2.5e-3;  // rayons extrêmes des disques (m)
const double L0 = 0.06;                    // largeur de la colonne (m)
const double dt_factor = 30.0;             // nombre de pas de temps par contact
const double t_save = 0.01;                // une image tous les t_save secondes
const double t_max = 3.0;                  // durée maximale de l'effondrement (s)

// Remplit la colonne de n disques de rayons aléatoires, sur un réseau
void setup_column(System& sys, int n) {
  // sol, mur gauche, et la porte (en dernier, pour pouvoir la retirer)
  sys.walls.push_back({{0.0, 0.0}, {0.0, 1.0}});   // 0 : sol
  sys.walls.push_back({{0.0, 0.0}, {1.0, 0.0}});   // 1 : mur gauche
  sys.walls.push_back({{L0, 0.0}, {-1.0, 0.0}});   // 2 : la porte

  std::mt19937 gen(2024);  // graine fixe : simulation reproductible
  std::uniform_real_distribution<double> radius(rmin, rmax), noise(-0.05, 0.05);
  const double spacing = 2.2 * rmax;
  const int ncols = static_cast<int>((L0 - 2.0 * rmax) / spacing);
  for (int k = 0; k < n; ++k) {
    const Vec2 pos{rmax + spacing * (0.5 + k % ncols),
                   rmax + spacing * (0.5 + k / ncols)};
    sys.particles.push_back(make_disk(pos, {noise(gen), noise(gen)}, radius(gen), rho));
  }
}

// ===================== PROGRAMME PRINCIPAL ===========================
int main(int argc, char** argv) {
  const double aspect = (argc > 1) ? std::atof(argv[1]) : 2.0;  // a = H0 / L0

  // ----------------------------- Matériau ------------------------------
  System sys;
  sys.prm.kn = 1.0e6;
  sys.prm.kt = 0.8e6;
  sys.prm.mu = 0.5;
  sys.prm.damp_rate = damping_ratio_from_restitution(re);
  // Résistance au roulement : sans elle, des disques roulent indéfiniment
  // sur le sol plat et le tas ne s'arrête jamais de s'étaler.
  sys.prm.mur = 0.1 * 0.5 * (rmin + rmax);
  sys.prm.kr = 0.25 * sys.prm.kn * rmin * rmin;
  // Pas de temps : contact entre deux petits disques (m_eff = m_min / 2)
  const double mmin = rho * M_PI * rmin * rmin;
  sys.prm.dt = critical_time(0.5 * mmin, sys.prm.kn) / dt_factor;
  sys.prm.DVerlet = 0.2 * rmin;

  // Nombre de disques pour atteindre H0 = aspect * L0 (compacité ~ 0,8)
  const double rmean = 0.5 * (rmin + rmax);
  const int n = static_cast<int>(0.8 * L0 * aspect * L0 / (M_PI * rmean * rmean * 1.04));
  setup_column(sys, n);
  std::printf("a = %.2f, %d disques, dt = %.2e s\n", aspect, n, sys.prm.dt);

  // ------------------- Phase 1 : remplissage de la colonne --------------
  update_interactions(sys, true);
  compute_forces(sys);
  while (sys.time < 0.3 || kinetic_energy(sys) > 1e-6) exec_one_iter(sys);

  double H0 = 0.0;
  for (const auto& p : sys.particles) H0 = std::max(H0, p.pos.y + p.radius);
  std::printf("colonne au repos à t = %.2f s : H0 = %.4f m, L0 = %.4f m, H0/L0 = %.2f\n",
              sys.time, H0, L0, H0 / L0);

  // ------------------- Phase 2 : on ouvre la porte ----------------------
  sys.walls.pop_back();            // la porte était la dernière paroi
  update_interactions(sys, true);  // la liste contenait des contacts avec la porte
  compute_forces(sys);
  sys.time = 0.0;                  // on compte le temps depuis l'ouverture

  viz::Recorder rec("sortie_etape8");
  const double view_width = L0 * (1.0 + 4.0 * aspect);  // fenêtre fixe pour l'animation
  double next_save = 0.0;
  while (sys.time < t_max) {
    if (sys.time >= next_save) {
      viz::Scene scene = make_scene(sys, sys.time);
      scene.xmin = -0.05 * view_width;
      scene.xmax = 1.05 * view_width;
      scene.ymin = -0.05 * H0;
      scene.ymax = 1.1 * H0;
      rec.save(scene);
      next_save += t_save;
    }
    exec_one_iter(sys);
    if (sys.time > 0.2 && kinetic_energy(sys) < 1e-7) break;  // écoulement arrêté
  }

  // ---------------------------- Mesures --------------------------------
  // L est pris au front du tas (centile 99,5 % des abscisses), pour ignorer
  // les quelques grains isolés qui ont roulé loin.
  std::vector<double> xs;
  double H = 0.0;
  for (const auto& p : sys.particles) {
    xs.push_back(p.pos.x + p.radius);
    H = std::max(H, p.pos.y + p.radius);
  }
  std::sort(xs.begin(), xs.end());
  const double L = xs[static_cast<size_t>(0.995 * (xs.size() - 1))];
  std::printf("arrêt à t = %.2f s après l'ouverture (%d images)\n", sys.time, rec.frames());
  std::printf("étalement final L = %.4f m, hauteur finale H = %.4f m\n", L, H);
  std::printf("(L - L0) / L0 = %.2f, H / L0 = %.2f\n", (L - L0) / L0, H / L0);
  std::printf("%d reconstructions de la liste\n", sys.nb_rebuilds);
  std::printf("images : ouvrir sortie_etape8/viewer.html\n");
  return 0;
}
