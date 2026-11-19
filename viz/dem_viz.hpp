// =====================================================================
//  dem_viz.hpp -- petite librairie de visualisation pour le tutoriel DEM 2D
//
//  >>> Ce fichier est FOURNI : il n'est pas nécessaire de le lire ni de le
//  >>> modifier pour suivre le tutoriel.
//
//  Il ne dépend d'aucune bibliothèque externe (C++17 standard uniquement).
//  On décrit une "scène" (disques, contacts, parois) puis on l'enregistre :
//
//    viz::Scene scene;
//    scene.time = t;
//    scene.disks.push_back({x, y, r, angle, vx, vy});   // un disque
//    scene.contacts.push_back({x1, y1, x2, y2, fn});    // un contact (optionnel)
//    scene.walls.push_back({px, py, nx, ny});           // une paroi (optionnel)
//
//    viz::Recorder rec("sortie");   // dossier de sortie
//    rec.save(scene);               // écrit une image + des fichiers ParaView
//
//  Pour chaque appel à save(), le Recorder écrit dans le dossier :
//    - frame_XXXXX.svg            image vectorielle (s'ouvre dans un navigateur)
//    - viewer.html                visionneuse : ouvrir ce fichier dans un
//                                 navigateur pour rejouer l'animation
//    - particles_XXXXX.vtk        disques, pour ParaView (filtre Glyph,
//                                 "Scale Array" = radius, facteur d'échelle 2)
//    - contacts_XXXXX.vtk         réseau des forces normales (lignes)
//    - particles.pvd / contacts.pvd  séries temporelles à ouvrir dans ParaView
//
//  Couleurs des images : vitesse des disques (bleu = lent, rouge = rapide).
//  Le trait dans chaque disque indique sa rotation. L'épaisseur d'un contact
//  est proportionnelle à la force normale.
// =====================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace viz {

struct Disk {
  double x, y;     // centre
  double r;        // rayon
  double angle;    // angle de rotation (rad)
  double vx, vy;   // vitesse (sert à la couleur)
};

struct Contact {
  double x1, y1;   // centre de la première particule
  double x2, y2;   // centre de la seconde (ou point de contact pour une paroi)
  double fn;       // force normale
};

struct Wall {
  double px, py;   // un point de la paroi
  double nx, ny;   // normale (orientée vers les particules)
};

struct Scene {
  double time = 0.0;
  std::vector<Disk> disks;
  std::vector<Contact> contacts;
  std::vector<Wall> walls;
  // Fenêtre affichée. Si xmax <= xmin, elle est calculée automatiquement
  // (disques et points des parois de la première image enregistrée).
  double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;
  // Vitesse correspondant au rouge. Si vmax <= 0 : le Recorder utilise la plus
  // grande vitesse rencontrée depuis le début de l'enregistrement.
  double vmax = 0.0;
};

namespace detail {

inline std::string number(int i) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%05d", i);
  return buf;
}

// Palette bleu -> cyan -> jaune -> rouge, s dans [0, 1]
inline std::string color(double s) {
  s = std::clamp(s, 0.0, 1.0);
  const double stops[4][3] = {{49, 54, 149}, {116, 173, 209}, {254, 224, 144}, {215, 48, 39}};
  const double u = s * 3.0;
  const int k = std::min(2, static_cast<int>(u));
  const double f = u - k;
  char buf[8];
  std::snprintf(buf, sizeof(buf), "#%02x%02x%02x",
                static_cast<int>(stops[k][0] + f * (stops[k + 1][0] - stops[k][0])),
                static_cast<int>(stops[k][1] + f * (stops[k + 1][1] - stops[k][1])),
                static_cast<int>(stops[k][2] + f * (stops[k + 1][2] - stops[k][2])));
  return buf;
}

inline void auto_box(Scene& s) {
  if (s.xmax > s.xmin) return;
  double x0 = std::numeric_limits<double>::max(), y0 = x0;
  double x1 = -x0, y1 = -x0;
  for (const auto& d : s.disks) {
    x0 = std::min(x0, d.x - d.r);
    x1 = std::max(x1, d.x + d.r);
    y0 = std::min(y0, d.y - d.r);
    y1 = std::max(y1, d.y + d.r);
  }
  for (const auto& w : s.walls) {  // les points des parois sont aussi inclus
    x0 = std::min(x0, w.px);
    x1 = std::max(x1, w.px);
    y0 = std::min(y0, w.py);
    y1 = std::max(y1, w.py);
  }
  if (x1 < x0) { x0 = y0 = 0.0; x1 = y1 = 1.0; }
  const double m = 0.05 * std::max(x1 - x0, y1 - y0);
  s.xmin = x0 - m; s.xmax = x1 + m; s.ymin = y0 - m; s.ymax = y1 + m;
}

}  // namespace detail

// ---------------------------------------------------------------------
// Image SVG d'une scène
// ---------------------------------------------------------------------
inline void write_svg(const std::string& filename, Scene s) {
  detail::auto_box(s);
  const double W = s.xmax - s.xmin, H = s.ymax - s.ymin;
  const int px_w = 800;
  const int px_h = static_cast<int>(px_w * H / W);
  std::ofstream out(filename);
  // On travaille directement en coordonnées physiques, l'axe y étant inversé.
  out << "<svg xmlns='http://www.w3.org/2000/svg' width='" << px_w << "' height='" << px_h
      << "' viewBox='" << s.xmin << " " << -s.ymax << " " << W << " " << H << "'>\n";
  out << "<rect x='" << s.xmin << "' y='" << -s.ymax << "' width='" << W << "' height='" << H
      << "' fill='white'/>\n";

  double vmax = s.vmax, fmax = 1e-300;
  if (vmax <= 0.0)
    for (const auto& d : s.disks) vmax = std::max(vmax, std::hypot(d.vx, d.vy));
  vmax = std::max(vmax, 1e-12);
  for (const auto& c : s.contacts) fmax = std::max(fmax, c.fn);

  // Parois : segment de la droite coupé par la fenêtre
  const double L = 2.0 * (W + H);
  for (const auto& w : s.walls) {
    const double tx = -w.ny, ty = w.nx;
    out << "<line x1='" << w.px - L * tx << "' y1='" << -(w.py - L * ty) << "' x2='"
        << w.px + L * tx << "' y2='" << -(w.py + L * ty)
        << "' stroke='#333' stroke-width='" << 0.006 * W << "'/>\n";
  }
  // Disques
  for (const auto& d : s.disks) {
    out << "<circle cx='" << d.x << "' cy='" << -d.y << "' r='" << d.r << "' fill='"
        << detail::color(std::hypot(d.vx, d.vy) / vmax) << "' stroke='#222' stroke-width='"
        << 0.08 * d.r << "'/>";
    out << "<line x1='" << d.x << "' y1='" << -d.y << "' x2='" << d.x + d.r * std::cos(d.angle)
        << "' y2='" << -(d.y + d.r * std::sin(d.angle)) << "' stroke='#222' stroke-width='"
        << 0.12 * d.r << "'/>\n";
  }
  // Contacts : épaisseur proportionnelle à la force normale
  double rmean = 0.0;
  for (const auto& d : s.disks) rmean += d.r;
  rmean = s.disks.empty() ? 0.01 * W : rmean / s.disks.size();
  for (const auto& c : s.contacts) {
    if (c.fn <= 0.0) continue;
    out << "<line x1='" << c.x1 << "' y1='" << -c.y1 << "' x2='" << c.x2 << "' y2='" << -c.y2
        << "' stroke='black' stroke-linecap='round' stroke-width='" << 0.5 * rmean * c.fn / fmax
        << "'/>\n";
  }
  char label[64];
  std::snprintf(label, sizeof(label), "t = %.4f s", s.time);
  out << "<text x='" << s.xmin + 0.02 * W << "' y='" << -s.ymax + 0.05 * H << "' font-size='"
      << 0.035 * H << "' font-family='sans-serif'>" << label << "</text>\n";
  out << "</svg>\n";
}

// ---------------------------------------------------------------------
// Fichiers VTK (format "legacy" ASCII) lisibles par ParaView
// ---------------------------------------------------------------------
inline void write_vtk_particles(const std::string& filename, const Scene& s) {
  std::ofstream out(filename);
  const size_t n = s.disks.size();
  out << "# vtk DataFile Version 3.0\nDEM 2D particles\nASCII\nDATASET POLYDATA\n";
  out << "POINTS " << n << " double\n";
  for (const auto& d : s.disks) out << d.x << " " << d.y << " 0\n";
  out << "VERTICES " << n << " " << 2 * n << "\n";
  for (size_t i = 0; i < n; ++i) out << "1 " << i << "\n";
  out << "POINT_DATA " << n << "\n";
  out << "SCALARS radius double 1\nLOOKUP_TABLE default\n";
  for (const auto& d : s.disks) out << d.r << "\n";
  out << "SCALARS angle double 1\nLOOKUP_TABLE default\n";
  for (const auto& d : s.disks) out << d.angle << "\n";
  out << "VECTORS velocity double\n";
  for (const auto& d : s.disks) out << d.vx << " " << d.vy << " 0\n";
}

inline void write_vtk_contacts(const std::string& filename, const Scene& s) {
  std::ofstream out(filename);
  const size_t n = s.contacts.size();
  out << "# vtk DataFile Version 3.0\nDEM 2D contacts\nASCII\nDATASET POLYDATA\n";
  out << "POINTS " << 2 * n << " double\n";
  for (const auto& c : s.contacts) out << c.x1 << " " << c.y1 << " 0\n" << c.x2 << " " << c.y2 << " 0\n";
  out << "LINES " << n << " " << 3 * n << "\n";
  for (size_t i = 0; i < n; ++i) out << "2 " << 2 * i << " " << 2 * i + 1 << "\n";
  out << "CELL_DATA " << n << "\n";
  out << "SCALARS fn double 1\nLOOKUP_TABLE default\n";
  for (const auto& c : s.contacts) out << c.fn << "\n";
}

// ---------------------------------------------------------------------
// Enregistreur : une image + des fichiers ParaView à chaque appel de save()
// ---------------------------------------------------------------------
class Recorder {
 public:
  explicit Recorder(const std::string& directory) : dir_(directory) {
    std::filesystem::create_directories(dir_);
  }

  void save(Scene s) {
    if (count_ == 0) {
      detail::auto_box(s);
      box_[0] = s.xmin; box_[1] = s.xmax; box_[2] = s.ymin; box_[3] = s.ymax;
    } else if (s.xmax <= s.xmin) {
      s.xmin = box_[0]; s.xmax = box_[1]; s.ymin = box_[2]; s.ymax = box_[3];
    }
    if (s.vmax <= 0.0) {
      for (const auto& d : s.disks) vmax_seen_ = std::max(vmax_seen_, std::hypot(d.vx, d.vy));
      s.vmax = vmax_seen_;
    }
    const std::string id = detail::number(count_);
    write_svg(dir_ + "/frame_" + id + ".svg", s);
    write_vtk_particles(dir_ + "/particles_" + id + ".vtk", s);
    write_vtk_contacts(dir_ + "/contacts_" + id + ".vtk", s);
    times_.push_back(s.time);
    ++count_;
    write_pvd("particles");
    write_pvd("contacts");
    write_viewer();
  }

  int frames() const { return count_; }

 private:
  void write_pvd(const std::string& name) const {
    std::ofstream out(dir_ + "/" + name + ".pvd");
    out << "<?xml version='1.0'?>\n<VTKFile type='Collection' version='0.1'>\n<Collection>\n";
    for (int i = 0; i < count_; ++i)
      out << "  <DataSet timestep='" << times_[i] << "' file='" << name << "_"
          << detail::number(i) << ".vtk'/>\n";
    out << "</Collection>\n</VTKFile>\n";
  }

  void write_viewer() const {
    std::ofstream out(dir_ + "/viewer.html");
    out << R"(<!doctype html><html><head><meta charset='utf-8'><title>DEM 2D</title>
<style>body{font-family:sans-serif;margin:16px;background:#f4f4f4}img{max-width:100%;background:#fff;border:1px solid #ccc}
.bar{display:flex;gap:8px;align-items:center;margin:8px 0}input[type=range]{flex:1}</style></head><body>
<div class='bar'><button id='play'>Lecture</button><input id='s' type='range' min='0' value='0'><span id='lab'></span></div>
<img id='im'><script>
const n=)" << count_ << R"(;const s=document.getElementById('s'),im=document.getElementById('im'),lab=document.getElementById('lab');
s.max=n-1;let timer=null;
function show(i){s.value=i;im.src='frame_'+String(i).padStart(5,'0')+'.svg';lab.textContent='image '+i+' / '+(n-1);}
s.oninput=()=>show(+s.value);
document.getElementById('play').onclick=function(){if(timer){clearInterval(timer);timer=null;this.textContent='Lecture';return;}
this.textContent='Pause';timer=setInterval(()=>show((+s.value+1)%n),80);};
show(0);</script></body></html>
)";
  }

  std::string dir_;
  int count_ = 0;
  std::vector<double> times_;
  double box_[4] = {0, 0, 0, 0};
  double vmax_seen_ = 0.0;
};

}  // namespace viz
