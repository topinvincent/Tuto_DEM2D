// =====================================================================
//  Tutoriel DEM 2D -- Étape 1 : une particule en chute libre (velocity Verlet)
//
//  Compilation : g++ -O2 -std=c++17 -Wall etape1_chute_libre.cpp -o etape1
// =====================================================================
#include <cmath>
#include <cstdio>
#include <iostream>

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

struct Particle {
  Vec2 pos;    // position du centre (m)
  Vec2 vel;    // vitesse (m/s)
  Vec2 force;  // force résultante
  double radius = 0.0;
  double mass = 0.0;
};

int main() {
  const Vec2 gravity{0.0, -9.81};
  const double rho = 2500.0;

  Particle p;
  p.radius = 0.01;
  p.mass = rho * M_PI * p.radius * p.radius;
  p.pos = {0.0, 1.0};
  p.vel = {0.5, 0.0};

  const double dt = 1.0e-3;
  p.force = p.mass * gravity;  // force initiale

  std::cout << "# t y y_exact diff" << std::endl;
  for (int step = 0; step <= 400; ++step) {
    // ... afficher t, y et la solution exacte y0 - g t^2 / 2 tous les 50 pas ...
    
    if(step%50==0){
      double t = step*dt;
      double y_exact = 1.0 - 0.5 * 9.81 * t * t;
      double diff = std::abs(p.pos.y - y_exact);
      std::cout<< step*dt << " " << p.pos.y << " " << y_exact << " " << diff << std::endl;
    }
    /* Euler explicit
    p.pos += dt*p.vel;
    p.force = p.mass * gravity;              // forces au temps t + dt
    p.vel += dt * p.force / p.mass;
    */ 
  
    p.vel += (0.5 * dt / p.mass) * p.force;  // kick
    p.pos += dt * p.vel;                     // drift
    p.force = p.mass * gravity;              // forces au temps t + dt
    p.vel += (0.5 * dt / p.mass) * p.force;  // kick
  
    }
}

