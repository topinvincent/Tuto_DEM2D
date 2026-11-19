# Génère les schémas SVG du tutoriel (images/schema_*.svg)
import math, os
OUT = os.path.join(os.path.dirname(__file__), '..', 'images')
INK, BLUE, RED, GREEN, GREY, LIGHT = '#222', '#1f5fa8', '#c0392b', '#2e7d32', '#888', '#e8eef7'
FONT = 'font-family="Helvetica, Arial, sans-serif"'

def svg(name, w, h, body):
    s = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" {FONT} font-size="15">
<defs>
 <marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{INK}"/></marker>
 <marker id="ab" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{BLUE}"/></marker>
 <marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{RED}"/></marker>
 <marker id="ag" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{GREEN}"/></marker>
 <pattern id="hatch" width="8" height="8" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="8" stroke="{GREY}" stroke-width="1.5"/></pattern>
</defs>
<rect width="{w}" height="{h}" fill="white"/>
{body}
</svg>
'''
    open(os.path.join(OUT, name), 'w').write(s)

def arrow(x1, y1, x2, y2, col=INK, m='a', w=2, dash=''):
    d = f' stroke-dasharray="{dash}"' if dash else ''
    return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{col}" stroke-width="{w}" marker-end="url(#{m})"{d}/>\n'

def dim(x1, y1, x2, y2, col=INK):  # cote à double flèche
    return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{col}" stroke-width="1.5" marker-start="url(#a)" marker-end="url(#a)"/>\n'

def text(x, y, t, col=INK, size=15, anchor='start', style=''):
    return f'<text x="{x:.1f}" y="{y:.1f}" fill="{col}" font-size="{size}" text-anchor="{anchor}" {style}>{t}</text>\n'

def S(t):  # indice
    return f'<tspan baseline-shift="sub" font-size="11">{t}</tspan>'

def it(t):  # variable en italique
    return f'<tspan font-style="italic">{t}</tspan>'

# ---------------------------------------------------------------------------
# 1. Algorithme d'un pas de temps
def fig_algo():
    w, h = 880, 700
    b = ''
    def box(y, hh, title, lines, fill=LIGHT, stroke=BLUE):
        nonlocal b
        b += f'<rect x="150" y="{y}" width="420" height="{hh}" rx="8" fill="{fill}" stroke="{stroke}" stroke-width="1.8"/>\n'
        b += text(360, y + 24, title, INK, 16, 'middle', 'font-weight="bold"')
        for k, l in enumerate(lines):
            if isinstance(l, tuple):
                b += text(250, y + 46 + 20 * k, l[0], INK, 14, 'start') + text(420, y + 46 + 20 * k, l[1], INK, 14, 'start')
            else:
                b += text(360, y + 46 + 20 * k, l, INK, 14, 'middle')
    box(20, 70, 'Initialisation', ['particules, parois, paramètres, Δt', 'voisins + calcul des forces F(0)'], '#f4f4f4', GREY)
    b += arrow(360, 90, 360, 118)
    box(120, 72, '1. Demi-pas sur les vitesses, pas sur les positions', [('v ← v + (Δt/2) F/m', 'ω ← ω + (Δt/2) T/I'), ('x ← x + Δt v', 'θ ← θ + Δt ω')])
    b += arrow(360, 192, 360, 220)
    box(222, 72, '2. Mise à jour des voisins (si nécessaire)', ['reconstruction de la liste de Verlet si un grain', "s'est déplacé de plus de D_Verlet / 2"])
    b += arrow(360, 294, 360, 322)
    box(324, 92, '3. Calcul des forces au temps t + Δt', ['F ← m g,  T ← 0', 'pour chaque interaction : δn, n, v_rel → fn, ft', 'F_i += f,  F_j −= f  (et moments)'])
    b += arrow(360, 416, 360, 444)
    box(446, 52, '4. Second demi-pas sur les vitesses', [('v ← v + (Δt/2) F/m', 'ω ← ω + (Δt/2) T/I')])
    b += arrow(360, 498, 360, 526)
    box(528, 52, '5. Sorties (tous les n pas)', ['images, énergies, mesures'], '#f4f4f4', GREY)
    # test
    b += arrow(360, 580, 360, 606)
    b += f'<polygon points="360,608 440,640 360,672 280,640" fill="white" stroke="{INK}" stroke-width="1.8"/>\n'
    b += text(360, 645, 't &lt; t_end ?', INK, 14, 'middle')
    b += text(450, 660, 'non → fin', INK, 14)
    # boucle
    b += f'<polyline points="280,640 90,640 90,156 146,156" fill="none" stroke="{INK}" stroke-width="2" marker-end="url(#a)"/>\n'
    b += text(100, 400, 'oui : t ← t + Δt', INK, 14, 'middle', 'transform="rotate(-90 100 400)"')
    # étapes du tutoriel correspondant à chaque bloc
    def tag(y, title, lines):
        nonlocal b
        b += f'<line x1="574" y1="{y}" x2="598" y2="{y}" stroke="{BLUE}" stroke-width="1.2" stroke-dasharray="3 3"/>'
        b += text(604, y - 4, title, BLUE, 13, 'start', 'font-weight="bold"')
        for k, l in enumerate(lines):
            b += text(604, y + 13 + 16 * k, l, INK, 12.5)
    tag(50, 'mise en données', ['étapes 1 à 4'])
    tag(150, 'intégration en temps', ['étape 1 : translation', 'étape 5 : rotation (θ, ω)'])
    tag(252, 'voisins', ['étape 5 : liste refaite à chaque pas', 'étape 6 : liste de Verlet, OBB', 'étape 7 : grille de cellules'])
    tag(352, 'loi de contact', ['étape 2 : contact avec le sol', 'étape 3 : contact entre disques', 'étape 4 : parois', 'étape 5 : frottement, roulement'])
    tag(466, 'intégration en temps', ['étape 1 (rotation : étape 5)'])
    tag(548, 'sorties', ['étape 3 : visualisation', 'étape 4 : bilan d\'énergie'])
    svg('schema_pas_de_temps.svg', w, h, b)

# ---------------------------------------------------------------------------
# 2. Contact disque-sol
def fig_sol():
    w, h = 560, 330
    cx, cy, R = 280, 170, 110
    floor = 260  # y écran du sol ; overlap = cy + R - floor
    b = ''
    b += f'<rect x="20" y="{floor}" width="520" height="50" fill="url(#hatch)"/>\n'
    b += f'<line x1="20" y1="{floor}" x2="540" y2="{floor}" stroke="{INK}" stroke-width="2.5"/>\n'
    # zone d'interpénétration
    dy = floor - cy
    half = math.sqrt(R * R - dy * dy)
    b += f'<path d="M{cx-half:.1f},{floor} A{R},{R} 0 0 0 {cx+half:.1f},{floor} Z" fill="{RED}" fill-opacity="0.35"/>\n'
    b += f'<circle cx="{cx}" cy="{cy}" r="{R}" fill="{LIGHT}" fill-opacity="0.6" stroke="{BLUE}" stroke-width="2.5"/>\n'
    b += f'<circle cx="{cx}" cy="{cy}" r="4" fill="{INK}"/>\n' + text(cx - 10, cy - 8, it('x'), INK, 15, 'end')
    # rayon
    ang = math.radians(-35)
    b += arrow(cx, cy, cx + R * math.cos(ang), cy + R * math.sin(ang), INK, 'a', 1.6)
    b += text(cx + 50, cy - 45, it('r'))
    # distance d
    b += dim(cx - 140, cy, cx - 140, floor)
    b += f'<line x1="{cx-150}" y1="{cy}" x2="{cx}" y2="{cy}" stroke="{GREY}" stroke-dasharray="4 4"/>\n'
    b += text(cx - 165, (cy + floor) / 2 + 5, it('d'), INK, 16, 'end')
    # delta_n
    b += dim(cx + 150, floor - 14, cx + 150, floor) + dim(cx + 150, cy + R + 14, cx + 150, cy + R)
    b += f'<line x1="{cx}" y1="{cy+R}" x2="{cx+160}" y2="{cy+R}" stroke="{GREY}" stroke-dasharray="4 4"/>\n'
    b += text(cx + 162, floor + 30, 'δ' + S('n') + ' = ' + it('r') + ' − ' + it('d'), RED, 16)
    # normale
    b += arrow(80, floor, 80, floor - 60, GREEN, 'ag', 2.5)
    b += text(66, floor - 45, '<tspan font-weight="bold">n</tspan>', GREEN, 16, 'end')
    b += f'<circle cx="80" cy="{floor}" r="4" fill="{INK}"/>' + text(60, floor + 22, it('x') + '<tspan font-size="11" dy="4">sol</tspan>')
    b += text(530, floor - 8, 'sol', INK, 14, 'end')
    svg('schema_contact_sol.svg', w, h, b)

# ---------------------------------------------------------------------------
# 3. Loi normale : schéma rhéologique + force au cours d'un choc
def fig_normale():
    w, h = 1000, 380
    b = ''
    # --- gauche : ressort + amortisseur
    x0, top, bot = 60, 70, 250
    b += f'<rect x="{x0}" y="{top-40}" width="200" height="40" rx="6" fill="{LIGHT}" stroke="{BLUE}" stroke-width="2"/>' + text(x0 + 100, top - 14, 'particule  (' + it('m') + '<tspan font-size="11" dy="4">eff</tspan><tspan dy="-4">)</tspan>', INK, 14, 'middle')
    b += f'<rect x="{x0-10}" y="{bot}" width="220" height="24" fill="url(#hatch)"/><line x1="{x0-10}" y1="{bot}" x2="{x0+210}" y2="{bot}" stroke="{INK}" stroke-width="2.5"/>'
    # ressort
    sx = x0 + 55
    pts = [(sx, top), (sx, top + 30)]
    n = 8
    for k in range(n):
        yy = top + 30 + (k + 0.5) * 120 / n
        pts.append((sx + (14 if k % 2 == 0 else -14), yy))
    pts += [(sx, top + 150), (sx, bot)]
    b += '<polyline points="' + ' '.join(f'{p[0]},{p[1]:.1f}' for p in pts) + f'" fill="none" stroke="{INK}" stroke-width="2"/>'
    b += text(sx - 24, top + 95, it('k') + '<tspan font-size="11" dy="4">n</tspan>', INK, 16, 'end')
    # amortisseur
    dx = x0 + 145
    b += f'<line x1="{dx}" y1="{top}" x2="{dx}" y2="{top+80}" stroke="{INK}" stroke-width="2"/>'
    b += f'<line x1="{dx-16}" y1="{top+80}" x2="{dx+16}" y2="{top+80}" stroke="{INK}" stroke-width="3"/>'
    b += f'<polyline points="{dx-22},{top+60} {dx-22},{top+120} {dx+22},{top+120} {dx+22},{top+60}" fill="none" stroke="{INK}" stroke-width="2"/>'
    b += f'<line x1="{dx}" y1="{top+120}" x2="{dx}" y2="{bot}" stroke="{INK}" stroke-width="2"/>'
    b += text(dx + 30, top + 95, it('c') + '<tspan font-size="11" dy="4">n</tspan>', INK, 16)
    b += text(x0 + 100, 305, it('f') + '<tspan font-size="11" dy="4">n</tspan><tspan dy="-4"> = max(0, </tspan>' + it('k') + '<tspan font-size="11" dy="4">n</tspan><tspan dy="-4"> δ</tspan><tspan font-size="11" dy="4">n</tspan><tspan dy="-4"> − </tspan>' + it('c') + '<tspan font-size="11" dy="4">n</tspan><tspan dy="-4"> </tspan>' + it('v') + '<tspan font-size="11" dy="4">n</tspan><tspan dy="-4">)</tspan>', INK, 15, 'middle')
    # --- droite : fn en fonction de delta_n pendant un choc (e = 0,5)
    e = 0.5
    le = math.log(e)
    z = -le / math.sqrt(math.pi ** 2 + le ** 2)
    wd = math.sqrt(1 - z * z)
    T = math.pi / wd
    pts = []  # (delta, f, vitesse de rapprochement)
    for k in range(2001):
        t = 1.2 * T * k / 2000
        d = math.exp(-z * t) * math.sin(wd * t) / wd
        dd = math.exp(-z * t) * (math.cos(wd * t) - z / wd * math.sin(wd * t))
        f = d + 2 * z * dd
        pts.append((d, f, dd))
        if d < 0: break
    dmax = max(p[0] for p in pts)
    fmax = max(p[1] for p in pts)
    gx, gy = 430, 50          # origine du repère (coin haut gauche du cadre)
    ox, oy = gx + 30, 290     # origine (0, 0)
    W, H = 380, 210
    def X(d): return ox + W * 0.85 * d / dmax
    def Y(f): return oy - H * 0.85 * f / fmax
    # aire dissipée
    trunc = [p for p in pts if p[1] >= 0]
    loop = ' '.join(f'{X(d):.1f},{Y(f):.1f}' for d, f, v in trunc)
    b += f'<polygon points="{ox},{oy} {loop}" fill="{BLUE}" fill-opacity="0.10" stroke="none"/>'
    b += arrow(ox, oy, ox + W + 20, oy) + arrow(ox, oy + 10, ox, oy - H - 25)
    b += text(ox + W + 20, oy + 24, 'δ' + S('n'), INK, 16, 'end')
    b += text(ox + 8, oy - H - 18, it('f') + S('n'), INK, 16)
    # loi élastique seule
    b += f'<line x1="{ox}" y1="{oy}" x2="{X(dmax*1.08):.1f}" y2="{Y(dmax*1.08):.1f}" stroke="{GREY}" stroke-width="1.6" stroke-dasharray="6 4"/>'
    b += text(X(dmax*1.08) + 6, Y(dmax*1.08) + 4, it('k') + S('n') + 'δ' + S('n') + ' (ressort seul)', GREY, 13)
    # trajectoire
    load = [p for p in trunc if p[2] >= 0]
    unload = [p for p in trunc if p[2] < 0]
    b += '<polyline points="' + f'{ox},{oy} ' + ' '.join(f'{X(d):.1f},{Y(f):.1f}' for d, f, v in load) + f'" fill="none" stroke="{BLUE}" stroke-width="2.6"/>'
    b += '<polyline points="' + ' '.join(f'{X(d):.1f},{Y(f):.1f}' for d, f, v in [load[-1]] + unload) + f'" fill="none" stroke="{BLUE}" stroke-width="2.6"/>'
    neg = [p for p in pts if p[1] < 0]
    if neg:
        b += '<polyline points="' + ' '.join(f'{X(d):.1f},{Y(f):.1f}' for d, f, v in [trunc[-1]] + neg) + f'" fill="none" stroke="{RED}" stroke-width="2" stroke-dasharray="5 4"/>'
        d0 = trunc[-1][0]
        b += f'<line x1="{X(d0):.1f}" y1="{oy}" x2="{ox}" y2="{oy}" stroke="{BLUE}" stroke-width="3.5"/>'
        dm, fm = neg[len(neg)//2][0], min(p[1] for p in neg)
        b += text(X(dm) + 20, Y(fm) + 8, 'partie attractive supprimée (' + it('f') + S('n') + ' ← 0)', RED, 13, 'start')
    # flèches de parcours
    def mark(p, q):
        return arrow(X(p[0]), Y(p[1]), X(q[0]), Y(q[1]), BLUE, 'ab', 2.6)
    i1 = len(load) // 3
    b += mark(load[i1], load[i1 + 8])
    j1 = len(unload) // 2
    b += mark(unload[j1], unload[j1 + 8])
    # annotations
    i2 = len(load) // 2
    b += text(ox + 30, Y(fmax) - 22, 'charge : ' + it('v') + S('n') + ' &lt; 0,  ' + it('f') + S('n') + ' &gt; ' + it('k') + S('n') + 'δ' + S('n'), BLUE, 13)
    j2 = len(unload) // 3
    b += text(X(unload[j2][0]) + 14, Y(unload[j2][1]) + 18, 'décharge : ' + it('v') + S('n') + ' &gt; 0', BLUE, 13)
    b += text(X(unload[j2][0]) + 14, Y(unload[j2][1]) + 34, it('f') + S('n') + ' &lt; ' + it('k') + S('n') + 'δ' + S('n'), BLUE, 13)
    b += text(X(0.45 * dmax), Y(0.35 * fmax) + 6, 'énergie dissipée', BLUE, 13, 'middle', 'font-style="italic"')
    b += text(ox - 6, Y(pts[0][1]) + 4, '−' + it('c') + S('n') + it('v') + S('n') + '(0)', BLUE, 12, 'end')
    svg('schema_loi_normale.svg', w, h, b)

# ---------------------------------------------------------------------------
# 4. Contact entre deux disques
def fig_disques():
    w, h = 640, 380
    xj, yj, rj = 220, 230, 130
    ri = 95
    ang = math.radians(-30)  # direction j -> i (écran, y vers le bas)
    nx, ny = math.cos(ang), math.sin(ang)
    dist = rj + ri - 30
    xi, yi = xj + dist * nx, yj + dist * ny
    b = ''
    b += f'<circle cx="{xj}" cy="{yj}" r="{rj}" fill="{LIGHT}" fill-opacity="0.7" stroke="{BLUE}" stroke-width="2.5"/>'
    b += f'<circle cx="{xi:.1f}" cy="{yi:.1f}" r="{ri}" fill="#fbeaea" fill-opacity="0.7" stroke="{RED}" stroke-width="2.5"/>'
    b += f'<circle cx="{xj}" cy="{yj}" r="4" fill="{INK}"/>' + text(xj - 10, yj + 22, it('x') + '<tspan font-size="11" dy="4">j</tspan>', INK, 16, 'end')
    b += f'<circle cx="{xi:.1f}" cy="{yi:.1f}" r="4" fill="{INK}"/>' + text(xi + 10, yi - 10, it('x') + '<tspan font-size="11" dy="4">i</tspan>', INK, 16)
    # vecteur branche
    b += arrow(xj, yj, xi - 6 * nx, yi - 6 * ny, INK, 'a', 1.8)
    b += text((xj + xi) / 2 - 30, (yj + yi) / 2 - 18, 'branche ' + it('x') + '<tspan font-size="11" dy="4">i</tspan><tspan dy="-4"> − </tspan>' + it('x') + '<tspan font-size="11" dy="4">j</tspan>', INK, 14, 'middle')
    # rayons
    a2 = math.radians(110)
    b += arrow(xj, yj, xj + rj * math.cos(a2), yj + rj * math.sin(a2), GREY, 'a', 1.5) + text(xj - 40, yj + 70, it('r') + '<tspan font-size="11" dy="4">j</tspan>', GREY, 15)
    a3 = math.radians(20)
    b += arrow(xi, yi, xi + ri * math.cos(a3), yi + ri * math.sin(a3), GREY, 'a', 1.5) + text(xi + 50, yi + 40, it('r') + '<tspan font-size="11" dy="4">i</tspan>', GREY, 15)
    # interpénétration
    p1 = (xj + rj * nx, yj + rj * ny)   # bord de j
    p2 = (xi - ri * nx, yi - ri * ny)   # bord de i
    b += f'<line x1="{p2[0]:.1f}" y1="{p2[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" stroke="{RED}" stroke-width="5"/>'
    c = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    b += f'<circle cx="{c[0]:.1f}" cy="{c[1]:.1f}" r="5" fill="{GREEN}"/>'
    b += text(c[0] - 14, c[1] + 28, it('c'), GREEN, 17, 'end')
    b += text(p1[0] + 20, p1[1] - 40, 'δ<tspan font-size="11" dy="4">n</tspan><tspan dy="-4"> = </tspan>' + it('r') + '<tspan font-size="11" dy="4">i</tspan><tspan dy="-4"> + </tspan>' + it('r') + '<tspan font-size="11" dy="4">j</tspan><tspan dy="-4"> − ‖</tspan>' + it('x') + '<tspan font-size="11" dy="4">i</tspan><tspan dy="-4"> − </tspan>' + it('x') + '<tspan font-size="11" dy="4">j</tspan><tspan dy="-4">‖</tspan>', RED, 15)
    b += f'<line x1="{p1[0]+18:.1f}" y1="{p1[1]-36:.1f}" x2="{c[0]+4:.1f}" y2="{c[1]-4:.1f}" stroke="{RED}" stroke-width="1"/>'
    # repère n, t au point de contact
    L = 70
    b += arrow(c[0], c[1], c[0] + L * nx, c[1] + L * ny, GREEN, 'ag', 2.5) + text(c[0] + L * nx + 6, c[1] + L * ny + 18, '<tspan font-weight="bold">n</tspan>', GREEN, 16)
    tx, ty = ny, -nx  # t = (-n_y, n_x) en repère math -> écran : rotation +90° visuelle
    b += arrow(c[0], c[1], c[0] + L * tx, c[1] + L * ty, GREEN, 'ag', 2.5) + text(c[0] + L * tx - 18, c[1] + L * ty, '<tspan font-weight="bold">t</tspan>', GREEN, 16)
    b += text(xj - 60, yj - rj - 12, 'disque ' + it('j'), BLUE, 15)
    b += text(xi + 20, yi - ri - 12, 'disque ' + it('i'), RED, 15)
    b += text(20, 365, 'n = (x_i − x_j) / ‖x_i − x_j‖ : normale de j vers i ;  t : tangente ;  c : point de contact', INK, 13)
    svg('schema_contact_disques.svg', w, h, b)

# ---------------------------------------------------------------------------
# 5. Vitesse relative au point de contact (rotation)
def fig_vitesse():
    w, h = 640, 360
    xj, yj, rj = 190, 190, 120
    xi, yi, ri = 190 + 120 + 100 - 10, 190, 100
    b = ''
    b += f'<circle cx="{xj}" cy="{yj}" r="{rj}" fill="{LIGHT}" fill-opacity="0.7" stroke="{BLUE}" stroke-width="2.5"/>'
    b += f'<circle cx="{xi}" cy="{yi}" r="{ri}" fill="#fbeaea" fill-opacity="0.7" stroke="{RED}" stroke-width="2.5"/>'
    c = (xj + rj - 5, yj)
    b += f'<circle cx="{xj}" cy="{yj}" r="4" fill="{INK}"/><circle cx="{xi}" cy="{yi}" r="4" fill="{INK}"/>'
    b += text(xj - 8, yj + 22, it('x') + '<tspan font-size="11" dy="4">j</tspan>', INK, 15, 'end') + text(xi + 8, yi + 22, it('x') + '<tspan font-size="11" dy="4">i</tspan>', INK, 15)
    # vitesses des centres
    b += arrow(xj, yj, xj + 70, yj - 30, INK, 'a', 2) + text(xj + 74, yj - 34, it('v') + '<tspan font-size="11" dy="4">j</tspan>')
    b += arrow(xi, yi, xi - 50, yi - 50, INK, 'a', 2) + text(xi - 54, yi - 56, it('v') + '<tspan font-size="11" dy="4">i</tspan>', INK, 15, 'end')
    # rotations (arcs)
    def rot(x, y, r, ccw, col, lab, lx, ly):
        a0, a1 = (200, 320) if ccw else (320, 200)
        p0 = (x + r * math.cos(math.radians(a0)), y - r * math.sin(math.radians(a0)))
        p1 = (x + r * math.cos(math.radians(a1)), y - r * math.sin(math.radians(a1)))
        sweep = 0 if ccw else 1
        return f'<path d="M{p0[0]:.1f},{p0[1]:.1f} A{r},{r} 0 0 {sweep} {p1[0]:.1f},{p1[1]:.1f}" fill="none" stroke="{col}" stroke-width="2" marker-end="url(#a)"/>' + text(lx, ly, lab, col, 16)
    b += rot(xj, yj, 50, True, INK, 'ω<tspan font-size="11" dy="4">j</tspan>', xj - 20, yj + 75)
    b += rot(xi, yi, 45, False, INK, 'ω<tspan font-size="11" dy="4">i</tspan>', xi - 10, yi + 70)
    # point de contact et repère
    b += f'<circle cx="{c[0]}" cy="{c[1]}" r="5" fill="{GREEN}"/>' + text(c[0] - 10, c[1] - 12, it('c'), GREEN, 16, 'end')
    b += arrow(c[0], c[1], c[0] + 60, c[1], GREEN, 'ag', 2.2) + text(c[0] + 64, c[1] + 5, '<tspan font-weight="bold">n</tspan>', GREEN, 15)
    b += arrow(c[0], c[1], c[0], c[1] - 60, GREEN, 'ag', 2.2) + text(c[0] + 6, c[1] - 62, '<tspan font-weight="bold">t</tspan>', GREEN, 15)
    # bras de levier
    b += f'<line x1="{xi}" y1="{yi}" x2="{c[0]}" y2="{c[1]}" stroke="{GREY}" stroke-dasharray="4 3"/>'
    b += text((xi + c[0]) / 2, yi + 18, it('c') + ' − ' + it('x') + '<tspan font-size="11" dy="4">i</tspan>', GREY, 13, 'middle')
    # formules
    b += text(20, 335, 'v_rel = [v_i + ω_i e_z × (c − x_i)] − [v_j + ω_j e_z × (c − x_j)]       v_n = v_rel · n       v_t = v_rel − v_n n', INK, 13)
    svg('schema_vitesse_contact.svg', w, h, b)

# ---------------------------------------------------------------------------
# 6. Loi tangentielle : schéma rhéologique, ft(u_t), critère de Coulomb
def fig_coulomb():
    w, h = 960, 360
    b = ''
    # --- a) schéma : ressort kt + patin
    b += text(20, 30, 'a) modèle tangentiel', INK, 15, 'start', 'font-weight="bold"')
    y = 170
    b += f'<rect x="20" y="{y+18}" width="270" height="22" fill="url(#hatch)"/><line x1="20" y1="{y+18}" x2="290" y2="{y+18}" stroke="{INK}" stroke-width="2.5"/>'
    b += f'<line x1="30" y1="{y-60}" x2="30" y2="{y+18}" stroke="{INK}" stroke-width="3"/>'
    pts = [(30, y - 20), (55, y - 20)]
    for k in range(7):
        pts.append((55 + (k + 0.5) * 70 / 7, y - 20 + (12 if k % 2 == 0 else -12)))
    pts += [(125, y - 20), (150, y - 20)]
    b += '<polyline points="' + ' '.join(f'{p[0]:.1f},{p[1]:.1f}' for p in pts) + f'" fill="none" stroke="{INK}" stroke-width="2"/>'
    b += text(90, y - 42, it('k') + '<tspan font-size="11" dy="4">t</tspan>', INK, 16, 'middle')
    b += f'<rect x="150" y="{y-40}" width="70" height="58" rx="4" fill="{LIGHT}" stroke="{BLUE}" stroke-width="2"/>'
    b += text(185, y - 6, 'patin', BLUE, 13, 'middle')
    b += text(185, y + 64, 'frottement μ', INK, 14, 'middle')
    b += arrow(220, y - 12, 280, y - 12, RED, 'ar', 2.5) + text(250, y - 22, it('f') + '<tspan font-size="11" dy="4">t</tspan>', RED, 15, 'middle')
    b += arrow(185, y - 70, 185, y - 44, INK, 'a', 2) + text(195, y - 64, it('f') + '<tspan font-size="11" dy="4">n</tspan>', INK, 15)
    b += text(20, 300, 'ressort élastique tant que', INK, 13) + text(20, 318, '|f_t| &lt; μ k_n δ_n, glissement ensuite', INK, 13)
    # --- b) ft en fonction du déplacement tangentiel
    gx, gy = 340, 40
    b += text(gx, 30, 'b) force tangentielle', INK, 15, 'start', 'font-weight="bold"')
    ox, oy = gx + 140, 190
    b += arrow(gx + 10, oy, gx + 290, oy) + arrow(ox, 320, ox, 50)
    b += text(gx + 290, oy + 22, it('u') + '<tspan font-size="11" dy="4">t</tspan>', INK, 15, 'end')
    b += text(ox + 8, 58, it('f') + '<tspan font-size="11" dy="4">t</tspan>', INK, 15)
    fs = 90  # seuil (pixels)
    b += f'<line x1="{gx+10}" y1="{oy-fs}" x2="{gx+290}" y2="{oy-fs}" stroke="{GREY}" stroke-dasharray="5 4"/>'
    b += f'<line x1="{gx+10}" y1="{oy+fs}" x2="{gx+290}" y2="{oy+fs}" stroke="{GREY}" stroke-dasharray="5 4"/>'
    b += text(gx + 12, oy - fs - 6, '+μ k_n δ_n', GREY, 13) + text(gx + 12, oy + fs + 18, '−μ k_n δ_n', GREY, 13)
    k = 2.2
    # chargement : pente kt jusqu'au seuil, glissement, décharge élastique
    xa = ox + fs / k
    path = f'M{ox},{oy} L{xa:.1f},{oy-fs} L{ox+120},{oy-fs} L{ox+120-2*fs/k:.1f},{oy+fs} L{gx+20},{oy+fs}'
    b += f'<path d="{path}" fill="none" stroke="{BLUE}" stroke-width="2.8" stroke-linejoin="round"/>'
    b += arrow(ox + 60, oy - fs - 14, ox + 100, oy - fs - 14, RED, 'ar', 1.6) + text(ox + 80, oy - fs - 22, 'glissement', RED, 13, 'middle')
    b += text(ox + 18, oy - 40, 'pente ' + it('k') + '<tspan font-size="11" dy="4">t</tspan>', BLUE, 13)
    b += text(ox + 92, oy + 30, 'décharge', BLUE, 13) + text(ox + 92, oy + 46, 'élastique', BLUE, 13)
    # --- c) critère de Coulomb dans le plan (fn, ft)
    cx0 = 690
    b += text(cx0, 30, 'c) critère de Coulomb', INK, 15, 'start', 'font-weight="bold"')
    ox2, oy2 = cx0 + 20, 190
    b += arrow(ox2, oy2, ox2 + 240, oy2) + arrow(ox2, 330, ox2, 50)
    b += text(ox2 + 240, oy2 + 22, it('f') + '<tspan font-size="11" dy="4">n</tspan>', INK, 15, 'end')
    b += text(ox2 + 8, 58, it('f') + '<tspan font-size="11" dy="4">t</tspan>', INK, 15)
    mu = 0.5
    L2 = 230
    b += f'<polygon points="{ox2},{oy2} {ox2+L2},{oy2-mu*L2*1.1:.1f} {ox2+L2},{oy2+mu*L2*1.1:.1f}" fill="{LIGHT}" stroke="none"/>'
    b += f'<line x1="{ox2}" y1="{oy2}" x2="{ox2+L2}" y2="{oy2-mu*L2*1.1:.1f}" stroke="{RED}" stroke-width="2.5"/>'
    b += f'<line x1="{ox2}" y1="{oy2}" x2="{ox2+L2}" y2="{oy2+mu*L2*1.1:.1f}" stroke="{RED}" stroke-width="2.5"/>'
    b += text(ox2 + 120, oy2 - 4, 'contact', BLUE, 13, 'middle') + text(ox2 + 120, oy2 + 14, 'adhérent', BLUE, 13, 'middle')
    b += text(ox2 + 150, oy2 - 105, '|f_t| = μ f_n', RED, 14, 'middle', 'transform="rotate(-29 ' + f'{ox2+150} {oy2-105}' + ')"')
    b += text(ox2 + 150, oy2 + 125, 'glissement', RED, 13, 'middle', 'transform="rotate(29 ' + f'{ox2+150} {oy2+125}' + ')"')
    b += text(ox2 + 30, oy2 - 116, 'états inaccessibles', GREY, 12)
    svg('schema_coulomb.svg', w, h, b)

# ---------------------------------------------------------------------------
# 7. OBB élargies et liste de Verlet
def fig_obb():
    w, h = 640, 330
    b = ''
    def square(cx, cy, half, ang, style):
        pts = []
        for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
            x, y = sx * half, sy * half
            c, s = math.cos(ang), math.sin(ang)
            pts.append((cx + c * x - s * y, cy + s * x + c * y))
        return '<polygon points="' + ' '.join(f'{p[0]:.1f},{p[1]:.1f}' for p in pts) + f'" {style}/>'
    r1, r2, D2 = 80, 62, 22
    c1 = (200, 170)
    c2 = (200 + r1 + r2 + 40 + 20, 150)
    a1, a2 = math.radians(20), math.radians(-15)
    for (cx, cy), r, a, col in [(c1, r1, a1, BLUE), (c2, r2, a2, RED)]:
        b += square(cx, cy, r + D2, a, f'fill="none" stroke="{col}" stroke-width="1.6" stroke-dasharray="7 5"')
        b += square(cx, cy, r, a, f'fill="none" stroke="{col}" stroke-width="1.6"')
        b += f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="{col}" fill-opacity="0.12" stroke="{col}" stroke-width="2.5"/>'
        b += f'<circle cx="{cx}" cy="{cy}" r="3.5" fill="{INK}"/>'
    b += text(c1[0], c1[1] + 5, 'disque i', BLUE, 14, 'middle')
    b += text(c2[0], c2[1] + 5, 'disque j', RED, 14, 'middle')
    b += text(20, 30, 'OBB (trait plein) et OBB élargie de D_Verlet / 2 (tirets)', INK, 14)
    b += text(330, 300, 'les disques ne se touchent pas, mais les OBB élargies se recouvrent :', INK, 13, 'middle')
    b += text(330, 318, 'la paire (i, j) est un contact candidat de la liste de Verlet', INK, 13, 'middle', 'font-weight="bold"')
    # cote D/2
    ang = a1
    px = c1[0] + math.cos(ang) * r1 - math.sin(ang) * (-r1 * 0.2)
    py = c1[1] + math.sin(ang) * r1 + math.cos(ang) * (-r1 * 0.2)
    b += text(c1[0] - 10, c1[1] + r1 + D2 + 28, 'D_Verlet / 2', INK, 13, 'middle')
    b += f'<line x1="{c1[0]-math.sin(a1)*(-r1)+0:.1f}" y1="0" x2="0" y2="0" stroke="none"/>'
    svg('schema_obb_verlet.svg', w, h, b)

# ---------------------------------------------------------------------------
# 8. Grille de cellules
def fig_cellules():
    import random
    random.seed(3)
    w, h = 520, 420
    b = ''
    x0, y0, cs, nx, ny = 40, 30, 70, 6, 5
    # cellules voisines surlignées
    ci, cj = 2, 2
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            col = '#cfe0f5' if (di, dj) != (0, 0) else '#9cc1ea'
            b += f'<rect x="{x0+(ci+di)*cs}" y="{y0+(cj+dj)*cs}" width="{cs}" height="{cs}" fill="{col}"/>'
    for k in range(nx + 1):
        b += f'<line x1="{x0+k*cs}" y1="{y0}" x2="{x0+k*cs}" y2="{y0+ny*cs}" stroke="{GREY}" stroke-width="1.2"/>'
    for k in range(ny + 1):
        b += f'<line x1="{x0}" y1="{y0+k*cs}" x2="{x0+nx*cs}" y2="{y0+k*cs}" stroke="{GREY}" stroke-width="1.2"/>'
    pts = []
    while len(pts) < 55:
        x, y = random.uniform(x0 + 10, x0 + nx * cs - 10), random.uniform(y0 + 10, y0 + ny * cs - 10)
        if all((x - a) ** 2 + (y - c) ** 2 > 22 ** 2 for a, c in pts):
            pts.append((x, y))
    ip = (x0 + ci * cs + 38, y0 + cj * cs + 30)
    for x, y in pts:
        if (x - ip[0]) ** 2 + (y - ip[1]) ** 2 < 30 ** 2:
            continue
        inside = abs((x - x0) // cs - ci) <= 1 and abs((y - y0) // cs - cj) <= 1
        b += f'<circle cx="{x:.1f}" cy="{y:.1f}" r="9" fill="{BLUE if inside else "#bbb"}" fill-opacity="{0.85 if inside else 0.6}"/>'
    b += f'<circle cx="{ip[0]}" cy="{ip[1]}" r="10" fill="{RED}"/>' + text(ip[0] + 14, ip[1] - 10, it('i'), RED, 16)
    b += dim(x0, y0 + ny * cs + 18, x0 + cs, y0 + ny * cs + 18)
    b += text(x0 + cs / 2, y0 + ny * cs + 40, 'côté de cellule', INK, 13, 'middle')
    b += text(x0 + cs + 20, y0 + ny * cs + 40, '≥ taille de la plus grande OBB élargie', INK, 13)
    b += text(x0, y0 + ny * cs + 66, 'seules les particules des 9 cellules voisines (en bleu) sont testées avec i', INK, 13)
    svg('schema_cellules.svg', w, h, b)


# ---------------------------------------------------------------------------
# Versions révisées (mise en page)
def fig_disques():
    w, h = 720, 420
    xj, yj, rj = 240, 250, 140
    ri = 100
    ang = math.radians(-20)
    nx, ny = math.cos(ang), math.sin(ang)
    ov = 26
    dist = rj + ri - ov
    xi, yi = xj + dist * nx, yj + dist * ny
    b = ''
    b += f'<circle cx="{xj}" cy="{yj}" r="{rj}" fill="{LIGHT}" fill-opacity="0.7" stroke="{BLUE}" stroke-width="2.5"/>'
    b += f'<circle cx="{xi:.1f}" cy="{yi:.1f}" r="{ri}" fill="#fbeaea" fill-opacity="0.7" stroke="{RED}" stroke-width="2.5"/>'
    b += f'<circle cx="{xj}" cy="{yj}" r="4" fill="{INK}"/>' + text(xj - 8, yj + 24, it('x') + S('j'), INK, 16, 'end')
    b += f'<circle cx="{xi:.1f}" cy="{yi:.1f}" r="4" fill="{INK}"/>' + text(xi + 4, yi + 26, it('x') + S('i'), INK, 16)
    # vecteur branche (de x_j vers x_i), décalé sous la ligne des centres
    off = 22
    ox, oy = -ny * off, nx * off   # décalage perpendiculaire (vers le bas à l'écran)
    b += arrow(xj + ox, yj + oy, xi + ox - 4 * nx, yi + oy - 4 * ny, INK, 'a', 1.6)
    b += f'<line x1="{xj}" y1="{yj}" x2="{xi:.1f}" y2="{yi:.1f}" stroke="{GREY}" stroke-width="1" stroke-dasharray="4 3"/>'
    mx, my = (xj + xi) / 2 + 3.2 * ox - 10, (yj + yi) / 2 + 3.2 * oy + 4
    b += text(mx, my, 'vecteur branche ' + it('x') + S('i') + ' − ' + it('x') + S('j'), INK, 14, 'middle')
    # rayons
    a2 = math.radians(125)
    b += arrow(xj, yj, xj + rj * math.cos(a2), yj + rj * math.sin(a2), GREY, 'a', 1.5) + text(xj - 60, yj + 60, it('r') + S('j'), GREY, 15)
    a3 = math.radians(60)
    b += arrow(xi, yi, xi + ri * math.cos(a3), yi + ri * math.sin(a3), GREY, 'a', 1.5) + text(xi + 34, yi + 50, it('r') + S('i'), GREY, 15)
    # interpénétration
    p1 = (xj + rj * nx, yj + rj * ny)
    p2 = (xi - ri * nx, yi - ri * ny)
    b += f'<line x1="{p2[0]:.1f}" y1="{p2[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" stroke="{RED}" stroke-width="5"/>'
    c = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
    lx, ly = c[0] - 40, 50
    b += f'<line x1="{lx+10:.1f}" y1="{ly+6:.1f}" x2="{c[0]:.1f}" y2="{c[1]-4:.1f}" stroke="{RED}" stroke-width="1"/>'
    b += text(lx - 120, ly, 'δ' + S('n') + ' = ' + it('r') + S('i') + ' + ' + it('r') + S('j') + ' − ‖' + it('x') + S('i') + ' − ' + it('x') + S('j') + '‖', RED, 15)
    b += f'<circle cx="{c[0]:.1f}" cy="{c[1]:.1f}" r="5" fill="{GREEN}"/>'
    b += text(c[0] - 12, c[1] + 26, it('c'), GREEN, 17, 'end')
    L = 62
    b += arrow(c[0], c[1], c[0] + L * nx, c[1] + L * ny, GREEN, 'ag', 2.5) + text(c[0] + L * nx + 4, c[1] + L * ny - 8, '<tspan font-weight="bold">n</tspan>', GREEN, 16)
    tx, ty = ny, -nx
    b += arrow(c[0], c[1], c[0] + L * tx, c[1] + L * ty, GREEN, 'ag', 2.5) + text(c[0] + L * tx - 10, c[1] + L * ty + 4, '<tspan font-weight="bold">t</tspan>', GREEN, 16, 'end')
    b += text(xj - 120, yj - rj + 10, 'disque ' + it('j'), BLUE, 15)
    b += text(xi + ri - 10, yi - ri + 10, 'disque ' + it('i'), RED, 15)
    b += text(20, 405, '<tspan font-weight="bold">n</tspan> : normale unitaire de j vers i ;  <tspan font-weight="bold">t</tspan> = (−n' + S('y') + ', n' + S('x') + ') : tangente ;  ' + it('c') + ' : point de contact (milieu de la zone de recouvrement)', INK, 13)
    svg('schema_contact_disques.svg', w, h, b)

def fig_vitesse():
    w, h = 660, 420
    xj, yj, rj = 200, 200, 120
    xi, yi, ri = 200 + 120 + 100 - 10, 200, 100
    b = ''
    b += f'<circle cx="{xj}" cy="{yj}" r="{rj}" fill="{LIGHT}" fill-opacity="0.7" stroke="{BLUE}" stroke-width="2.5"/>'
    b += f'<circle cx="{xi}" cy="{yi}" r="{ri}" fill="#fbeaea" fill-opacity="0.7" stroke="{RED}" stroke-width="2.5"/>'
    c = (xj + rj - 5, yj)
    b += f'<circle cx="{xj}" cy="{yj}" r="4" fill="{INK}"/><circle cx="{xi}" cy="{yi}" r="4" fill="{INK}"/>'
    b += text(xj - 8, yj + 22, it('x') + S('j'), INK, 15, 'end') + text(xi + 8, yi + 22, it('x') + S('i'), INK, 15)
    b += arrow(xj, yj, xj + 60, yj - 50, INK, 'a', 2) + text(xj + 64, yj - 54, it('v') + S('j'))
    b += arrow(xi, yi, xi + 40, yi - 60, INK, 'a', 2) + text(xi + 44, yi - 64, it('v') + S('i'))
    def rot(x, y, r, ccw, lab, lx, ly):
        a0, a1 = (200, 320) if ccw else (320, 200)
        p0 = (x + r * math.cos(math.radians(a0)), y - r * math.sin(math.radians(a0)))
        p1 = (x + r * math.cos(math.radians(a1)), y - r * math.sin(math.radians(a1)))
        sweep = 0 if ccw else 1
        return f'<path d="M{p0[0]:.1f},{p0[1]:.1f} A{r},{r} 0 0 {sweep} {p1[0]:.1f},{p1[1]:.1f}" fill="none" stroke="{INK}" stroke-width="2" marker-end="url(#a)"/>' + text(lx, ly, lab, INK, 16)
    b += rot(xj, yj, 55, True, 'ω' + S('j'), xj - 8, yj + 82)
    b += rot(xi, yi, 50, False, 'ω' + S('i'), xi - 8, yi + 78)
    b += f'<line x1="{xi}" y1="{yi}" x2="{c[0]}" y2="{c[1]}" stroke="{GREY}" stroke-dasharray="4 3"/>'
    b += f'<circle cx="{c[0]}" cy="{c[1]}" r="5" fill="{GREEN}"/>' + text(c[0] - 10, c[1] + 22, it('c'), GREEN, 16, 'end')
    b += arrow(c[0], c[1], c[0] + 55, c[1], GREEN, 'ag', 2.2) + text(c[0] + 50, c[1] - 8, '<tspan font-weight="bold">n</tspan>', GREEN, 15)
    b += arrow(c[0], c[1], c[0], c[1] - 60, GREEN, 'ag', 2.2) + text(c[0] + 8, c[1] - 50, '<tspan font-weight="bold">t</tspan>', GREEN, 15)
    y0 = 360
    b += text(20, y0, 'v' + S('rel') + ' = [ v' + S('i') + ' + ω' + S('i') + ' e' + S('z') + ' × (c − x' + S('i') + ') ] − [ v' + S('j') + ' + ω' + S('j') + ' e' + S('z') + ' × (c − x' + S('j') + ') ]', INK, 14)
    b += text(20, y0 + 26, 'v' + S('n') + ' = v' + S('rel') + ' · n  (partie normale)        v' + S('t') + ' = v' + S('rel') + ' − v' + S('n') + ' n  (glissement)', INK, 14, 'start', 'xml:space="preserve"')
    svg('schema_vitesse_contact.svg', w, h, b)

def fig_coulomb():
    w, h = 980, 370
    b = ''
    knd = 'μ ' + it('k') + S('n') + ' δ' + S('n')
    b += text(20, 30, 'a) modèle tangentiel', INK, 15, 'start', 'font-weight="bold"')
    y = 170
    b += f'<rect x="20" y="{y+18}" width="270" height="22" fill="url(#hatch)"/><line x1="20" y1="{y+18}" x2="290" y2="{y+18}" stroke="{INK}" stroke-width="2.5"/>'
    b += f'<line x1="30" y1="{y-60}" x2="30" y2="{y+18}" stroke="{INK}" stroke-width="3"/>'
    pts = [(30, y - 20), (55, y - 20)]
    for k in range(7):
        pts.append((55 + (k + 0.5) * 70 / 7, y - 20 + (12 if k % 2 == 0 else -12)))
    pts += [(125, y - 20), (150, y - 20)]
    b += '<polyline points="' + ' '.join(f'{p[0]:.1f},{p[1]:.1f}' for p in pts) + f'" fill="none" stroke="{INK}" stroke-width="2"/>'
    b += text(90, y - 42, it('k') + S('t'), INK, 16, 'middle')
    b += f'<rect x="150" y="{y-40}" width="70" height="58" rx="4" fill="{LIGHT}" stroke="{BLUE}" stroke-width="2"/>'
    b += text(185, y - 6, 'patin', BLUE, 13, 'middle')
    b += text(185, y + 66, 'frottement μ', INK, 14, 'middle')
    b += arrow(220, y - 12, 280, y - 12, RED, 'ar', 2.5) + text(250, y - 22, it('f') + S('t'), RED, 15, 'middle')
    b += arrow(185, y - 76, 185, y - 44, INK, 'a', 2) + text(195, y - 66, it('f') + S('n'), INK, 15)
    b += text(20, 300, 'ressort élastique tant que |' + it('f') + S('t') + '| &lt; ' + knd + ',', INK, 13)
    b += text(20, 320, 'glissement du patin au-delà', INK, 13)
    # b)
    gx = 350
    b += text(gx, 30, 'b) force tangentielle', INK, 15, 'start', 'font-weight="bold"')
    ox, oy = gx + 130, 190
    b += arrow(gx + 10, oy, gx + 300, oy) + arrow(ox, 330, ox, 50)
    b += text(gx + 300, oy - 10, it('u') + S('t'), INK, 15, 'end')
    b += text(ox + 8, 58, it('f') + S('t'), INK, 15)
    fs = 90
    b += f'<line x1="{gx+10}" y1="{oy-fs}" x2="{gx+300}" y2="{oy-fs}" stroke="{GREY}" stroke-dasharray="5 4"/>'
    b += f'<line x1="{gx+10}" y1="{oy+fs}" x2="{gx+300}" y2="{oy+fs}" stroke="{GREY}" stroke-dasharray="5 4"/>'
    b += text(gx + 6, oy - fs - 6, '+' + knd, GREY, 13) + text(gx + 6, oy + fs + 18, '−' + knd, GREY, 13)
    k = 2.2
    xa = ox + fs / k
    xb = ox + 130
    path = f'M{ox},{oy} L{xa:.1f},{oy-fs} L{xb},{oy-fs} L{xb-2*fs/k:.1f},{oy+fs} L{gx+20},{oy+fs}'
    b += f'<path d="{path}" fill="none" stroke="{BLUE}" stroke-width="2.8" stroke-linejoin="round"/>'
    b += arrow(xa + 20, oy - fs - 14, xb - 10, oy - fs - 14, RED, 'ar', 1.6) + text((xa + xb) / 2, oy - fs - 22, 'glissement', RED, 13, 'middle')
    b += text(ox - 8, oy - 50, 'pente ' + it('k') + S('t'), BLUE, 13, 'end')
    b += text(xb + 6, oy + 50, 'décharge', BLUE, 13) + text(xb + 6, oy + 66, 'élastique', BLUE, 13)
    # c)
    cx0 = 700
    b += text(cx0, 30, 'c) critère de Coulomb', INK, 15, 'start', 'font-weight="bold"')
    ox2, oy2 = cx0 + 20, 190
    L2 = 200
    sl = 0.55
    b += f'<polygon points="{ox2},{oy2} {ox2+L2},{oy2-sl*L2:.1f} {ox2+L2},{oy2+sl*L2:.1f}" fill="{LIGHT}" stroke="none"/>'
    b += f'<line x1="{ox2}" y1="{oy2}" x2="{ox2+L2}" y2="{oy2-sl*L2:.1f}" stroke="{RED}" stroke-width="2.5"/>'
    b += f'<line x1="{ox2}" y1="{oy2}" x2="{ox2+L2}" y2="{oy2+sl*L2:.1f}" stroke="{RED}" stroke-width="2.5"/>'
    b += arrow(ox2, oy2, ox2 + 245, oy2) + arrow(ox2, 340, ox2, 50)
    b += text(ox2 + 245, oy2 + 22, it('f') + S('n'), INK, 15, 'end')
    b += text(ox2 + 8, 58, it('f') + S('t'), INK, 15)
    b += text(ox2 + 125, oy2 - 30, 'contact', BLUE, 13, 'middle') + text(ox2 + 125, oy2 - 14, 'adhérent', BLUE, 13, 'middle')
    ang = math.degrees(math.atan(sl))
    px, py = ox2 + 95, oy2 - sl * 95 - 10
    b += text(px, py, '|' + it('f') + S('t') + '| = μ ' + it('f') + S('n'), RED, 14, 'middle', f'transform="rotate({-ang:.1f} {px} {py:.1f})"')
    px, py = ox2 + 95, oy2 + sl * 95 + 22
    b += text(px, py, 'glissement', RED, 13, 'middle', f'transform="rotate({ang:.1f} {px} {py:.1f})"')
    b += text(ox2 + 30, 75, 'états inaccessibles', GREY, 12)
    svg('schema_coulomb.svg', w, h, b)

def fig_obb():
    w, h = 640, 370
    b = ''
    def corner(cx, cy, a, u, v):
        c, s = math.cos(a), math.sin(a)
        return (cx + c * u - s * v, cy + s * u + c * v)
    def square(cx, cy, half, ang, style):
        pts = [corner(cx, cy, ang, sx * half, sy * half) for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
        return '<polygon points="' + ' '.join(f'{p[0]:.1f},{p[1]:.1f}' for p in pts) + f'" {style}/>'
    r1, r2, D2 = 80, 62, 24
    c1 = (190, 185)
    c2 = (190 + r1 + r2 + 60, 165)
    a1, a2 = math.radians(20), math.radians(-15)
    for (cx, cy), r, a, col in [(c1, r1, a1, BLUE), (c2, r2, a2, RED)]:
        b += square(cx, cy, r + D2, a, f'fill="none" stroke="{col}" stroke-width="1.6" stroke-dasharray="7 5"')
        b += square(cx, cy, r, a, f'fill="none" stroke="{col}" stroke-width="1.6"')
        b += f'<circle cx="{cx}" cy="{cy}" r="{r}" fill="{col}" fill-opacity="0.12" stroke="{col}" stroke-width="2.5"/>'
        b += f'<circle cx="{cx}" cy="{cy}" r="3.5" fill="{INK}"/>'
    b += text(c1[0], c1[1] + 22, 'disque i', BLUE, 14, 'middle')
    b += text(c2[0], c2[1] + 22, 'disque j', RED, 14, 'middle')
    # cote D_Verlet/2 sur le côté bas gauche de l'OBB de i
    pa = corner(c1[0], c1[1], a1, -r1 * 0.3, r1)
    pb = corner(c1[0], c1[1], a1, -r1 * 0.3, r1 + D2)
    b += dim(pa[0], pa[1], pb[0], pb[1])
    b += text(pb[0] - 14, pb[1] + 26, 'D' + S('Verlet') + ' / 2', INK, 14, 'middle')
    # cote rayon (demi-côté de l'OBB)
    b += arrow(c1[0], c1[1], *corner(c1[0], c1[1], a1, -r1, 0), GREY, 'a', 1.4)
    b += text(c1[0] - 50, c1[1] - 26, it('r') + S('i'), GREY, 14)
    b += text(20, 28, 'OBB (trait plein) et OBB élargie de D' + S('Verlet') + ' / 2 (tirets)', INK, 14)
    b += text(330, 335, 'les disques ne se touchent pas, mais les OBB élargies se recouvrent :', INK, 13, 'middle')
    b += text(330, 355, 'la paire (i, j) est un contact candidat de la liste de Verlet', INK, 13, 'middle', 'font-weight="bold"')
    svg('schema_obb_verlet.svg', w, h, b)

_old_cells = fig_cellules
def fig_cellules():
    _old_cells()
    f = os.path.join(OUT, 'schema_cellules.svg')
    t = open(f).read().replace('height="420" viewBox="0 0 520 420"', 'height="470" viewBox="0 0 520 470"').replace('<rect width="520" height="420"', '<rect width="520" height="470"')
    open(f, 'w').write(t)

for f in [fig_algo, fig_sol, fig_normale, fig_disques, fig_vitesse, fig_coulomb, fig_obb, fig_cellules]:
    f()
print('ok')
