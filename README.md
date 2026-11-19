# Tutoriel DEM 2D en C++

Construire pas à pas un petit code DEM pour des disques en 2D, sur les principes d'exaDEM et de Rockable.

- **Le tutoriel** : [TUTORIEL.md](TUTORIEL.md) (8 étapes progressives), également consultable en ligne : <https://topinvincent.github.io/Tuto_DEM2D/>.
- **Les corrigés** : `corriges/`, un programme autonome par étape.
- **La visualisation** : `viz/dem_viz.hpp`, librairie fournie (images SVG, visionneuse HTML, fichiers ParaView).

```bash
make          # compile les corrigés dans build/ (plus etape5_tests : cas analytiques)
make test     # les compile et les exécute (environ 1 min)
make etape4   # une seule étape
```

Les animations sont dans `build/sortie_etapeN/viewer.html` (à ouvrir dans un navigateur).

Testé avec g++ 13 (C++17), sans dépendance externe.

## Site web

Le site est généré à partir de `TUTORIEL.md` et publié automatiquement sur GitHub Pages à chaque push sur `main` (`.github/workflows/pages.yml`). Pour le prévisualiser localement :

```bash
pip install mkdocs-material
python3 site/build.py
mkdocs serve -f site/mkdocs.yml
```
