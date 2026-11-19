# Tutoriel DEM 2D en C++

Construire pas à pas un petit code DEM pour des disques en 2D, sur les principes d'exaDEM et de Rockable.

- **Le tutoriel** : [TUTORIEL.md](TUTORIEL.md) (8 étapes progressives).
- **Les corrigés** : `corriges/`, un programme autonome par étape.
- **La visualisation** : `viz/dem_viz.hpp`, librairie fournie (images SVG, visionneuse HTML, fichiers ParaView).

```bash
make          # compile les corrigés dans build/ (plus etape5_tests : cas analytiques)
make test     # les compile et les exécute (environ 1 min)
make etape4   # une seule étape
```

Les animations sont dans `build/sortie_etapeN/viewer.html` (à ouvrir dans un navigateur).

Testé avec g++ 13 (C++17), sans dépendance externe.
