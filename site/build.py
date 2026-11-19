#!/usr/bin/env python3
"""Génère les sources du site MkDocs à partir de TUTORIEL.md.

Usage (depuis la racine du dépôt) :
    python3 site/build.py          # écrit site/docs/ et site/mkdocs.yml
    mkdocs serve -f site/mkdocs.yml

TUTORIEL.md reste la seule source : chaque section « ## » devient une page,
les liens internes (#ancre) sont réécrits vers la bonne page, et les corrigés
sont affichés et proposés au téléchargement.
"""
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "site"
DOCS = OUT / "docs"
REPO_URL = "https://github.com/topinvincent/Tuto_DEM2D"


def slugify(text):
    """Ancre au format GitHub (même règle que pymdownx.slugs.slugify, case=lower)."""
    text = re.sub(r"<[^>]+>", "", text).replace("`", "").replace("*", "")
    text = re.sub(r"[^\w\- ]", "", text.strip().lower())
    return text.replace(" ", "-")


def page_name(title, idx):
    m = re.match(r"Étape (\d+)", title)
    if m:
        return f"etape{m.group(1)}.md"
    m = re.match(r"Annexe ([A-Z])", title)
    if m:
        return f"annexe-{m.group(1).lower()}.md"
    if title.startswith("0."):
        return "prerequis.md"
    return f"page{idx}.md"


def is_fence(line):
    return line.lstrip().startswith("```")


def split_sections(lines):
    """Découpe en sections de niveau 2, en ignorant les « # » des blocs de code."""
    intro, sections, cur, in_code = [], [], None, False
    for line in lines:
        if is_fence(line):
            in_code = not in_code
        if not in_code and line.startswith("## "):
            cur = {"title": line[3:].strip(), "lines": [line]}
            sections.append(cur)
            continue
        (cur["lines"] if cur else intro).append(line)
    return intro, sections


def headings(lines):
    in_code = False
    for line in lines:
        if is_fence(line):
            in_code = not in_code
        m = None if in_code else re.match(r"^#{1,6} (.+)$", line)
        if m:
            yield m.group(1).strip()


def strip_trailing_rule(text):
    return re.sub(r"(\n\s*-{3,}\s*)+$", "\n", text.rstrip() + "\n")


def main():
    lines = (ROOT / "TUTORIEL.md").read_text(encoding="utf-8").splitlines()
    intro, sections = split_sections(lines)
    sections = [s for s in sections if s["title"] != "Sommaire"]

    if DOCS.exists():
        shutil.rmtree(DOCS)
    DOCS.mkdir(parents=True)

    anchors = {}
    for i, s in enumerate(sections):
        s["file"] = page_name(s["title"], i)
        for h in headings(s["lines"]):
            anchors.setdefault(slugify(h), s["file"])

    def fix_links(md, current):
        def repl(m):
            label, target = m.group(1), m.group(2)
            if target.startswith("#"):
                page = anchors.get(target[1:])
                if page and page != current:
                    target = page + target
            elif re.match(r"(corriges|viz)/[\w.]+\.(cpp|hpp)$", target):
                target = "code/" + Path(target).name.replace(".", "_") + ".md"
            elif target == "TUTORIEL.md":
                target = "index.md"
            return f"[{label}]({target})"
        return re.sub(r"\[([^\]]*)\]\(([^)\s]+)\)", repl, md)

    # Pages de contenu : « ## Titre » devient le titre de page, les niveaux remontent d'un cran
    for s in sections:
        out, in_code = [], False
        for line in s["lines"]:
            if is_fence(line):
                in_code = not in_code
            if not in_code and re.match(r"^#{2,6} ", line):
                line = line[1:]
            out.append(line)
        body = strip_trailing_rule("\n".join(out))
        (DOCS / s["file"]).write_text(fix_links(body, s["file"]), encoding="utf-8")

    # Accueil : introduction + parcours + téléchargements
    home = strip_trailing_rule(fix_links("\n".join(intro), "index.md"))
    home += "\n## Parcours\n\n" + "\n".join(
        f"- [{s['title']}]({s['file']})" for s in sections) + "\n"
    home += ("\n## Télécharger\n\n"
             "- [Archive complète](telechargements/tutoriel-dem2d.zip) : texte, corrigés, "
             "librairie de visualisation `viz/dem_viz.hpp` et Makefile.\n"
             "- Code source de chaque étape : rubrique **Corrigés** du menu.\n"
             f"- Dépôt GitHub : [{REPO_URL.split('github.com/')[1]}]({REPO_URL}).\n")
    (DOCS / "index.md").write_text(home, encoding="utf-8")

    shutil.copytree(ROOT / "images", DOCS / "images")
    shutil.copytree(OUT / "javascripts", DOCS / "javascripts")

    # Corrigés et librairie de visualisation : une page par fichier + fichier brut
    code_dir, raw_dir = DOCS / "code", DOCS / "telechargements"
    code_dir.mkdir()
    raw_dir.mkdir()
    code_pages = []
    files = sorted((ROOT / "corriges").glob("*.?pp")) + sorted((ROOT / "viz").glob("*.hpp"))
    for f in files:
        shutil.copy(f, raw_dir / f.name)
        name = f.name.replace(".", "_") + ".md"
        src = f.read_text(encoding="utf-8").rstrip()
        (code_dir / name).write_text(
            f"# {f.parent.name}/{f.name}\n\n"
            f"[Télécharger {f.name}](../telechargements/{f.name}){{ .md-button }}\n\n"
            f"````cpp linenums=\"1\"\n{src}\n````\n",
            encoding="utf-8")
        code_pages.append((f"{f.parent.name}/{f.name}", "code/" + name))

    # Archive complète du tutoriel
    tmp = OUT / "_archive"
    if tmp.exists():
        shutil.rmtree(tmp)
    dest = tmp / "tutoriel-dem2d"
    dest.mkdir(parents=True)
    for item in ["TUTORIEL.md", "README.md", "Makefile", "corriges", "viz", "images"]:
        p = ROOT / item
        if p.is_dir():
            shutil.copytree(p, dest / item)
        elif p.exists():
            shutil.copy(p, dest / item)
    shutil.make_archive(str(raw_dir / "tutoriel-dem2d"), "zip", tmp)
    shutil.rmtree(tmp)

    # Navigation
    nav = ["  - Accueil: index.md"]
    nav += [f'  - "{s["title"]}": {s["file"]}' for s in sections]
    nav.append("  - Corrigés:")
    nav += [f'    - "{n}": {p}' for n, p in code_pages]
    config = (OUT / "mkdocs.base.yml").read_text(encoding="utf-8")
    (OUT / "mkdocs.yml").write_text(
        "# Fichier généré par site/build.py : modifier mkdocs.base.yml\n"
        + config + "\nnav:\n" + "\n".join(nav) + "\n", encoding="utf-8")
    print(f"{len(sections)} pages, {len(code_pages)} fichiers de code -> {DOCS}")


if __name__ == "__main__":
    main()
