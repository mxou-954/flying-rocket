#!/usr/bin/env python3
"""Étude de convergence sur le pas de temps dt.

Relance la simulation pour plusieurs valeurs de dt (mêmes paramètres, même
graine) et compare le point d'impact au résultat obtenu avec le dt le plus fin,
pris comme référence.

Le point d'impact est obtenu en interpolant linéairement les deux derniers pas
de la trajectoire jusqu'à z = 0 : sans cette interpolation, on mesurerait
surtout le dépassement discret du dernier pas sous le sol, pas l'erreur
d'intégration.

Le binaire et le fichier de configuration sont cherchés à la racine du dépôt : le
script peut être lancé depuis n'importe quel répertoire.

    python tools/convergence.py [--exe chemin/vers/m31] [--config config.txt]
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import tempfile

DTS = [0.5, 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002]

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Emplacements possibles du binaire selon le generateur CMake (mono ou multi-config).
EXE_CANDIDATES = [
    "build/m31", "build/m31.exe",
    "build/Release/m31.exe", "build/Debug/m31.exe",
]


def find_exe():
    for rel in EXE_CANDIDATES:
        path = os.path.join(ROOT, *rel.split("/"))
        if os.path.isfile(path):
            return path
    sys.exit("Binaire introuvable. Compilez d'abord :\n"
             "    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release\n"
             "    cmake --build build --config Release\n"
             "ou indiquez le chemin avec --exe.")


def read_config(path):
    cfg = {}
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].split() # .split() écoupe selon les espaces seulement
        if len(line) >= 2:
            cfg[line[0]] = line[1]
    return cfg


def impact_from_csv(path):
    """Interpole le passage z = 0 entre les deux derniers points de la trajectoire."""
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    a, b = rows[-2], rows[-1]
    za, zb = float(a["z"]), float(b["z"])
    s = 0.0 if za == zb else za / (za - zb)   # fraction du dernier pas jusqu'à z = 0
    lerp = lambda k: float(a[k]) + s * (float(b[k]) - float(a[k]))
    return (lerp("x"), lerp("y")), lerp("t")


def run(exe, cfg, dt, workdir):
    cfg = dict(cfg)
    cfg["dt"] = repr(dt)
    cfg["output_file"] = os.path.join(workdir, "traj_%g.csv" % dt)
    cfg_path = os.path.join(workdir, "config_%g.txt" % dt)
    with open(cfg_path, "w", encoding="utf-8") as f:
        for k, v in cfg.items():
            f.write("%s %s\n" % (k, v))

    subprocess.run([exe, cfg_path], capture_output=True, text=True, check=True)
    return impact_from_csv(cfg["output_file"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=None, help="chemin du binaire (auto-detecte par defaut)")
    ap.add_argument("--config", default=os.path.join(ROOT, "config.txt"))
    args = ap.parse_args()

    exe = os.path.abspath(args.exe) if args.exe else find_exe()
    base = read_config(args.config)
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for dt in DTS:
            results.append((dt, *run(exe, base, dt, tmp)))

    ref_pos = results[-1][1]  # dt le plus fin = référence

    print("dt        t_impact    x_impact       y_impact       ecart/ref (m)  ordre obs.")
    prev_err, prev_dt = None, None
    for dt, pos, t in results:
        err = math.dist(pos, ref_pos) # distance euclidienne point 2d
        order = ""
        if prev_err and err > 0:
            order = "%.2f" % (math.log(prev_err / err) / math.log(prev_dt / dt))
        print("%-9g %-11.4f %-14.3f %-14.3f %-14.3f %s"
              % (dt, t, pos[0], pos[1], err, order))
        prev_err, prev_dt = err, dt

    print("\nReference : dt = %g" % results[-1][0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
