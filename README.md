# M31 – simulation de trajectoire guidée (modèle simplifié)

Simulation en C++ d'une trajectoire de roquette guidée en 3D, écrite comme projet
d'apprentissage de la modélisation numérique. Le code intègre un point matériel
soumis à la gravité, à une traînée aérodynamique simplifiée et à une poussée, avec
un correcteur PD qui ramène le mobile vers une trajectoire de référence parabolique.

**Ce projet n'est pas un simulateur de vol validé.** Les paramètres portent le nom
d'un système réel mais aucune donnée aérodynamique, propulsive ou de guidage réelle
n'a été utilisée, et aucun résultat n'a été confronté à des mesures. Voir
[Hypothèses](#hypothèses-du-modèle) et [Limites](#limites-connues).

---

## Modèle

À chaque pas de temps :

1. **Trajectoire de référence** — parabole symétrique reliant le point de départ à la
   cible, d'altitude maximale `apex`. Interpolation linéaire en (x, y), parabolique
   en z. Le temps de vol `Thit = 2·√(2·apex/g)` en est déduit ; ce n'est pas un
   paramètre de configuration.
2. **Poussée** — verticale, modulée par l'écart d'altitude à la référence
   (`facteur = clamp(Δz / 500, 0, 1)`), jusqu'à épuisement du carburant. La masse
   décroît au taux `burn_rate`.
3. **Traînée** — `a = −½·ρ(z)·Cd·A·‖v−v_vent‖·(v−v_vent) / m`.
4. **Atmosphère** — densité exponentielle `ρ(z) = 1.225·exp(−z/8500)`, vitesse du son
   ISA troposphérique `c = √(γ·R·T)` avec `T(z) = max(288.15 − 0.0065·z, 216.65)`.
5. **Vent** — vecteur horizontal constant tiré une fois au démarrage
   (0–15 m/s, direction uniforme), à partir d'une graine fixée.
6. **Guidage PD** — `correction = Kp·erreur_position + Kd·erreur_vitesse`, appliquée
   à la vitesse, bornée par `‖v‖·tan(max_angle_deg)`.
   Kp = 1.0 / Kd = 1.2 en croisière ; Kp = 6.0 / Kd = 4.0 dans les 30 dernières
   secondes avant le temps de vol théorique.
7. **Intégration** — Euler semi-implicite (symplectique) : `v ← v + a·dt`, puis
   `x ← x + v·dt`. Arrêt quand `z ≤ 0` ou `t > T`.

### Hypothèses du modèle

- Point matériel : ni orientation, ni moments d'inertie, ni dynamique d'attitude.
  `nose_dir` existe dans la structure mais n'est pas utilisé.
- Gravité constante `g = 9.81 m/s²`, Terre plate, pas de rotation ni de Coriolis.
- Poussée strictement verticale et instantanément modulable — ce n'est pas le
  comportement d'un propulseur à propergol solide, qui brûle selon une loi imposée.
- Coefficient de traînée `Cd` constant : aucune dépendance au nombre de Mach, alors
  que la trajectoire est transsonique/supersonique. Le nombre de Mach est calculé et
  journalisé, mais **n'entre pas** dans le calcul de la traînée.
- Le guidage agit directement sur le vecteur vitesse : il n'y a pas de gouvernes,
  pas de portance, pas de retard d'actionneur. `max_angle_deg` sert seulement à
  borner l'amplitude de la correction.
- Vent constant en temps et en espace, purement horizontal, sans rafales ni
  cisaillement vertical.
- Consommation de carburant indépendante de la modulation de poussée (incohérence
  physique assumée du modèle actuel).

### Limites connues

- **Aucune validation** : les résultats ne sont comparés à aucune référence
  analytique, expérimentale ou logicielle. La précision d'impact affichée (quelques
  mètres) mesure la performance du correcteur PD *dans ce modèle*, pas une précision
  balistique réelle.
- Les paramètres de `config.txt` (masse, poussée, Cd, surface) sont des ordres de
  grandeur plausibles, pas des données constructeur.
- Le correcteur PD applique une correction proportionnelle à `dt` : le système
  simulé dépend légèrement du pas de temps (voir [docs/convergence.md](docs/convergence.md)).
- L'intégrateur est d'ordre 1 ; l'ordre de convergence observé est ≈ 1.
- Un seul tirage de vent par exécution : pas d'analyse statistique de dispersion.

---

## Compilation

Prérequis : CMake ≥ 3.15 et un compilateur C++17 (GCC, Clang ou MSVC).

```bash
git clone https://github.com/mxou-954/M31-simulation.git
cd M31-simulation
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Cibles produites : `m31` (simulation) et `m31_tests` (tests unitaires).

## Exécution

Depuis la racine du dépôt (les chemins de `config.txt` sont relatifs au répertoire
courant) :

```bash
mkdir -p data
./build/m31            # utilise ./config.txt
./build/m31 autre.txt  # ou un fichier de configuration explicite
```

Sous Windows / MSVC, le binaire se trouve dans `build/Release/m31.exe`.

Le programme retourne un code de sortie non nul et un message explicite si le
fichier de configuration est absent, mal formé, contient une clé inconnue, une
valeur non numérique ou un paramètre hors domaine (`dt ≤ 0`, `mass ≤ 0`,
`fuel ≥ mass`, `max_angle_deg ∉ ]0,90[`, …), ou si le CSV de sortie ne peut pas
être écrit.

## Tests

```bash
ctest --test-dir build --output-on-failure
```

Neuf groupes d'assertions : algèbre `Vec3`, densité de l'air, vitesse du son,
temps de vol théorique, validation des paramètres, chargement de configuration
(fichier manquant / valeur invalide / clé inconnue), reproductibilité par graine,
cohérence d'une simulation nominale, et gestion d'un chemin de sortie invalide.

La CI GitHub Actions (`.github/workflows/ci.yml`) compile et exécute les tests sur
Ubuntu et Windows à chaque push et pull request.

## Reproductibilité

Le seul élément aléatoire est le tirage du vent. Il utilise un `std::mt19937` seedé
par le paramètre `seed` de la configuration :

```
seed 42
```

À configuration identique, deux exécutions produisent **exactement** la même
trajectoire, quelle que soit la machine (un test le vérifie). Changer `seed` change
le vent, et donc la trajectoire.

## Étude de convergence

Le pas de temps a été balayé de 0.5 s à 0.002 s ; l'écart au point d'impact décroît
d'ordre ≈ 1 et passe sous 3 cm à `dt = 0.01 s`. Détails, tableau et protocole :
[docs/convergence.md](docs/convergence.md).

```bash
python tools/convergence.py
```

Le script localise seul le binaire (`build/m31`, `build/Release/m31.exe`, …) et
`config.txt` à la racine du dépôt ; il peut être lancé depuis n'importe quel
répertoire. `--exe` permet de forcer un chemin.

---

## Configuration

`config.txt` : une paire `clé valeur` par ligne, `#` pour les commentaires.

| Clé | Unité | Description |
|---|---|---|
| `missile_x/y/z` | m | Position initiale |
| `target_x/y/z` | m | Position de la cible |
| `dt` | s | Pas de temps |
| `T` | s | Durée maximale simulée |
| `apex` | m | Altitude max de la trajectoire de référence |
| `mass` | kg | Masse initiale totale (structure + carburant) |
| `fuel` | kg | Masse de carburant (doit être < `mass`) |
| `thrust` | N | Poussée nominale |
| `burn_rate` | kg/s | Débit de carburant |
| `max_angle_deg` | ° | Borne d'amplitude de la correction de guidage |
| `drag_cd` | – | Coefficient de traînée (constant) |
| `drag_area` | m² | Surface de référence |
| `seed` | – | Graine du vent aléatoire |
| `output_file` | – | Chemin du CSV de sortie |

## Sortie CSV

| Colonne | Description |
|---|---|
| `t` | Temps (s) |
| `x, y, z` | Position simulée (m) |
| `speed` | Norme de la vitesse (m/s) |
| `x_theory, y_theory, z_theory` | Position de référence (m) |
| `a_thrust_x/y/z` | Accélération de poussée (m/s²) |
| `rho` | Densité de l'air (kg/m³) |
| `drag_x/y/z` | Accélération de traînée (m/s²) |
| `kinetic_energy` | Énergie cinétique (J) |
| `pos_error` | Distance à la trajectoire de référence (m) |
| `correction_x/y/z` | Correction de guidage avant saturation |
| `mach` | Nombre de Mach (journalisé uniquement, non utilisé dans la physique) |

Visualisation (nécessite `pandas` et `matplotlib`) :

```bash
cd tools && python3 plot.py
```

## Structure

```
.
├── src/
│   ├── main.cpp        # Point d'entrée
│   ├── config.h/.cpp   # SimulationConfig, chargement et validation
│   ├── simulation.h/.cpp
│   ├── entities.h      # Missile, ArrivalPoint
│   └── Vec3.h
├── tests/tests.cpp     # Tests unitaires (CTest)
├── tools/
│   ├── plot.py         # Visualisation matplotlib
│   └── convergence.py  # Étude de convergence sur dt
├── docs/convergence.md
├── data/               # Sorties CSV (non versionnées)
├── .github/workflows/ci.yml
├── CMakeLists.txt
└── config.txt
```

## Exemple de sortie console

```
Thit = 106.8544 s

=== Rapport de simulation ===
Durée totale               : 106.85 s
Position finale missile    : (55002.11958, 15000.63506, -1.20038423)
Écart cible – missile      : (2.119584, 0.6350601, -1.20038423)
Distance scalaire à la cible : 2.517307 m
Extinction moteur          : t = 28.1 s
Position à l'extinction    : (14460.83, 3944.457, 10853.79)
Carburant consommé         : 154.55 kg
```

Le « carburant consommé » vaut `burn_rate × t_extinction` et peut légèrement
dépasser la masse de carburant initiale à cause de la discrétisation du dernier pas.
