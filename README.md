# Roquette M31 – LRU (Lance-Roquettes Unitaire)
### Simulation de trajectoire balistique guidée en C++

---

## Présentation

Simulation physique complète de la trajectoire d'une roquette M31 tirée depuis un LRU.  
Le modèle intègre la poussée moteur, la traînée aérodynamique, la densité atmosphérique, le vent aléatoire et un système de guidage par contrôleur **PD adaptatif** pour corriger la trajectoire en vol vers une cible donnée.

---

## Fonctionnement

La simulation calcule à chaque pas de temps :

1. **Trajectoire théorique de référence** — parabole symétrique interpolant départ et cible avec une altitude maximale configurable (`apex`)
2. **Poussée moteur** — modulée proportionnellement à l'erreur d'altitude, jusqu'à épuisement du carburant
3. **Traînée aérodynamique** — modèle `½ · ρ · Cd · A · v²` avec densité atmosphérique variable (modèle ISA exponentiel)
4. **Vent aléatoire** — vecteur horizontal tiré une seule fois (0–15 m/s, direction quelconque)
5. **Guidage PD adaptatif** — corrige l'écart position/vitesse entre trajectoire réelle et référence ; gains renforcés dans la phase terminale (< 30 s avant impact)
6. **Intégration Euler symplectique** — vitesse mise à jour avant la position pour une meilleure stabilité énergétique

Un rapport final est affiché en console à l'impact (distance à la cible, position d'extinction moteur, carburant consommé).

---

## Stack technique

- **C++17**
- **CMake** (build system)
- Pas de dépendance externe — stdlib uniquement

---

## Structure du projet

```
.
├── src/
│   ├── main.cpp          # Point d'entrée, lecture config, lancement simulation
│   ├── simulation.cpp    # Boucle de simulation, guidage, physique
│   └── simulation.h      # Déclarations
├── data/                 # Fichiers de sortie CSV (trajectoires)
├── tools/                # Scripts d'analyse / visualisation
├── CMakeLists.txt
└── config.txt            # Paramètres de simulation (cible, apex, dt…)
```

---

## Installation & compilation

### Prérequis

- CMake ≥ 3.15
- Compilateur C++17 (GCC, Clang ou MSVC)

### Build

```bash
git clone https://github.com/mxou-954/M31-simulation.git
cd M31-simulation
mkdir build && cd build
cmake ..
cmake --build .
```

---

## Utilisation

Modifier `config.txt` pour définir les paramètres de tir :

```
# Exemple de configuration
target_x = 15000
target_y = 0
target_z = 0
apex     = 4000
dt       = 0.1
T        = 200
```

Puis lancer la simulation :

```bash
./main
```

Les résultats sont exportés dans `data/` au format CSV avec les colonnes :

| Colonne | Description |
|---|---|
| `t` | Temps (s) |
| `x, y, z` | Position réelle (m) |
| `speed` | Vitesse scalaire (m/s) |
| `mach` | Nombre de Mach |
| `x_theory, y_theory, z_theory` | Position théorique (m) |
| `pos_error` | Écart à la trajectoire théorique (m) |
| `rho` | Densité de l'air (kg/m³) |
| `drag_x/y/z` | Accélération de traînée (m/s²) |
| `kinetic_energy` | Énergie cinétique (J) |
| `correction_x/y/z` | Correction de guidage appliquée |

---

## Modèles physiques

### Atmosphère
Densité selon un modèle exponentiel isotherme :
```
ρ(z) = 1.225 · exp(−z / 8500)
```
Vitesse du son selon le modèle ISA standard (valable jusqu'à la tropopause) :
```
T(z) = max(288.15 − 0.0065·z, 216.65)  [K]
c(z) = √(γ · R · T)
```

### Guidage PD
```
correction = Kp · pos_error + Kd · vel_error
```
- Phase de croisière : Kp = 1.0, Kd = 1.2  
- Phase terminale (< 30 s) : Kp = 6.0, Kd = 4.0  
- Correction bornée par l'angle de gouverne maximal du missile

---

## Rapport de simulation (exemple console)

```
Thit = 57.3 s

=== Rapport de simulation ===
Durée totale               : 57.2 s
Position finale missile    : (14997.3, 1.2, -0.4)
Écart cible – missile      : (-2.7, 1.2, -0.4)
Distance scalaire à la cible : 3.1 m
Extinction moteur          : t = 12.4 s
Position à l'extinction    : (2845.1, 0.3, 1923.7)
Carburant consommé         : 24.8 kg
```
