# Étude de convergence sur `dt`

## Protocole

Même configuration (`config.txt`, `seed 42`, `apex 14000`, cible à 55 000 / 15 000 m),
seul `dt` varie. Le point d'impact est obtenu en interpolant linéairement les deux
derniers pas jusqu'à `z = 0` — sinon on mesurerait le dépassement discret du dernier
pas sous le sol plutôt que l'erreur d'intégration.

La solution à `dt = 0.002 s` sert de référence ; l'écart est la distance horizontale
entre le point d'impact obtenu et cette référence.

Reproduction :

```bash
cmake -S . -B build && cmake --build build
python tools/convergence.py
```

## Résultats

| dt (s) | t_impact (s) | x_impact (m) | y_impact (m) | écart / réf (m) | ordre observé |
|--------|--------------|--------------|--------------|-----------------|---------------|
| 0.5    | 106.8475     | 54996.545    | 14999.098    | 4.587           | –             |
| 0.2    | 106.8567     | 55000.396    | 15000.165    | 0.592           | 2.24          |
| 0.1    | 106.8572     | 55000.693    | 15000.246    | 0.284           | 1.06          |
| 0.05   | 106.8575     | 55000.813    | 15000.279    | 0.159           | 0.84          |
| 0.02   | 106.8577     | 55000.912    | 15000.306    | 0.057           | 1.12          |
| 0.01   | 106.8577     | 55000.941    | 15000.314    | 0.026           | 1.13          |
| 0.005  | 106.8577     | 55000.957    | 15000.318    | 0.010           | 1.41          |
| 0.002  | 106.8578     | 55000.966    | 15000.321    | référence       | –             |

## Lecture

- La solution converge : l'écart décroît de façon monotone quand `dt` diminue.
- L'ordre observé est proche de **1** (ordre attendu pour l'intégrateur d'Euler
  semi-implicite utilisé ici). Il n'est pas exactement 1 car le correcteur PD
  applique une correction proportionnelle à `dt` : le système simulé dépend
  légèrement de `dt` lui-même, il ne s'agit donc pas d'une convergence vers une
  unique solution continue bien définie.
- L'écart chute sous **0.3 m** dès `dt = 0.1 s` et sous **0.06 m** à `dt = 0.02 s`.
  Le temps d'impact est stable à 10⁻³ s près à partir de `dt = 0.1 s`.

## Conclusion pratique

`dt = 0.01 s` (valeur par défaut) est largement suffisant : diviser encore le pas
par 5 ne déplace le point d'impact que de ~2 cm sur 57 km de portée. `dt = 0.1 s`
reste acceptable pour des essais rapides. Au-delà de `dt = 0.2 s`, l'erreur devient
visible (plusieurs mètres) et le comportement du correcteur commence à se dégrader.
