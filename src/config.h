#ifndef _config_h
#define _config_h

#include <string>
#include "Vec3.h"
#include "entities.h"

/// Paramètres d'une simulation, chargés depuis un fichier clé/valeur.
/// Les valeurs par défaut correspondent à config.txt.
struct SimulationConfig {
    Vec3   missile_pos    = Vec3(1, 1, 1);
    Vec3   target_pos     = Vec3(55000, 15000, 0);

    double dt             = 0.01;    // pas de temps (s)
    double T              = 400.0;   // durée max simulée (s)
    double apex           = 14000.0; // altitude max de la trajectoire de référence (m)

    double mass           = 302.0;   // masse totale au départ (kg)
    double fuel           = 155.0;   // carburant (kg)
    double thrust         = 45000.0; // poussée (N)
    double burn_rate      = 5.5;     // consommation (kg/s)
    double max_angle_deg  = 5.0;     // angle de gouverne max (deg)
    double drag_cd        = 0.3;     // coefficient de traînée (-)
    double drag_area      = 0.0404;  // surface de référence (m²)

    unsigned int seed     = 42;      // graine du vent aléatoire (reproductibilité)
    std::string output_file = "data/traj.csv";
};

/// Vérifie la cohérence physique des paramètres.
/// @return  message d'erreur, ou chaîne vide si la configuration est valide.
std::string validateConfig(const SimulationConfig& c);

/// Charge une configuration depuis un fichier clé/valeur.
/// @return  false si le fichier est illisible, mal formé ou invalide (err renseigné).
bool loadConfig(const std::string& path, SimulationConfig& c, std::string& err);

/// Construit le missile initial à partir de la configuration.
Missile makeMissile(const SimulationConfig& c);

#endif
