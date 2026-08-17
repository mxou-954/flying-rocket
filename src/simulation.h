#ifndef _simulation_h
#define _simulation_h

#include "Vec3.h"
#include "entities.h"
#include "config.h"

#include <string>

// --- Modèles physiques élémentaires (exposés pour les tests) ---

/// Vitesse du son (m/s) selon le modèle ISA troposphérique.
double speedOfSound(double altitude);

/// Nombre de Mach pour une vitesse donnée à une altitude donnée.
double convertSpeedInMach(double speed, double altitude);

/// Densité de l'air (kg/m³), modèle exponentiel isotherme.
double airDensity(double altitude);

/// Temps de vol d'une parabole symétrique atteignant `apex` sous gravité seule (s).
double computeFlightTime(double apex, double g);

/// Lance la simulation et écrit la trajectoire au format CSV.
/// @return  false si le fichier de sortie n'a pas pu être ouvert.
bool runSimulation(Missile& m, ArrivalPoint& target, const SimulationConfig& cfg,
                   bool verbose = true);

#endif
