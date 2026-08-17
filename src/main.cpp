#include <iostream>
#include <string>

#include "config.h"
#include "simulation.h"
#include "entities.h"

int main(int argc, char** argv)
{
    // Chemin de config : argv[1] si fourni, sinon config.txt du répertoire courant.
    const std::string configPath = (argc > 1) ? argv[1] : "config.txt";

    SimulationConfig cfg;
    std::string err;
    if (!loadConfig(configPath, cfg, err)) {
        std::cerr << "Erreur de configuration : " << err << "\n";
        return 1;
    }

    Missile m = makeMissile(cfg);
    ArrivalPoint target;
    target.pos = cfg.target_pos;

    if (!runSimulation(m, target, cfg)) {
        return 1;
    }
    return 0;
}
