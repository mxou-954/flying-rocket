#include "config.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

std::string validateConfig(const SimulationConfig& c)
{
    if (c.dt <= 0.0)              return "dt doit etre > 0";
    if (c.T <= 0.0)               return "T doit etre > 0";
    if (c.dt > c.T)               return "dt doit etre <= T";
    if (c.apex <= 0.0)            return "apex doit etre > 0";
    if (c.mass <= 0.0)            return "mass doit etre > 0";
    if (c.fuel < 0.0)             return "fuel doit etre >= 0";
    if (c.fuel >= c.mass)         return "fuel doit etre < mass (mass = structure + carburant)";
    if (c.thrust < 0.0)           return "thrust doit etre >= 0";
    if (c.burn_rate <= 0.0)       return "burn_rate doit etre > 0";
    if (c.max_angle_deg <= 0.0 || c.max_angle_deg >= 90.0)
                                  return "max_angle_deg doit etre dans ]0, 90[";
    if (c.drag_cd < 0.0)          return "drag_cd doit etre >= 0";
    if (c.drag_area <= 0.0)       return "drag_area doit etre > 0";
    if (c.output_file.empty())    return "output_file ne doit pas etre vide";
    return "";
}

bool loadConfig(const std::string& path, SimulationConfig& c, std::string& err)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        err = "impossible d'ouvrir le fichier de configuration : " + path;
        return false;
    }

    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;

        // Commentaires (#) et lignes vides
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);

        std::istringstream ls(line);
        std::string key, raw;
        if (!(ls >> key)) continue;            // ligne vide
        if (!(ls >> raw)) {
            err = "ligne " + std::to_string(lineNo) + " : valeur manquante pour '" + key + "'";
            return false;
        }

        if (key == "output_file") { c.output_file = raw; continue; }

        double v = 0.0;
        try {
            std::size_t used = 0;
            v = std::stod(raw, &used);
            if (used != raw.size()) throw std::invalid_argument("trailing");
        } catch (const std::exception&) {
            err = "ligne " + std::to_string(lineNo) + " : valeur numerique invalide '"
                + raw + "' pour '" + key + "'";
            return false;
        }

        if      (key == "missile_x")      c.missile_pos.x  = v;
        else if (key == "missile_y")      c.missile_pos.y  = v;
        else if (key == "missile_z")      c.missile_pos.z  = v;
        else if (key == "target_x")       c.target_pos.x   = v;
        else if (key == "target_y")       c.target_pos.y   = v;
        else if (key == "target_z")       c.target_pos.z   = v;
        else if (key == "dt")             c.dt             = v;
        else if (key == "T")              c.T              = v;
        else if (key == "apex")           c.apex           = v;
        else if (key == "mass")           c.mass           = v;
        else if (key == "fuel")           c.fuel           = v;
        else if (key == "thrust")         c.thrust         = v;
        else if (key == "burn_rate")      c.burn_rate      = v;
        else if (key == "max_angle_deg")  c.max_angle_deg  = v;
        else if (key == "drag_cd")        c.drag_cd        = v;
        else if (key == "drag_area")      c.drag_area      = v;
        else if (key == "seed")           c.seed           = static_cast<unsigned int>(v);
        else {
            err = "ligne " + std::to_string(lineNo) + " : cle inconnue '" + key + "'";
            return false;
        }
    }

    err = validateConfig(c);
    return err.empty();
}

Missile makeMissile(const SimulationConfig& c)
{
    Missile m;
    m.pos           = c.missile_pos;
    m.mass          = c.mass;
    m.fuel          = c.fuel;
    m.thrust        = c.thrust;
    m.burn_rate     = c.burn_rate;
    m.max_angle_deg = c.max_angle_deg;
    m.cd            = c.drag_cd;
    m.area          = c.drag_area;
    return m;
}
