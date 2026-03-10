// ============================================================
//  Roquette M31 – LRU (Lance-Roquettes Unitaire)
//  Simulation de trajectoire balistique guidée
// ============================================================

#include "simulation.h"



double speedOfSound(double altitude) {
    // Modèle atmosphère standard ISA
    // T(z) = 288.15 - 0.0065 * z  (valable jusqu'à 11 000 m)
    double T = std::max(288.15 - 0.0065 * altitude, 216.65); // plancher tropopause
    return std::sqrt(1.4 * 287.05 * T); // γ * R * T
}

double convertSpeedInMach(double speed, double altitude) {
    return speed / speedOfSound(altitude);
}

// ------------------------------------------------------------
//  Modèle atmosphérique
// ------------------------------------------------------------

/// Densité de l'air selon un modèle exponentiel isotherme.
/// @param altitude  Altitude en mètres (z)
/// @return          Densité en kg/m³
double airDensity(double altitude)
{
    const double rho0 = 1.225;   // densité au niveau de la mer (kg/m³)
    const double H    = 8500.0;  // hauteur d'échelle atmosphérique (m)
    return rho0 * std::exp(-altitude / H);
}

// ------------------------------------------------------------
//  Génération du vent aléatoire
// ------------------------------------------------------------

/// Tire un vecteur vent horizontal aléatoire (0–15 m/s, direction quelconque).
/// @return  Vec3 vent en m/s (composante z nulle)
static Vec3 randomWind()
{
    double speed = ((double)rand() / RAND_MAX) * 15.0;          // m/s
    double angle = ((double)rand() / RAND_MAX) * 2.0 * M_PI;   // radians
    return Vec3(speed * std::cos(angle),
                speed * std::sin(angle),
                0.0);
}

// ------------------------------------------------------------
//  Calcul du temps de vol théorique (trajectoire parabolique)
// ------------------------------------------------------------

/// Calcule le temps d'impact pour atteindre l'apex donné sous gravité seule.
/// La parabole symétrique vérifie z(Thit/2) = apex et z(0) = z(Thit) = 0.
/// @param apex  Altitude maximale souhaitée (m)
/// @param g     Accélération gravitationnelle (m/s²)
/// @return      Temps de vol total (s)
static double computeFlightTime(double apex, double g)
{
    return 2.0 * std::sqrt(2.0 * apex / g);
}

// ------------------------------------------------------------
//  Trajectoire théorique de référence
// ------------------------------------------------------------

/// Position théorique à l'instant t sur une trajectoire parabolique
/// interpolant linéairement en (x,y) et paraboliquement en z.
/// @param start   Position de départ
/// @param target  Position cible
/// @param apex    Altitude maximale (m)
/// @param t       Temps courant (s)
/// @param Thit    Temps de vol total (s)
static Vec3 theoreticalPosition(const Vec3& start, const Vec3& target,
                                double apex, double t, double Thit)
{
    double ratio = std::min(t / Thit, 1.0);
    double z     = start.z + apex * 4.0 * (t / Thit) * (1.0 - t / Thit);
    return Vec3(
        start.x + (target.x - start.x) * ratio,
        start.y + (target.y - start.y) * ratio,
        z
    );
}

/// Vitesse théorique à l'instant t (dérivée analytique de theoreticalPosition).
static Vec3 theoreticalVelocity(const Vec3& start, const Vec3& target,
                                double apex, double t, double Thit)
{
    return Vec3(
        (target.x - start.x) / Thit,
        (target.y - start.y) / Thit,
        apex * 4.0 / Thit * (1.0 - 2.0 * t / Thit)
    );
}

// ------------------------------------------------------------
//  Correction de guidage (contrôleur PD adaptatif)
// ------------------------------------------------------------

/// Applique une correction de vitesse par contrôleur Proportionnel–Dérivé.
/// Les gains sont augmentés dans la phase terminale (< 20 s avant impact)
/// pour améliorer la précision au point d'impact.
///
/// La correction est bornée par l'angle de gouverne maximal du missile
/// afin de rester physiquement réaliste.
///
/// @param m          Missile (m.vel modifié en place)
/// @param pos_error  Écart position théorique – position réelle (m)
/// @param vel_error  Écart vitesse théorique – vitesse réelle (m/s)
/// @param time_to_impact  Temps restant avant impact (s)
/// @param dt         Pas de temps (s)
/// @return           Vecteur correction appliqué (pour journalisation)
static Vec3 applyGuidance(Missile& m,
                           const Vec3& pos_error,
                           const Vec3& vel_error,
                           double time_to_impact,
                           double dt)
{
    // Gains PD – augmentés en phase terminale
    double kp = 1.0, kd = 1.2;
    if (time_to_impact < 30.0) {
        kp = 6.0;
        kd = 4.0;
    }

    Vec3   correction     = pos_error * kp + vel_error * kd;
    double correction_len = length(correction);

    if (correction_len > 0.0) {
        Vec3   dir            = correction * (1.0 / correction_len);
        double max_angle_rad  = m.max_angle_deg * M_PI / 180.0;
        double max_correction = length(m.vel) * std::tan(max_angle_rad);
        double magnitude      = std::min(correction_len, max_correction);
        m.vel += dir * magnitude * dt;
    }

    return correction;
}

// ------------------------------------------------------------
//  Boucle principale de simulation
// ------------------------------------------------------------

void runSimulation(
    Missile&           m,
    ArrivalPoint&      target,
    double             dt,
    double             T,
    double             Thit,      // recalculé en interne
    const std::string& outputPath,
    double             apex
)
{
    // -- Constantes physiques --
    const double g       = 9.81;
    const Vec3   gravity = Vec3(0.0, 0.0, -g);

    // -- Temps de vol théorique (trajectoire parabolique symétrique) --
    Thit = computeFlightTime(apex, g);
    std::cout << "Thit = " << Thit << " s\n";

    // -- Conditions initiales --
    const Vec3 start_pos = m.pos;
    m.engineOn = true;

    // Vitesse initiale alignée sur la trajectoire théorique
    m.vel = theoreticalVelocity(start_pos, target.pos, apex, 0.0, Thit);

    // -- Vent (tiré une seule fois pour toute la simulation) --
    const Vec3 wind = randomWind();

    // -- Journalisation --
    std::ofstream out(outputPath);
    out << "t,x,y,z,speed,"
        << "x_theory,y_theory,z_theory,"
        << "a_thrust_x,a_thrust_y,a_thrust_z,"
        << "rho,drag_x,drag_y,drag_z,"
        << "kinetic_energy,pos_error,"
        << "correction_x,correction_y,correction_z,"
        << "mach,";

    // -- Variables d'état moteur (pour le rapport final) --
    double engineSetOff       = 0.0;
    Vec3   positionEngineOff  = Vec3(0.0, 0.0, 0.0);

    // --------------------------------------------------------
    //  Boucle temporelle (intégration Euler symplectique)
    // --------------------------------------------------------
    double t = 0.0;
    for (; t <= T; t += dt)
    {
        // ---- 1. Gestion moteur ----
        if (m.fuel <= 0.5 && m.engineOn) {
            m.engineOn       = false;
            engineSetOff     = t;
            positionEngineOff = m.pos;
        }

        // ---- 2. Référence théorique ----
        Vec3 pos_theory = theoreticalPosition(start_pos, target.pos, apex, t, Thit);
        Vec3 vel_theory = theoreticalVelocity (start_pos, target.pos, apex, t, Thit);

        // ---- 3. Poussée moteur ----
        //  La poussée est modulée proportionnellement à l'erreur d'altitude,
        //  saturée entre 0 et 1 pour éviter une sur-correction.
        Vec3 a_thrust = Vec3(0.0, 0.0, 0.0);
        if (m.engineOn) {
            double z_error      = pos_theory.z - m.pos.z;
            double thrust_factor = std::min(std::max(z_error / 500.0, 0.0), 1.0);
            a_thrust = Vec3(0.0, 0.0, m.thrust * thrust_factor / m.mass);

            // Consommation de carburant (indépendante de thrust_factor)
            m.fuel -= m.burn_rate * dt;
            m.mass -= m.burn_rate * dt;
        }

        // ---- 4. Traînée aérodynamique ----
        //  Modèle : F_drag = -½ · ρ · Cd · A · v_rel² · v̂_rel
        //  En accélération : a_drag = F_drag / m
        double rho           = airDensity(m.pos.z);
        Vec3   vel_rel       = m.vel - wind;
        double speed_rel     = length(vel_rel);
        Vec3   drag          = (speed_rel > 0.0)
            ? vel_rel * (-0.5 * rho * m.cd * m.area * speed_rel / m.mass)
            : Vec3(0.0, 0.0, 0.0);
        //  Note : le facteur speed_rel est déjà dans vel_rel (‖vel_rel‖ · v̂),
        //  donc on multiplie seulement par speed_rel (et non speed_rel²).

        // ---- 5. Accélération totale ----
        Vec3 a = gravity + drag + a_thrust;

        // ---- 6. Guidage (correction de trajectoire) ----
        Vec3 pos_error = pos_theory - m.pos;
        Vec3 vel_error = vel_theory - m.vel;
        double time_to_impact = Thit - t;

        Vec3 correction = applyGuidance(m, pos_error, vel_error, time_to_impact, dt);

        // ---- 7. Journalisation de l'état courant ----
        double speed          = length(m.vel);
        double kinetic_energy = 0.5 * m.mass * speed * speed;
        double pos_err        = length(pos_error);

        double mach = convertSpeedInMach(speed, m.pos.z);

        out << t
            << "," << m.pos.x    << "," << m.pos.y    << "," << m.pos.z
            << "," << speed
            << "," << pos_theory.x << "," << pos_theory.y << "," << pos_theory.z
            << "," << a_thrust.x << "," << a_thrust.y << "," << a_thrust.z
            << "," << rho
            << "," << drag.x     << "," << drag.y     << "," << drag.z
            << "," << kinetic_energy
            << "," << pos_err
            << "," << correction.x << "," << correction.y << "," << correction.z
            << "," << mach
            << "\n";

        // ---- 8. Intégration Euler symplectique ----
        //  v(t+dt) = v(t) + a(t)·dt   → mise à jour vitesse en premier
        //  x(t+dt) = x(t) + v(t+dt)·dt → puis position avec la nouvelle vitesse
        m.vel += a       * dt;
        m.pos += m.vel   * dt;

        // ---- 9. Condition d'arrêt (impact sol) ----
        if (m.pos.z <= 0.0)
            break;
    }

    // --------------------------------------------------------
    //  Rapport final
    // --------------------------------------------------------
    Vec3 delta = m.pos - target.pos;

    std::cout << "\n=== Rapport de simulation ===\n";
    std::cout << "Durée totale               : " << t              << " s\n";
    std::cout << "Position finale missile    : ("
              << m.pos.x << ", " << m.pos.y << ", " << m.pos.z << ")\n";
    std::cout << "Écart cible – missile      : ("
              << delta.x << ", " << delta.y << ", " << delta.z << ")\n";
    std::cout << "Distance scalaire à la cible : " << length(delta) << " m\n";
    std::cout << "Extinction moteur          : t = " << engineSetOff << " s\n";
    std::cout << "Position à l'extinction    : ("
              << positionEngineOff.x << ", "
              << positionEngineOff.y << ", "
              << positionEngineOff.z << ")\n";
    std::cout << "Carburant consommé         : "
              << (m.burn_rate * engineSetOff) << " kg\n";
}