// Tests unitaires minimalistes (sans dépendance externe).
// Lancés par CTest : ctest --test-dir build

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "Vec3.h"
#include "config.h"
#include "entities.h"
#include "simulation.h"

static int g_failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::cerr << "FAIL " << __FILE__ << ":" << __LINE__            \
                      << "  " << #cond << "\n";                            \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)

static bool nearly(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ---------------------------------------------------------------- 1. Vec3
static void test_vec3()
{
    Vec3 a(3, 4, 0), b(1, 1, 1);
    CHECK(nearly(length(a), 5.0, 1e-12));
    CHECK(nearly((a + b).x, 4.0, 1e-12));
    CHECK(nearly((a - b).y, 3.0, 1e-12));
    CHECK(nearly((a * 2.0).x, 6.0, 1e-12));
    a += b;
    CHECK(nearly(a.z, 1.0, 1e-12));
}

// ------------------------------------------------------- 2. Atmosphère
static void test_air_density()
{
    CHECK(nearly(airDensity(0.0), 1.225, 1e-12));
    CHECK(airDensity(5000.0) < airDensity(0.0));      // décroissance monotone
    CHECK(airDensity(20000.0) < airDensity(5000.0));
    CHECK(airDensity(100000.0) > 0.0);                // jamais négative
}

// ------------------------------------------------------ 3. Vitesse du son
static void test_speed_of_sound()
{
    CHECK(nearly(speedOfSound(0.0), 340.3, 0.5));     // ~340 m/s au niveau de la mer
    CHECK(speedOfSound(10000.0) < speedOfSound(0.0)); // plus froid => plus lent
    // Plancher tropopause : constante au-delà de 11 km
    CHECK(nearly(speedOfSound(15000.0), speedOfSound(20000.0), 1e-9));
    CHECK(nearly(convertSpeedInMach(speedOfSound(0.0), 0.0), 1.0, 1e-9));
}

// --------------------------------------------------- 4. Temps de vol
static void test_flight_time()
{
    // z(t) = v0 t - g t²/2, apex = v0²/(2g)  =>  Thit = 2 sqrt(2 apex / g)
    CHECK(nearly(computeFlightTime(1000.0, 9.81), 2.0 * std::sqrt(2000.0 / 9.81), 1e-9));
    CHECK(computeFlightTime(14000.0, 9.81) > computeFlightTime(1000.0, 9.81));
}

// ------------------------------------------- 5. Validation des paramètres
static void test_validate_config()
{
    SimulationConfig ok;
    CHECK(validateConfig(ok).empty());

    SimulationConfig c = ok; c.dt = 0.0;
    CHECK(!validateConfig(c).empty());

    c = ok; c.dt = -1.0;         CHECK(!validateConfig(c).empty());
    c = ok; c.T = 0.0;           CHECK(!validateConfig(c).empty());
    c = ok; c.mass = 0.0;        CHECK(!validateConfig(c).empty());
    c = ok; c.fuel = c.mass;     CHECK(!validateConfig(c).empty());
    c = ok; c.apex = -10.0;      CHECK(!validateConfig(c).empty());
    c = ok; c.max_angle_deg = 90.0; CHECK(!validateConfig(c).empty());
    c = ok; c.drag_area = 0.0;   CHECK(!validateConfig(c).empty());
}

// ------------------------------------------- 6. Chargement de config
static void test_load_config()
{
    SimulationConfig c;
    std::string err;

    // Fichier inexistant : échec propre, pas de crash
    CHECK(!loadConfig("fichier_qui_n_existe_pas.txt", c, err));
    CHECK(!err.empty());

    // Fichier valide
    {
        std::ofstream f("tmp_ok.txt");
        f << "# commentaire\n\ndt 0.05\nT 120\napex 5000\nseed 7\n"
             "output_file tmp_out.csv\n";
    }
    SimulationConfig good;
    CHECK(loadConfig("tmp_ok.txt", good, err));
    CHECK(err.empty());
    CHECK(nearly(good.dt, 0.05, 1e-12));
    CHECK(nearly(good.T, 120.0, 1e-12));
    CHECK(good.seed == 7u);
    CHECK(good.output_file == "tmp_out.csv");

    // Valeur non numérique
    { std::ofstream f("tmp_bad.txt"); f << "dt abc\n"; }
    SimulationConfig bad;
    CHECK(!loadConfig("tmp_bad.txt", bad, err));

    // Clé inconnue
    { std::ofstream f("tmp_key.txt"); f << "vitesse_lumiere 3e8\n"; }
    CHECK(!loadConfig("tmp_key.txt", bad, err));

    // Paramètre hors domaine détecté au chargement
    { std::ofstream f("tmp_neg.txt"); f << "dt -0.1\n"; }
    CHECK(!loadConfig("tmp_neg.txt", bad, err));

    std::remove("tmp_ok.txt");
    std::remove("tmp_bad.txt");
    std::remove("tmp_key.txt");
    std::remove("tmp_neg.txt");
}

// --------------------------------- 7. Reproductibilité (même graine)
static void test_reproducibility()
{
    SimulationConfig cfg;
    cfg.dt = 0.05;
    cfg.output_file = "tmp_repro.csv";

    auto run = [](SimulationConfig c) {
        Missile m = makeMissile(c);
        ArrivalPoint t; t.pos = c.target_pos;
        CHECK(runSimulation(m, t, c, /*verbose=*/false));
        return m.pos;
    };

    Vec3 a = run(cfg);
    Vec3 b = run(cfg);                       // même graine
    CHECK(nearly(a.x, b.x, 1e-12));
    CHECK(nearly(a.y, b.y, 1e-12));
    CHECK(nearly(a.z, b.z, 1e-12));

    cfg.seed = 1234;                          // graine différente
    Vec3 c = run(cfg);
    CHECK(length(c - a) > 1e-9);

    std::remove("tmp_repro.csv");
}

// ------------------------ 8. Simulation nominale : impact plausible
static void test_simulation_reaches_ground()
{
    SimulationConfig cfg;
    cfg.dt = 0.05;
    cfg.output_file = "tmp_run.csv";

    Missile m = makeMissile(cfg);
    ArrivalPoint t; t.pos = cfg.target_pos;
    CHECK(runSimulation(m, t, cfg, /*verbose=*/false));

    CHECK(m.pos.z <= 0.0);                 // la simulation se termine au sol
    CHECK(m.mass > 0.0);                   // masse jamais négative
    CHECK(m.fuel <= cfg.fuel);             // le carburant ne remonte pas
    CHECK(length(m.pos - t.pos) < 5000.0); // le guidage amène près de la cible

    // Le CSV produit contient un en-tête et des données
    std::ifstream f("tmp_run.csv");
    CHECK(f.is_open());
    std::string header, first;
    std::getline(f, header);
    std::getline(f, first);
    CHECK(header.rfind("t,x,y,z,speed", 0) == 0);
    CHECK(!first.empty());
    f.close();

    std::remove("tmp_run.csv");
}

// ---------------------- 9. Sortie non ouvrable : échec propre
static void test_bad_output_path()
{
    SimulationConfig cfg;
    cfg.dt = 0.5;
    cfg.output_file = "repertoire_inexistant/sous/traj.csv";

    Missile m = makeMissile(cfg);
    ArrivalPoint t; t.pos = cfg.target_pos;
    CHECK(!runSimulation(m, t, cfg, /*verbose=*/false));
}

int main()
{
    test_vec3();
    test_air_density();
    test_speed_of_sound();
    test_flight_time();
    test_validate_config();
    test_load_config();
    test_reproducibility();
    test_simulation_reaches_ground();
    test_bad_output_path();

    if (g_failures == 0) {
        std::cout << "Tous les tests sont passes.\n";
        return 0;
    }
    std::cerr << g_failures << " assertion(s) en echec.\n";
    return 1;
}
