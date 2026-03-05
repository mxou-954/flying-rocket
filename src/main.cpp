#include <iostream>
#include <fstream>
#include <cmath>

#include "Vec3.h"
#include "simulation.h"
#include "entities.h"

using namespace std;

int main() {
    ifstream config("../config.txt");
    string line;

    double Thit = 0;
    double T = 0;
    double dt = 0;
    double apex = 0;
    string output_file = "";

    Missile m;
    ArrivalPoint target;

    if(config.is_open()){
            string key;
            string strValue;
            while(config >> key >> strValue) {
                if(key == "output_file") { output_file = strValue; continue; }
                double value = stod(strValue);
                if(key == "missile_x") m.pos.x = value;
                if(key == "missile_y") m.pos.y = value;
                if(key == "missile_z") m.pos.z = value;
                if(key == "target_x") target.pos.x = value;
                if(key == "target_y") target.pos.y = value;
                if(key == "target_z") target.pos.z = value;
                if(key == "Thit") Thit = value;
                if(key == "dt") dt = value;
                if(key == "T") T = value;
                if(key == "mass") m.mass = value;
                if(key == "drag") m.drag = value;
                if(key == "apex") apex = value;
                if(key == "fuel") m.fuel = value;
                if(key == "thrust") m.thrust = value;
            }
        config.close();
    } else {
        cout << "Erreur : Impossible de lire le fichier." << endl;
    }

    runSimulation(
        m, 
        target, 
        dt, 
        T, 
        Thit, 
        output_file, 
        apex
    );
}