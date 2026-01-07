#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "DynamicsEquations.h"
#include "Integrator.h"
#include "RigidBodyState.h"

using namespace LinAlg;

#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>

using namespace std;

bool load_config(const string& filename, RigidBodyState& state, VehicleConfig& config, double& dt, double& t_max) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open config file: " << filename << "\n";
        return false;
    }
    string line;
    while(getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        istringstream iss(line);
        string key;
        char equals;
        double value;
        if (iss >> key >> equals >> value && equals == '=') {
            if (key == "initial_position_x") state.position[0] = value;
            else if (key == "initial_position_y") state.position[1] = value;
            else if (key == "initial_position_z") state.position[2] = value;
            else if (key == "initial_velocity_x") state.velocity_body[0] = value;
            else if (key == "initial_velocity_y") state.velocity_body[1] = value;
            else if (key == "initial_velocity_z") state.velocity_body[2] = value;
            else if (key == "initial_pitch_deg") {
                double rad = value * M_PI/180.0;
                state.orientation = Quaternion<double>::fromAxisAngle(
                    Vector<double>({0.0, 1.0, 0.0}), rad);
            } else if (key == "initial_angvel_p") state.angular[0] = value;
            else if (key == "initial_angvel_q") state.angular[1] = value;
            else if (key == "initial_angvel_r") state.angular[2] = value;
            else if (key == "reference_area") config.reference_area = value;
            else if (key == "mass") config.mass = value;
            else if (key == "Ixx") config.inertia_tensor(0, 0) = value;
            else if (key == "Iyy") config.inertia_tensor(1, 1) = value;
            else if (key == "Izz") config.inertia_tensor(2, 2) = value;
            else if (key == "Ixz") config.inertia_tensor(0, 2) = value;
            else if (key == "thrust_magnitude") config.thrust_magnitude = value;
            else if (key == "thrust_dir_x") config.thrust_direction[0] = value;
            else if (key == "thrust_dir_y") config.thrust_direction[1] = value;
            else if (key == "thrust_dir_z") config.thrust_direction[2] = value;
            else if (key == "CL_alpha") config.CL_alpha = value;
            else if (key == "CD_alpha") config.CD_alpha = value;
            else if (key == "CL_zero") config.CL_zero = value;
            else if (key == "CD_zero") config.CD_zero = value;
            else if (key == "CY_beta") config.CY_beta = value;
            else if (key == "CM_alpha") config.CM_alpha = value;
            else if (key == "CM_zero") config.CM_zero = value;
            else if (key == "c_bar") config.c_bar = value;
            else if (key == "CL_beta") config.CL_beta = value;
            else if (key == "CN_beta") config.CN_beta = value;
            else if (key == "C_lp") config.C_lp = value;
            else if (key == "C_nr") config.C_nr = value;
            else if (key == "C_mq") config.C_mq = value;
            else if (key == "aspect_ratio") config.aspect_ratio = value;
            else if (key == "oswald_eff") config.oswald_eff = value;
            else if (key == "time_step") dt = value;
            else if (key == "max_time") t_max = value;
        }
    }
    return true;
}

int main(int argc, char* argv[]) {
    try {
        RigidBodyState state;
        state.position = Vector<double>(3);  // Starting 1 km up
        state.velocity_body = Vector<double>(3);  // Starting 100 m/s forward
        state.angular = Vector<double>(3);

        VehicleConfig config;

        double dt = 0.01;
        double t_max = 30.0;
        double t = 0.0;
        double s_area = 10.0;

        // Load config file
        filesystem::path exe_dir = filesystem::current_path();
        filesystem::path config_path = exe_dir.parent_path().parent_path() / "config.txt";
        string config_file = config_path.string();
        if (!load_config(config_file, state, config, dt, t_max)) {
            std::cerr << "Using default values\n";
            state.position = Vector<double>({0.0, 0.0, 1000.0});
            state.velocity_body = Vector<double>({100.0, 0.0, 0.0});
            state.angular = Vector<double>({0.0, 0.0, 0.0});
            state.orientation = Quaternion<double>(1.0, 0.0, 0.0, 0.0);
        }

        std::cout << "Loaded configuration from: " << config_file << "\n";

        cout << "Starting 6-DOF simulation...\n\n";

        while (t < t_max) {
            step_rk4(state, dt, t, config);
            t += dt;

            // Log every 0.5 seconds
            if (std::fmod(t, 0.5) < dt) {
                cout << "Time = " << t << " seconds\n" 
                << "Position: " << state.position << "\n" 
                << "Velocity (body): " << state.velocity_body << "\n"
                << "Attitude q: " << state.orientation << "\n"
                << "Euler angles: " << state.orientation.toEulerXYZ() << "\n\n";
            }

            // Stop if hits the ground
            if (state.position[2] <= 0.0) {
                cout << "\n GROUND IMPACT at t = " << t << " s\n";
                cout << "Final position: " << state.position << "\n";
                cout << "Impact velocity: " << state.velocity_body << "\n";
                break;
            }
        }

    } catch (const std::exception& e) {
        cerr << "\n ERROR: " << e.what() << endl;
        return 1;
    }

    return 0;
}