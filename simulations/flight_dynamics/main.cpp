#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "DynamicsEquations.h"
#include "Integrator.h"
#include "RigidBodyState.h"

using namespace LinAlg;

int main() {
    try {
        RigidBodyState state;
        state.position = Vector<double>({0.0, 0.0, 1000.0});  // Starting 1 km up
        state.velocity_body = Vector<double>({100.0, 0.0, 0.0});  // Starting 100 m/s forward
        state.orientation = Quaternion<double>(1.0, 0.0, 0.0, 0.0);  // Starting with identity orientation
        state.angular = Vector<double>({0.0, 0.0, 0.0});

        double dt = 0.01;
        double t = 0.00;
        double t_max = 30.0;

        std::cout << "Starting 6-DOF simulation...\n\n";

        while (t < t_max) {
            step(state, dt);
            t += dt;

            // Log every 0.5 seconds
            if (std::fmod(t, 0.5) < dt) {
                std::cout << "Time = " << t << " seconds\n" 
                << "Position: " << state.position << "\n" 
                << "Velocity (body): " << state.velocity_body << "\n"
                << "Attitude q: " << state.orientation << "\n"
                << "Euler angles: " << state.orientation.toEulerXYZ() << "\n\n";
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "\n❌ ERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}