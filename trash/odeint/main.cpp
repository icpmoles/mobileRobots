#include <vector>
#include <fstream>
#include <iostream>

#include "first_order_tf.h"
#include "PID.h"

int main() {
    double Ts = 0.01;

    // Create plant and controller
    first_order_tf plant(1.0, 10.0, 2.0, Ts);
    PID controller(1.0, 10.0, Ts);

    // Initialize controller and plant
    plant.initialize();
    controller.initialize();

    // Variables to store simulation data
    int numStep = 10000;
    std::vector<double> t_vect(numStep+1, 0.0);
    std::vector<double> y_vect(numStep+1, 0.0);
    std::vector<double> ysp_vect(numStep+1, 0.0);
    std::vector<double> u_vect(numStep+1, 0.0);

    // Compute a step response
    double y = 0.0;
    double ysp = 5.0;
    for (int k=0; k<=numStep; k++) {
        // Update the controller
        controller.setMeasurement(y);
        controller.setReference(ysp);

        // Execute the controller
        controller.execute();
        double u = controller.getControl();

        // Update the plant
        plant.setInput(u);

        // Simulate the plant
        plant.simulate();
        y = plant.getOutput();

        // Store variables for plotting
        t_vect.at(k) = k*Ts;
        y_vect.at(k) = y;
        ysp_vect.at(k) = ysp;
        u_vect.at(k) = u;
    }

    // Create an output csv file
    std::ofstream resultFile("simulation_results.csv");

    // Write results to file
    resultFile << "t, ysp, y, u\n";
    for(int k=0; k<=numStep; k++)
    {
        resultFile << t_vect.at(k) << ", " << ysp_vect.at(k) << ", " << y_vect.at(k) << ", " << u_vect.at(k) << "\n";
    }

    // Close the file
    resultFile.close();

    return 0;
}
