#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>

using namespace std;

// Runge Kutta 4 Integrator
double rk4(function<double(double)> dxdt, double x, double dt) {
    double k1 = dt * dxdt(x);
    double k2 = dt * dxdt(x + k1 / 2.0);
    double k3 = dt * dxdt(x + k2 / 2.0);
    double k4 = dt * dxdt(x + k3);
    return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
}

double Dynamics(double q,double u)
{
    //Define constants
    const double Mq = -0.61;
    const double Md = -6.65;
    const double Theta = -0.01;

    //Known Regressor
    double Phi = tanh((360/M_PI) * q);
    double f = Theta*Phi;

    //Pitch Rate
    double dqdt = (Mq*q) + Md*(u + f);
    return dqdt;
}

vector<double> AdaptiveLaws(double q,double q_ref, double q_cmd)
{
    //Define Constants
    const double gamma_q = 6000;
    const double gamma_cmd = 6000;
    const double Gamma_theta = 8;

    double Phi = tanh(360/M_PI * q);
    double kq_dot = gamma_q * q * (q - q_ref);
    double kcmd_dot = gamma_cmd * q_cmd * (q - q_ref);
    double theta_dot = -Gamma_theta * Phi * (q - q_ref);

    return {kq_dot, kcmd_dot, theta_dot};
}

double Controller(double k_q,double k_q_cmd,double theta,double q_cmd,double q)
{
    double Phi = tanh((360/M_PI)*q);
    double u = k_q * q + k_q_cmd * q_cmd - theta*Phi;
    return u;
}
double ReferenceModel(double q_ref,double q_cmd)
{
    double q_ref_dot = 4 * (q_cmd - q_ref);
    return q_ref_dot;
}

int main() {
    // Open a file to save the data
    ofstream outFile("q_output.txt");
    if (!outFile) {
        cerr << "Error opening file!" << endl;
        return -1;
    }
    // Write header
    outFile << "q_cpp = " << endl;

    // Initial conditions
    double q = 0.01;
    double q_ref = 0.01;

    // Adaptive parameters
    double k_q = 0.01;
    double k_q_cmd = 0.01;
    double theta = 0.01;

    //Simulation Time
    double t0 = 0.0;
    double t_end = 30;
    double dt = 0.01;

    // Sine wave parameters for q_cmd
    const double amplitude = 1.0/M_PI;

    // Simulation loop
    for (double t = t0; t <= t_end; t += dt) {
        // Command signal: sine wave
        double q_cmd = amplitude * sin(t);

        // Compute control input
        double u = Controller(k_q, k_q_cmd, theta, q_cmd, q);

        // Integrate system dynamics
        q = rk4([&](double q) { return Dynamics(q, u); }, q, dt);

        // Integrate reference model
        q_ref = rk4([&](double q_ref) { return ReferenceModel(q_ref, q_cmd); }, q_ref, dt);

        // Integrate adaptive laws
        vector<double> adaptiveRates = AdaptiveLaws(q, q_ref, q_cmd);
        k_q = rk4([&](double k_q) { return adaptiveRates[0]; }, k_q, dt);
        k_q_cmd = rk4([&](double k_q_cmd) { return adaptiveRates[1]; }, k_q_cmd, dt);
        theta = rk4([&](double theta) { return adaptiveRates[2]; }, theta, dt);

        // Tracking error
        double e = q - q_ref;
        cout << "Tracking error: " << e << endl;

        // Save q
        outFile << q << " , " << endl;
    }

    // Close the file
    outFile.close();

    return 0;
}
