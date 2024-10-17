// cyber-physical-systems.cpp : Defines the entry point for the application.
//

#include "cyber-physical-systems.h"
using namespace std;

const int m = 1475;                             // mass in kg
const int k = 61;                               // friction 
const double dt = 0.10;                         // time step in seconds
const double g = 9.80665;                       // gravity in m/s2

void simulate_car(double F, double x0, double v0, double theta, double t_max, vector<double>& t, vector<double>& x, vector<double>& v) {
    int n = static_cast <int> (t_max / dt);                           // calculate number of steps

    // Initialize vectors time, position and velocity to be of size n
    t.resize(n);
    x.resize(n);
    v.resize(n);

    // Initial conditions
    t[0] = 0.0;
    x[0] = x0;
    v[0] = v0;

    // Calculate force due to road grade
    double disturbance = m * g * sin(theta * M_PI / 180);   // in Case 1 & 2, disturbance will evaluate to 0

    // Simulating steps using Trapezoidal approximation

    for (int i = 1; i < n; ++i) {
        t[i] = i * dt;

        // acceleration at previous step
        double a_prev = (F - k * v[i - 1] - disturbance) / m;

        // Predict velocity and position
        double v_predict = v[i - 1] + a_prev * dt;
        double x_predict = x[i - 1] + v[i - 1] * dt;

        // Acceleration at predicted time 
        double a_predict = (F - k * v_predict) / m;

        // Correct velocity and position
        v[i] = v[i - 1] + 0.5 * (a_prev + a_predict) * dt;
        x[i] = x[i - 1] + 0.5 * (v[i - 1] + v_predict) * dt;
    }



}
// Function to print simulation results
void print_results(const vector<double>& t, const vector<double>& x, const vector<double>& v, const string& case_description) {
    cout << "\n" << case_description << "\n";
    cout << setw(10) << "Time(s)" << setw(15) << "Position(m)" << setw(15) << "Velocity(m/s)" << "\n";

    for (size_t i = 0; i < t.size(); ++i) {
        cout << setw(10) << t[i] << setw(15) << x[i] << setw(15) << v[i] << "\n";
    }
}

void save_results_to_csv(const vector<double>& t, const vector<double>& x, const vector<double>& v, const string& filename) {
    ofstream file(filename);
    if (file.is_open()) {
        file << "Time(s),Position(m),Velocity(m/s)\n";
        for (size_t i = 0; i < t.size(); ++i) {
            file << t[i] << "," << x[i] << "," << v[i] << "\n";
        }
        file.close();
        cout << "Results saved to " << filename << "\n";
    }
    else {
        cerr << "Unable to open file " << filename << "\n";
    }
}


int main() {

    cout << "Current working directory: " << filesystem::current_path() << "\n";
    exit;
    // Case 1: F = 0, x(0) = 0, v(0) = 15 m/s
    vector<double> t1, x1, v1;
    string def1 = "Case 1: F = 0, x(0) = 0, v(0) = 15 m/s";
    simulate_car(0, 0.0, 15, 0, 60.0, t1, x1, v1);
    // print_results(t1, x1, v1, def1);
    save_results_to_csv(t1, x1, v1, "case1.csv");


    // Case 2: F = 575 N, x(0) = 0, v(0) = 0
    vector<double> t2, x2, v2;
    string def2 = "Case 2: F = 575 N, x(0) = 0, v(0) = 0";
    simulate_car(575, 0.0, 0.0, 0, 60.0, t2, x2, v2);
    //print_results(t2, x2, v2, def2);
    save_results_to_csv(t2, x2, v2, "case2.csv");

    // Case 3: F = 575 N, x(0) = 0, v(0) = 0, theta = 4.5 degrees
    vector<double> t3, x3, v3;
    string def3 = "Case 3: F = 575 N, x(0) = 0, v(0) = 0, theta = 4.5 degrees";
    simulate_car(575, 0.0, 0.0, 4.5, 60.0, t3, x3, v3);
    print_results(t3, x3, v3, def3);
    save_results_to_csv(t3, x3, v3, "case3.csv");

    // Case 4: F = 575 N, x(0) = 0, v(0) = 0, theta = 9 degrees
    vector<double> t4, x4, v4;
    string def4 = "Case 4: F = 575 N, x(0) = 0, v(0) = 0, theta = 9 degrees";
    simulate_car(575.0, 0.0, 0.0, 9.0, 60.0, t4, x4, v4);
    print_results(t4, x4, v4, def4);
    save_results_to_csv(t4, x4, v4, "case4.csv");

    return 0;
}



