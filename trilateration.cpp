#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// Constants
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

vector<double> UWB_measured = {1.28, 1.4, 1.55};
vector<double> ACC_measured = {0, 0, -9.8066};
vector<double> error = {0.08, 0.01};
vector<vector<double>> calib_U, calib_A;
vector<double> offset_acc = {0, 0, 0};
vector<double> offset_uwb = {0, 0, 0};

double x1 = 0, y1 = 1, z1 = 0;
double x2 = 0, y2 = 0, z2 = 0;
double x3 = 1, y3 = 0, z3 = 0;

double Fs = 400 / 14.586690;
double Fc = 0.5;
int filterOrder = 10;
int bufferSize = filterOrder + 1;

vector<double> FIR_coeff(bufferSize, 0);
vector<vector<double>> buffer_accel(3, vector<double>(bufferSize, 0));
vector<vector<double>> buffer_uwb(3, vector<double>(bufferSize, 0));

// Function to generate FIR coefficients
void generate_FIR_coeff() {
    for (int i = 0; i <= filterOrder; ++i) {
        double hamming = 0.54 - 0.46 * cos(2 * M_PI * i / filterOrder);
        FIR_coeff[i] = hamming * (sin(2 * M_PI * Fc / (Fs / 2) * (i - filterOrder / 2)) / (i - filterOrder / 2));
        if (i == filterOrder / 2) FIR_coeff[i] = hamming * (2 * Fc / (Fs / 2));
    }
}

// Function to sample UWB and ACC data
pair<vector<double>, vector<double>> sampler(vector<double>& UWB, vector<double>& ACC, vector<double>& error) {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dist;

    vector<double> ind_u, ind_a;

    for (double u : UWB) {
        dist = uniform_real_distribution<>(u - error[0], u + error[0]);
        ind_u.push_back(round(dist(gen) * 1000) / 1000); // Rounded to 3 decimals
    }

    for (double a : ACC) {
        dist = uniform_real_distribution<>(a - error[1], a + error[1]);
        ind_a.push_back(round(dist(gen) * 1000) / 1000); // Rounded to 3 decimals
    }

    return {ind_u, ind_a};
}

// Function to convert raw accelerometer data to 'g' values
double convert_raw_to_g(double raw_value) {
    double sensitivity = 0.00976;
    if (raw_value >= 32768)
        raw_value -= 65536;
    return raw_value * sensitivity;
}

// Trilateration function
void trilateration(double s1, double s2, double s3) {
    MatrixXd A(3, 4);
    A << 1, -2 * x1, -2 * y1, -2 * z1,
         1, -2 * x2, -2 * y2, -2 * z2,
         1, -2 * x3, -2 * y3, -2 * z3;

    VectorXd B(3);
    B << pow(s1, 2) - pow(x1, 2) - pow(y1, 2) - pow(z1, 2),
         pow(s2, 2) - pow(x2, 2) - pow(y2, 2) - pow(z2, 2),
         pow(s3, 2) - pow(x3, 2) - pow(y3, 2) - pow(z3, 2);

    // Solve for xp
    VectorXd xp = A.colPivHouseholderQr().solve(B);

    // Homogeneous solution xh
    VectorXd xh(4);
    xh << 0, 0, 0, 1;

    double C = xp(1) * xp(1) + xp(2) * xp(2) + xp(3) * xp(3);
    double D = 2 * (xp(1) * xh(1) + xp(2) * xh(2) + xp(3) * xh(3));
    double E = xh(1) * xh(1) + xh(2) * xh(2) + xh(3) * xh(3);

    // Quadratic coefficients
    double a = E, b = D - xh(0), c = C - xp(0);

    // Solve quadratic equation
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        cout << "No real solutions for t." << endl;
        return;
    }

    double t1 = (-b + sqrt(discriminant)) / (2 * a);
    double t2 = (-b - sqrt(discriminant)) / (2 * a);

    // Compute the solutions
    VectorXd X1 = xp + t1 * xh;

    cout << "X: " << X1(1) << " Y: " << X1(2) << " Z: " << X1(3) << endl;
}

int main() {
    generate_FIR_coeff();

    vector<double> fir_accel(3, 0);
    vector<double> fir_uwb(3, 0);

    for (int i = 0; i < 200; ++i) {
        auto [raw_U, raw_A] = sampler(UWB_measured, ACC_measured, error);

        double a_x = convert_raw_to_g(raw_A[0]);
        double a_y = convert_raw_to_g(raw_A[1]);
        double a_z = convert_raw_to_g(raw_A[2]);

        if (calib_A.size() < 100) {
            calib_A.push_back(raw_A);
        } else {
            a_x -= offset_acc[0];
            a_y -= offset_acc[1];
            a_z -= offset_acc[2];

            for (int j = 0; j < bufferSize - 1; ++j) {
                buffer_accel[0][j] = buffer_accel[0][j + 1];
                buffer_accel[1][j] = buffer_accel[1][j + 1];
                buffer_accel[2][j] = buffer_accel[2][j + 1];
            }
            buffer_accel[0][bufferSize - 1] = a_x;
            buffer_accel[1][bufferSize - 1] = a_y;
            buffer_accel[2][bufferSize - 1] = a_z;

            for (int j = 0; j < 3; ++j) {
                fir_accel[j] = inner_product(buffer_accel[j].begin(), buffer_accel[j].end(), FIR_coeff.begin(), 0.0);
            }
        }

        if (calib_U.size() < 50) {
            calib_U.push_back(raw_U);
        } else {
            for (int j = 0; j < 3; ++j) {
                raw_U[j] -= offset_uwb[j];
                for (int k = 0; k < bufferSize - 1; ++k)
                    buffer_uwb[j][k] = buffer_uwb[j][k + 1];
                buffer_uwb[j][bufferSize - 1] = raw_U[j];
                fir_uwb[j] = inner_product(buffer_uwb[j].begin(), buffer_uwb[j].end(), FIR_coeff.begin(), 0.0);
            }
        }

        if (calib_A.size() == 100 && calib_U.size() == 50) {
            for (int j = 0; j < 3; ++j) {
                offset_acc[j] = accumulate(calib_A.begin(), calib_A.end(), 0.0, [j](double acc, const vector<double>& val) {
                                   return acc + val[j];
                               }) /
                                calib_A.size() -
                                ACC_measured[j];
                offset_uwb[j] = accumulate(calib_U.begin(), calib_U.end(), 0.0, [j](double acc, const vector<double>& val) {
                                   return acc + val[j];
                               }) /
                                calib_U.size() -
                                UWB_measured[j];
            }
            break;
        }

        // Apply trilateration using FIR-filtered UWB values
        double s1 = fir_uwb[0];
        double s2 = fir_uwb[1];
        double s3 = fir_uwb[2];
        cout << s1 << s2 <<s3 << endl;

        trilateration(1.35088,1.47198,1.65333);
    }

    return 0;
}
