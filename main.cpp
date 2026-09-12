#include <iostream>
#include <vector>
#include <random>
#include <iomanip>

int main() {
    // set precision for floating point output
    std::cout << std::fixed << std::setprecision(6);
    
    constexpr int kIMUFrequency = 100;
    constexpr int kGPSFrequency = 1;
    constexpr int kDopplerFrequency = 10;
    constexpr int kSimSeconds  = 50;
    constexpr int kNumSamples  = kSimSeconds * kIMUFrequency + 1;  // exact integer arithmetic
    constexpr double kDt = 1.0 / kIMUFrequency;
    constexpr int kBeaconPosition = 1000;
    
    constexpr double kAccelInit = 0.5;
    constexpr double kPositionInit = 0.0;
    constexpr double kVelocityInit = 0.0;

    constexpr unsigned int kSeed = 42;

    constexpr double kAccelBiasSigma = 0.05;
    constexpr double kAccelNoiseSigma = 0.02;
    constexpr double kGPSPositionSigma = 2.0;
    constexpr double kGPSVelocitySigma = 0.15;
    constexpr double kDopplerRangeSigma = 0.5;
    constexpr double kDopplerRRSigma = 0.05;

    struct TrueTrajectory {
        double position;
        double velocity;
        double acceleration;
    };

    std::vector<TrueTrajectory> true_trajectory(kNumSamples);

    // std::random_device rd;
    std::mt19937_64 gen(kSeed);

    // noise for monte carlo simulation of IMU measurements
    std::normal_distribution<double> accel_bias(0.0, kAccelBiasSigma);

    // accelerometer sample-to-sample noise
    std::normal_distribution<double> accel_noise(0.0, kAccelNoiseSigma);

    // GPS sample-to-sample noise
    std::normal_distribution<double> gps_position_noise(0.0, kGPSPositionSigma);
    std::normal_distribution<double> gps_velocity_noise(0.0, kGPSVelocitySigma);

    // beacon doppler sample-to-sample noise
    std::normal_distribution<double> doppler_range_noise(0.0, kDopplerRangeSigma);
    std::normal_distribution<double> doppler_rr_noise(0.0, kDopplerRRSigma);

    for (int i=0; i < kNumSamples; i++) {
        double t = i * kDt;
        true_trajectory.at(i).position = kPositionInit + kVelocityInit * t + 0.5 * kAccelInit * t * t;
        true_trajectory.at(i).velocity = kVelocityInit + kAccelInit * t;
        true_trajectory.at(i).acceleration = kAccelInit;
    }

    std::cout << "last position_true: " << true_trajectory.back().position << '\n';
    std::cout << "last velocity_true: " << true_trajectory.back().velocity << '\n';
    std::cout << "last acceleration_true: " << true_trajectory.back().acceleration << '\n';

    return 0;
}