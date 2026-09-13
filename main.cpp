#include <iostream>
#include <vector>
#include <random>
#include <iomanip>

struct State {
    double position;
    double velocity;
    double acceleration;
};

struct GPSMeasurement {
    bool valid;
    double position;
    double velocity;
};

std::vector<State> generate_true_trajectory(int, double, double, double, double);
std::vector<double> generate_imu_measurements(const std::vector<State>&, std::mt19937_64&, std::normal_distribution<double>&, double);

int main() {
    // set precision for floating point output
    std::cout << std::fixed << std::setprecision(6);
    
    constexpr int kIMUFrequency = 100;
    constexpr int kGPSFrequency = 1;
    constexpr int kDopplerFrequency = 10;
    constexpr int kSimSeconds  = 50;
    constexpr int kNumSamples  = kSimSeconds * kIMUFrequency + 1;  // exact integer arithmetic
    constexpr double kDt = 1.0 / kIMUFrequency;
    constexpr double kBeaconPosition = 1000.0;
    
    constexpr double kAccelInit = 0.5;
    constexpr double kPositionInit = 0.0;
    constexpr double kVelocityInit = 0.0;

    constexpr unsigned int kSeed = 42;

    constexpr double kAccelBias = 0.05;
    constexpr double kAccelNoiseSigma = 0.0;
    constexpr double kGPSPositionSigma = 2.0;
    constexpr double kGPSVelocitySigma = 0.15;
    constexpr double kDopplerRangeSigma = 0.5;
    constexpr double kDopplerRRSigma = 0.05;

    static_assert(kIMUFrequency % kGPSFrequency == 0, "IMU frequency must be a multiple of GPS frequency");
    static_assert(kIMUFrequency % kDopplerFrequency == 0, "IMU frequency must be a multiple of Doppler frequency");

    constexpr int kGPSStep = kIMUFrequency / kGPSFrequency;
    constexpr int kDopplerStep = kIMUFrequency / kDopplerFrequency;

    // std::random_device rd;
    std::mt19937_64 gen(kSeed);

    // accelerometer sample-to-sample noise
    std::normal_distribution<double> accel_noise(0.0, kAccelNoiseSigma);

    // GPS sample-to-sample noise
    std::normal_distribution<double> gps_position_noise(0.0, kGPSPositionSigma);
    std::normal_distribution<double> gps_velocity_noise(0.0, kGPSVelocitySigma);

    // beacon doppler sample-to-sample noise
    std::normal_distribution<double> doppler_range_noise(0.0, kDopplerRangeSigma);
    std::normal_distribution<double> doppler_rr_noise(0.0, kDopplerRRSigma);

    auto true_trajectory = generate_true_trajectory(kNumSamples, kDt, kAccelInit, kPositionInit, kVelocityInit);

    std::cout << "last position_true: " << true_trajectory.back().position << '\n';
    std::cout << "last velocity_true: " << true_trajectory.back().velocity << '\n';
    std::cout << "last acceleration_true: " << true_trajectory.back().acceleration << '\n';

    auto imu_measurements = generate_imu_measurements(true_trajectory, gen, accel_noise, kAccelBias);

    std::vector<State> estimated_trajectory(kNumSamples);
    estimated_trajectory.at(0).position = kPositionInit;
    estimated_trajectory.at(0).velocity = kVelocityInit;
    // estimated_trajectory.at(0).acceleration = kAccelInit;

    for (size_t i = 0; i < imu_measurements.size() - 1; ++i) {
        // std::cout << "imu[" << i << "] = " << imu_measurements[i] << '\n';
        estimated_trajectory.at(i).acceleration = imu_measurements.at(i);
        estimated_trajectory.at(i + 1).velocity = estimated_trajectory.at(i).velocity + estimated_trajectory.at(i).acceleration * kDt;
        estimated_trajectory.at(i + 1).position = estimated_trajectory.at(i).position + estimated_trajectory.at(i).velocity * kDt + 0.5 * estimated_trajectory.at(i).acceleration * kDt * kDt;
    }
    std::cout << "estimated velocity: " << estimated_trajectory.back().velocity << '\n';
    std::cout << "estimated position: " << estimated_trajectory.back().position << '\n';

    std::cout << "last position_error: " << std::abs(true_trajectory.back().position - estimated_trajectory.back().position) << '\n';

    return 0;
}

std::vector<State> generate_true_trajectory(int num_samples, double dt, double accel_init, double position_init, double velocity_init) {
    std::vector<State> trajectory(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        double t = i * dt;
        trajectory.at(i).position = position_init + velocity_init * t + 0.5 * accel_init * t * t;
        trajectory.at(i).velocity = velocity_init + accel_init * t;
        trajectory.at(i).acceleration = accel_init;
    }

    return trajectory;
}

std::vector<double> generate_imu_measurements(const std::vector<State>& true_trajectory, std::mt19937_64& gen, std::normal_distribution<double>& accel_noise, double accel_bias) {
    
    std::vector<double> imu_measurements(true_trajectory.size());

    for (size_t i = 0; i < true_trajectory.size(); ++i) {
        double noise = accel_noise(gen);
        imu_measurements[i] = true_trajectory.at(i).acceleration + accel_bias + noise;
    }

    return imu_measurements;
}

