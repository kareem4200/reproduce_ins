#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

struct State
{
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

struct GPSMeasurement
{
    bool valid = false
    double position = 0.0;
    double velocity = 0.0;
};

struct DopplerMeasurement
{
    bool valid = false;
    double range = 0.0;
    double range_rate = 0.0;
};

std::vector<State> generate_true_trajectory(int, double, double, double, double);
std::vector<double> generate_imu_measurements(const std::vector<State>&, std::mt19937_64&,
                                              std::normal_distribution<double>&, double);
std::vector<GPSMeasurement> generate_gps_measurements(const std::vector<State>&, std::mt19937_64&,
                                                      std::normal_distribution<double>&,
                                                      std::normal_distribution<double>&, int);
std::vector<DopplerMeasurement> generate_doppler_measurements(const std::vector<State>&,
                                                              std::mt19937_64&,
                                                              std::normal_distribution<double>&,
                                                              std::normal_distribution<double>&,
                                                              double, int);
void write_measurements_csv(const std::vector<State>&, const std::vector<State>&,
                            const std::vector<GPSMeasurement>&,
                            const std::vector<DopplerMeasurement>&, double);

int main()
{
    // set precision for floating point output
    std::cout << std::fixed << std::setprecision(6);

    constexpr int kIMUFrequency = 100;
    constexpr int kGPSFrequency = 1;
    constexpr int kDopplerFrequency = 10;
    constexpr int kSimSeconds = 50;
    constexpr int kNumSamples = kSimSeconds * kIMUFrequency + 1; // exact integer arithmetic
    constexpr double kDt = 1.0 / kIMUFrequency;
    constexpr double kBeaconPosition = 1000.0;

    constexpr double kAccelInit = 0.5;
    constexpr double kPositionInit = 0.0;
    constexpr double kVelocityInit = 0.0;

    constexpr unsigned int kSeed = 42;

    constexpr double kAccelBias = 0.05;
    constexpr double kAccelNoiseSigma = 0.02;
    constexpr double kGPSPositionSigma = 2.0;
    constexpr double kGPSVelocitySigma = 0.15;
    constexpr double kDopplerRangeSigma = 0.5;
    constexpr double kDopplerRRSigma = 0.05;

    static_assert(kIMUFrequency % kGPSFrequency == 0,
                  "IMU frequency must be a multiple of GPS frequency");
    static_assert(kIMUFrequency % kDopplerFrequency == 0,
                  "IMU frequency must be a multiple of Doppler frequency");

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

    auto true_trajectory =
        generate_true_trajectory(kNumSamples, kDt, kAccelInit, kPositionInit, kVelocityInit);

    std::cout << "last position_true: " << true_trajectory.back().position << '\n';
    std::cout << "last velocity_true: " << true_trajectory.back().velocity << '\n';
    std::cout << "last acceleration_true: " << true_trajectory.back().acceleration << '\n';

    auto imu_measurements =
        generate_imu_measurements(true_trajectory, gen, accel_noise, kAccelBias);

    std::vector<State> estimated_trajectory(kNumSamples);
    estimated_trajectory.at(0).position = kPositionInit;
    estimated_trajectory.at(0).velocity = kVelocityInit;
    estimated_trajectory.at(0).acceleration = kAccelInit;

    for (size_t i = 0; i < imu_measurements.size() - 1; ++i)
    {
        // std::cout << "imu[" << i << "] = " << imu_measurements[i] << '\n';
        estimated_trajectory.at(i).acceleration = imu_measurements.at(i);
        estimated_trajectory.at(i + 1).velocity =
            estimated_trajectory.at(i).velocity + estimated_trajectory.at(i).acceleration * kDt;
        estimated_trajectory.at(i + 1).position =
            estimated_trajectory.at(i).position + estimated_trajectory.at(i).velocity * kDt +
            0.5 * estimated_trajectory.at(i).acceleration * kDt * kDt;
    }

    std::cout << "estimated velocity: " << estimated_trajectory.back().velocity << '\n';
    std::cout << "estimated position: " << estimated_trajectory.back().position << '\n';

    std::cout << "last position_error: "
              << std::abs(true_trajectory.back().position - estimated_trajectory.back().position)
              << '\n';

    auto gps_measurements = generate_gps_measurements(true_trajectory, gen, gps_position_noise,
                                                      gps_velocity_noise, kGPSStep);

    auto doppler_measurements = generate_doppler_measurements(
        true_trajectory, gen, doppler_range_noise, doppler_rr_noise, kBeaconPosition, kDopplerStep);

    write_measurements_csv(true_trajectory, estimated_trajectory, gps_measurements,
                           doppler_measurements, kDt);

    return 0;
}

std::vector<State> generate_true_trajectory(int num_samples, double dt, double accel_init,
                                            double position_init, double velocity_init)
{
    std::vector<State> trajectory(num_samples);

    for (int i = 0; i < num_samples; ++i)
    {
        double t = i * dt;
        trajectory.at(i).position = position_init + velocity_init * t + 0.5 * accel_init * t * t;
        trajectory.at(i).velocity = velocity_init + accel_init * t;
        trajectory.at(i).acceleration = accel_init;
    }

    return trajectory;
}

std::vector<double> generate_imu_measurements(const std::vector<State>& true_trajectory,
                                              std::mt19937_64& gen,
                                              std::normal_distribution<double>& accel_noise_dist,
                                              double accel_bias)
{

    std::vector<double> imu_measurements(true_trajectory.size());

    for (size_t i = 0; i < true_trajectory.size(); ++i)
    {
        const double noise = accel_noise_dist(gen);
        imu_measurements.at(i) = true_trajectory.at(i).acceleration + accel_bias + noise;
    }

    return imu_measurements;
}

std::vector<GPSMeasurement>
generate_gps_measurements(const std::vector<State>& true_trajectory, std::mt19937_64& gen,
                          std::normal_distribution<double>& position_noise_dist,
                          std::normal_distribution<double>& velocity_noise_dist, int gps_step)
{
    std::vector<GPSMeasurement> gps_measurements(true_trajectory.size());

    for (size_t i = 0; i < true_trajectory.size(); ++i)
    {
        if (i % gps_step == 0)
        {
            const double position_noise = position_noise_dist(gen);
            const double velocity_noise = velocity_noise_dist(gen);
            gps_measurements.at(i).valid = true;
            gps_measurements.at(i).position = true_trajectory.at(i).position + position_noise;
            gps_measurements.at(i).velocity = true_trajectory.at(i).velocity + velocity_noise;
        }
        else
        {
            gps_measurements.at(i).valid = false;
            gps_measurements.at(i).position = std::numeric_limits<double>::quiet_NaN();
            gps_measurements.at(i).velocity = std::numeric_limits<double>::quiet_NaN();
        }
    }

    return gps_measurements;
}

std::vector<DopplerMeasurement>
generate_doppler_measurements(const std::vector<State>& true_trajectory, std::mt19937_64& gen,
                              std::normal_distribution<double>& range_noise_dist,
                              std::normal_distribution<double>& rr_noise_dist,
                              double beacon_position, int doppler_step)
{
    std::vector<DopplerMeasurement> doppler_measurement(true_trajectory.size());

    for (size_t i = 0; i < true_trajectory.size(); ++i)
    {
        if (i % doppler_step == 0)
        {
            // in C++, it costs nothing to allocate stack in the loop (unless it is an object with
            // expensive constructor)
            const double range_noise = range_noise_dist(gen);
            const double rr_noise = rr_noise_dist(gen);
            const double delta_position = true_trajectory.at(i).position - beacon_position;
            doppler_measurement.at(i).valid = true;
            doppler_measurement.at(i).range = std::abs(delta_position) + range_noise;
            const double sign =
                (delta_position > 1e-12) ? 1.0 : -1.0; // positive is moving away from beacon
            doppler_measurement.at(i).range_rate = sign * true_trajectory.at(i).velocity + rr_noise;
        }
        else
        {
            doppler_measurement.at(i).valid = false;
            doppler_measurement.at(i).range = std::numeric_limits<double>::quiet_NaN();
            doppler_measurement.at(i).range_rate = std::numeric_limits<double>::quiet_NaN();
        }
    }

    return doppler_measurement;
}

void write_measurements_csv(const std::vector<State>& true_trajectory,
                            const std::vector<State>& estimated_trajectory,
                            const std::vector<GPSMeasurement>& gps_measurements,
                            const std::vector<DopplerMeasurement>& doppler_measurements, double dt)
{
    std::ofstream file("../measurements.csv");

    file << "time,true_pos,true_vel,true_acc,est_pos,est_vel,est_acc,gps_valid,gps_pos,gps_vel,"
            "doppler_valid,doppler_range,doppler_rr\n";

    for (size_t i = 0; i < true_trajectory.size(); ++i)
    {
        const double time = static_cast<double>(i) * dt;

        file << time << "," << true_trajectory.at(i).position << ","
             << true_trajectory.at(i).velocity << "," << true_trajectory.at(i).acceleration << ","
             << estimated_trajectory.at(i).position << "," << estimated_trajectory.at(i).velocity
             << "," << estimated_trajectory.at(i).acceleration << ","
             << gps_measurements.at(i).valid << "," << gps_measurements.at(i).position << ","
             << gps_measurements.at(i).velocity << "," << doppler_measurements.at(i).valid << ","
             << doppler_measurements.at(i).range << "," << doppler_measurements.at(i).range_rate
             << "\n";
    }

    file.close();
}