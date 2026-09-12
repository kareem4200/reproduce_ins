#include <iostream>
#include <vector>

int main() {
    // std::cout << "Hello, World!" << std::endl;
    constexpr int kImuRateHz   = 100;
    constexpr int kSimSeconds  = 50;
    constexpr int kNumSamples  = kSimSeconds * kImuRateHz + 1;  // exact integer arithmetic
    constexpr double kDt       = 1.0 / kImuRateHz;

    int gps_freq = 1;
    int doppler_freq = 10;
    float a_true = 0.5;
    float p0_true = 0.0;
    float v0_true = 0.0;

    std::vector<float> position_true(kNumSamples);
    std::vector<float> velocity_true(kNumSamples);
    std::vector<float> acceleration_true(kNumSamples);

    for (int i=0; i < kNumSamples; i++) {
        float t = i * kDt;
        position_true.at(i) = p0_true + v0_true * t + 0.5 * a_true * t * t;
        velocity_true.at(i) = v0_true + a_true * t;
        acceleration_true.at(i) = a_true;
    }

    std::cout << "last position_true: " << position_true.back() << std::endl;
    std::cout << "last velocity_true: " << velocity_true.back() << std::endl;
    std::cout << "last acceleration_true: " << acceleration_true.back() << std::endl;

    return 0;
}