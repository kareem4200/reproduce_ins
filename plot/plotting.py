from matplotlib import pyplot as plt
import pandas as pd

measurements = pd.read_csv("../measurements.csv")
gps_valid_measurements = measurements[measurements['gps_valid'] == 1]
doppler_valid_measurements = measurements[measurements['doppler_valid'] == 1]

plt.figure(figsize=(10, 6))
plt.plot(measurements['time'], measurements['true_pos'], label='True Position')
plt.plot(measurements['time'], measurements['est_pos'], label='Estimated Position')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()

plt.figure(figsize=(10, 6))
plt.plot(measurements['time'], measurements['true_pos'], label='True Position')
plt.scatter(gps_valid_measurements['time'], gps_valid_measurements['true_pos'], label='GPS Measurements', color='red', s=10)
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()

plt.figure(figsize=(10, 6))
plt.plot(doppler_valid_measurements['time'], doppler_valid_measurements['doppler_range'], label='Range Measurements')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()

plt.figure(figsize=(10, 6))
plt.plot(doppler_valid_measurements['time'], doppler_valid_measurements['doppler_rr'], label='Velocity Measurements')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

plt.show()