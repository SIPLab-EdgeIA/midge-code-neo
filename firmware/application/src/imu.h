#ifndef MBFW_IMU_H
#define MBFW_IMU_H

#include <inttypes.h>

/** Number of samples collected for each IMU calibration measurement. */
#define IMU_CALIBRATION_SAMPLE_COUNT 100

/**
 * @brief Stores the average accelerometer and gyroscope measurements
 *        for a single orientation.
 */
struct imu_calibration_measurement {
    float accel[3]; /**< Average accelerometer readings for X, Y, and Z. */
    float gyro[3];  /**< Average gyroscope readings for X, Y, and Z. */
};

/**
 * @brief Identifies the six orientations used for IMU calibration.
 */
enum imu_calibration_orientation {
    IMU_CALIBRATION_ORIENTATION_X_POS = 0, /**< Positive X axis. */
    IMU_CALIBRATION_ORIENTATION_X_NEG = 1, /**< Negative X axis. */
    IMU_CALIBRATION_ORIENTATION_Y_POS = 2, /**< Positive Y axis. */
    IMU_CALIBRATION_ORIENTATION_Y_NEG = 3, /**< Negative Y axis. */
    IMU_CALIBRATION_ORIENTATION_Z_POS = 4, /**< Positive Z axis. */
    IMU_CALIBRATION_ORIENTATION_Z_NEG = 5, /**< Negative Z axis. */

    IMU_CALIBRATION_ORIENTATION_COUNT = 6 /**< Total number of orientations. */
};

/**
 * @brief Stores calibration measurements for all six orientations.
 *
 * Each array entry corresponds to the orientation with the same index
 * in enum imu_calibration_orientation.
 */
struct imu_calibration_data {
    struct imu_calibration_measurement
        measurements[IMU_CALIBRATION_ORIENTATION_COUNT];
};

uint8_t imu_sensor_get_status();

int imu_sensor_init();

int imu_sensor_start(int sample_iter, uint16_t acc_fsr, uint16_t gyr_fsr, uint16_t datarate);

int imu_sensor_stop();

int cmd_start_imu(uint8_t* data);

int cmd_stop_imu(uint8_t* data);

#endif  // MBFW_IMU_H
