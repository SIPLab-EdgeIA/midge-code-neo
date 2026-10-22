#ifndef MBFW_IMU_H
#define MBFW_IMU_H

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>

/** Number of samples collected for each IMU calibration measurement. */
#define IMU_CALIBRATION_SAMPLE_COUNT 10

/** Maximum allowed gyroscope reading while the device is stationary (rad/s). */
#define IMU_CALIBRATION_GYRO_ERROR 0.05f

/** Maximum allowed accelerometer variation between samples (m/s^2). */
#define IMU_CALIBRATION_ACCEL_ERROR 0.15f

#define IMU_CALIBRATION_ORIENTATION_TOLERANCE 0.2f

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
 * @brief Result of an IMU calibration orientation check.
 */
enum imu_calibration_result {
    IMU_CALIBRATION_OK = 0,
    IMU_CALIBRATION_MOVEMENT_DETECTED,
    IMU_CALIBRATION_WRONG_ORIENTATION
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

    bool completed[IMU_CALIBRATION_ORIENTATION_COUNT];
};

uint8_t imu_sensor_get_status();

int imu_sensor_init();

int imu_sensor_start(int sample_iter, uint16_t acc_fsr, uint16_t gyr_fsr, uint16_t datarate);

int imu_sensor_stop();

int cmd_start_imu(uint8_t* data);

int cmd_stop_imu(uint8_t* data);

int imu_calibration_check_stationary(void);


/**
 * @brief Collects IMU samples, verifies their stability, and calculates
 *        the average accelerometer and gyroscope readings.
 *
 * The average is calculated only if all collected samples meet the
 * stationary criteria.
 *
 * @param measurement Pointer to the structure where the averages
 *                    will be stored.
 *
 * @return 0 on success,
 *         -EINVAL if measurement is NULL,
 *         -EAGAIN if movement or excessive variation is detected,
 *         or another negative error code if a sensor read fails.
 */
int imu_calibration_collect_average(
    struct imu_calibration_measurement *measurement);


int imu_calibration_check_orientation(
    enum imu_calibration_orientation orientation,
    struct imu_calibration_measurement *measurement);    

int imu_calibration_store_measurement(
    struct imu_calibration_data *calibration_data,
    enum imu_calibration_orientation orientation,
    const struct imu_calibration_measurement *measurement);

int imu_calibration_get_missing_orientations(
    const struct imu_calibration_data *calibration_data,
    enum imu_calibration_orientation *missing_orientations,
    size_t max_orientations);



#endif  // MBFW_IMU_H
