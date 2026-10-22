#include "imu.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <math.h>

#include "imu_interface.h"
#include "midge_protocol.h"
#include "storage.h"
#include "utility.h"


LOG_MODULE_REGISTER(imu);

static uint8_t imu_sensor_state = SENSOR_STATE_DISABLED;

uint8_t imu_sensor_get_status() { return imu_sensor_state; }

int imu_sensor_init() {
    int ret = imu_drv_api.init();
    if (ret != 0) {
        imu_sensor_state = SENSOR_STATE_ERR;
        LOG_ERR("Failed to initialize IMU driver: %d", ret);
    } else {
        imu_sensor_state = SENSOR_STATE_STOP;
        LOG_INF("IMU driver initialized successfully");
    }
    return ret;
}

int imu_sensor_start(int sample_iter, uint16_t acc_fsr, uint16_t gyr_fsr, uint16_t datarate) {
    struct imu_config config = {
        .acc_fsr = acc_fsr,
        .gyr_fsr = gyr_fsr,
        .datarate = datarate,
    };
    // set config
    int ret = imu_drv_api.set_config(&config);
    if (ret != 0) {
        LOG_ERR("Failed to set IMU config: %d", ret);
        return ret;
    }

    // start sampling
    ret = imu_drv_api.start(sample_iter);
    if (ret != 0) {
        LOG_ERR("Failed to start IMU sampling: %d", ret);
        imu_sensor_state = SENSOR_STATE_ERR;
    } else {
        LOG_INF("IMU sampling started successfully");
        imu_sensor_state = SENSOR_STATE_ACTIVE;
    }
    return ret;
}

int imu_sensor_stop() {
    int ret = imu_drv_api.stop();
    if (ret != 0) {
        LOG_ERR("Failed to stop IMU sampling: %d", ret);
        imu_sensor_state = SENSOR_STATE_ERR;
    } else {
        LOG_INF("IMU sampling stopped successfully");
        imu_sensor_state = SENSOR_STATE_STOP;
    }
    return ret;
}



struct start_imu_work_ctx {
    struct k_work work;
    struct k_sem done;
    uint16_t sample_id;
    uint16_t acc_fsr;
    uint16_t gyr_fsr;
    uint16_t datarate;
    int ret;
};

static void start_imu_work_handler(struct k_work* work) {
    struct start_imu_work_ctx* ctx = CONTAINER_OF(work, struct start_imu_work_ctx, work);
    ctx->ret = imu_sensor_start(ctx->sample_id, ctx->acc_fsr, ctx->gyr_fsr, ctx->datarate);
    k_sem_give(&ctx->done);
}

int cmd_start_imu(uint8_t* data) {
    struct cmd_start_imu_request* req_data = (struct cmd_start_imu_request*)data;
    struct start_imu_work_ctx ctx = {.sample_id = req_data->sample_id,
                                     .acc_fsr = req_data->acc_fsr,
                                     .gyr_fsr = req_data->gyr_fsr,
                                     .datarate = req_data->datarate};
    k_sem_init(&ctx.done, 0, 1);
    k_work_init(&ctx.work, start_imu_work_handler);
    int ret = k_work_submit(&ctx.work);
    if (ret < 0) {
        LOG_ERR("Failed to submit start imu work: %d", ret);
    } else {
        ret = k_sem_take(&ctx.done, K_FOREVER);
        if (ret != 0) {
            LOG_ERR("Failed to take semaphore for start imu work: %d", ret);
        } else {
            ret = ctx.ret;
        }
    }

    struct cmd_start_imu_response* resp_data = (struct cmd_start_imu_response*)data;
    memset(resp_data, 0, sizeof(struct cmd_start_imu_response));
    resp_data->status_code = ret;
    return ret;
}

static void stop_imu_work_handler(struct k_work* work) {
    struct simple_work_ctx* ctx = CONTAINER_OF(work, struct simple_work_ctx, work);
    ctx->ret = imu_sensor_stop();
    k_sem_give(&ctx->done);
}

int cmd_stop_imu(uint8_t* data) {
    // struct cmd_stop_imu_request* req_data = (struct cmd_stop_imu_request*)data;
    struct cmd_stop_imu_response* resp_data = (struct cmd_stop_imu_response*)data;
    struct simple_work_ctx ctx;
    k_work_init(&ctx.work, stop_imu_work_handler);
    k_sem_init(&ctx.done, 0, 1);

    int ret;
    int submit_ret = k_work_submit(&ctx.work);
    if (submit_ret < 0) {
        ret = submit_ret;
    } else {
        k_sem_take(&ctx.done, K_FOREVER);
        ret = ctx.ret;
    }
    resp_data->status_code = ret;
    return ret;
}

/**
 * @brief Checks whether two IMU samples meet the stationary criteria.
 *
 * Checks that the accelerometer and gyroscope readings do not vary
 * more than their configured tolerances. It also checks that each
 * gyroscope axis remains close to zero.
 *
 * When the same sample is passed as both arguments, the function
 * checks only whether the gyroscope readings are close to zero.
 *
 * @param previous Pointer to the previous IMU sample.
 * @param current Pointer to the current IMU sample.
 *
 * @return 0 if the readings meet the criteria,
 *         -EINVAL if either pointer is NULL,
 *         or -EAGAIN if a tolerance is exceeded.
 */
static int imu_calibration_calculate_error(
    const struct imu_sample *previous,
    const struct imu_sample *current)
{
    if (previous == NULL || current == NULL) {
        return -EINVAL;
    }

    for (int axis = 0; axis < 3; axis++) {
        // Check that the gyroscope reading is close to zero.
        if (fabsf(current->gyro[axis]) >
            IMU_CALIBRATION_GYRO_ERROR) {
            return -EAGAIN;
        }

        // Check the variation between consecutive accelerometer samples.
        float accel_diff =
            current->accel[axis] - previous->accel[axis];

        if (fabsf(accel_diff) > IMU_CALIBRATION_ACCEL_ERROR) {
            return -EAGAIN;
        }

        // Check the variation between consecutive gyroscope samples.
        float gyro_diff =
            current->gyro[axis] - previous->gyro[axis];

        if (fabsf(gyro_diff) > IMU_CALIBRATION_GYRO_ERROR) {
            return -EAGAIN;
        }
    }

    return 0;
}

/**
 * @brief Checks whether the device remains stationary during sampling.
 *
 * Reads IMU_CALIBRATION_SAMPLE_COUNT samples and checks each sample
 * against the previous one using imu_calibration_calculate_error().
 *
 * @return 0 if all samples meet the stationary criteria,
 *         -EAGAIN if movement or excessive variation is detected,
 *         or another negative error code if a sensor read fails.
 */
int imu_calibration_check_stationary(void)
{
    struct imu_sample previous_sample;

    // Read the first sample.
    int ret = imu_drv_api.read_sample(&previous_sample);
    if (ret != 0) {
        LOG_ERR("Failed to read initial stationary check sample: %d", ret);
        return ret;
    }

    // Check the initial gyroscope readings.
    ret = imu_calibration_calculate_error(
        &previous_sample, &previous_sample);

    if (ret != 0) {
        return ret;
    }

    // Read and validate the remaining samples.
    for (int i = 1; i < IMU_CALIBRATION_SAMPLE_COUNT; i++) {
        struct imu_sample current_sample;

        ret = imu_drv_api.read_sample(&current_sample);
        if (ret != 0) {
            LOG_ERR("Failed to read stationary check sample %d: %d",
                    i, ret);
            return ret;
        }

        ret = imu_calibration_calculate_error(
            &previous_sample, &current_sample);

        if (ret != 0) {
            return ret;
        }

        // Keep the current sample for the next comparison.
        previous_sample = current_sample;
    }

    return 0;
}

/**
 * @brief Collects IMU samples, verifies stability, and calculates averages.
 *
 * Each sample is checked before its readings are added to the sums.
 * The averages are stored only if all samples meet the stationary criteria.
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
    struct imu_calibration_measurement *measurement)
{
    if (measurement == NULL) {
        return -EINVAL;
    }

    // Initialize the sums for the accelerometer and gyroscope axes.
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};

    struct imu_sample previous_sample = {0};

    // Read and validate the configured number of samples.
    for (int i = 0; i < IMU_CALIBRATION_SAMPLE_COUNT; i++) {
        struct imu_sample current_sample;

        int ret = imu_drv_api.read_sample(&current_sample);
        if (ret != 0) {
            LOG_ERR("Failed to read calibration sample %d: %d", i, ret);
            return ret;
        }

        // Validate the first sample and compare subsequent samples.
        if (i == 0) {
            ret = imu_calibration_calculate_error(
                &current_sample, &current_sample);
        } else {
            ret = imu_calibration_calculate_error(
                &previous_sample, &current_sample);
        }

        if (ret != 0) {
            LOG_WRN("Movement detected during calibration at sample %d", i);
            return ret;
        }

        // Accumulate readings only after the sample passes validation.
        for (int axis = 0; axis < 3; axis++) {
            accel_sum[axis] += current_sample.accel[axis];
            gyro_sum[axis] += current_sample.gyro[axis];
        }

        // Save the current sample for the next comparison.
        previous_sample = current_sample;
    }

    // Calculate and store the average for each axis.
    for (int axis = 0; axis < 3; axis++) {
        measurement->accel[axis] =
            accel_sum[axis] / (float)IMU_CALIBRATION_SAMPLE_COUNT;

        measurement->gyro[axis] =
            gyro_sum[axis] / (float)IMU_CALIBRATION_SAMPLE_COUNT;
    }

    return 0;
}