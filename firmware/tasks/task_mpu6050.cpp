#include "task_context.h"
#include "task_messages.h"
#include "task_failsafe.h"

#include "freertos/task.h"
#include "imu_mpu6050.h"
#include "esp_log.h"

namespace quad {
namespace {
void publish_imu(TaskContext& context, const ImuFrame& frame) {
    // Report health before overwriting the mailbox: faults must not be lost.
    failsafe_update_imu(frame);
    xQueueOverwrite(context.imu_frames, &frame);
}

void run_mpu6050(TaskContext& context) {
    ESP_LOGI("TASK MPU 6050", "RUN TASK MPU 6050\n");
    // Only this task accesses the driver, including calibration and cleanup.
    Mpu6050 imu;
    ImuFrame frame{};
    const ImuConfig config{};
    ESP_LOGI("TASK MPU 6050", "Initializing MPU6050: SDA=GPIO%d SCL=GPIO%d address=0x%02x",
             static_cast<int>(config.sda), static_cast<int>(config.scl), config.address);
    publish_imu(context, frame);
    frame.error = imu.init(config);
    if (frame.error != ESP_OK) {
        ESP_LOGE("TASK MPU 6050", "MPU6050 initialization failed: %s", esp_err_to_name(frame.error));
        frame.state = ImuState::Fault;
        publish_imu(context, frame);
        return;
    }

    frame.state = ImuState::Calibrating;
    ESP_LOGI("TASK MPU 6050", "MPU6050 initialized; keep level/still, +Z up for calibration");
    publish_imu(context, frame);
    vTaskDelay(pdMS_TO_TICKS(2000));
    frame.error = imu.calibrate();
    if (frame.error != ESP_OK) {
        ESP_LOGE("TASK MPU 6050", "MPU6050 calibration failed: %s; see mpu6050 diagnostics",
                 esp_err_to_name(frame.error));
        frame.state = ImuState::Fault;
        publish_imu(context, frame);
        return;
    }
    frame.bias = imu.bias();
    ESP_LOGI("TASK MPU 6050", "MPU6050 calibration successful; starting sample stream");
    frame.state = ImuState::Streaming;
    while (true) {
        frame.error = imu.read(frame.sample);
        if (frame.error == ESP_OK) {
            ++frame.sample_count;
        } else {
            ++frame.read_error_count;
        }
        // Never wait for logging or estimation to catch up with the sensor.
        publish_imu(context, frame);
        if (frame.error != ESP_OK) vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}  // namespace

void task_mpu6050_init(void* context) {
    ESP_LOGI("TASK_MPU6050","INITIALIZE TASK MPU6050\n");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    run_mpu6050(*static_cast<TaskContext*>(context));
    // The local driver is destroyed before deleting this task on failure.
    vTaskDelete(nullptr);
}

}  // namespace quad
