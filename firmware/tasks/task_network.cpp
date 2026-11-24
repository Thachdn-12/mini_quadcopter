#include <atomic>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

// Newlib's BSD headers expose a legacy type macro with our namespace's name.
#ifdef quad
#undef quad
#endif

#include "task_network.h"
#include "task_context.h"
#include "task_messages.h"
#include "task_failsafe.h"
#include "task_pid.h"
#include "network/network_transport.h"

namespace quad {
namespace {
constexpr char kTag[] = "TASK NETWORK";
constexpr int64_t kCommandAgeUs = 100000;
enum class Result : uint8_t { Ok, Invalid, Expired, NotReady, NoController, Busy };
struct ReceivedPacket {
    uint8_t bytes[kNetworkPacketSize];
    uint32_t session;
    int64_t received_us;
};
std::atomic<uint32_t> connection{0};
QueueHandle_t inbox = nullptr;
TaskContext* task_context = nullptr;
portMUX_TYPE pilot_mux = portMUX_INITIALIZER_UNLOCKED;
PilotCommand latest;
uint32_t latest_session = 0;
uint32_t last_sequence = 0;
uint32_t reported_sequence = 0;
uint8_t last_result = 0;

uint16_t read16(const uint8_t* p) { return p[0] | (uint16_t(p[1]) << 8); }
uint32_t read32(const uint8_t* p) { return read16(p) | (uint32_t(read16(p + 2)) << 16); }
void write16(uint8_t* p, uint16_t n) { p[0] = n; p[1] = n >> 8; }
void write32(uint8_t* p, uint32_t n) { write16(p, n); write16(p + 2, n >> 16); }
float signed_value(const uint8_t* p) {
    const int n = read16(p);
    return (n >= 32768 ? n - 65536 : n) / 100.0f;
}
uint16_t clock_ms() { return static_cast<uint16_t>(esp_timer_get_time() / 1000); }

Result process(const ReceivedPacket& received) {
    const auto* p = received.bytes;
    const auto current = connection.load();
    if (!current || received.session != current || read32(p + 4) != current)
        return Result::Expired;
    const int64_t now = esp_timer_get_time();
    const auto clock_age_ms = static_cast<uint16_t>(static_cast<uint16_t>(now / 1000) - read16(p + 2));
    const int64_t clock_age_us = clock_age_ms * 1000LL + now % 1000;
    if (now - received.received_us >= kCommandAgeUs || clock_age_us >= kCommandAgeUs)
        return Result::Expired;
    const uint32_t sequence = read32(p + 8);
    const uint32_t delta = sequence - last_sequence;
    if (p[0] != 1 || p[1] > 3 || delta == 0 || delta >= 0x80000000u)
        return Result::Invalid;
    PilotCommand command{read16(p + 12) / 1000.0f, signed_value(p + 14),
                         signed_value(p + 16), signed_value(p + 18), sequence,
                         received.received_us, now - clock_age_us + kCommandAgeUs};
    if (command.expires_us > received.received_us + kCommandAgeUs)
        command.expires_us = received.received_us + kCommandAgeUs;
    if (command.throttle > 1 || command.roll_deg < -30 || command.roll_deg > 30 ||
        command.pitch_deg < -30 || command.pitch_deg > 30 ||
        command.yaw_rate_dps < -180 || command.yaw_rate_dps > 180)
        return Result::Invalid;
    const bool zero_demand = command.throttle == 0 && command.roll_deg == 0 &&
        command.pitch_deg == 0 && command.yaw_rate_dps == 0;
    if (p[1] != 0 && !zero_demand) return Result::Invalid;
    if (p[1] == 0 && pid_controller_available() &&
        failsafe_accept_pilot_command(zero_demand, command.received_us, command.expires_us) != ESP_OK)
        return Result::NotReady;
    portENTER_CRITICAL(&pilot_mux);
    last_sequence = sequence;
    if (p[1] == 0) {
        latest = command;
        latest_session = current;
    } else {
        latest = {};
        latest_session = 0;
    }
    portEXIT_CRITICAL(&pilot_mux);
    switch (p[1]) {
        case 0: return Result::Ok;
        // PID only produces corrections. Explicit remote arming uses the same
        // supervisor as local commands and requires a fresh zero pilot packet.
        case 1:
            if (!pid_controller_available()) return Result::NoController;
            return request_arm() == ESP_OK ? Result::Ok : Result::NotReady;
        case 2: return request_disarm() == ESP_OK ? Result::Ok : Result::NotReady;
        case 3: return request_emergency_stop() == ESP_OK ? Result::Ok : Result::NotReady;
    }
    return Result::Invalid;
}
}  // namespace

uint32_t network_open_connection() {
    uint32_t expected = 0;
    uint32_t session;
    do { session = esp_random(); } while (!session);
    return connection.compare_exchange_strong(expected, session) ? session : 0;
}

void network_close_connection(uint32_t session) {
    if (session) connection.compare_exchange_strong(session, 0);
}

bool network_receive(uint32_t session, const uint8_t* packet, size_t size) {
    if (!inbox || !session || session != connection.load() || !packet ||
        size != kNetworkPacketSize) return false;
    ReceivedPacket received{};
    for (size_t i = 0; i < size; ++i) received.bytes[i] = packet[i];
    received.session = session;
    received.received_us = esp_timer_get_time();
    return xQueueSend(inbox, &received, 0) == pdTRUE;
}

esp_err_t get_network_pilot_command(PilotCommand& command) {
    portENTER_CRITICAL(&pilot_mux);
    const PilotCommand copy = latest;
    const uint32_t session = latest_session;
    portEXIT_CRITICAL(&pilot_mux);
    const int64_t now = esp_timer_get_time();
    if (!session || session != connection.load() || copy.received_us <= 0 ||
        now < copy.received_us || now - copy.received_us >= kCommandAgeUs || now >= copy.expires_us)
        return ESP_ERR_INVALID_STATE;
    command = copy;
    return ESP_OK;
}

void network_get_telemetry(uint8_t (&p)[kNetworkPacketSize]) {
    for (auto& b : p) b = 0;
    FailsafeStatus safety{};
    (void)get_failsafe_status(safety);
    PilotCommand pilot;
    const bool input_fresh = get_network_pilot_command(pilot) == ESP_OK;
    AttitudeFrame attitude{};
    const bool valid = task_context &&
        xQueuePeek(task_context->attitude_frames, &attitude, 0) == pdTRUE &&
        attitude.valid && sample_is_fresh(attitude.imu.sample, esp_timer_get_time());
    p[0] = 1;
    p[1] = 0x80;
    p[2] = static_cast<uint8_t>(safety.state);
    p[3] = safety.imu_ready | (valid << 1) | (safety.motor_ready << 2) |
           (input_fresh << 3) | (pid_controller_available() << 4);
    write32(p + 4, connection.load());
    portENTER_CRITICAL(&pilot_mux);
    const uint32_t sequence = reported_sequence;
    const uint8_t result = last_result;
    portEXIT_CRITICAL(&pilot_mux);
    write32(p + 8, sequence);
    if (valid) {
        write16(p + 12, static_cast<int16_t>(attitude.roll_deg * 100));
        // Same FRD body pitch convention used by task_pid and pilot setpoints.
        write16(p + 14, static_cast<int16_t>(-attitude.pitch_deg * 100));
    }
    p[16] = static_cast<uint8_t>(safety.reason);
    p[17] = result;
    write16(p + 18, clock_ms());
}

void task_network_init(void* argument) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    task_context = static_cast<TaskContext*>(argument);
    inbox = xQueueCreate(16, sizeof(ReceivedPacket));
    esp_err_t error = inbox ? nvs_flash_init() : ESP_ERR_NO_MEM;
    if (error == ESP_OK) error = start_network_transport();
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "Network startup failed: %s", esp_err_to_name(error));
        if (inbox) { vQueueDelete(inbox); inbox = nullptr; }
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(kTag, "Pilot link ready; explicit arming requires PID and healthy zero-demand input");
    uint32_t observed_session = 0;
    while (true) {
        const auto current = connection.load();
        if (current != observed_session) {
            if (observed_session) {
                FailsafeStatus safety{};
                if (get_failsafe_status(safety) == ESP_OK && safety.state == FailsafeState::Armed)
                    (void)request_emergency_stop();
            }
            portENTER_CRITICAL(&pilot_mux);
            latest = {};
            latest_session = 0;
            last_sequence = 0;
            reported_sequence = 0;
            last_result = 0;
            portEXIT_CRITICAL(&pilot_mux);
            observed_session = current;
        }
        ReceivedPacket received{};
        if (xQueueReceive(inbox, &received, pdMS_TO_TICKS(20)) == pdTRUE) {
            // A newly connected client may have enqueued before this loop saw it.
            if (received.session != observed_session) continue;
            const Result result = process(received);
            portENTER_CRITICAL(&pilot_mux);
            reported_sequence = last_sequence;
            last_result = static_cast<uint8_t>(result);
            portEXIT_CRITICAL(&pilot_mux);
            if (result == Result::Invalid) failsafe_invalid_command();
        }
    }
}
}  // namespace quad
