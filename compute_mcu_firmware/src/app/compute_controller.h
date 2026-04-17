#pragma once

#include <Arduino.h>

#include "app/auto_home_session.h"
#include "app/self_check_runner.h"
#include "diag/health_monitor.h"
#include "health_report.h"
#include "imu/mpu6500_service.h"
#include "imu/imu_settings_store.h"
#include "station/kran_fsm.h"
#include "transport/i2c_stepper_client.h"
#include "transport/spi_slave_backend.h"

namespace robot
{
namespace app
{
class ComputeController
{
public:
    void begin();
    void loop();

private:
    static constexpr uint8_t kTxQueueDepth = 6;

    bool sendMessage(protocol::MessageType type, const uint8_t *payload, uint8_t len, uint8_t flags = 0, uint8_t ack_seq = 0);
    bool enqueuePacket(const protocol::Packet &packet);
    void serviceTxQueue();
    void handlePacket(const protocol::Packet &packet, uint32_t now_ms);
    void pollStationLink(uint32_t now_ms, const HealthReport &report, const protocol::StepperStatusPayload &stepper_status);
    void pollKranFsm(uint32_t now_ms, const protocol::StepperStatusPayload &stepper_status);
    void handleStationRequest(const char *line, uint32_t now_ms, const HealthReport &report, const protocol::StepperStatusPayload &stepper_status);
    station::KranInputs buildKranInputs(uint32_t now_ms, const protocol::StepperStatusPayload &stepper_status) const;
    bool stationPrizmSummaryFresh(uint32_t now_ms) const;
    bool dispatchKranCommand(station::KranCommand command, int16_t arg);
    bool readAllBoomSlots(uint32_t now_ms,
                          int16_t slot_targets_mm[11],
                          const char *&error_code);
    bool writeBoomSlot(uint32_t now_ms,
                       uint8_t slot,
                       int16_t mm,
                       const char *&error_code);
    bool copyRequestId(char *dst, size_t dst_size, const char *src) const;
    void clearActiveStationRequest();
    void emitStationResp(const char *request_id);
    void emitStationRespStage(const char *request_id, uint8_t stage);
    void emitStationErr(const char *request_id, const char *error_code);
    void emitBoomSlots(const char *request_id,
                       const int16_t slot_targets_mm[11]);
    void emitSystemSummary(uint32_t now_ms, const HealthReport &report, const protocol::StepperStatusPayload &stepper_status);
    const char *kranErrorCodeToString(station::KranErrorCode error_code) const;
    const char *stepperConfigResultToErrorCode(protocol::StepperConfigResult result) const;
    uint8_t computeRobotStatus(bool prizm_summary_fresh, const HealthReport &report, const protocol::StepperStatusPayload &stepper_status) const;
    uint8_t computePreStatus(bool prizm_summary_fresh, const HealthReport &report) const;
    uint8_t computeCrsfStatus(bool prizm_summary_fresh) const;
    uint8_t computeBatteryStatus(bool prizm_summary_fresh) const;
    void publishImuState(uint32_t now_ms);
    void probeImu();
    void updateStatusLed(uint32_t now_ms);
    void traceBoot(const __FlashStringHelper *msg);

    transport::SpiSlaveBackend spi_;
    transport::I2cStepperClient stepper_;
    imu::ImuSettingsStore imu_settings_;
    imu::Mpu6500RuntimeConfig imu_config_{};
    imu::Mpu6500Service imu_;
    SelfCheckRunner self_check_;
    diag::HealthMonitor health_;

    uint8_t next_seq_ = 1;
    uint8_t last_seq_from_prizm_ = 0;
    uint32_t last_rx_ms_ = 0;
    uint32_t last_diag_ms_ = 0;
    uint32_t last_imu_probe_ms_ = 0;
    uint32_t last_imu_publish_ms_ = 0;
    uint32_t last_prizm_summary_ms_ = 0;
    bool wire_timeout_seen_ = false;
    bool tx_loaded_ = false;
    bool prizm_start_gate_open_ = false;
    bool prizm_crsf_fresh_ = false;
    uint16_t prizm_battery_cV_ = 0;
    protocol::Packet tx_queue_[kTxQueueDepth]{};
    uint8_t tx_queue_head_ = 0;
    uint8_t tx_queue_tail_ = 0;
    uint8_t tx_queue_count_ = 0;
    AutoHomeSession auto_home_session_{};
    station::KranFsm kran_fsm_;
    char station_rx_buffer_[32]{};
    uint8_t station_rx_length_ = 0;
    char active_station_request_id_[16]{};
};

} // namespace app
} // namespace robot
