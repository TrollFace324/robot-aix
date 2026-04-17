#include "app/compute_controller.h"

#include <math.h>
#include <stdio.h>
#include <Wire.h>
#include <string.h>

#include "config/debug_config.h"
#include "config/imu_config.h"
#include "config/pin_map_compute.h"
#include "config/transport_config.h"

namespace
{
constexpr uint32_t kRxTimeoutMs = 500;
constexpr uint32_t kEchoCaps = 0x0000000F;
constexpr uint32_t kImuProbeRetryMs = 1000;
constexpr uint32_t kImuPublishPeriodMs = robot::config::kImuPublishPeriodMs;
constexpr uint32_t kLedSlowBlinkMs = 600;
constexpr uint32_t kLedFastBlinkMs = 180;
constexpr uint32_t kLedPulseCycleMs = 1200;
constexpr uint32_t kLedPulseOnMs = 90;
constexpr uint32_t kLedPulseGapMs = 170;
constexpr uint32_t kStationPrizmFreshMs = 1000;
constexpr uint32_t kStationSerialBaud = 9600;
constexpr uint8_t kStationBoomSlotCount = 11;

bool isDigitsOnly(const char *value)
{
    if (value == nullptr || *value == '\0')
    {
        return false;
    }

    for (const char *cursor = value; *cursor != '\0'; ++cursor)
    {
        if (*cursor < '0' || *cursor > '9')
        {
            return false;
        }
    }

    return true;
}

bool parseStationRequestId(const char *line, char *request_id_out, size_t request_id_size)
{
    if (line == nullptr || request_id_out == nullptr || request_id_size < 2)
    {
        return false;
    }

    static constexpr char kPrefix[] = "REQ,";
    if (strncmp(line, kPrefix, sizeof(kPrefix) - 1) != 0)
    {
        return false;
    }

    const char *payload = line + (sizeof(kPrefix) - 1);
    if (*payload == '\0')
    {
        return false;
    }

    size_t length = 0;
    bool previous_was_underscore = false;
    while (payload[length] != '\0')
    {
        const char ch = payload[length];
        if ((ch >= '0' && ch <= '9'))
        {
            previous_was_underscore = false;
        }
        else if (ch == '_')
        {
            if (previous_was_underscore)
            {
                return false;
            }
            previous_was_underscore = true;
        }
        else
        {
            return false;
        }

        if (length + 1 >= request_id_size)
        {
            return false;
        }
        request_id_out[length] = ch;
        ++length;
    }

    if (previous_was_underscore)
    {
        return false;
    }

    request_id_out[length] = '\0';
    return true;
}

bool splitRequestId(const char *request_id,
                    char *base_out,
                    size_t base_size,
                    int16_t *args_out,
                    size_t max_args,
                    size_t *arg_count_out)
{
    if (request_id == nullptr || base_out == nullptr || base_size < 2 ||
        args_out == nullptr || arg_count_out == nullptr)
    {
        return false;
    }

    *arg_count_out = 0;
    const char *separator = strchr(request_id, '_');
    if (separator == nullptr)
    {
        const size_t length = strlen(request_id);
        if (length == 0 || length >= base_size)
        {
            return false;
        }
        memcpy(base_out, request_id, length + 1);
        return true;
    }

    const size_t base_length = static_cast<size_t>(separator - request_id);
    if (base_length == 0 || base_length >= base_size)
    {
        return false;
    }

    memcpy(base_out, request_id, base_length);
    base_out[base_length] = '\0';

    const char *cursor = separator + 1;
    while (*cursor != '\0')
    {
        if (*arg_count_out >= max_args)
        {
            return false;
        }

        const char *next_separator = strchr(cursor, '_');
        const size_t arg_length =
            (next_separator == nullptr)
                ? strlen(cursor)
                : static_cast<size_t>(next_separator - cursor);
        if (arg_length == 0 || arg_length >= 8)
        {
            return false;
        }

        char arg_text[8]{};
        memcpy(arg_text, cursor, arg_length);
        if (!isDigitsOnly(arg_text))
        {
            return false;
        }

        long parsed = 0;
        for (size_t index = 0; index < arg_length; ++index)
        {
            parsed = (parsed * 10L) + static_cast<long>(arg_text[index] - '0');
            if (parsed > 32767L)
            {
                return false;
            }
        }

        args_out[*arg_count_out] = static_cast<int16_t>(parsed);
        ++(*arg_count_out);

        if (next_separator == nullptr)
        {
            break;
        }
        cursor = next_separator + 1;
    }
    return true;
}

bool isImmediateStageStubRequest(const char *base_request_id)
{
    return strcmp(base_request_id, "6") == 0 ||
           strcmp(base_request_id, "16") == 0 ||
           strcmp(base_request_id, "17") == 0 ||
           strcmp(base_request_id, "20") == 0 ||
           strcmp(base_request_id, "21") == 0 ||
           strcmp(base_request_id, "22") == 0 ||
           strcmp(base_request_id, "23") == 0 ||
           strcmp(base_request_id, "24") == 0;
}

bool isSimpleAckRequest(const char *base_request_id)
{
    return strcmp(base_request_id, "1") == 0 ||
           strcmp(base_request_id, "2") == 0 ||
           strcmp(base_request_id, "3") == 0 ||
           strcmp(base_request_id, "4") == 0 ||
           strcmp(base_request_id, "5") == 0 ||
           strcmp(base_request_id, "7") == 0 ||
           strcmp(base_request_id, "8") == 0 ||
           strcmp(base_request_id, "9") == 0 ||
           strcmp(base_request_id, "10") == 0 ||
           strcmp(base_request_id, "13") == 0 ||
           strcmp(base_request_id, "14") == 0 ||
           strcmp(base_request_id, "15") == 0 ||
           strcmp(base_request_id, "18") == 0 ||
           strcmp(base_request_id, "19") == 0 ||
           strcmp(base_request_id, "25") == 0 ||
           strcmp(base_request_id, "26") == 0 ||
           strcmp(base_request_id, "27") == 0 ||
           strcmp(base_request_id, "28") == 0 ||
           strcmp(base_request_id, "29") == 0;
}

bool pulseTrain(uint32_t now_ms, uint8_t count)
{
    if (count == 0)
    {
        return false;
    }

    const uint32_t phase = now_ms % kLedPulseCycleMs;
    for (uint8_t i = 0; i < count; ++i)
    {
        const uint32_t start = static_cast<uint32_t>(i) * (kLedPulseOnMs + kLedPulseGapMs);
        if (phase >= start && phase < (start + kLedPulseOnMs))
        {
            return true;
        }
    }
    return false;
}

int16_t clampInt16(long value)
{
    if (value > 32767L)
    {
        return 32767;
    }
    if (value < -32768L)
    {
        return -32768;
    }
    return static_cast<int16_t>(value);
}

void encodeInt16Le(uint8_t *out, int16_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFF);
    out[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}
}

namespace robot
{
namespace app
{
void ComputeController::begin()
{
    pinMode(config::kStatusLedPin, OUTPUT);
    digitalWrite(config::kStatusLedPin, HIGH);
    // Firmware boot signature: 5 short flashes to confirm fresh compute build is running.
    for (uint8_t i = 0; i < 5; ++i)
    {
        digitalWrite(config::kStatusLedPin, HIGH);
        delay(60);
        digitalWrite(config::kStatusLedPin, LOW);
        delay(60);
    }

    Serial.begin(config::kEnableBootSerialTrace ? config::kBootSerialBaud : kStationSerialBaud);
    delay(20);
    traceBoot(F("CMP: begin"));

    traceBoot(F("CMP: wire ready"));

    spi_.begin();
    traceBoot(F("CMP: spi ready"));
    stepper_.begin(Wire, config::kStepperI2cAddress);
    traceBoot(F("CMP: stepper client ready"));
    bool loaded_from_eeprom = false;
    imu_config_ = imu_settings_.loadOrDefaults(&loaded_from_eeprom);
    imu_.configure(imu_config_);
    probeImu();
    last_imu_probe_ms_ = millis();
    last_imu_publish_ms_ = millis();
    traceBoot(F("CMP: imu probe done"));

    self_check_.begin(&imu_, &stepper_, config::kRequireStepperOnlineForReady);
    health_.begin();

    next_seq_ = 1;
    last_seq_from_prizm_ = 0;
    last_rx_ms_ = 0;
    last_diag_ms_ = millis();
    wire_timeout_seen_ = false;
    tx_loaded_ = false;
    prizm_start_gate_open_ = false;
    prizm_crsf_fresh_ = false;
    prizm_battery_cV_ = 0;
    auto_home_session_.reset();
    last_prizm_summary_ms_ = 0;
    tx_queue_head_ = 0;
    tx_queue_tail_ = 0;
    tx_queue_count_ = 0;
    station_rx_length_ = 0;
    active_station_request_id_[0] = '\0';

    // Initial beacon frame: allows PRIZM to validate MISO path before handshake.
    {
        protocol::Packet beacon;
        beacon.type = protocol::MessageType::TEST_FLAG;
        beacon.source = protocol::NodeId::COMPUTE;
        beacon.target = protocol::NodeId::PRIZM;
        beacon.seq = next_seq_++;
        beacon.length = 4;
        beacon.payload[0] = 'B';
        beacon.payload[1] = 'E';
        beacon.payload[2] = 'A';
        beacon.payload[3] = 'C';
        enqueuePacket(beacon);
    }
    serviceTxQueue();

    digitalWrite(config::kStatusLedPin, LOW);
    traceBoot(F("CMP: begin done"));
}

void ComputeController::loop()
{
    const uint32_t now_ms = millis();

    spi_.poll(now_ms);
    const protocol::StepperStatusPayload stepper_status_before = stepper_.status();
    const bool stepper_active =
        stepper_.online() &&
        (stepper_status_before.status == protocol::StepperStatusCode::STATUS_MOVING ||
         stepper_status_before.status == protocol::StepperStatusCode::STATUS_HOMING);
    if (!stepper_active)
    {
        imu_.update(now_ms);
    }
    self_check_.update(now_ms);
    const HealthReport &report = self_check_.report();
    const protocol::StepperStatusPayload stepper_status = stepper_.status();

    protocol::Packet packet;
    uint8_t processed_packets = 0;
    while (processed_packets < 4 && spi_.receivePacket(packet))
    {
        tx_loaded_ = false;
        if (packet.source != protocol::NodeId::PRIZM)
        {
            serviceTxQueue();
            ++processed_packets;
            continue;
        }

        if (packet.seq == last_seq_from_prizm_)
        {
            serviceTxQueue();
            ++processed_packets;
            continue;
        }

        last_seq_from_prizm_ = packet.seq;
        last_rx_ms_ = now_ms;
        health_.onPacketRx(packet);
        handlePacket(packet, now_ms);
        serviceTxQueue();
        ++processed_packets;
    }

    if ((last_rx_ms_ != 0) && (now_ms - last_rx_ms_ > kRxTimeoutMs))
    {
        health_.onTimeout();
    }

#if defined(WIRE_TIMEOUT)
    if (Wire.getWireTimeoutFlag())
    {
        wire_timeout_seen_ = true;
        Wire.clearWireTimeoutFlag();
        // Recover frozen TWI state machine and continue serving SPI.
        stepper_.begin(Wire, config::kStepperI2cAddress);
        probeImu();
        last_imu_probe_ms_ = now_ms;
    }
#endif

    if (!imu_.healthy() && (now_ms - last_imu_probe_ms_ >= kImuProbeRetryMs))
    {
        probeImu();
        last_imu_probe_ms_ = now_ms;
    }

    publishImuState(now_ms);
    pollKranFsm(now_ms, stepper_status);
    pollStationLink(now_ms, report, stepper_status);
    updateStatusLed(now_ms);
    serviceTxQueue();

    if (config::kEnableBootSerialTrace && (now_ms - last_diag_ms_ >= 1000))
    {
        const protocol::LinkHealth &h = spi_.health();
        const HealthReport &r = self_check_.report();
        Serial.print(F("CMP: rx="));
        Serial.print(h.rx_packets);
        Serial.print(F(" tx="));
        Serial.print(h.tx_packets);
        Serial.print(F(" crc="));
        Serial.print(h.crc_errors);
        Serial.print(F(" dec="));
        Serial.print(h.decode_errors);
        Serial.print(F(" imu_h="));
        Serial.print(r.imu_healthy ? 1 : 0);
        Serial.print(F(" imu_v="));
        Serial.print(r.imu_ok ? 1 : 0);
        Serial.print(F(" tilt="));
        Serial.print(r.tilt_fault ? 1 : 0);
        Serial.print(F(" step="));
        Serial.print(r.stepper_online ? 1 : 0);
        Serial.print(F(" ready="));
        Serial.print(r.startup_ready ? 1 : 0);
        Serial.print(F(" wt="));
        Serial.println(wire_timeout_seen_ ? 1 : 0);
        last_diag_ms_ = now_ms;
    }
}

bool ComputeController::sendMessage(protocol::MessageType type, const uint8_t *payload, uint8_t len, uint8_t flags, uint8_t ack_seq)
{
    protocol::Packet out;
    out.type = type;
    out.source = protocol::NodeId::COMPUTE;
    out.target = protocol::NodeId::PRIZM;
    out.seq = next_seq_++;
    out.ack_seq = ack_seq;
    out.flags = flags;
    out.length = len;
    if (payload && len > 0)
    {
        memcpy(out.payload, payload, len);
    }
    const bool queued = enqueuePacket(out);
    serviceTxQueue();
    return queued;
}

bool ComputeController::enqueuePacket(const protocol::Packet &packet)
{
    if (tx_queue_count_ >= kTxQueueDepth)
    {
        return false;
    }

    tx_queue_[tx_queue_tail_] = packet;
    tx_queue_tail_ = static_cast<uint8_t>((tx_queue_tail_ + 1) % kTxQueueDepth);
    ++tx_queue_count_;
    return true;
}

void ComputeController::serviceTxQueue()
{
    if (tx_loaded_ || tx_queue_count_ == 0)
    {
        return;
    }

    if (!spi_.sendPacket(tx_queue_[tx_queue_head_]))
    {
        return;
    }

    tx_queue_head_ = static_cast<uint8_t>((tx_queue_head_ + 1) % kTxQueueDepth);
    --tx_queue_count_;
    tx_loaded_ = true;
}

void ComputeController::handlePacket(const protocol::Packet &packet, uint32_t now_ms)
{
    const HealthReport &report = self_check_.report();

    switch (packet.type)
    {
    case protocol::MessageType::HELLO:
    {
        uint8_t payload[4] = {protocol::kVersion, 0x01, 0x00, 0x00};
        sendMessage(protocol::MessageType::HELLO_ACK, payload, sizeof(payload), protocol::FLAG_ACK, packet.seq);
        break;
    }

    case protocol::MessageType::GET_CAPABILITIES:
    {
        uint8_t payload[4];
        payload[0] = static_cast<uint8_t>(kEchoCaps & 0xFF);
        payload[1] = static_cast<uint8_t>((kEchoCaps >> 8) & 0xFF);
        payload[2] = static_cast<uint8_t>((kEchoCaps >> 16) & 0xFF);
        payload[3] = static_cast<uint8_t>((kEchoCaps >> 24) & 0xFF);
        sendMessage(protocol::MessageType::CAPABILITIES, payload, sizeof(payload), protocol::FLAG_ACK, packet.seq);
        break;
    }

    case protocol::MessageType::GET_STATUS:
    {
        uint8_t status_payload[2] = {health_.healthByte(report), static_cast<uint8_t>(report.startup_ready ? 1 : 0)};
        sendMessage(protocol::MessageType::STATUS, status_payload, sizeof(status_payload));

        uint8_t check_payload[8] = {
            static_cast<uint8_t>(report.startup_ready ? 1 : 0),
            static_cast<uint8_t>(report.imu_ok ? 1 : 0),
            static_cast<uint8_t>(report.i2c_bus_ok ? 1 : 0),
            static_cast<uint8_t>(report.stepper_online ? 1 : 0),
            stepper_.lastWriteStatus(),
            stepper_.lastRequestCount(),
            stepper_.lastDiagCode(),
            stepper_.lastStatusSeq()};
        sendMessage(protocol::MessageType::STARTUP_CHECK_RESULT, check_payload, sizeof(check_payload));

        uint8_t sensor_payload[2] = {
            static_cast<uint8_t>(report.stepper_status_code),
            static_cast<uint8_t>(report.tilt_fault ? 1 : 0)};
        sendMessage(protocol::MessageType::SENSOR_STATE, sensor_payload, sizeof(sensor_payload));

        if (report.startup_ready)
        {
            sendMessage(protocol::MessageType::READY_TO_RUN, nullptr, 0);
        }
        else
        {
            uint8_t fault[1] = {report.fault_flags};
            sendMessage(protocol::MessageType::RUN_LOCKED, fault, sizeof(fault));
        }
        break;
    }

    case protocol::MessageType::HEARTBEAT:
        sendMessage(protocol::MessageType::HEARTBEAT_ACK, nullptr, 0, protocol::FLAG_ACK, packet.seq);
        break;

    case protocol::MessageType::ECHO_TEST:
        sendMessage(protocol::MessageType::ECHO_TEST, packet.payload, packet.length, protocol::FLAG_ACK, packet.seq);
        break;

    case protocol::MessageType::TEST_FLAG:
        sendMessage(protocol::MessageType::TEST_FLAG, packet.payload, packet.length, protocol::FLAG_ACK, packet.seq);
        break;

    case protocol::MessageType::ACTUATOR_COMMAND:
        if (packet.length >= 3)
        {
            const protocol::StepperCommand cmd = static_cast<protocol::StepperCommand>(packet.payload[0]);
            const int16_t arg0 = static_cast<int16_t>(packet.payload[1] | (packet.payload[2] << 8));
            stepper_.sendCommand(cmd, arg0, 0);
        }
        break;

    case protocol::MessageType::SET_MODE:
    {
        const protocol::SetModeSummaryPayload summary =
            protocol::decodeSetModeSummaryPayload(packet.payload, packet.length);
        prizm_start_gate_open_ = summary.start_gate_open;
        prizm_crsf_fresh_ = summary.crsf_fresh;
        prizm_battery_cV_ = summary.battery_cV;
        last_prizm_summary_ms_ = now_ms;
        break;
    }

    default:
        break;
    }
}

bool ComputeController::stationPrizmSummaryFresh(uint32_t now_ms) const
{
    return (last_prizm_summary_ms_ != 0U) &&
           ((now_ms - last_prizm_summary_ms_) <= kStationPrizmFreshMs);
}

station::KranInputs ComputeController::buildKranInputs(uint32_t now_ms,
                                                       const protocol::StepperStatusPayload &stepper_status) const
{
    station::KranInputs inputs;
    inputs.now_ms = now_ms;
    inputs.start_gate_open = prizm_start_gate_open_;
    inputs.prizm_summary_fresh = stationPrizmSummaryFresh(now_ms);
    inputs.stepper_online = stepper_.online();
    inputs.stepper_homed = stepper_status.is_homed;
    inputs.home_switch_active = stepper_status.home_switch_state;
    inputs.stepper_fault =
        (stepper_status.fault_flags != 0) ||
        (stepper_status.status == protocol::StepperStatusCode::STATUS_ERROR);
    inputs.current_position_mm = stepper_status.current_position_mm;
    inputs.target_position_mm = stepper_status.target_position_mm;

    switch (stepper_status.status)
    {
    case protocol::StepperStatusCode::STATUS_MOVING:
        inputs.stepper_state = station::StepperMotionState::Moving;
        break;

    case protocol::StepperStatusCode::STATUS_HOMING:
        inputs.stepper_state = station::StepperMotionState::Homing;
        break;

    case protocol::StepperStatusCode::STATUS_ERROR:
        inputs.stepper_state = station::StepperMotionState::Fault;
        break;

    case protocol::StepperStatusCode::STATUS_IDLE:
    case protocol::StepperStatusCode::STATUS_STOPPED:
    default:
        inputs.stepper_state = station::StepperMotionState::Idle;
        break;
    }

    return inputs;
}

bool ComputeController::dispatchKranCommand(station::KranCommand command, int16_t arg)
{
    switch (command)
    {
    case station::KranCommand::Home:
        return stepper_.sendCommand(protocol::StepperCommand::CMD_HOME, 0, 0);

    case station::KranCommand::GotoSlot:
        return stepper_.sendCommand(protocol::StepperCommand::CMD_GOTO_SLOT, arg, 0);

    case station::KranCommand::None:
    default:
        return true;
    }
}

bool ComputeController::readAllBoomSlots(
    uint32_t now_ms,
    int16_t slot_targets_mm[kStationBoomSlotCount],
    const char *&error_code)
{
    error_code = nullptr;
    for (uint8_t slot = 0; slot < kStationBoomSlotCount; ++slot)
    {
        int16_t slot_mm = 0;
        protocol::StepperConfigResult result =
            protocol::StepperConfigResult::Ok;
        if (!stepper_.readSlotMm(now_ms, slot, slot_mm, result))
        {
            error_code = "STEPPER_READ_FAIL";
            return false;
        }

        if (result != protocol::StepperConfigResult::Ok)
        {
            error_code = stepperConfigResultToErrorCode(result);
            return false;
        }

        slot_targets_mm[slot] = slot_mm;
    }

    return true;
}

bool ComputeController::writeBoomSlot(uint32_t now_ms,
                                      uint8_t slot,
                                      int16_t mm,
                                      const char *&error_code)
{
    error_code = nullptr;
    protocol::StepperConfigResult result =
        protocol::StepperConfigResult::Ok;
    if (!stepper_.writeSlotMm(now_ms, slot, mm, result))
    {
        error_code = "STEPPER_WRITE_FAIL";
        return false;
    }

    if (result != protocol::StepperConfigResult::Ok)
    {
        error_code = stepperConfigResultToErrorCode(result);
        return false;
    }

    return true;
}

bool ComputeController::copyRequestId(char *dst, size_t dst_size, const char *src) const
{
    if (dst == nullptr || src == nullptr || dst_size == 0)
    {
        return false;
    }

    const size_t length = strlen(src);
    if (length + 1 > dst_size)
    {
        return false;
    }

    memcpy(dst, src, length + 1);
    return true;
}

void ComputeController::clearActiveStationRequest()
{
    active_station_request_id_[0] = '\0';
}

void ComputeController::emitStationResp(const char *request_id)
{
    Serial.print(F("RESP,"));
    Serial.println(request_id);
}

void ComputeController::emitStationRespStage(const char *request_id, uint8_t stage)
{
    Serial.print(F("RESP,"));
    Serial.print(request_id);
    Serial.print(F(","));
    Serial.println(stage);
}

void ComputeController::emitStationErr(const char *request_id, const char *error_code)
{
    Serial.print(F("ERR,"));
    Serial.print(request_id);
    Serial.print(F(","));
    Serial.println(error_code);
}

void ComputeController::emitBoomSlots(
    const char *request_id,
    const int16_t slot_targets_mm[kStationBoomSlotCount])
{
    Serial.print(F("SLOTS,"));
    Serial.print(request_id);
    for (uint8_t slot = 0; slot < kStationBoomSlotCount; ++slot)
    {
        Serial.print(F(","));
        Serial.print(slot_targets_mm[slot]);
    }
    Serial.println();
}

const char *ComputeController::kranErrorCodeToString(station::KranErrorCode error_code) const
{
    switch (error_code)
    {
    case station::KranErrorCode::Busy:
        return "BUSY";
    case station::KranErrorCode::NotReady:
        return "NOT_READY";
    case station::KranErrorCode::InvalidSlot:
        return "INVALID_SLOT";
    case station::KranErrorCode::StepperFault:
        return "STEPPER_FAULT";
    case station::KranErrorCode::None:
    default:
        return "UNKNOWN";
    }
}

const char *ComputeController::stepperConfigResultToErrorCode(
    protocol::StepperConfigResult result) const
{
    switch (result)
    {
    case protocol::StepperConfigResult::InvalidSlot:
        return "INVALID_SLOT";
    case protocol::StepperConfigResult::InvalidMm:
        return "INVALID_MM";
    case protocol::StepperConfigResult::Busy:
        return "BUSY";
    case protocol::StepperConfigResult::EepromSaveFail:
        return "EEPROM_SAVE_FAIL";
    case protocol::StepperConfigResult::Ok:
    default:
        return "STEPPER_CONFIG_FAIL";
    }
}

uint8_t ComputeController::computeRobotStatus(bool prizm_summary_fresh,
                                              const HealthReport &report,
                                              const protocol::StepperStatusPayload &stepper_status) const
{
    const bool stepper_fault =
        (stepper_status.fault_flags != 0) ||
        (stepper_status.status == protocol::StepperStatusCode::STATUS_ERROR);

    if (!prizm_summary_fresh || !report.stepper_online || stepper_fault)
    {
        return 0;
    }

    if (!report.startup_ready || !stepper_status.is_homed)
    {
        return 1;
    }

    return 2;
}

uint8_t ComputeController::computePreStatus(bool prizm_summary_fresh,
                                            const HealthReport &report) const
{
    if (!prizm_summary_fresh)
    {
        return 1;
    }

    if (report.startup_ready)
    {
        return 2;
    }

    if (report.imu_healthy || report.stepper_online)
    {
        return 1;
    }

    return 0;
}

uint8_t ComputeController::computeCrsfStatus(bool prizm_summary_fresh) const
{
    if (!prizm_summary_fresh)
    {
        return 1;
    }

    return prizm_crsf_fresh_ ? 2 : 0;
}

uint8_t ComputeController::computeBatteryStatus(bool prizm_summary_fresh) const
{
    if (!prizm_summary_fresh)
    {
        return 1;
    }

    if (prizm_battery_cV_ >= 1140U)
    {
        return 2;
    }

    if (prizm_battery_cV_ >= 1080U)
    {
        return 1;
    }

    return 0;
}

void ComputeController::emitSystemSummary(uint32_t now_ms,
                                          const HealthReport &report,
                                          const protocol::StepperStatusPayload &stepper_status)
{
    const bool prizm_summary_fresh = stationPrizmSummaryFresh(now_ms);
    const uint8_t robot_status = computeRobotStatus(prizm_summary_fresh, report, stepper_status);
    const uint8_t pre_status = computePreStatus(prizm_summary_fresh, report);
    const uint8_t crsf_status = computeCrsfStatus(prizm_summary_fresh);
    const uint8_t battery_status = computeBatteryStatus(prizm_summary_fresh);
    char line[80];
    snprintf(line,
             sizeof(line),
             "STAT,0,robot=%u,pre=%u,crsf=%u,batt=%u,v=%u",
             robot_status,
             pre_status,
             crsf_status,
             battery_status,
             static_cast<unsigned int>(prizm_battery_cV_));
    Serial.println(line);
}

void ComputeController::pollKranFsm(uint32_t now_ms,
                                    const protocol::StepperStatusPayload &stepper_status)
{
    const station::KranInputs inputs = buildKranInputs(now_ms, stepper_status);
    auto_home_session_.update(inputs.start_gate_open, inputs.stepper_fault, inputs.stepper_state);

    AutoHomeInputs auto_home_inputs;
    auto_home_inputs.prizm_summary_fresh = inputs.prizm_summary_fresh;
    auto_home_inputs.start_gate_open = inputs.start_gate_open;
    auto_home_inputs.stepper_online = inputs.stepper_online;
    auto_home_inputs.stepper_fault = inputs.stepper_fault;
    auto_home_inputs.kran_request_active = kran_fsm_.hasActiveRequest();
    auto_home_inputs.stepper_state = inputs.stepper_state;

    if (auto_home_session_.shouldIssueHome(auto_home_inputs) &&
        stepper_.sendCommand(protocol::StepperCommand::CMD_HOME, 0, 0))
    {
        auto_home_session_.markHomeIssued();
    }

    const station::KranOutputs outputs = kran_fsm_.update(inputs, nullptr);
    if (outputs.command != station::KranCommand::None &&
        !dispatchKranCommand(outputs.command, outputs.command_arg) &&
        active_station_request_id_[0] != '\0')
    {
        emitStationErr(active_station_request_id_, "STEPPER_WRITE_FAIL");
        clearActiveStationRequest();
        return;
    }

    if (outputs.emit_error && active_station_request_id_[0] != '\0')
    {
        emitStationErr(active_station_request_id_, kranErrorCodeToString(outputs.error_code));
        clearActiveStationRequest();
        return;
    }

    if (outputs.emit_second_response && active_station_request_id_[0] != '\0')
    {
        emitStationRespStage(active_station_request_id_, 1);
        clearActiveStationRequest();
    }
}

void ComputeController::handleStationRequest(const char *line,
                                             uint32_t now_ms,
                                             const HealthReport &report,
                                             const protocol::StepperStatusPayload &stepper_status)
{
    char request_id[16]{};
    if (!parseStationRequestId(line, request_id, sizeof(request_id)))
    {
        return;
    }

    char base_request_id[8]{};
    int16_t request_args[3]{};
    size_t request_arg_count = 0;
    if (!splitRequestId(request_id,
                        base_request_id,
                        sizeof(base_request_id),
                        request_args,
                        3,
                        &request_arg_count))
    {
        emitStationErr(request_id, "BAD_REQUEST_ID");
        return;
    }

    if (strcmp(base_request_id, "0") == 0)
    {
        emitSystemSummary(now_ms, report, stepper_status);
        return;
    }

    if (strcmp(base_request_id, "11") == 0 || strcmp(base_request_id, "12") == 0)
    {
        const station::KranActionRequest request{
            strcmp(base_request_id, "11") == 0
                ? station::KranRequestType::Home
                : station::KranRequestType::Throw,
            static_cast<uint8_t>((request_arg_count == 0) ? 0 : request_args[0])};

        const station::KranOutputs outputs = kran_fsm_.update(
            buildKranInputs(now_ms, stepper_status),
            &request);

        if (outputs.emit_error)
        {
            emitStationErr(request_id, kranErrorCodeToString(outputs.error_code));
            return;
        }

        if (!copyRequestId(active_station_request_id_,
                           sizeof(active_station_request_id_),
                           request_id))
        {
            emitStationErr(request_id, "BAD_REQUEST_ID");
            clearActiveStationRequest();
            return;
        }

        if (outputs.emit_first_response)
        {
            emitStationRespStage(request_id, 0);
        }

        if (outputs.command != station::KranCommand::None &&
            !dispatchKranCommand(outputs.command, outputs.command_arg))
        {
            emitStationErr(request_id, "STEPPER_WRITE_FAIL");
            clearActiveStationRequest();
        }
        return;
    }

    if (strcmp(base_request_id, "30") == 0)
    {
        if (request_arg_count != 0)
        {
            emitStationErr(request_id, "BAD_REQUEST_ID");
            return;
        }

        int16_t slot_targets_mm[kStationBoomSlotCount]{};
        const char *error_code = nullptr;
        if (!readAllBoomSlots(now_ms, slot_targets_mm, error_code))
        {
            emitStationErr(request_id,
                           (error_code != nullptr) ? error_code
                                                   : "STEPPER_READ_FAIL");
            return;
        }

        emitBoomSlots(request_id, slot_targets_mm);
        return;
    }

    if (strcmp(base_request_id, "31") == 0)
    {
        if (request_arg_count != 2)
        {
            emitStationErr(request_id, "BAD_REQUEST_ID");
            return;
        }

        const char *error_code = nullptr;
        if (!writeBoomSlot(now_ms,
                           static_cast<uint8_t>(request_args[0]),
                           request_args[1],
                           error_code))
        {
            emitStationErr(request_id,
                           (error_code != nullptr) ? error_code
                                                   : "STEPPER_WRITE_FAIL");
            return;
        }

        emitStationResp(request_id);
        return;
    }

    if (isImmediateStageStubRequest(base_request_id))
    {
        emitStationRespStage(request_id, 0);
        emitStationRespStage(request_id, 1);
        return;
    }

    if (isSimpleAckRequest(base_request_id))
    {
        emitStationResp(request_id);
        return;
    }

    emitStationErr(request_id, "UNSUPPORTED");
}

void ComputeController::pollStationLink(uint32_t now_ms,
                                        const HealthReport &report,
                                        const protocol::StepperStatusPayload &stepper_status)
{
    while (Serial.available() > 0)
    {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\r')
        {
            continue;
        }

        if (ch == '\n')
        {
            if (station_rx_length_ > 0)
            {
                station_rx_buffer_[station_rx_length_] = '\0';
                handleStationRequest(station_rx_buffer_, now_ms, report, stepper_status);
                station_rx_length_ = 0;
            }
            continue;
        }

        if (station_rx_length_ + 1U >= sizeof(station_rx_buffer_))
        {
            station_rx_length_ = 0;
            continue;
        }

        station_rx_buffer_[station_rx_length_++] = ch;
    }
}

void ComputeController::probeImu()
{
    imu_.begin(Wire, config::kImuI2cAddress);
}

void ComputeController::publishImuState(uint32_t now_ms)
{
    if (now_ms - last_imu_publish_ms_ < kImuPublishPeriodMs)
    {
        return;
    }

    last_imu_publish_ms_ = now_ms;
    if (tx_queue_count_ >= (kTxQueueDepth - 1))
    {
        return;
    }

    uint8_t payload[10]{};
    uint8_t flags = 0;
    if (imu_.healthy())
    {
        flags |= 0x01;
    }
    if (imu_.dataValid())
    {
        flags |= 0x02;
    }
    if (imu_.calibrating())
    {
        flags |= 0x04;
    }
    if (imu_.tiltFault())
    {
        flags |= 0x08;
    }

    payload[0] = flags;
    encodeInt16Le(&payload[1], clampInt16(lroundf(imu_.yawDeg() * 100.0f)));
    encodeInt16Le(&payload[3], clampInt16(lroundf(imu_.yawRateDps() * 10.0f)));
    encodeInt16Le(&payload[5], clampInt16(lroundf(imu_.rollDeg() * 100.0f)));
    encodeInt16Le(&payload[7], clampInt16(lroundf(imu_.pitchDeg() * 100.0f)));

    const uint32_t age_ms = (imu_.lastSampleMs() == 0) ? 255U : (now_ms - imu_.lastSampleMs());
    payload[9] = static_cast<uint8_t>((age_ms > 255U) ? 255U : age_ms);

    sendMessage(protocol::MessageType::IMU_STATE, payload, sizeof(payload));
}

void ComputeController::updateStatusLed(uint32_t now_ms)
{
    if (!config::kEnableStatusLed)
    {
        return;
    }

    bool on = false;

    const HealthReport &r = self_check_.report();
    const bool ss_inactive =
        (!config::kSpiUseExternalSs) &&
        (!config::kForceSsOutputLowInNoSsMode) &&
        (digitalRead(SS) == HIGH);

    if (ss_inactive)
    {
        // Four pulses: SPI slave SS is not held low, AVR slave interface inactive.
        on = pulseTrain(now_ms, 4);
    }
    else if (wire_timeout_seen_)
    {
        on = (now_ms % kLedFastBlinkMs) < (kLedFastBlinkMs / 2);
    }
    else if (last_rx_ms_ == 0)
    {
        // One pulse: no SPI frames received yet from PRIZM.
        on = pulseTrain(now_ms, 1);
    }
    else if (now_ms - last_rx_ms_ > kRxTimeoutMs)
    {
        // Two pulses: link lost / stale packets.
        on = pulseTrain(now_ms, 2);
    }
    else if (!r.startup_ready)
    {
        // Three pulses: SPI link is alive, but self-check not ready.
        on = pulseTrain(now_ms, 3);
    }
    else
    {
        // Solid ON: link active and startup-ready.
        on = true;
    }

    digitalWrite(config::kStatusLedPin, on ? HIGH : LOW);
}

void ComputeController::traceBoot(const __FlashStringHelper *msg)
{
    if (!config::kEnableBootSerialTrace)
    {
        return;
    }
    Serial.println(msg);
}

} // namespace app
} // namespace robot
