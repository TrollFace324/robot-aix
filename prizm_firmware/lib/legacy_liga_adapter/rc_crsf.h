#pragma once

#include <Arduino.h>

struct RcChannels
{
    uint16_t raw[16];
    float right_x;
    float right_y;
    float left_x;
    bool valid;
    uint32_t last_update_ms;
};

class RcCrsf
{
public:
    void begin(HardwareSerial &serial, uint32_t baud = 115200UL);
    bool poll(RcChannels &out);
    bool hasByteActivity(uint32_t timeout_ms = 250) const;
    void sendBatteryTelemetry(uint16_t voltage_cV,
                              uint16_t current_dA = 0,
                              uint32_t capacity_mAh = 0,
                              uint8_t remaining_pct = 0);

private:
    HardwareSerial *serial_ = nullptr;
    uint8_t frame_[64];
    uint8_t index_ = 0;
    uint8_t expected_size_ = 0;
    uint32_t last_byte_ms_ = 0;
    RcChannels last_{};

    bool handleFrame(const uint8_t *frame, uint8_t size, RcChannels &out);
    void writeFrame(uint8_t dest, uint8_t type, const uint8_t *payload, uint8_t payload_len);
    static uint8_t crc8(const uint8_t *data, uint8_t len);
    static float normalizeChannel(uint16_t value, bool invert = false);
};
