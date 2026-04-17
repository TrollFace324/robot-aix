#include "rc_crsf.h"

namespace
{
constexpr uint8_t CRSF_ADDRESS_CRSF_RECEIVER = 0xEC;

constexpr uint8_t CRSF_FRAME_RC_CHANNELS_PACKED = 0x16;
constexpr uint8_t CRSF_FRAME_SUBSET_RC_CHANNELS_PACKED = 0x17;
constexpr uint8_t CRSF_FRAME_BATTERY_SENSOR = 0x08;
constexpr uint8_t CRSF_RC_PAYLOAD_SIZE = 22;
constexpr uint8_t CRSF_RC_PAYLOAD_SIZE_WITH_STATUS = 23;
constexpr uint8_t CRSF_BATTERY_REMAINING_MAX = 100;

constexpr uint8_t CH_RIGHT_X = 0; // CH1
constexpr uint8_t CH_RIGHT_Y = 1; // CH2
constexpr uint8_t CH_LEFT_X = 3;  // CH4

constexpr uint16_t CRSF_MIN = 172;
constexpr uint16_t CRSF_MID = 992;
constexpr uint16_t CRSF_MAX = 1811;

bool isLikelyCrsfAddress(uint8_t addr)
{
    // Common CRSF addresses seen on UART links.
    return addr == 0xC8 || addr == 0xEA || addr == 0xEC || addr == 0xEE;
}
} // namespace

void RcCrsf::begin(HardwareSerial &serial, uint32_t baud)
{
    serial_ = &serial;
    serial_->begin(baud);
    memset(&last_, 0, sizeof(last_));
    for (uint8_t i = 0; i < 16; ++i)
    {
        last_.raw[i] = CRSF_MID;
    }
    last_byte_ms_ = 0;
}

bool RcCrsf::poll(RcChannels &out)
{
    if (!serial_)
    {
        return false;
    }

    bool got_new_frame = false;

    while (serial_->available() > 0)
    {
        const uint8_t byte_in = static_cast<uint8_t>(serial_->read());
        last_byte_ms_ = millis();
        if (index_ == 0)
        {
            if (!isLikelyCrsfAddress(byte_in))
            {
                continue;
            }
            frame_[index_++] = byte_in;
            continue;
        }

        if (index_ == 1)
        {
            if (byte_in < 2 || byte_in > 62)
            {
                index_ = 0;
                expected_size_ = 0;
                continue;
            }
            frame_[index_++] = byte_in;
            expected_size_ = static_cast<uint8_t>(byte_in + 2);
            continue;
        }

        frame_[index_++] = byte_in;
        if (index_ >= expected_size_)
        {
            RcChannels parsed = last_;
            if (handleFrame(frame_, expected_size_, parsed))
            {
                last_ = parsed;
                out = last_;
                got_new_frame = true;
            }
            index_ = 0;
            expected_size_ = 0;
        }
    }

    if (!got_new_frame)
    {
        out = last_;
    }
    return got_new_frame;
}

bool RcCrsf::hasByteActivity(uint32_t timeout_ms) const
{
    if (last_byte_ms_ == 0)
    {
        return false;
    }
    return (millis() - last_byte_ms_) <= timeout_ms;
}

void RcCrsf::sendBatteryTelemetry(uint16_t voltage_cV,
                                  uint16_t current_dA,
                                  uint32_t capacity_mAh,
                                  uint8_t remaining_pct)
{
    uint8_t payload[8];
    // 0x08 Battery Sensor payload (big-endian):
    // Voltage and current use 0.1 units in most radios (0.1V and 0.1A).
    // PRIZM returns centivolts, so scale down by 10.
    const uint16_t voltage_dV = static_cast<uint16_t>((static_cast<uint32_t>(voltage_cV) + 5U) / 10U);
    payload[0] = static_cast<uint8_t>(voltage_dV >> 8);
    payload[1] = static_cast<uint8_t>(voltage_dV & 0xFF);
    payload[2] = static_cast<uint8_t>(current_dA >> 8);
    payload[3] = static_cast<uint8_t>(current_dA & 0xFF);

    if (capacity_mAh > 0xFFFFFFUL)
    {
        capacity_mAh = 0xFFFFFFUL;
    }
    payload[4] = static_cast<uint8_t>((capacity_mAh >> 16) & 0xFF);
    payload[5] = static_cast<uint8_t>((capacity_mAh >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>(capacity_mAh & 0xFF);
    payload[7] = (remaining_pct > CRSF_BATTERY_REMAINING_MAX) ? CRSF_BATTERY_REMAINING_MAX : remaining_pct;

    writeFrame(CRSF_ADDRESS_CRSF_RECEIVER, CRSF_FRAME_BATTERY_SENSOR, payload, sizeof(payload));
}

bool RcCrsf::handleFrame(const uint8_t *frame, uint8_t size, RcChannels &out)
{
    if (size < 5)
    {
        return false;
    }

    const uint8_t len = frame[1];
    if (size != static_cast<uint8_t>(len + 2))
    {
        return false;
    }

    const uint8_t type = frame[2];
    const uint8_t *data = &frame[3];
    const uint8_t data_len_without_crc = static_cast<uint8_t>(len - 2);
    const uint8_t crc_received = frame[size - 1];
    const uint8_t crc_computed = crc8(&frame[2], static_cast<uint8_t>(len - 1));
    if (crc_received != crc_computed)
    {
        return false;
    }

    bool parsed_channels = false;
    if (type == CRSF_FRAME_RC_CHANNELS_PACKED &&
        (data_len_without_crc == CRSF_RC_PAYLOAD_SIZE || data_len_without_crc == CRSF_RC_PAYLOAD_SIZE_WITH_STATUS))
    {
        uint32_t bit_buffer = 0;
        uint8_t bits_in_buffer = 0;
        uint8_t payload_index = 0;
        // ELRS can append one optional status byte. Channel data is still first 22 bytes.
        const uint8_t channel_bytes = CRSF_RC_PAYLOAD_SIZE;
        for (uint8_t ch = 0; ch < 16; ++ch)
        {
            while (bits_in_buffer < 11 && payload_index < channel_bytes)
            {
                bit_buffer |= static_cast<uint32_t>(data[payload_index++]) << bits_in_buffer;
                bits_in_buffer = static_cast<uint8_t>(bits_in_buffer + 8);
            }

            out.raw[ch] = static_cast<uint16_t>(bit_buffer & 0x07FF);
            bit_buffer >>= 11;
            bits_in_buffer = static_cast<uint8_t>(bits_in_buffer - 11);
        }
        parsed_channels = true;
    }
    else if (type == CRSF_FRAME_SUBSET_RC_CHANNELS_PACKED && data_len_without_crc >= 2)
    {
        // Subset format used by ELRS:
        // byte0: start_channel(0..4 bits), resolution(5..6 bits), digital flag(bit7)
        const uint8_t cfg = data[0];
        const uint8_t start_channel = static_cast<uint8_t>(cfg & 0x1F);
        const uint8_t res_conf = static_cast<uint8_t>((cfg >> 5) & 0x03);
        const uint8_t bits_per_ch = static_cast<uint8_t>(10 + res_conf); // 10..13 bits
        const uint8_t payload_bytes = static_cast<uint8_t>(data_len_without_crc - 1);
        const uint8_t ch_count = static_cast<uint8_t>((payload_bytes * 8) / bits_per_ch);

        uint32_t bit_buffer = 0;
        uint8_t bits_in_buffer = 0;
        uint8_t payload_index = 1;
        const uint16_t max_subset = static_cast<uint16_t>((1U << bits_per_ch) - 1U);

        for (uint8_t i = 0; i < ch_count; ++i)
        {
            while (bits_in_buffer < bits_per_ch && payload_index < data_len_without_crc)
            {
                bit_buffer |= static_cast<uint32_t>(data[payload_index++]) << bits_in_buffer;
                bits_in_buffer = static_cast<uint8_t>(bits_in_buffer + 8);
            }

            const uint16_t subset_value = static_cast<uint16_t>(bit_buffer & max_subset);
            bit_buffer >>= bits_per_ch;
            bits_in_buffer = static_cast<uint8_t>(bits_in_buffer - bits_per_ch);

            const uint8_t ch = static_cast<uint8_t>(start_channel + i);
            if (ch < 16)
            {
                // Scale subset value into CRSF 11-bit range so downstream thresholds keep working.
                out.raw[ch] = static_cast<uint16_t>(
                    CRSF_MIN + ((static_cast<uint32_t>(subset_value) * (CRSF_MAX - CRSF_MIN)) / max_subset));
            }
        }
        parsed_channels = true;
    }

    if (!parsed_channels)
    {
        return false;
    }

    // Strict stick mapping with the current transmitter setup:
    // Right stick: X=CH1, Y=CH2
    // Left stick:  X=CH4
    // CH3 (left stick Y) is intentionally ignored.
    out.right_x = normalizeChannel(out.raw[CH_RIGHT_X]);
    out.right_y = normalizeChannel(out.raw[CH_RIGHT_Y], true);
    out.left_x = normalizeChannel(out.raw[CH_LEFT_X], true);
    out.valid = true;
    out.last_update_ms = millis();
    return true;
}

void RcCrsf::writeFrame(uint8_t dest, uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    if (!serial_)
    {
        return;
    }

    // CRSF: [dest][len][type][payload][crc], where len = type + payload + crc.
    const uint8_t len = static_cast<uint8_t>(payload_len + 2U);
    uint8_t tx[64];
    tx[0] = dest;
    tx[1] = len;
    tx[2] = type;
    for (uint8_t i = 0; i < payload_len; ++i)
    {
        tx[3 + i] = payload[i];
    }
    tx[3 + payload_len] = crc8(&tx[2], static_cast<uint8_t>(1U + payload_len));
    serial_->write(tx, static_cast<size_t>(payload_len + 4U));
}

uint8_t RcCrsf::crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b)
        {
            if (crc & 0x80)
            {
                crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

float RcCrsf::normalizeChannel(uint16_t value, bool invert)
{
    if (value < CRSF_MIN)
    {
        value = CRSF_MIN;
    }
    if (value > CRSF_MAX)
    {
        value = CRSF_MAX;
    }

    float norm = static_cast<float>(static_cast<int32_t>(value) - CRSF_MID) /
                 static_cast<float>(CRSF_MAX - CRSF_MID);
    if (norm < -1.0f)
    {
        norm = -1.0f;
    }
    if (norm > 1.0f)
    {
        norm = 1.0f;
    }
    return invert ? -norm : norm;
}
