#pragma once

#include <cctype>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace control_station
{
namespace master
{
enum class ParsedSlaveMessageKind
{
    Response,
    Error,
    SystemSummary,
    BoomSlots
};

struct SystemSummary
{
    int robot = 0;
    int pre = 0;
    int crsf = 0;
    int batt = 0;
    int voltage_cV = 0;
};

struct ParsedSlaveMessage
{
    ParsedSlaveMessageKind kind = ParsedSlaveMessageKind::Response;
    std::string request_id;
    std::optional<int> state;
    std::string raw;
    std::string error_code;
    std::optional<SystemSummary> summary;
    std::vector<int> boom_slots;
};

inline bool isValidRequestId(std::string_view request_id)
{
    if (request_id.empty())
    {
        return false;
    }

    bool previous_was_underscore = false;
    for (const char ch : request_id)
    {
        const unsigned char byte = static_cast<unsigned char>(ch);
        if (std::isdigit(byte))
        {
            previous_was_underscore = false;
            continue;
        }

        if (ch == '_')
        {
            if (previous_was_underscore)
            {
                return false;
            }

            previous_was_underscore = true;
            continue;
        }

        return false;
    }

    return !previous_was_underscore;
}

inline bool isValidErrorCode(std::string_view error_code)
{
    if (error_code.empty())
    {
        return false;
    }

    for (const char ch : error_code)
    {
        const unsigned char byte = static_cast<unsigned char>(ch);
        if (std::isalnum(byte) || ch == '_')
        {
            continue;
        }
        return false;
    }

    return true;
}

inline std::optional<int> parseSummaryValue(std::string_view value)
{
    if (value.empty())
    {
        return std::nullopt;
    }

    int parsed = 0;
    for (const char ch : value)
    {
        const unsigned char byte = static_cast<unsigned char>(ch);
        if (!std::isdigit(byte))
        {
            return std::nullopt;
        }
        parsed = (parsed * 10) + (ch - '0');
    }
    return parsed;
}

inline std::optional<ParsedSlaveMessage> parseRespMessage(std::string_view line)
{
    static constexpr std::string_view prefix = "RESP,";
    const std::string_view payload = line.substr(prefix.size());
    const size_t comma_pos = payload.find(',');

    if (comma_pos == std::string_view::npos)
    {
        if (!isValidRequestId(payload))
        {
            return std::nullopt;
        }

        ParsedSlaveMessage parsed;
        parsed.kind = ParsedSlaveMessageKind::Response;
        parsed.request_id = std::string(payload);
        parsed.raw = std::string(line);
        return parsed;
    }

    const std::string_view request_id = payload.substr(0, comma_pos);
    const std::string_view state_part = payload.substr(comma_pos + 1);
    if (!isValidRequestId(request_id) || state_part.size() != 1U ||
        (state_part != "0" && state_part != "1"))
    {
        return std::nullopt;
    }

    ParsedSlaveMessage parsed;
    parsed.kind = ParsedSlaveMessageKind::Response;
    parsed.request_id = std::string(request_id);
    parsed.state = (state_part == "0") ? 0 : 1;
    parsed.raw = std::string(line);
    return parsed;
}

inline std::optional<ParsedSlaveMessage> parseErrMessage(std::string_view line)
{
    static constexpr std::string_view prefix = "ERR,";
    const std::string_view payload = line.substr(prefix.size());
    const size_t comma_pos = payload.find(',');
    if (comma_pos == std::string_view::npos)
    {
        return std::nullopt;
    }

    const std::string_view request_id = payload.substr(0, comma_pos);
    const std::string_view error_code = payload.substr(comma_pos + 1);
    if (!isValidRequestId(request_id) || !isValidErrorCode(error_code))
    {
        return std::nullopt;
    }

    ParsedSlaveMessage parsed;
    parsed.kind = ParsedSlaveMessageKind::Error;
    parsed.request_id = std::string(request_id);
    parsed.error_code = std::string(error_code);
    parsed.raw = std::string(line);
    return parsed;
}

inline std::optional<ParsedSlaveMessage> parseStatMessage(std::string_view line)
{
    static constexpr std::string_view prefix = "STAT,";
    const std::string_view payload = line.substr(prefix.size());
    const size_t first_comma = payload.find(',');
    if (first_comma == std::string_view::npos)
    {
        return std::nullopt;
    }

    const std::string_view request_id = payload.substr(0, first_comma);
    if (!isValidRequestId(request_id))
    {
        return std::nullopt;
    }

    SystemSummary summary{};
    bool seen_robot = false;
    bool seen_pre = false;
    bool seen_crsf = false;
    bool seen_batt = false;
    bool seen_voltage = false;

    std::string_view cursor = payload.substr(first_comma + 1);
    while (!cursor.empty())
    {
        const size_t next_comma = cursor.find(',');
        const std::string_view token =
            next_comma == std::string_view::npos ? cursor : cursor.substr(0, next_comma);

        const size_t equal_pos = token.find('=');
        if (equal_pos == std::string_view::npos)
        {
            return std::nullopt;
        }

        const std::string_view key = token.substr(0, equal_pos);
        const std::string_view value = token.substr(equal_pos + 1);
        const std::optional<int> parsed_value = parseSummaryValue(value);
        if (!parsed_value.has_value())
        {
            return std::nullopt;
        }

        if (key == "robot")
        {
            summary.robot = *parsed_value;
            seen_robot = true;
        }
        else if (key == "pre")
        {
            summary.pre = *parsed_value;
            seen_pre = true;
        }
        else if (key == "crsf")
        {
            summary.crsf = *parsed_value;
            seen_crsf = true;
        }
        else if (key == "batt")
        {
            summary.batt = *parsed_value;
            seen_batt = true;
        }
        else if (key == "v")
        {
            summary.voltage_cV = *parsed_value;
            seen_voltage = true;
        }
        else
        {
            return std::nullopt;
        }

        if (next_comma == std::string_view::npos)
        {
            break;
        }
        cursor.remove_prefix(next_comma + 1);
    }

    if (!seen_robot || !seen_pre || !seen_crsf || !seen_batt || !seen_voltage)
    {
        return std::nullopt;
    }

    ParsedSlaveMessage parsed;
    parsed.kind = ParsedSlaveMessageKind::SystemSummary;
    parsed.request_id = std::string(request_id);
    parsed.raw = std::string(line);
    parsed.summary = summary;
    return parsed;
}

inline std::optional<ParsedSlaveMessage> parseSlotsMessage(std::string_view line)
{
    static constexpr std::string_view prefix = "SLOTS,";
    const std::string_view payload = line.substr(prefix.size());
    const size_t first_comma = payload.find(',');
    if (first_comma == std::string_view::npos)
    {
        return std::nullopt;
    }

    const std::string_view request_id = payload.substr(0, first_comma);
    if (!isValidRequestId(request_id))
    {
        return std::nullopt;
    }

    std::vector<int> boom_slots;
    boom_slots.reserve(11);
    std::string_view cursor = payload.substr(first_comma + 1);
    while (!cursor.empty())
    {
        const size_t next_comma = cursor.find(',');
        const std::string_view token =
            next_comma == std::string_view::npos ? cursor : cursor.substr(0, next_comma);
        const std::optional<int> parsed_value = parseSummaryValue(token);
        if (!parsed_value.has_value())
        {
            return std::nullopt;
        }

        boom_slots.push_back(*parsed_value);
        if (next_comma == std::string_view::npos)
        {
            break;
        }
        cursor.remove_prefix(next_comma + 1);
    }

    if (boom_slots.size() != 11)
    {
        return std::nullopt;
    }

    ParsedSlaveMessage parsed;
    parsed.kind = ParsedSlaveMessageKind::BoomSlots;
    parsed.request_id = std::string(request_id);
    parsed.raw = std::string(line);
    parsed.boom_slots = std::move(boom_slots);
    return parsed;
}

inline std::optional<ParsedSlaveMessage> parseSlaveMessage(std::string_view line)
{
    if (line.rfind("RESP,", 0) == 0)
    {
        return parseRespMessage(line);
    }

    if (line.rfind("ERR,", 0) == 0)
    {
        return parseErrMessage(line);
    }

    if (line.rfind("STAT,", 0) == 0)
    {
        return parseStatMessage(line);
    }

    if (line.rfind("SLOTS,", 0) == 0)
    {
        return parseSlotsMessage(line);
    }

    return std::nullopt;
}

} // namespace master
} // namespace control_station
