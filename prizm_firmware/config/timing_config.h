#pragma once

namespace robot
{
namespace config
{
constexpr uint32_t kLoopPeriodMs = 5;
constexpr uint32_t kTxTimeoutMs = 40;
constexpr uint32_t kHeartbeatPeriodMs = 100;
constexpr uint32_t kLinkDeadMs = 500;
constexpr uint32_t kHelloRetryMs = 200;
constexpr uint8_t kMaxRetries = 3;
constexpr uint32_t kRcFlsafeMs = 400;

} // namespace config
} // namespace robot
