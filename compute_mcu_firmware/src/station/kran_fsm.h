#pragma once

#include <stdint.h>

namespace robot
{
namespace station
{
enum class KranState : uint8_t
{
    NotReady = 0,
    ReadyHome = 1,
    HomeOnlyActive = 2,
    ThrowExtend = 3,
    ThrowDwell = 4,
    ThrowHome = 5,
    FaultLatched = 6
};

enum class StepperMotionState : uint8_t
{
    Idle = 0,
    Moving = 1,
    Homing = 2,
    Fault = 3
};

enum class KranRequestType : uint8_t
{
    Home = 0,
    Throw = 1
};

enum class KranCommand : uint8_t
{
    None = 0,
    Home = 1,
    GotoSlot = 2
};

enum class KranErrorCode : uint8_t
{
    None = 0,
    Busy = 1,
    NotReady = 2,
    InvalidSlot = 3,
    StepperFault = 4
};

struct KranInputs
{
    uint32_t now_ms = 0;
    bool start_gate_open = false;
    bool prizm_summary_fresh = false;
    bool stepper_online = false;
    bool stepper_homed = false;
    bool home_switch_active = false;
    bool stepper_fault = false;
    StepperMotionState stepper_state = StepperMotionState::Idle;
    int16_t current_position_mm = 0;
    int16_t target_position_mm = 0;
};

struct KranActionRequest
{
    KranRequestType type = KranRequestType::Home;
    uint8_t slot = 0;
};

struct KranOutputs
{
    KranCommand command = KranCommand::None;
    int16_t command_arg = 0;
    bool emit_first_response = false;
    bool emit_second_response = false;
    bool emit_error = false;
    KranErrorCode error_code = KranErrorCode::None;
};

class KranFsm
{
public:
    KranOutputs update(const KranInputs &inputs, const KranActionRequest *request);
    KranState state() const;
    bool hasActiveRequest() const;
    KranRequestType activeRequestType() const;
    uint8_t activeSlot() const;

private:
    static constexpr uint8_t kMaxSlot = 10;
    static constexpr uint32_t kThrowDwellMs = 2000;

    bool readyForHome(const KranInputs &inputs) const;
    bool readyForCommand(const KranInputs &inputs) const;
    bool isIdleState() const;
    bool targetReached(const KranInputs &inputs) const;
    bool throwMotionObserved(const KranInputs &inputs);

    KranState state_ = KranState::NotReady;
    bool active_request_ = false;
    KranRequestType active_type_ = KranRequestType::Home;
    uint8_t active_slot_ = 0;
    uint32_t dwell_started_ms_ = 0;
    // True once the stepper has observably entered HOMING since our last
    // dispatched CMD_HOME. Used to ignore stale fault_flags during the I2C
    // round-trip between dispatching the command and the stepper clearing
    // the fault in beginHomeSequence().
    bool home_recovery_observed_ = true;
    bool throw_motion_observed_ = false;
    int16_t throw_start_position_mm_ = 0;
    int16_t throw_start_target_mm_ = 0;
};

} // namespace station
} // namespace robot
