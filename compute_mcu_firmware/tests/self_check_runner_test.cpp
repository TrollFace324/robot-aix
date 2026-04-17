#include <cassert>
#include <cstdint>

#include "app/self_check_runner.h"

namespace
{
class FakeImu final : public robot::IImuService
{
public:
    bool healthy_value = true;
    bool data_valid_value = false;
    bool calibrating_value = true;
    bool tilt_fault_value = false;

    bool begin(TwoWire &, uint8_t) override
    {
        return true;
    }

    void update(uint32_t) override {}

    bool healthy() const override
    {
        return healthy_value;
    }

    bool dataValid() const override
    {
        return data_valid_value;
    }

    bool calibrating() const override
    {
        return calibrating_value;
    }

    bool tiltFault() const override
    {
        return tilt_fault_value;
    }

    uint32_t lastSampleMs() const override
    {
        return data_valid_value ? 250U : 0U;
    }

    float yawDeg() const override
    {
        return 0.0f;
    }

    float yawRateDps() const override
    {
        return 0.0f;
    }

    float rollDeg() const override
    {
        return 0.0f;
    }

    float pitchDeg() const override
    {
        return 0.0f;
    }
};

class FakeStepper final : public robot::IStepperLink
{
public:
    bool request_status_result = true;
    bool online_value = true;
    robot::protocol::StepperStatusPayload status_value{};
    uint32_t request_count = 0;

    void begin(TwoWire &, uint8_t) override {}

    bool requestStatus(uint32_t) override
    {
        ++request_count;
        return request_status_result;
    }

    bool sendCommand(robot::protocol::StepperCommand, int16_t, int16_t) override
    {
        return true;
    }

    bool online() const override
    {
        return online_value;
    }

    robot::protocol::StepperStatusPayload status() const override
    {
        return status_value;
    }
};

void testTiltFaultIsSuppressedUntilImuDataIsValid()
{
    FakeImu imu;
    imu.data_valid_value = false;
    imu.calibrating_value = true;
    imu.tilt_fault_value = true;

    FakeStepper stepper;
    stepper.status_value.status = robot::protocol::StepperStatusCode::STATUS_IDLE;

    robot::app::SelfCheckRunner runner;
    runner.begin(&imu, &stepper, true);
    runner.update(250);

    const robot::HealthReport &report = runner.report();
    assert(report.imu_healthy);
    assert(report.imu_calibrating);
    assert(!report.tilt_fault);
    assert(!report.imu_ok);
    assert(!report.startup_ready);
    assert((report.fault_flags & 0x04U) == 0);
    assert(stepper.request_count == 1);
}

void testTiltFaultIsReportedAfterImuBecomesValid()
{
    FakeImu imu;
    imu.data_valid_value = true;
    imu.calibrating_value = false;
    imu.tilt_fault_value = true;

    FakeStepper stepper;
    stepper.status_value.status = robot::protocol::StepperStatusCode::STATUS_IDLE;

    robot::app::SelfCheckRunner runner;
    runner.begin(&imu, &stepper, true);
    runner.update(250);

    const robot::HealthReport &report = runner.report();
    assert(report.tilt_fault);
    assert(!report.imu_ok);
    assert(!report.startup_ready);
    assert((report.fault_flags & 0x04U) != 0);
}
} // namespace

int main()
{
    testTiltFaultIsSuppressedUntilImuDataIsValid();
    testTiltFaultIsReportedAfterImuBecomesValid();
    return 0;
}
