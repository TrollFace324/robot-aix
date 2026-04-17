#include <cassert>
#include <string>

#include "../station_protocol.h"

namespace
{
using control_station::master::ParsedSlaveMessage;
using control_station::master::ParsedSlaveMessageKind;
using control_station::master::parseSlaveMessage;

void testParsesErrorMessage()
{
    const auto parsed = parseSlaveMessage("ERR,12_4,BUSY");
    assert(parsed.has_value());
    assert(parsed->kind == ParsedSlaveMessageKind::Error);
    assert(parsed->request_id == "12_4");
    assert(parsed->error_code == "BUSY");
}

void testParsesSystemSummaryMessage()
{
    const auto parsed =
        parseSlaveMessage("STAT,0,robot=2,pre=1,crsf=0,batt=2,v=1140");
    assert(parsed.has_value());
    assert(parsed->kind == ParsedSlaveMessageKind::SystemSummary);
    assert(parsed->request_id == "0");
    assert(parsed->summary.has_value());
    assert(parsed->summary->robot == 2);
    assert(parsed->summary->pre == 1);
    assert(parsed->summary->crsf == 0);
    assert(parsed->summary->batt == 2);
    assert(parsed->summary->voltage_cV == 1140);
}

void testRejectsMalformedSystemSummary()
{
    const auto parsed = parseSlaveMessage("STAT,0,robot=2,pre=1,crsf=0,batt=2");
    assert(!parsed.has_value());
}

void testParsesBoomSlotsMessage()
{
    const auto parsed = parseSlaveMessage("SLOTS,30,110,99,88,77,66,55,44,33,22,11,0");
    assert(parsed.has_value());
    assert(parsed->request_id == "30");
    assert(parsed->boom_slots.size() == 11);
    assert(parsed->boom_slots.front() == 110);
    assert(parsed->boom_slots.back() == 0);
}

void testRejectsMalformedBoomSlotsMessage()
{
    const auto parsed = parseSlaveMessage("SLOTS,30,110,99");
    assert(!parsed.has_value());
}
} // namespace

int main()
{
    testParsesErrorMessage();
    testParsesSystemSummaryMessage();
    testRejectsMalformedSystemSummary();
    testParsesBoomSlotsMessage();
    testRejectsMalformedBoomSlotsMessage();
    return 0;
}
