#include "station/hc12_backend.h"

namespace robot
{
namespace station
{
void Hc12Backend::begin(Stream &stream)
{
    stream_ = &stream;
    available_ = false;
}

void Hc12Backend::poll()
{
    available_ = (stream_ != nullptr) && (stream_->available() > 0);
}

bool Hc12Backend::available() const
{
    return available_;
}

} // namespace station
} // namespace robot
