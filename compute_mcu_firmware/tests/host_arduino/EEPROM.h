#pragma once

#include <cstring>
#include <cstdint>

class EEPROMClass
{
public:
    template <typename T>
    void get(int address, T &out) const
    {
        std::memcpy(&out, storage_ + address, sizeof(T));
    }

    template <typename T>
    void put(int address, const T &value)
    {
        std::memcpy(storage_ + address, &value, sizeof(T));
    }

    void clear()
    {
        std::memset(storage_, 0, sizeof(storage_));
    }

private:
    uint8_t storage_[1024]{};
};

inline EEPROMClass EEPROM{};
