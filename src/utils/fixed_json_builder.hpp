#pragma once

#include <stddef.h>
#include <stdint.h>

#include <cstdio>
#include <cstring>

#include <etl/string.h>

namespace Utils {

template <size_t Capacity>
class FixedJsonBuilder {
public:
    bool Append(const char* text)
    {
        return Append(text, std::strlen(text));
    }

    bool Append(char c)
    {
        if (m_out.available() == 0) {
            m_truncated = true;
            return false;
        }
        m_out.push_back(c);
        return true;
    }

    bool AppendUnsigned(unsigned long value)
    {
        char buffer[24];
        const int written = std::snprintf(buffer, sizeof(buffer), "%lu", value);
        return AppendFormatted(buffer, sizeof(buffer), written);
    }

    bool AppendSigned(long value)
    {
        char buffer[24];
        const int written = std::snprintf(buffer, sizeof(buffer), "%ld", value);
        return AppendFormatted(buffer, sizeof(buffer), written);
    }

    bool AppendDouble(double value, uint8_t decimals)
    {
        char buffer[40];
        const int written = std::snprintf(buffer,
                                          sizeof(buffer),
                                          "%.*f",
                                          static_cast<int>(decimals),
                                          value);
        return AppendFormatted(buffer, sizeof(buffer), written);
    }

    const char* CStr() const
    {
        return m_out.c_str();
    }

    size_t Size() const
    {
        return m_out.size();
    }

    bool Truncated() const
    {
        return m_truncated;
    }

private:
    bool Append(const char* text, size_t length)
    {
        const size_t writable = length <= m_out.available() ? length : m_out.available();
        if (writable > 0) {
            m_out.append(text, writable);
        }
        if (writable != length) {
            m_truncated = true;
            return false;
        }
        return true;
    }

    bool AppendFormatted(const char* buffer, size_t bufferSize, int written)
    {
        if (written < 0 || static_cast<size_t>(written) >= bufferSize) {
            m_truncated = true;
            return false;
        }
        return Append(buffer, static_cast<size_t>(written));
    }

    etl::string<Capacity> m_out;
    bool m_truncated = false;
};

} // namespace Utils
