#ifndef SERIAL_COMM_INTERFACE_HPP
#define SERIAL_COMM_INTERFACE_HPP

#include <cstdint>
#include <cstddef> // For size_t

/**
 * @brief Abstract interface for serial communication.
 *
 * Implementations of this interface will handle the actual byte transmission
 * (e.g., UART, UDP sockets, console printing).
 */
class ISerialComm {
public:
    virtual ~ISerialComm() = default; // Virtual destructor is important for interfaces

    /**
     * @brief Sends an array of bytes over the communication channel.
     * @param buffer Pointer to the data buffer to send.
     * @param length Number of bytes to send from the buffer.
     * @return true if sending was successful (or queued successfully), false otherwise.
     */
    virtual bool send_bytes(const uint8_t* buffer, size_t length) = 0;

    /**
     * @brief Gets the current time in microseconds since boot or epoch.
     * 
     * This function allows hardware-specific time implementations.
     * For embedded systems without std::chrono, this can be implemented
     * using hardware timers or system ticks.
     * 
     * @return Current timestamp in microseconds
     */
    virtual uint64_t get_time_us() = 0;

    // Add virtual receive_bytes function later if needed for two-way communication
    // virtual size_t receive_bytes(uint8_t* buffer, size_t max_length) = 0;
};

#endif // SERIAL_COMM_INTERFACE_HPP 