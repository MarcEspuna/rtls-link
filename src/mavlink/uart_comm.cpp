#include "uart_comm.hpp"

UartComm::UartComm(HardwareSerial& serial) : serial_(serial) {}

bool UartComm::send_bytes(const uint8_t* data, size_t length) {
    return serial_.write(data, length) == length;
}

uint64_t UartComm::get_time_us() {
    return micros();
}