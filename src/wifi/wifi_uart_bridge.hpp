#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

#include "wifi_backend.hpp"

/**
 * @brief It basically needs a Serial port and a UDP port and IP address to forward the communication between the two.
 * 
 */

class WifiUartBridge : public WifiBackend {
private: 
public:
    WifiUartBridge(HardwareSerial &serial, IPAddress gsc_ip, uint16_t local_udp_port);

    void Update() override;

private:
    static constexpr uint32_t max_packet_size = 1024;
private:
    HardwareSerial& m_Serial;
    IPAddress gsc_ip;
    uint16_t m_UdpPort;
    WiFiUDP m_Udp;
    uint8_t incomingPacket[max_packet_size];
    uint8_t incomingSerialPacket[max_packet_size];
    const IPAddress GSCIp;

    // Auto-discovery and buffering
    IPAddress m_TargetIp;
    uint32_t m_LastSendTime = 0;
    uint16_t m_BufferIndex = 0;
    static constexpr uint16_t kBufferThreshold = 256; // Send if buffer exceeds this
    static constexpr uint32_t kTimeThresholdMs = 10;  // Send if older than this

};