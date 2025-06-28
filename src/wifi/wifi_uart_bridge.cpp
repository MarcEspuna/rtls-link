#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "wifi_uart_bridge.hpp"

#include "bsp/board.hpp"


WifiUartBridge::WifiUartBridge(HardwareSerial &serial, IPAddress gsc_ip, uint16_t local_udp_port)
    : m_Serial(serial), gsc_ip(gsc_ip), m_UdpPort(local_udp_port), GSCIp(gsc_ip)
{
    m_Udp.begin(m_UdpPort);

    const auto& uart_pins = bsp::kBoardConfig.mavlink_uart;
    m_Serial.begin(921600, SERIAL_8N1, uart_pins.rx_pin, uart_pins.tx_pin);
}

/**
 * @brief Maybe around 50 to 100hz update rate could be ok?
 * 
 */
void WifiUartBridge::Update()
{
    // Only process packets if WiFi is connected in STATION mode or if in AP mode (status check might not be strictly needed for AP)
    if (WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP) {
        int packetSize = m_Udp.parsePacket(); 
        if (packetSize ) {  // Forward from GSC to UART(Ardupilot FC)
          // Check remote IP only if we received a packet
          if (m_Udp.remoteIP() == GSCIp) {
            // receive incoming UDP packets
            int len = m_Udp.read(incomingPacket, max_packet_size);
            m_Serial.write(incomingPacket, len);
          } else {
            // Discard packet from unknown source
            m_Udp.flush();
          }
        }

        // Check if data is available on the Serial1 port
        if (m_Serial.available()) {
            // read the incoming byte:
            size_t len = m_Serial.read(incomingSerialPacket, max_packet_size);
            // Send the byte via UDP *only if connected*
            m_Udp.beginPacket(GSCIp, m_UdpPort);
            m_Udp.write(incomingSerialPacket, len);
            m_Udp.endPacket();
        }
    } else {
      // Optional: If WiFi is not connected, clear any pending serial data to prevent buffer buildup?
      // Or simply do nothing and let the data be dropped implicitly by not sending.
      // Clearing might be safer if large amounts of serial data could arrive while disconnected.
      while (m_Serial.available()) {
        m_Serial.read(); // Read and discard
      }
    }

}