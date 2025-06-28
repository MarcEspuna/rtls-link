#include "beacon_protocool.hpp"
#include "bsp/board.hpp"

#include "app.hpp"

namespace bcn_konex {

// Static member initialization
etl::array<double, 12> BeaconProtocol::s_locations_to_echo = {};
SemaphoreHandle_t BeaconProtocol::s_anchor_mutex = NULL;

// Initialize the serial communication
void BeaconProtocol::Init() {
    s_anchor_mutex = xSemaphoreCreateMutex();
}

void BeaconProtocol::SetAnchorsToEcho(const etl::array<double, 12>& anchor_locations) {
    xSemaphoreTake(s_anchor_mutex, portMAX_DELAY);
    s_locations_to_echo = anchor_locations;
    xSemaphoreGive(s_anchor_mutex);
}

void BeaconProtocol::SendAddAnchorEcho() {
    HardwareSerial& serial = App::GetArdupilotSerial();

    static int32_t last_echo = 0;

    // Send the anchors to echo every ticks seconds
    if (last_echo < 1) {
        xSemaphoreTake(s_anchor_mutex, portMAX_DELAY);
        uint8_t anchorId = 0;
        for (uint32_t i = 0; i < s_locations_to_echo.size(); i += 3) {
            RelativeLocation loc = {
                static_cast<float>(s_locations_to_echo[i]), 
                static_cast<float>(s_locations_to_echo[i+1]), 
                static_cast<float>(s_locations_to_echo[i+2])
            };
            
            AddAnchor frame = {PREAMBLE, HDR_ADD_ANCHOR, anchorId, loc, 0xFFFF}; // Example values for preamble, header, and crc
            
            // --- Logging --- 
            // printf("[BeaconProtocol LOG] SendAddAnchorEcho - ID: %d, X: %.2f, Y: %.2f, Z: %.2f\n\r", 
            //        anchorId, loc.x__m, loc.y__m, loc.z__m);
            // ---------------
            
            serial.write((uint8_t*)&frame, sizeof(frame));
            anchorId++;
        }
        xSemaphoreGive(s_anchor_mutex);
        last_echo = 5;  // Every 5 updates (0.5 seconds at 10Hz)
    }
    last_echo--;
}

void BeaconProtocol::ACKListener() {
    HardwareSerial& serial = App::GetArdupilotSerial();
    // For now we will simply flush the data reception. We don't care about the ACK
    if (serial.available()) {
        char receivedChar = serial.read(); // Read the incoming character
        // ACK received 
    }
}

void BeaconProtocol::SendSample(float x_m, float y_m, float z_m, uint16_t error) {
    HardwareSerial& serial = App::GetArdupilotSerial();
    Sample frame = {PREAMBLE, HDR_SAMPLE, {x_m, y_m, z_m}, error, 0xFFFF}; // Example values for preamble, header, and crc
    serial.write((uint8_t*)&frame, sizeof(frame));
}

void BeaconProtocol::SendRangeSample(uint8_t id, float range) {
    HardwareSerial& serial = App::GetArdupilotSerial();
    // Check if the id is valid
    if (id >= 4) {
        return;
    }

    RangeSample frame = {PREAMBLE, HDR_RANGE_SAMPLE, id, range, 0xFFFF}; // Example values for preamble, header, and crc
    serial.write((uint8_t*)&frame, sizeof(frame));
}

} // namespace bcn_konex
