#pragma once

#include "config/features.hpp"

#if defined(USE_MAVLINK) && defined(USE_UWB_MODE_TDOA_TAG)

#include <Arduino.h>
#include <etl/array.h>
#include <etl/span.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "uwb/uwb_params.hpp"

namespace ardupilot {

class RTLSLinkBeaconProtocol {
public:
    static void Init();
    static void Update();
    static void SetAnchors(etl::span<const UWBAnchorParam> anchors);
    static void SendTDoA(uint8_t anchor_a, uint8_t anchor_b, float distance_diff_m, float sigma_m);
    static void SendPosition(float x_m, float y_m, float z_m);
    static bool ConfigAccepted();

private:
    struct PendingPosition {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        bool valid = false;
    };

    static constexpr uint8_t FRAME_MAGIC_1 = 0x52;
    static constexpr uint8_t FRAME_MAGIC_2 = 0x42;
    static constexpr uint8_t PROTOCOL_VERSION = 1;
    static constexpr uint8_t PAYLOAD_LEN_MAX = 32;
    static constexpr uint8_t PENDING_POSITION_COUNT = 8;

    enum class MsgId : uint8_t {
        HELLO = 1,
        ANCHOR = 2,
        POSITION = 3,
        TDOA = 4,
        CONFIG_END = 5,
        ACK = 0x80,
    };

    enum class AckStatus : uint8_t {
        OK = 0,
        UNSUPPORTED_VERSION = 1,
        BAD_CONFIG = 2,
        BAD_FRAME = 3,
    };

    enum class ParseState : uint8_t {
        MAGIC_1,
        MAGIC_2,
        MSG_ID,
        LEN,
        SEQ,
        PAYLOAD,
        CRC_LOW,
        CRC_HIGH,
    };

    static void send_config();
    static void send_frame(MsgId msg_id, const uint8_t* payload, uint8_t payload_len);
    static void send_anchor(const UWBAnchorParam& anchor, uint8_t anchor_id);
    static void send_position_frame(float x_m, float y_m, float z_m);
    static void queue_position(float x_m, float y_m, float z_m);
    static void flush_pending_positions();
    static bool position_enabled_now();
    static void process_rx_byte(uint8_t b);
    static void reset_parser();
    static void handle_ack();
    static uint16_t crc16_update(uint16_t crc, uint8_t b);
    static int32_t meters_to_mm(float meters);
    static uint16_t meters_to_u16_mm(float meters);
    static void write_i32_le(uint8_t* p, int32_t v);
    static void write_u16_le(uint8_t* p, uint16_t v);

    static SemaphoreHandle_t tx_mutex;
    static etl::array<UWBAnchorParam, UWBParams::maxAnchorCount> anchor_cache;
    static uint8_t anchor_count;
    static bool config_accepted;
    static uint32_t config_accepted_ms;
    static uint32_t last_config_tx_ms;
    static uint8_t tx_seq;

    static etl::array<PendingPosition, PENDING_POSITION_COUNT> pending_positions;
    static uint8_t pending_position_next;

    static ParseState parse_state;
    static uint8_t rx_msg_id;
    static uint8_t rx_payload_len;
    static uint8_t rx_seq;
    static uint8_t rx_payload[PAYLOAD_LEN_MAX];
    static uint8_t rx_payload_idx;
    static uint16_t rx_crc;
    static uint16_t rx_frame_crc;
};

} // namespace ardupilot

#endif
