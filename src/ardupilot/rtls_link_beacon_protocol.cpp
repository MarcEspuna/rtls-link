#include "ardupilot/rtls_link_beacon_protocol.hpp"

#if defined(USE_MAVLINK) && defined(USE_UWB_MODE_TDOA_TAG)

#include <algorithm>
#include <cmath>

#include "app.hpp"
#include "front.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"

namespace ardupilot {

SemaphoreHandle_t RTLSLinkBeaconProtocol::tx_mutex = nullptr;
etl::array<UWBAnchorParam, UWBParams::maxAnchorCount> RTLSLinkBeaconProtocol::anchor_cache = {};
uint8_t RTLSLinkBeaconProtocol::anchor_count = 0;
bool RTLSLinkBeaconProtocol::config_accepted = false;
uint32_t RTLSLinkBeaconProtocol::config_accepted_ms = 0;
uint32_t RTLSLinkBeaconProtocol::last_config_tx_ms = 0;
uint8_t RTLSLinkBeaconProtocol::tx_seq = 0;
etl::array<RTLSLinkBeaconProtocol::PendingPosition, RTLSLinkBeaconProtocol::PENDING_POSITION_COUNT> RTLSLinkBeaconProtocol::pending_positions = {};
uint8_t RTLSLinkBeaconProtocol::pending_position_next = 0;
RTLSLinkBeaconProtocol::ParseState RTLSLinkBeaconProtocol::parse_state = RTLSLinkBeaconProtocol::ParseState::MAGIC_1;
uint8_t RTLSLinkBeaconProtocol::rx_msg_id = 0;
uint8_t RTLSLinkBeaconProtocol::rx_payload_len = 0;
uint8_t RTLSLinkBeaconProtocol::rx_seq = 0;
uint8_t RTLSLinkBeaconProtocol::rx_payload[RTLSLinkBeaconProtocol::PAYLOAD_LEN_MAX] = {};
uint8_t RTLSLinkBeaconProtocol::rx_payload_idx = 0;
uint16_t RTLSLinkBeaconProtocol::rx_crc = 0xffff;
uint16_t RTLSLinkBeaconProtocol::rx_frame_crc = 0;

void RTLSLinkBeaconProtocol::Init()
{
    if (tx_mutex == nullptr) {
        tx_mutex = xSemaphoreCreateMutex();
    }
}

void RTLSLinkBeaconProtocol::Update()
{
    HardwareSerial& serial = App::GetArdupilotSerial();
    while (serial.available()) {
        const int c = serial.read();
        if (c >= 0) {
            process_rx_byte((uint8_t)c);
        }
    }

    if (!config_accepted && anchor_count > 0 && (millis() - last_config_tx_ms) >= 500) {
        send_config();
        last_config_tx_ms = millis();
    }
}

void RTLSLinkBeaconProtocol::SetAnchors(etl::span<const UWBAnchorParam> anchors)
{
    anchor_count = std::min<uint8_t>(anchors.size(), UWBParams::maxAnchorCount);
    for (uint8_t i = 0; i < anchor_count; i++) {
        anchor_cache[i] = anchors[i];
    }
    config_accepted = false;
    config_accepted_ms = 0;
    last_config_tx_ms = 0;
}

void RTLSLinkBeaconProtocol::SendTDoA(uint8_t anchor_a, uint8_t anchor_b, float distance_diff_m, float sigma_m)
{
    if (!config_accepted || anchor_a >= anchor_count || anchor_b >= anchor_count || anchor_a == anchor_b) {
        return;
    }

    uint8_t payload[8] {};
    payload[0] = anchor_a;
    payload[1] = anchor_b;
    write_i32_le(&payload[2], meters_to_mm(distance_diff_m));
    write_u16_le(&payload[6], meters_to_u16_mm(sigma_m));
    send_frame(MsgId::TDOA, payload, sizeof(payload));
}

void RTLSLinkBeaconProtocol::SendPosition(float x_m, float y_m, float z_m)
{
    if (!config_accepted) {
        queue_position(x_m, y_m, z_m);
        return;
    }
    if (!position_enabled_now()) {
        return;
    }
    send_position_frame(x_m, y_m, z_m);
}

bool RTLSLinkBeaconProtocol::ConfigAccepted()
{
    return config_accepted;
}

void RTLSLinkBeaconProtocol::send_config()
{
    uint8_t hello[3] {};
    hello[0] = PROTOCOL_VERSION;
    hello[1] = 0;
    hello[2] = anchor_count;
    send_frame(MsgId::HELLO, hello, sizeof(hello));

    for (uint8_t i = 0; i < anchor_count; i++) {
        send_anchor(anchor_cache[i], i);
    }

    uint8_t end[1] { anchor_count };
    send_frame(MsgId::CONFIG_END, end, sizeof(end));
}

void RTLSLinkBeaconProtocol::send_anchor(const UWBAnchorParam& anchor, uint8_t anchor_id)
{
    uint8_t payload[13] {};
    payload[0] = anchor_id;
    write_i32_le(&payload[1], meters_to_mm(anchor.x));
    write_i32_le(&payload[5], meters_to_mm(anchor.y));
    write_i32_le(&payload[9], meters_to_mm(anchor.z));
    send_frame(MsgId::ANCHOR, payload, sizeof(payload));
}

void RTLSLinkBeaconProtocol::send_position_frame(float x_m, float y_m, float z_m)
{
    const auto& params = Front::uwbLittleFSFront.GetParams();
    uint8_t payload[14] {};
    write_i32_le(&payload[0], meters_to_mm(x_m));
    write_i32_le(&payload[4], meters_to_mm(y_m));
    write_i32_le(&payload[8], meters_to_mm(z_m));
    write_u16_le(&payload[12], params.apBeaconPositionErrorMm);
    send_frame(MsgId::POSITION, payload, sizeof(payload));
}

void RTLSLinkBeaconProtocol::send_frame(MsgId msg_id, const uint8_t* payload, uint8_t payload_len)
{
    if (payload_len > PAYLOAD_LEN_MAX || tx_mutex == nullptr) {
        return;
    }
    if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
        return;
    }

    uint8_t frame[2 + 3 + PAYLOAD_LEN_MAX + 2] {};
    uint8_t len = 0;
    frame[len++] = FRAME_MAGIC_1;
    frame[len++] = FRAME_MAGIC_2;
    frame[len++] = (uint8_t)msg_id;
    frame[len++] = payload_len;
    frame[len++] = tx_seq++;
    for (uint8_t i = 0; i < payload_len; i++) {
        frame[len++] = payload[i];
    }

    uint16_t crc = 0xffff;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc16_update(crc, frame[i]);
    }
    write_u16_le(&frame[len], crc);
    len += 2;

    App::GetArdupilotSerial().write(frame, len);
    xSemaphoreGive(tx_mutex);
}

void RTLSLinkBeaconProtocol::queue_position(float x_m, float y_m, float z_m)
{
    auto& pos = pending_positions[pending_position_next];
    pos.x = x_m;
    pos.y = y_m;
    pos.z = z_m;
    pos.valid = true;
    pending_position_next = (pending_position_next + 1) % PENDING_POSITION_COUNT;
}

void RTLSLinkBeaconProtocol::flush_pending_positions()
{
    for (uint8_t i = 0; i < PENDING_POSITION_COUNT; i++) {
        const uint8_t idx = (pending_position_next + i) % PENDING_POSITION_COUNT;
        auto& pos = pending_positions[idx];
        if (!pos.valid) {
            continue;
        }
        if (position_enabled_now()) {
            send_position_frame(pos.x, pos.y, pos.z);
        }
        pos.valid = false;
    }
}

bool RTLSLinkBeaconProtocol::position_enabled_now()
{
    const auto& params = Front::uwbLittleFSFront.GetParams();
    switch (params.apBeaconPositionMode) {
    case APBeaconPositionMode::POSITION_DISABLED:
        return false;
    case APBeaconPositionMode::CONTINUOUS:
        return true;
    case APBeaconPositionMode::STARTUP_WINDOW:
    default:
        return config_accepted && (millis() - config_accepted_ms) <= params.apBeaconPositionStartupMs;
    }
}

void RTLSLinkBeaconProtocol::process_rx_byte(uint8_t b)
{
    switch (parse_state) {
    case ParseState::MAGIC_1:
        if (b == FRAME_MAGIC_1) {
            rx_crc = crc16_update(0xffff, b);
            parse_state = ParseState::MAGIC_2;
        }
        break;
    case ParseState::MAGIC_2:
        if (b != FRAME_MAGIC_2) {
            reset_parser();
            if (b == FRAME_MAGIC_1) {
                rx_crc = crc16_update(0xffff, b);
                parse_state = ParseState::MAGIC_2;
            }
            break;
        }
        rx_crc = crc16_update(rx_crc, b);
        parse_state = ParseState::MSG_ID;
        break;
    case ParseState::MSG_ID:
        rx_msg_id = b;
        rx_crc = crc16_update(rx_crc, b);
        parse_state = ParseState::LEN;
        break;
    case ParseState::LEN:
        rx_payload_len = b;
        if (rx_payload_len > PAYLOAD_LEN_MAX) {
            reset_parser();
            break;
        }
        rx_crc = crc16_update(rx_crc, b);
        parse_state = ParseState::SEQ;
        break;
    case ParseState::SEQ:
        rx_seq = b;
        rx_crc = crc16_update(rx_crc, b);
        rx_payload_idx = 0;
        parse_state = rx_payload_len == 0 ? ParseState::CRC_LOW : ParseState::PAYLOAD;
        break;
    case ParseState::PAYLOAD:
        rx_payload[rx_payload_idx++] = b;
        rx_crc = crc16_update(rx_crc, b);
        if (rx_payload_idx >= rx_payload_len) {
            parse_state = ParseState::CRC_LOW;
        }
        break;
    case ParseState::CRC_LOW:
        rx_frame_crc = b;
        parse_state = ParseState::CRC_HIGH;
        break;
    case ParseState::CRC_HIGH:
        rx_frame_crc |= (uint16_t)b << 8;
        if (rx_frame_crc == rx_crc && rx_msg_id == (uint8_t)MsgId::ACK) {
            handle_ack();
        }
        reset_parser();
        break;
    }
}

void RTLSLinkBeaconProtocol::reset_parser()
{
    parse_state = ParseState::MAGIC_1;
    rx_msg_id = 0;
    rx_payload_len = 0;
    rx_seq = 0;
    rx_payload_idx = 0;
    rx_crc = 0xffff;
    rx_frame_crc = 0;
}

void RTLSLinkBeaconProtocol::handle_ack()
{
    if (rx_payload_len < 3) {
        return;
    }
    const MsgId acked = (MsgId)rx_payload[0];
    const AckStatus status = (AckStatus)rx_payload[1];
    const uint8_t version = rx_payload[2];
    if (acked == MsgId::CONFIG_END && status == AckStatus::OK && version == PROTOCOL_VERSION) {
        if (!config_accepted) {
            config_accepted_ms = millis();
            config_accepted = true;
            flush_pending_positions();
        }
    }
}

uint16_t RTLSLinkBeaconProtocol::crc16_update(uint16_t crc, uint8_t b)
{
    crc ^= (uint16_t)b << 8;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

int32_t RTLSLinkBeaconProtocol::meters_to_mm(float meters)
{
    if (!std::isfinite(meters)) {
        return 0;
    }
    return (int32_t)lroundf(meters * 1000.0f);
}

uint16_t RTLSLinkBeaconProtocol::meters_to_u16_mm(float meters)
{
    if (!std::isfinite(meters) || meters <= 0.0f) {
        return 0;
    }
    const long mm = lroundf(meters * 1000.0f);
    return (uint16_t)std::min<long>(mm, UINT16_MAX);
}

void RTLSLinkBeaconProtocol::write_i32_le(uint8_t* p, int32_t v)
{
    const uint32_t u = (uint32_t)v;
    p[0] = u & 0xff;
    p[1] = (u >> 8) & 0xff;
    p[2] = (u >> 16) & 0xff;
    p[3] = (u >> 24) & 0xff;
}

void RTLSLinkBeaconProtocol::write_u16_le(uint8_t* p, uint16_t v)
{
    p[0] = v & 0xff;
    p[1] = v >> 8;
}

} // namespace ardupilot

#endif
