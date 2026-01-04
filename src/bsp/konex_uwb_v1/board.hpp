#pragma once

#include <cstdint>

#include "bsp/board_def.hpp"

namespace bsp {

    static constexpr BoardConfig kBoardConfig = {
        .uwb = {
            .pins = {
                .spi_clk_pin = 12,
                .spi_mosi_pin = 11,
                .spi_miso_pin = 13,
                .spi_cs_pin = 10,
                .reset_pin = 9,
                .int_pin = 8,
            },
            .long_base_address = "XX:00:5B:D5:A9:9A:E2:9C",
            .slow_spi_clk_hz = 1400000,
            .fast_spi_clk_hz = 20000000
        },
        // Serial2 for mavlink
        .mavlink_uart = {
            .rx_pin = 5,
            .tx_pin = 4,
        },
        // Serial1 for UWB data
        .uwb_data_uart = {
            .rx_pin = 18,
            .tx_pin = 17,
        },
        // Serial0 (UART0) for rangefinder MAVLink input
        .rangefinder_uart = {
            .rx_pin = 44,
            .tx_pin = 43,
        },
        .led_pin = 36,
        .led2_pin = 35,  // LED 2 for device identification
    };

} // namespace bsp


