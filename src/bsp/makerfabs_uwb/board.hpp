#pragma once

#include "bsp/board_def.hpp"

namespace bsp {

    static constexpr BoardConfig kBoardConfig = {
        .uwb = {
            .pins = {
                .spi_clk_pin = 18,
                .spi_mosi_pin = 23,
                .spi_miso_pin = 19,
                .spi_cs_pin = 4,
                .reset_pin = 27,
                .int_pin = 34,
            },
            .long_base_address = "81:00:5B:D5:A9:9A:E2:9C",
            .slow_spi_clk_hz = 2000000,
            .fast_spi_clk_hz = 16000000
        },
        // Serial2 for mavlink
        .mavlink_uart = {
            .rx_pin = 26,
            .tx_pin = 25,
        },
        // Serial1 for UWB data
        .uwb_data_uart = {
            .rx_pin = 21,
            .tx_pin = 22,
        },
        // Rangefinder not supported on Makerfabs board (use 0xFFFF to indicate disabled)
        .rangefinder_uart = {
            .rx_pin = 0xFFFF,
            .tx_pin = 0xFFFF,
        },
        .led_pin = -1,
        .led2_pin = -1,  // LED 2 not available on this board
    };
} // namespace bsp