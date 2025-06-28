#pragma once

#include <cstdint>
#include <etl/string_view.h>

namespace bsp {

    struct UWBPinout {
        uint16_t spi_clk_pin;
        uint16_t spi_mosi_pin;
        uint16_t spi_miso_pin;
        uint16_t spi_cs_pin;
        uint16_t reset_pin;
        uint16_t int_pin;
    };
    
    struct UWBConfig {
        UWBPinout pins;
        etl::string_view long_base_address;
        uint32_t slow_spi_clk_hz;
        uint32_t fast_spi_clk_hz;
    };    

    struct UARTPinout {
        uint16_t rx_pin;
        uint16_t tx_pin;
    };

    struct BoardConfig {
        UWBConfig uwb;
        UARTPinout mavlink_uart;
        UARTPinout uwb_data_uart;
        int16_t led_pin;
    };

}   // namespace bsp