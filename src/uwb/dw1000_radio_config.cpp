#include "config/features.hpp"  // MUST be first project include

#ifdef USE_UWB_TDOA_MODES

#include "dw1000_radio_config.hpp"

namespace dw1000_radio {

const uint8_t* ModeByIndex(uint8_t index)
{
    switch (index) {
        case 0: return MODE_SHORTDATA_FAST_ACCURACY;   // 6.8Mb/s, 64MHz PRF, 128 preamble
        case 1: return MODE_LONGDATA_FAST_ACCURACY;    // 6.8Mb/s, 64MHz PRF, 1024 preamble
        case 2: return MODE_SHORTDATA_FAST_LOWPOWER;   // 6.8Mb/s, 16MHz PRF, 128 preamble
        case 3: return MODE_LONGDATA_FAST_LOWPOWER;    // 6.8Mb/s, 16MHz PRF, 1024 preamble
        case 4: return MODE_SHORTDATA_MID_ACCURACY;    // 850kb/s, 64MHz PRF, 128 preamble
        case 5: return MODE_LONGDATA_MID_ACCURACY;     // 850kb/s, 64MHz PRF, 1024 preamble
        case 6: return MODE_LONGDATA_RANGE_ACCURACY;   // 110kb/s, 64MHz PRF, 2048 preamble
        case 7: return MODE_LONGDATA_RANGE_LOWPOWER;   // 110kb/s, 16MHz PRF, 2048 preamble
        default: return MODE_SHORTDATA_FAST_ACCURACY;
    }
}

bool Is64MHzPrfMode(uint8_t index)
{
    return index != 2 && index != 3 && index != 7;
}

void ApplyTxPower(dwDevice_t* dev, uint8_t powerLevel, uint8_t smartPowerEnable)
{
    if (smartPowerEnable) {
        dwUseSmartPower(dev, true);
        return;
    }

    dwUseSmartPower(dev, false);
    switch (powerLevel) {
        case 0:
            dwSetTxPower(dev, 0x07070707ul);
            break;
        case 1:
            dwSetTxPower(dev, 0x0F0F0F0Ful);
            break;
        case 2:
            dwSetTxPower(dev, 0x17171717ul);
            break;
        case 3:
        default:
            dwEnableLargePower(dev);
            break;
    }
}

void ApplyTdoaRadioParams(dwDevice_t* dev, const UWBParams& params)
{
    dwEnableMode(dev, ModeByIndex(params.dwMode));
    dwSetChannel(dev, params.channel);
    dwSetPreambleCode(dev, Is64MHzPrfMode(params.dwMode)
        ? PREAMBLE_CODE_64MHZ_9
        : PREAMBLE_CODE_16MHZ_4);
    ApplyTxPower(dev, params.txPowerLevel, params.smartPowerEnable);
}

} // namespace dw1000_radio

#endif // USE_UWB_TDOA_MODES
