/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * statsCnt.h - utitlity for logging rates
 */

#pragma once

#include <stdint.h>

/**
 * @brief A struct used to track event rates
 */
typedef struct {
    uint32_t count;
    uint32_t latestCount;
    uint32_t latestAveragingMs;
    float latestRate;
    uint32_t intervalMs;
} statsCntRateCounter_t;

/**
 * @brief Struct to use a rate counter together with the log usb system.
 */
typedef struct {
    // logByFunction_t must be the first element in this struct since pointers
    // to this struct are cast to logByFunction_t* and used by the log module
    // logByFunction_t logByFunction;
    statsCntRateCounter_t rateCounter;
} statsCntRateLogger_t;