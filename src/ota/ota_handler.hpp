/**
 * @file ota_handler.hpp
 * @brief OTA (Over-The-Air) update handler interface
 *
 * Provides HTTP firmware update functionality using the ESP32 Update library.
 * Integrates with the existing ESPAsyncWebServer for OTA uploads from the desktop app/CLI.
 */

#pragma once

#include "config/features.hpp"

#ifdef USE_OTA_WEB

#include <ESPAsyncWebServer.h>

namespace ota {

/**
 * @brief Initialize OTA routes on the web server
 *
 * Registers the following routes:
 *   POST /update - Handles firmware binary upload
 *
 * @param server Reference to the AsyncWebServer instance
 */
void initOtaRoutes(AsyncWebServer& server);

} // namespace ota

#endif // USE_OTA_WEB
