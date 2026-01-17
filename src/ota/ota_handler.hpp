/**
 * @file ota_handler.hpp
 * @brief OTA (Over-The-Air) update handler interface
 *
 * Provides web-based firmware update functionality using the ESP32 Update library.
 * Integrates with the existing ESPAsyncWebServer for OTA uploads.
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
 *   GET  /update - Serves the firmware upload HTML page
 *   POST /update - Handles firmware binary upload
 *
 * @param server Reference to the AsyncWebServer instance
 */
void initOtaRoutes(AsyncWebServer& server);

/**
 * @brief Get the HTML page for firmware upload
 * @return HTML content as a string
 */
const char* getUpdatePageHtml();

} // namespace ota

#endif // USE_OTA_WEB
