/**
 * @file ota_handler.cpp
 * @brief OTA update handler implementation
 */

#include "ota_handler.hpp"

#ifdef USE_OTA_WEB

#include <Update.h>
#include <Arduino.h>

namespace ota {

// HTML page for firmware upload
static const char* UPDATE_PAGE_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>RTLS-Link Firmware Update</title>
    <style>
        * { box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            margin: 0; padding: 20px;
            background: #1a1a2e; color: #eee;
            min-height: 100vh;
        }
        .container {
            max-width: 500px; margin: 0 auto;
            background: #16213e; padding: 30px;
            border-radius: 12px; box-shadow: 0 4px 20px rgba(0,0,0,0.3);
        }
        h1 { margin: 0 0 10px; color: #00d9ff; font-size: 1.5em; }
        .subtitle { color: #888; margin-bottom: 25px; font-size: 0.9em; }
        .upload-area {
            border: 2px dashed #444; border-radius: 8px;
            padding: 40px 20px; text-align: center;
            transition: all 0.3s; cursor: pointer;
            margin-bottom: 20px;
        }
        .upload-area:hover, .upload-area.drag-over {
            border-color: #00d9ff; background: rgba(0,217,255,0.05);
        }
        .upload-area input { display: none; }
        .upload-icon { font-size: 48px; margin-bottom: 10px; }
        .file-name { color: #00d9ff; margin-top: 10px; word-break: break-all; }
        button {
            width: 100%; padding: 15px; border: none;
            background: #00d9ff; color: #000; font-weight: bold;
            border-radius: 8px; cursor: pointer; font-size: 1em;
            transition: all 0.3s;
        }
        button:hover:not(:disabled) { background: #00b8d9; }
        button:disabled { opacity: 0.5; cursor: not-allowed; }
        .progress-container {
            display: none; margin-top: 20px;
        }
        .progress-bar {
            height: 8px; background: #333; border-radius: 4px;
            overflow: hidden;
        }
        .progress-fill {
            height: 100%; width: 0%; background: #00d9ff;
            transition: width 0.3s;
        }
        .progress-text {
            text-align: center; margin-top: 10px; color: #888;
        }
        .status {
            margin-top: 20px; padding: 15px;
            border-radius: 8px; display: none;
        }
        .status.success { background: rgba(0,255,136,0.15); color: #00ff88; display: block; }
        .status.error { background: rgba(255,68,68,0.15); color: #ff4444; display: block; }
        .warning {
            background: rgba(255,170,0,0.15); color: #ffaa00;
            padding: 12px; border-radius: 6px; margin-bottom: 20px;
            font-size: 0.85em;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Firmware Update</h1>
        <p class="subtitle">RTLS-Link OTA Update</p>

        <div class="warning">
            Ensure stable power during update. Do not disconnect the device.
        </div>

        <form id="uploadForm" method="POST" enctype="multipart/form-data">
            <div class="upload-area" id="dropZone">
                <div class="upload-icon">&#128190;</div>
                <div>Drop firmware.bin here or click to select</div>
                <div class="file-name" id="fileName"></div>
                <input type="file" name="firmware" id="fileInput" accept=".bin">
            </div>
            <button type="submit" id="uploadBtn" disabled>Upload Firmware</button>
        </form>

        <div class="progress-container" id="progressContainer">
            <div class="progress-bar">
                <div class="progress-fill" id="progressFill"></div>
            </div>
            <div class="progress-text" id="progressText">Uploading... 0%</div>
        </div>

        <div class="status" id="status"></div>
    </div>

    <script>
        const dropZone = document.getElementById('dropZone');
        const fileInput = document.getElementById('fileInput');
        const fileName = document.getElementById('fileName');
        const uploadBtn = document.getElementById('uploadBtn');
        const uploadForm = document.getElementById('uploadForm');
        const progressContainer = document.getElementById('progressContainer');
        const progressFill = document.getElementById('progressFill');
        const progressText = document.getElementById('progressText');
        const status = document.getElementById('status');

        dropZone.addEventListener('click', () => fileInput.click());

        ['dragenter', 'dragover'].forEach(e => {
            dropZone.addEventListener(e, (ev) => {
                ev.preventDefault();
                dropZone.classList.add('drag-over');
            });
        });

        ['dragleave', 'drop'].forEach(e => {
            dropZone.addEventListener(e, (ev) => {
                ev.preventDefault();
                dropZone.classList.remove('drag-over');
            });
        });

        dropZone.addEventListener('drop', (e) => {
            const files = e.dataTransfer.files;
            if (files.length) {
                fileInput.files = files;
                updateFileName(files[0]);
            }
        });

        fileInput.addEventListener('change', () => {
            if (fileInput.files.length) {
                updateFileName(fileInput.files[0]);
            }
        });

        function updateFileName(file) {
            fileName.textContent = file.name + ' (' + (file.size / 1024).toFixed(1) + ' KB)';
            uploadBtn.disabled = false;
        }

        uploadForm.addEventListener('submit', async (e) => {
            e.preventDefault();
            if (!fileInput.files.length) return;

            const formData = new FormData();
            formData.append('firmware', fileInput.files[0]);

            uploadBtn.disabled = true;
            progressContainer.style.display = 'block';
            status.className = 'status';
            status.style.display = 'none';

            try {
                const xhr = new XMLHttpRequest();
                xhr.open('POST', '/update', true);

                xhr.upload.onprogress = (e) => {
                    if (e.lengthComputable) {
                        const pct = Math.round((e.loaded / e.total) * 100);
                        progressFill.style.width = pct + '%';
                        progressText.textContent = 'Uploading... ' + pct + '%';
                    }
                };

                xhr.onload = () => {
                    if (xhr.status === 200) {
                        progressText.textContent = 'Update complete! Rebooting...';
                        status.className = 'status success';
                        status.textContent = 'Firmware updated successfully. Device is rebooting...';
                        setTimeout(() => location.reload(), 10000);
                    } else {
                        throw new Error(xhr.responseText || 'Upload failed');
                    }
                };

                xhr.onerror = () => {
                    throw new Error('Network error');
                };

                xhr.send(formData);
            } catch (err) {
                status.className = 'status error';
                status.textContent = 'Error: ' + err.message;
                uploadBtn.disabled = false;
            }
        });
    </script>
</body>
</html>
)rawliteral";

const char* getUpdatePageHtml() {
    return UPDATE_PAGE_HTML;
}

void initOtaRoutes(AsyncWebServer& server) {
    // GET /update - Serve the upload page
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", UPDATE_PAGE_HTML);
    });

    // POST /update - Handle firmware upload
    server.on("/update", HTTP_POST,
        // Response handler (called after upload completes)
        [](AsyncWebServerRequest* request) {
            bool success = !Update.hasError();
            AsyncWebServerResponse* response = request->beginResponse(
                success ? 200 : 500,
                "text/plain",
                success ? "Update successful. Rebooting..." : "Update failed"
            );
            response->addHeader("Connection", "close");
            request->send(response);

            if (success) {
                // Schedule reboot
                delay(500);
                ESP.restart();
            }
        },
        // File upload handler (called for each chunk)
        [](AsyncWebServerRequest* request, const String& filename, size_t index,
           uint8_t* data, size_t len, bool final) {

            if (index == 0) {
                printf("[OTA] Starting update: %s\n", filename.c_str());

                // Calculate available space
                size_t maxSize = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
                printf("[OTA] Max firmware size: %u bytes\n", maxSize);

                if (!Update.begin(maxSize, U_FLASH)) {
                    printf("[OTA] Update.begin failed: %s\n", Update.errorString());
                    return;
                }
            }

            // Write chunk
            if (Update.write(data, len) != len) {
                printf("[OTA] Update.write failed: %s\n", Update.errorString());
                return;
            }

            if (final) {
                if (Update.end(true)) {
                    printf("[OTA] Update complete. Size: %u bytes\n", index + len);
                } else {
                    printf("[OTA] Update.end failed: %s\n", Update.errorString());
                }
            }
        }
    );

    printf("[OTA] Web OTA routes registered at /update\n");
}

} // namespace ota

#endif // USE_OTA_WEB
