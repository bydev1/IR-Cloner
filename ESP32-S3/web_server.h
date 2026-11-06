/**
 * @file web_server.h
 * @brief Wi-Fi AP + LittleFS + REST API for Universal IR Transceiver (Demo)
 * @note Target: ESP32-S3, Arduino-ESP32 framework
 *       Library: WebServer.h (built-in), ArduinoJson v7, LittleFS
 *
 * วิธีใช้ใน main.cpp:
 *   SpiMaster spi;
 *   IrWebServer web(spi);
 *
 *   void setup() { spi.begin(); web.begin(); }
 *   void loop()  { web.handle(); }
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "spi_master.h"   // SpiMaster, MAX_IR_WORDS, STATUS_* masks

// =============================================================================
// 1. Configuration
// =============================================================================
#define WIFI_AP_SSID        "IR-Transceiver"
#define WIFI_AP_PASS        "12345678"       // ต้องยาว >= 8 ตัวอักษร
#define HTTP_PORT           80

#define PROFILE_DIR         "/profiles"
#define PROFILE_NAME_MAX    32               // ความยาวชื่อโปรไฟล์สูงสุด
#define INDEX_FILE          "/index.html"

#define TX_BUSY_WAIT_MS     600              // รอ tx_busy เคลียร์ (FPGA watchdog = 500 ms)

// =============================================================================
// 2. IrWebServer Class
// =============================================================================
class IrWebServer {
public:
    explicit IrWebServer(SpiMaster &spi);

    bool begin();    // เริ่ม LittleFS + Wi-Fi AP + ลงทะเบียน route (เรียกหลัง spi.begin())
    void handle();   // เรียกใน loop() ทุกรอบ ห้ามมี delay() ยาวๆ

private:
    SpiMaster      &_spi;
    WebServer       _server;
    SemaphoreHandle_t _spiMutex;

    // Static buffer สำหรับพัลส์ (2 KB) ไม่ใช้ Stack
    uint16_t        _pulseBuf[MAX_IR_WORDS];

    // ---- Route handlers ----
    void handleIndex();
    void handleStatus();          // GET  /api/status
    void handleCaptureArm();      // POST /api/capture/arm
    void handleCapturePoll();     // GET  /api/capture/poll
    void handleProfileList();     // GET  /api/profiles
    void handleProfileSave();     // POST /api/profiles/save
    void handleProfileDelete();   // POST /api/profiles/delete
    void handleSend();            // POST /api/send
    void handleNotFound();

    // ---- Helpers ----
    void sendJson(int code, const String &body);
    void sendError(int code, const char *msg);
    bool validName(const String &name) const;          // a-z A-Z 0-9 _ - , ความยาว 1..32
    String profilePath(const String &name) const;      // "/profiles/<name>.json"

    bool spiLock(uint32_t timeoutMs = 500);
    void spiUnlock();
};

#endif // WEB_SERVER_H
