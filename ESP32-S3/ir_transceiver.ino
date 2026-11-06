/**
 * @file ir_transceiver.ino
 * @brief Entry point: Universal IR Transceiver (ESP32-S3 + Cyclone IV FPGA)
 *
 * Board : ESP32-S3 DevKitC-1 (N16R8)
 * IDE   : Arduino IDE 2.3.x, ESP32 core v3.x
 * Libs  : WebServer, LittleFS (built-in), ArduinoJson v7
 *
 * Tools menu (แนะนำ):
 *   - USB CDC On Boot : Enabled   (เพื่อให้ Serial Monitor ทำงานผ่านพอร์ต USB)
 *   - Flash Size      : 16MB
 *   - PSRAM           : OPI PSRAM
 *   - Partition Scheme: เลือกแบบที่มีพาร์ติชัน SPIFFS/LittleFS (ใช้เก็บ index.html และโปรไฟล์)
 *
 * หมายเหตุเรื่องขา SPI:
 *   ขา SCK/MOSI/MISO/CS ถูกกำหนดใน spi_master.h (PIN_SPI_*)
 *   ไม่ได้ส่งผ่าน Constructor เพราะ SpiMaster() ไม่รับพารามิเตอร์
 */

#include <Arduino.h>
#include "spi_master.h"
#include "web_server.h"

// -----------------------------------------------------------------------------
// Global instances
// -----------------------------------------------------------------------------
SpiMaster   spi;
IrWebServer web(spi);

static bool g_ready = false;   // true เมื่อ SPI และ Web Server พร้อมทั้งคู่

// -----------------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------------
void setup() {
    // ---- Serial (USB CDC) ----
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) {   // รอ USB CDC สูงสุด 3 วินาที
        delay(10);
    }
    delay(200);
    Serial.println();
    Serial.println("=== Universal IR Transceiver ===");

    // ---- SPI Master ----
    esp_err_t err = spi.begin();
    if (err != ESP_OK) {
        Serial.printf("[SPI] begin() FAILED: %s\n", esp_err_to_name(err));
        Serial.println("[SYS] Halt: Web Server not started (SPI is not ready)");
        return;   // g_ready ยังเป็น false
    }
    Serial.printf("[SPI] begin() OK  (SCK=%d MOSI=%d MISO=%d CS=%d, %d Hz)\n",
                  (int)PIN_SPI_SCK, (int)PIN_SPI_MOSI,
                  (int)PIN_SPI_MISO, (int)PIN_SPI_CS_N, SPI_FREQ_HZ);

    if (spi.ping()) {
        Serial.println("[SPI] FPGA ping OK");
    } else {
        Serial.println("[SPI] FPGA ping FAILED (check wiring / FPGA config) - continuing anyway");
    }

    // ---- Wi-Fi AP + LittleFS + Web Server ----
    if (!web.begin()) {
        Serial.println("[WEB] begin() FAILED (see log above)");
        return;   // g_ready ยังเป็น false
    }
    Serial.println("[WEB] Server started. Connect to Wi-Fi AP and open http://192.168.4.1");

    g_ready = true;
}

// -----------------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------------
void loop() {
    if (g_ready) {
        web.handle();   // รับ HTTP request (handler ไม่บล็อก)
    }
    delay(1);           // คืน CPU ให้ Idle task / Wi-Fi stack
}
