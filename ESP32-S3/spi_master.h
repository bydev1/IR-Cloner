/**
 * @file spi_master.h
 * @brief SPI Master Driver for Universal IR Transceiver
 * @note Target: ESP32-S3 (ESP-IDF Framework)
 *       Industrial-grade implementation with DMA and Strict CS_N framing.
 */

#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

// =============================================================================
// 1. Hardware Pinout & Configuration (ESP32-S3)
// =============================================================================
#define PIN_SPI_SCK         GPIO_NUM_12
#define PIN_SPI_MOSI        GPIO_NUM_11
#define PIN_SPI_MISO        GPIO_NUM_10
#define PIN_SPI_CS_N        GPIO_NUM_13  // Software Manual CS_N

#define SPI_FREQ_HZ         250000       // 250 kHz (ป้องกัน Oversampling Trap)
#define MAX_IR_WORDS        1024         // ขีดจำกัดสูงสุดของ FPGA RAM (2048 ไบต์)

// =============================================================================
// 2. Opcode Definitions (ตรงกับ spi_slave.v 100%)
// =============================================================================
#define CMD_STATUS          0x01
#define CMD_GET_LEN         0x02
#define CMD_READ_BUF        0x03
#define CMD_SET_LEN         0x04
#define CMD_WRITE_BUF       0x05
#define CMD_START_TX        0x06
#define CMD_ARM_RX          0x07

// =============================================================================
// 3. Status Bit Masks (อ้างอิงจาก RTL บรรทัด 185: {..., rx_overflow, tx_busy, rx_done})
// =============================================================================
#define STATUS_RX_DONE      (1 << 0)     // 0x01 (Active-High: รับสัญญาณเสร็จสิ้น)
#define STATUS_TX_BUSY      (1 << 1)     // 0x02 (Active-High: กำลังส่งพัลส์ IR)
#define STATUS_RX_OVERFLOW  (1 << 2)     // 0x04 (Active-High: บัฟเฟอร์ล้นเกิน 1024 เวิร์ด)

// =============================================================================
// 4. SpiMaster Class Definition
// =============================================================================
class SpiMaster {
public:
    SpiMaster();
    ~SpiMaster();

    // System API
    esp_err_t begin();
    bool      ping();

    // Core Protocol API
    esp_err_t get_status(uint8_t *status);
    esp_err_t arm_rx();
    esp_err_t get_rx_len(uint16_t *len);
    esp_err_t read_rx_buf(uint16_t *buf, uint16_t len);
    
    esp_err_t set_tx_len(uint16_t len);
    esp_err_t write_tx_buf(const uint16_t *buf, uint16_t len);
    esp_err_t start_tx();

private:
    spi_device_handle_t spi_handle;

    // Helper method สำหรับจัดการ Transaction และคุม CS_N แบบ Strict Recovery
    esp_err_t transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len);
};

#endif // SPI_MASTER_H