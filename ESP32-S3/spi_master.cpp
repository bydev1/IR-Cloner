/**
 * @file spi_master.cpp
 * @brief Implementation of SPI Master Driver for ESP32-S3
 * @note Code Reviewed & Patched: DMA Stack alignment, Safe Ping, and 32-bit Heap boundary.
 */

#include "spi_master.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "IR_SPI_MASTER";

SpiMaster::SpiMaster() : spi_handle(NULL) {}

SpiMaster::~SpiMaster() {
    if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_bus_free(SPI2_HOST);
    }
}

// =============================================================================
// Initialization (Mode 0, 250 kHz, Manual CS_N)
// =============================================================================
esp_err_t SpiMaster::begin() {
    esp_err_t ret;

    // 1. ตั้งค่าขา CS_N เป็น Output และดึง HIGH (Idle State)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PIN_SPI_CS_N);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level(PIN_SPI_CS_N, 1);

    // 2. ตั้งค่าบัส SPI เปิดใช้งาน DMA_CH_AUTO
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = PIN_SPI_SCK;
    buscfg.mosi_io_num = PIN_SPI_MOSI;
    buscfg.miso_io_num = PIN_SPI_MISO;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    // 3. ตั้งค่า SPI Device (ล็อกความเร็วที่ 250 kHz, Mode 0)
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = SPI_FREQ_HZ;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1; // Manual CS_N
    devcfg.queue_size = 1;
    devcfg.flags = 0;

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    return ret;
}

// =============================================================================
// Core Transfer Logic (Stack Buffer Safe & Strict CS_N Recovery)
// =============================================================================
esp_err_t SpiMaster::transfer(const uint8_t *tx_data, uint8_t *rx_data, size_t len) {
    if (len == 0) return ESP_OK;

    spi_transaction_t t = {};
    t.length = len * 8; // ความยาวเป็นบิต

    // [FIX 1] ป้องกัน Stack Buffer vs DMA Conflict สำหรับคำสั่งขนาดสั้น
    if (len <= 4) {
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        if (tx_data) memcpy(t.tx_data, tx_data, len);
    } else {
        t.tx_buffer = tx_data;
        t.rx_buffer = rx_data;
    }

    gpio_set_level(PIN_SPI_CS_N, 0); // Start Frame
    
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    
    gpio_set_level(PIN_SPI_CS_N, 1); // End Frame (Strict Recovery)

    // คัดลอกข้อมูลกลับหากใช้โหมดโอนถ่ายขนาดสั้น
    if (len <= 4 && ret == ESP_OK && rx_data) {
        memcpy(rx_data, t.rx_data, len);
    }

    return ret;
}

// =============================================================================
// Sanity Check
// =============================================================================
bool SpiMaster::ping() {
    uint8_t status;
    esp_err_t err = get_status(&status);
    
    // [FIX 2] ป้องกัน False Positive (ตรวจสอบ MISO ลอย และ Data Integrity)
    // - ไม่ใช่ 0xFF (สายหลุด)
    // - บิตที่ 7 ถึง 3 ต้องเป็น 0 เสมอตามโครงสร้าง {5'b0, rx_ovf, tx_busy, rx_done}
    if (err == ESP_OK && status != 0xFF && (status & 0xF8) == 0x00) {
        return true;
    }
    return false;
}

// =============================================================================
// Opcodes Implementation
// =============================================================================
esp_err_t SpiMaster::get_status(uint8_t *status) {
    uint8_t tx[2] = {CMD_STATUS, 0x00};
    uint8_t rx[2] = {0x00, 0x00};
    esp_err_t err = transfer(tx, rx, 2); // len <= 4 จะใช้ SPI_TRANS_USE_RXDATA อัตโนมัติ
    if (err == ESP_OK && status != nullptr) {
        *status = rx[1];
    }
    return err;
}

esp_err_t SpiMaster::arm_rx() {
    uint8_t tx[1] = {CMD_ARM_RX};
    uint8_t rx[1] = {0x00};
    return transfer(tx, rx, 1);
}

esp_err_t SpiMaster::get_rx_len(uint16_t *len) {
    uint8_t tx[3] = {CMD_GET_LEN, 0x00, 0x00};
    uint8_t rx[3] = {0x00, 0x00, 0x00};
    esp_err_t err = transfer(tx, rx, 3);
    if (err == ESP_OK && len != nullptr) {
        *len = (rx[1] << 8) | rx[2];
    }
    return err;
}

esp_err_t SpiMaster::read_rx_buf(uint16_t *buf, uint16_t len) {
    if (len == 0) return ESP_OK;
    if (len > MAX_IR_WORDS) return ESP_ERR_INVALID_SIZE;

    size_t transfer_bytes = 1 + (len * 2);
    
    // [FIX 3] ปัดเศษขนาด DMA Buffer ให้เป็น 4-byte Aligned ป้องกัน Memory Overwrite
    size_t dma_alloc_bytes = (transfer_bytes + 3) & ~3;

    uint8_t *tx = (uint8_t *)heap_caps_calloc(dma_alloc_bytes, 1, MALLOC_CAP_DMA);
    uint8_t *rx = (uint8_t *)heap_caps_calloc(dma_alloc_bytes, 1, MALLOC_CAP_DMA);
    
    if (!tx || !rx) {
        if (tx) free(tx);
        if (rx) free(rx);
        return ESP_ERR_NO_MEM;
    }

    tx[0] = CMD_READ_BUF;
    // ส่ง transfer_bytes ไปให้ฟังก์ชันหลัก (ไม่ใช่ dma_alloc_bytes เพื่อส่งเฉพาะข้อมูลจริง)
    esp_err_t err = transfer(tx, rx, transfer_bytes); 

    if (err == ESP_OK && buf != nullptr) {
        for (uint16_t i = 0; i < len; i++) {
            buf[i] = (rx[1 + (i * 2)] << 8) | rx[2 + (i * 2)];
        }
    }

    free(tx);
    free(rx);
    return err;
}

esp_err_t SpiMaster::set_tx_len(uint16_t len) {
    if (len > MAX_IR_WORDS) return ESP_ERR_INVALID_SIZE;

    uint8_t tx[3] = {CMD_SET_LEN, (uint8_t)(len >> 8), (uint8_t)(len & 0xFF)};
    uint8_t rx[3] = {0x00, 0x00, 0x00};
    return transfer(tx, rx, 3);
}

esp_err_t SpiMaster::write_tx_buf(const uint16_t *buf, uint16_t len) {
    if (len == 0) return ESP_OK;
    if (len > MAX_IR_WORDS) return ESP_ERR_INVALID_SIZE;

    size_t transfer_bytes = 1 + (len * 2);
    
    // [FIX 3] ปัดเศษขนาด DMA Buffer ให้เป็น 4-byte Aligned
    size_t dma_alloc_bytes = (transfer_bytes + 3) & ~3;

    uint8_t *tx = (uint8_t *)heap_caps_calloc(dma_alloc_bytes, 1, MALLOC_CAP_DMA);
    uint8_t *rx = (uint8_t *)heap_caps_calloc(dma_alloc_bytes, 1, MALLOC_CAP_DMA);

    if (!tx || !rx) {
        if (tx) free(tx);
        if (rx) free(rx);
        return ESP_ERR_NO_MEM;
    }

    tx[0] = CMD_WRITE_BUF;
    
    for (uint16_t i = 0; i < len; i++) {
        tx[1 + (i * 2)] = (buf[i] >> 8) & 0xFF;
        tx[2 + (i * 2)] = buf[i] & 0xFF;
    }

    esp_err_t err = transfer(tx, rx, transfer_bytes);

    free(tx);
    free(rx);
    return err;
}

esp_err_t SpiMaster::start_tx() {
    uint8_t tx[1] = {CMD_START_TX};
    uint8_t rx[1] = {0x00};
    return transfer(tx, rx, 1);
}