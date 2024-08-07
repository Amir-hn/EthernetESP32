#include <SPI.h>
#include <Wire.h>  // Include if you are using I2C for EEPROM or other peripherals
#include <EtherCard.h>
#include <cstring>  // for memcpy
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_eth.h"
#include "esp_system.h"

// ENC28J60 Configuration
#define ENC28J60_SPI_CS_PIN 10 // Chip Select Pin
#define ENC28J60_RESET_PIN 9   // Reset Pin

// Timeout Definitions
#define ENC28J60_SPI_LOCK_TIMEOUT_MS 50
#define ENC28J60_REG_TRANS_LOCK_TIMEOUT_MS 150
#define ENC28J60_PHY_OPERATION_TIMEOUT_US 150
#define ENC28J60_SYSTEM_RESET_ADDITION_TIME_US 1000
#define ENC28J60_TX_READY_TIMEOUT_MS 2000

// Buffer Size and Regions
#define ENC28J60_BUFFER_SIZE 0x2000 // 8KB built-in buffer
#define ENC28J60_BUF_RX_START 0
#define ENC28J60_BUF_RX_END (ENC28J60_BUF_TX_START - 1)
#define ENC28J60_BUF_TX_START ((ENC28J60_BUFFER_SIZE / 4) * 3)
#define ENC28J60_BUF_TX_END (ENC28J60_BUFFER_SIZE - 1)

// Status Vector Sizes
#define ENC28J60_RSV_SIZE 6
#define ENC28J60_TSV_SIZE 6

typedef struct {
    uint8_t next_packet_low;
    uint8_t next_packet_high;
    uint8_t length_low;
    uint8_t length_high;
    uint8_t status_low;
    uint8_t status_high;
} enc28j60_rx_header_t;

typedef struct {
    uint16_t byte_cnt;
    uint8_t collision_cnt:4;
    uint8_t crc_err:1;
    uint8_t len_check_err:1;
    uint8_t len_out_range:1;
    uint8_t tx_done:1;
    uint8_t multicast:1;
    uint8_t broadcast:1;
    uint8_t pkt_defer:1;
    uint8_t excessive_defer:1;
    uint8_t excessive_collision:1;
    uint8_t late_collision:1;
    uint8_t giant:1;
    uint8_t underrun:1;
    uint16_t bytes_on_wire;
    uint8_t ctrl_frame:1;
    uint8_t pause_ctrl_frame:1;
    uint8_t backpressure_app:1;
    uint8_t vlan_frame:1;
} enc28j60_tsv_t;

typedef struct {
    SPIClass* spi;
    uint8_t csPin;
    SemaphoreHandle_t lock;
} eth_spi_info_t;

typedef struct {
    void *ctx;
    void *(*init)(const void *spi_config);
    void (*deinit)(void *spi_ctx);
    void (*read)(void *spi_ctx, uint32_t cmd, uint32_t addr, void *data, uint32_t data_len);
    void (*write)(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *data, uint32_t data_len);
} eth_spi_custom_driver_t;

typedef struct {
    eth_spi_custom_driver_t spi;
    SemaphoreHandle_t reg_trans_lock;
    SemaphoreHandle_t tx_ready_sem;
    TaskHandle_t rx_task_hdl;
    uint32_t sw_reset_timeout_ms;
    uint32_t next_packet_ptr;
    uint32_t last_tsv_addr;
    int int_gpio_num;
    uint32_t poll_period_ms;
    uint8_t addr[6];
    uint8_t last_bank;
    bool packets_remain;
} emac_enc28j60_t;

static eth_spi_info_t* enc28j60_spi_init(uint8_t csPin)
{
    eth_spi_info_t* spi = new eth_spi_info_t();
    if (!spi) {
        return NULL;
    }

    spi->spi = &SPI;
    spi->csPin = csPin;

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH); // Deselect the device

    // Create mutex-like semaphore
    spi->lock = xSemaphoreCreateMutex();
    if (!spi->lock) {
        delete spi;
        return NULL;
    }

    return spi;
}

static void enc28j60_spi_deinit(eth_spi_info_t* spi)
{
    if (spi) {
        if (spi->lock) {
            vSemaphoreDelete(spi->lock);
        }
        delete spi;
    }
}

static inline bool enc28j60_spi_lock(eth_spi_info_t* spi)
{
    return xSemaphoreTake(spi->lock, pdMS_TO_TICKS(ENC28J60_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline bool enc28j60_spi_unlock(eth_spi_info_t* spi)
{
    return xSemaphoreGive(spi->lock) == pdTRUE;
}



// ENC28J60 SPI Commands
#define ENC28J60_SPI_CMD_WCR 0x40 // Write Control Register
#define ENC28J60_SPI_CMD_RCR 0x00 // Read Control Register

// Timeout Definitions
#define ENC28J60_REG_TRANS_LOCK_TIMEOUT_MS 150

typedef struct {
    SPIClass* spi;
    uint8_t csPin;
    SemaphoreHandle_t reg_trans_lock;
} emac_enc28j60_t;

static esp_err_t enc28j60_spi_write(emac_enc28j60_t *emac, uint32_t cmd, uint32_t addr, const void *value, uint32_t len) {
    // Arduino's SPI.transfer does not handle commands and addresses separately.
    // We need to implement the correct command format in SPI communication.

    // Begin SPI transaction
    digitalWrite(emac->csPin, LOW);

    // Send command
    SPI.transfer(cmd);

    // Send address (assuming address is in byte format)
    SPI.transfer((uint8_t)addr);

    // Send data
    for (uint32_t i = 0; i < len; i++) {
        SPI.transfer(((uint8_t*)value)[i]);
    }

    // End SPI transaction
    digitalWrite(emac->csPin, HIGH);

    return ESP_OK; // In Arduino, you will need to handle errors differently
}

static esp_err_t enc28j60_spi_read(emac_enc28j60_t *emac, uint32_t cmd, uint32_t addr, void *value, uint32_t len) {
    // Arduino's SPI.transfer does not handle commands and addresses separately.
    // We need to implement the correct command format in SPI communication.

    // Begin SPI transaction
    digitalWrite(emac->csPin, LOW);

    // Send command
    SPI.transfer(cmd);

    // Send address (assuming address is in byte format)
    SPI.transfer((uint8_t)addr);

    // Receive data
    for (uint32_t i = 0; i < len; i++) {
        ((uint8_t*)value)[i] = SPI.transfer(0x00); // Send dummy byte to receive data
    }

    // End SPI transaction
    digitalWrite(emac->csPin, HIGH);

    return ESP_OK; // In Arduino, you will need to handle errors differently
}

static inline bool enc28j60_reg_trans_lock(emac_enc28j60_t *emac) {
    // Implement locking mechanism if needed, or replace with a placeholder
    return true; // Placeholder: always successful
}

static inline bool enc28j60_reg_trans_unlock(emac_enc28j60_t *emac) {
    // Implement unlocking mechanism if needed, or replace with a placeholder
    return true; // Placeholder: always successful
}

static inline uint32_t enc28j60_next_ptr_align_odd(uint32_t next_packet_ptr, uint32_t start, uint32_t end) {
    uint32_t erxrdpt;
    if ((next_packet_ptr - 1 < start) || (next_packet_ptr - 1 > end)) {
        erxrdpt = end;
    } else {
        erxrdpt = next_packet_ptr - 1;
    }
    return erxrdpt;
}

static inline uint32_t enc28j60_rx_packet_start(uint32_t start_addr, uint32_t off) {
    if (start_addr + off > ENC28J60_BUF_RX_END) {
        return (start_addr + off) - (ENC28J60_BUF_RX_END - ENC28J60_BUF_RX_START + 1);
    } else {
        return start_addr + off;
    }
}

static esp_err_t enc28j60_do_register_write(emac_enc28j60_t *emac, uint8_t reg_addr, uint8_t value) {
    return enc28j60_spi_write(emac, ENC28J60_SPI_CMD_WCR, reg_addr, &value, sizeof(value));
}

static esp_err_t enc28j60_do_register_read(emac_enc28j60_t *emac, bool is_eth_reg, uint8_t reg_addr, uint8_t *value) {
    uint8_t tmp[2];
    esp_err_t ret = enc28j60_spi_read(emac, ENC28J60_SPI_CMD_RCR, reg_addr, tmp, is_eth_reg ? 1 : 2);
    if (ret == ESP_OK) {
        *value = is_eth_reg ? tmp[0] : tmp[1];
    }
    return ret;
}

// ENC28J60 SPI Commands
#define ENC28J60_SPI_CMD_BFS 0x80 // Bit Field Set
#define ENC28J60_SPI_CMD_BFC 0xA0 // Bit Field Clear
#define ENC28J60_SPI_CMD_WBM 0x7A // Write Buffer Memory
#define ENC28J60_SPI_CMD_RBM 0x3A // Read Buffer Memory
#define ENC28J60_SPI_CMD_SRC 0xFF // Software Reset

// ENC28J60 Register Addresses
#define ENC28J60_ECON1 0x1F
#define ENC28J60_MISTAT 0x0A
#define ENC28J60_MIREGADR 0x14
#define ENC28J60_MIWRL 0x12
#define ENC28J60_MIWRH 0x13
#define ENC28J60_MICMD 0x12
#define ENC28J60_MIRDL 0x06
#define ENC28J60_MIRDH 0x07

// ENC28J60 Constants
#define ENC28J60_SYSTEM_RESET_ADDITION_TIME_US 1000
#define ENC28J60_PHY_OPERATION_TIMEOUT_US 5000

typedef struct {
    SPIClass* spi;
    uint8_t csPin;
    uint8_t last_bank;
} emac_enc28j60_t;

static esp_err_t enc28j60_spi_write(emac_enc28j60_t *emac, uint32_t cmd, uint32_t addr, const void *value, uint32_t len) {
    digitalWrite(emac->csPin, LOW);
    SPI.transfer(cmd);
    SPI.transfer(addr);
    for (uint32_t i = 0; i < len; i++) {
        SPI.transfer(((uint8_t*)value)[i]);
    }
    digitalWrite(emac->csPin, HIGH);
    return ESP_OK; // Placeholder for error handling
}

static esp_err_t enc28j60_spi_read(emac_enc28j60_t *emac, uint32_t cmd, uint32_t addr, void *value, uint32_t len) {
    digitalWrite(emac->csPin, LOW);
    SPI.transfer(cmd);
    SPI.transfer(addr);
    for (uint32_t i = 0; i < len; i++) {
        ((uint8_t*)value)[i] = SPI.transfer(0x00); // Send dummy byte to receive data
    }
    digitalWrite(emac->csPin, HIGH);
    return ESP_OK; // Placeholder for error handling
}

static esp_err_t enc28j60_do_bitwise_set(emac_enc28j60_t *emac, uint8_t reg_addr, uint8_t mask) {
    return enc28j60_spi_write(emac, ENC28J60_SPI_CMD_BFS, reg_addr, &mask, sizeof(mask));
}

static esp_err_t enc28j60_do_bitwise_clr(emac_enc28j60_t *emac, uint8_t reg_addr, uint8_t mask) {
    return enc28j60_spi_write(emac, ENC28J60_SPI_CMD_BFC, reg_addr, &mask, sizeof(mask));
}

static esp_err_t enc28j60_do_memory_write(emac_enc28j60_t *emac, uint8_t *buffer, uint32_t len) {
    return enc28j60_spi_write(emac, ENC28J60_SPI_CMD_WBM, 0x1A, buffer, len);
}

static esp_err_t enc28j60_do_memory_read(emac_enc28j60_t *emac, uint8_t *buffer, uint32_t len) {
    return enc28j60_spi_read(emac, ENC28J60_SPI_CMD_RBM, 0x1A, buffer, len);
}

static esp_err_t enc28j60_do_reset(emac_enc28j60_t *emac) {
    enc28j60_spi_write(emac, ENC28J60_SPI_CMD_SRC, 0x1F, NULL, 0);
    delayMicroseconds(ENC28J60_SYSTEM_RESET_ADDITION_TIME_US);
    return ESP_OK; // Placeholder for error handling
}

static esp_err_t enc28j60_switch_register_bank(emac_enc28j60_t *emac, uint8_t bank) {
    if (bank != emac->last_bank) {
        enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, 0x03);
        enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, bank & 0x03);
        emac->last_bank = bank;
    }
    return ESP_OK;
}

static esp_err_t enc28j60_register_write(emac_enc28j60_t *emac, uint16_t reg_addr, uint8_t value) {
    enc28j60_switch_register_bank(emac, (reg_addr & 0xF00) >> 8);
    return enc28j60_spi_write(emac, ENC28J60_SPI_CMD_WCR, reg_addr & 0xFF, &value, sizeof(value));
}

static esp_err_t enc28j60_register_read(emac_enc28j60_t *emac, uint16_t reg_addr, uint8_t *value) {
    enc28j60_switch_register_bank(emac, (reg_addr & 0xF00) >> 8);
    return enc28j60_spi_read(emac, ENC28J60_SPI_CMD_RCR, reg_addr & 0xFF, value, sizeof(*value));
}

static esp_err_t enc28j60_read_packet(emac_enc28j60_t *emac, uint32_t addr, uint8_t *packet, uint32_t len) {
    enc28j60_register_write(emac, ENC28J60_ERDPTL, addr & 0xFF);
    enc28j60_register_write(emac, ENC28J60_ERDPTH, (addr & 0xFF00) >> 8);
    return enc28j60_do_memory_read(emac, packet, len);
}

static esp_err_t emac_enc28j60_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value) {
    emac_enc28j60_t *emac = (emac_enc28j60_t *)mac; // Adjust based on your container_of implementation
    uint8_t mii_status;

    enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status);
    if (mii_status & 0x01) return ESP_ERR_INVALID_STATE; // Replace with the correct error code if PHY is busy

    enc28j60_register_write(emac, ENC28J60_MIREGADR, phy_reg & 0xFF);
    enc28j60_register_write(emac, ENC28J60_MIWRL, reg_value & 0xFF);
    enc28j60_register_write(emac, ENC28J60_MIWRH, (reg_value & 0xFF00) >> 8);

    uint32_t to = 0;
    do {
        delayMicroseconds(15);
        enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status);
        to += 15;
    } while ((mii_status & 0x01) && to < ENC28J60_PHY_OPERATION_TIMEOUT_US);

    if (mii_status & 0x01) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

static esp_err_t emac_enc28j60_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value) {
    emac_enc28j60_t *emac = (emac_enc28j60_t *)mac; // Adjust based on your container_of implementation
    uint8_t mii_status;
    uint8_t mii_cmd;

    enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status);
    if (mii_status & 0x01) return ESP_ERR_INVALID_STATE; // Replace with the correct error code if PHY is busy

    enc28j60_register_write(emac, ENC28J60_MIREGADR, phy_reg & 0xFF);
    mii_cmd = 0x01; // MICMD_MIIRD
    enc28j60_register_write(emac, ENC28J60_MICMD, mii_cmd);

    uint32_t to = 0;
    do {
        delayMicroseconds(15);
        enc28j60_register_read(emac, ENC28J60_MISTAT, &mii_status);
        to += 15;
    } while ((mii_status & 0x01) && to < ENC28J60_PHY_OPERATION_TIMEOUT_US);

    if (mii_status & 0x01) return ESP_ERR_TIMEOUT;

    mii_cmd &= (~0x01); // Clear MICMD_MIIRD
    enc28j60_register_write(emac, ENC28J60_MICMD, mii_cmd);

    uint8_t value_l = 0;
    uint8_t value_h = 0;
    enc28j60_register_read(emac, ENC28J60_MIRDL, &value_l);
    enc28j60_register_read(emac, ENC28J60_MIRDH, &value_h);
    *reg_value = (value_h << 8) | value_l;

    return ESP_OK;
}

// Define the ENC28J60 register addresses
#define ENC28J60_EREVID 0x1F
#define ENC28J60_MAADR1 0x00
#define ENC28J60_MAADR2 0x01
#define ENC28J60_MAADR3 0x02
#define ENC28J60_MAADR4 0x03
#define ENC28J60_MAADR5 0x04
#define ENC28J60_MAADR6 0x05
#define ENC28J60_EHT0   0x06
#define ENC28J60_ERXSTL 0x07
#define ENC28J60_ERXSTH 0x08
#define ENC28J60_ERXNDL 0x09
#define ENC28J60_ERXNDH 0x0A
#define ENC28J60_ERXRDPTL 0x0B
#define ENC28J60_ERXRDPTH 0x0C
#define ENC28J60_ETXSTL 0x0D
#define ENC28J60_ETXSTH 0x0E
#define ENC28J60_ERXFCON 0x0F
#define ENC28J60_MACON1 0x10
#define ENC28J60_MACON3 0x11
#define ENC28J60_MACON4 0x12
#define ENC28J60_MABBIPG 0x13
#define ENC28J60_MAIPGL 0x14
#define ENC28J60_MAIPGH 0x15

#define ENC28J60_REV_B1 1
#define ENC28J60_REV_B7 7

#define ERXFCON_UCEN 0x01
#define ERXFCON_CRCEN 0x02
#define ERXFCON_BCEN 0x04
#define ERXFCON_MCEN 0x08

#define MACON1_MARXEN 0x01
#define MACON1_RXPAUS 0x02
#define MACON1_TXPAUS 0x04

#define MACON3_PADCFG0 0x01
#define MACON3_TXCRCEN 0x02
#define MACON3_FRMLNEN 0x04

#define MACON4_DEFER 0x01

#define ECON1_RXEN 0x01


int enc28j60_set_mediator(emac_enc28j60_t *emac, esp_eth_mediator_t *eth) {
    if (!eth) {
        return ESP_ERR_INVALID_ARG;
    }
    emac->eth = eth;
    return ESP_OK;
}

int enc28j60_verify_id(emac_enc28j60_t *emac) {
    uint8_t revision;
    if (enc28j60_register_read(ENC28J60_EREVID, &revision, 1) != 0) {
        return ESP_FAIL;
    }
    emac->revision = revision;
    Serial.print("revision: ");
    Serial.println(emac->revision);
    if (emac->revision < ENC28J60_REV_B1 || emac->revision > ENC28J60_REV_B7) {
        return ESP_ERR_INVALID_VERSION;
    }
    return ESP_OK;
}

int enc28j60_set_mac_addr(emac_enc28j60_t *emac) {
    if (enc28j60_register_write(ENC28J60_MAADR6, emac->addr[5]) != 0 ||
        enc28j60_register_write(ENC28J60_MAADR5, emac->addr[4]) != 0 ||
        enc28j60_register_write(ENC28J60_MAADR4, emac->addr[3]) != 0 ||
        enc28j60_register_write(ENC28J60_MAADR3, emac->addr[2]) != 0 ||
        enc28j60_register_write(ENC28J60_MAADR2, emac->addr[1]) != 0 ||
        enc28j60_register_write(ENC28J60_MAADR1, emac->addr[0]) != 0) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

int enc28j60_clear_multicast_table(emac_enc28j60_t *emac) {
    for (int i = 0; i < 7; i++) {
        if (enc28j60_register_write(ENC28J60_EHT0 + i, 0x00) != 0) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

int enc28j60_setup_default(emac_enc28j60_t *emac) {
    if (enc28j60_register_write(ENC28J60_ERXSTL, ENC28J60_BUF_RX_START & 0xFF) != 0 ||
        enc28j60_register_write(ENC28J60_ERXSTH, (ENC28J60_BUF_RX_START >> 8) & 0xFF) != 0 ||
        enc28j60_register_write(ENC28J60_ERXNDL, ENC28J60_BUF_RX_END & 0xFF) != 0 ||
        enc28j60_register_write(ENC28J60_ERXNDH, (ENC28J60_BUF_RX_END >> 8) & 0xFF) != 0) {
        return ESP_FAIL;
    }
    
    uint32_t erxrdpt = enc28j60_next_ptr_align_odd(ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_END);
    if (enc28j60_register_write(ENC28J60_ERXRDPTL, erxrdpt & 0xFF) != 0 ||
        enc28j60_register_write(ENC28J60_ERXRDPTH, (erxrdpt >> 8) & 0xFF) != 0) {
        return ESP_FAIL;
    }

    if (enc28j60_register_write(ENC28J60_ETXSTL, ENC28J60_BUF_TX_START & 0xFF) != 0 ||
        enc28j60_register_write(ENC28J60_ETXSTH, (ENC28J60_BUF_TX_START >> 8) & 0xFF) != 0) {
        return ESP_FAIL;
    }

    if (enc28j60_register_write(ENC28J60_ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN | ERXFCON_MCEN) != 0 ||
        enc28j60_register_write(ENC28J60_MACON1, MACON1_MARXEN | MACON1_RXPAUS | MACON1_TXPAUS) != 0 ||
        enc28j60_register_write(ENC28J60_MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN) != 0 ||
        enc28j60_register_write(ENC28J60_MACON4, MACON4_DEFER) != 0 ||
        enc28j60_register_write(ENC28J60_MABBIPG, 0x12) != 0 ||
        enc28j60_register_write(ENC28J60_MAIPGL, 0x12) != 0 ||
        enc28j60_register_write(ENC28J60_MAIPGH, 0x0C) != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


// Helper macro to check function return values
#define MAC_CHECK(a, str, label, ret_val, ...) \
    do { \
        if (!(a)) { \
            ESP_LOGE(TAG, "%s (%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            ret = ret_val; \
            goto label; \
        } \
    } while (0)

// Helper macro to check and ignore errors for non-critical operations
#define MAC_CHECK_NO_RET(a, str, label, ...) \
    do { \
        if (!(a)) { \
            ESP_LOGE(TAG, "%s (%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto label; \
        } \
    } while (0)

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    uint8_t addr[6];
    int int_gpio_num;
    SemaphoreHandle_t tx_ready_sem;
    TaskHandle_t rx_task_hdl;
    esp_timer_handle_t poll_timer;
    uint32_t poll_period_ms;
    uint8_t last_tsv_addr;
    uint8_t packets_remain;
    uint8_t revision;
    uint16_t next_packet_ptr;
} emac_enc28j60_t;

static esp_err_t emac_enc28j60_start(esp_eth_mac_t *mac) {
    esp_err_t ret = ESP_OK;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    
    // Enable interrupt
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, 0xFF) == ESP_OK, "clear EIR failed", out, ESP_FAIL);
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_PKTIE | EIE_INTIE | EIE_TXERIE) == ESP_OK, "set EIE.[PKTIE|INTIE] failed", out, ESP_FAIL);
    
    // Enable RX logic
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_RXEN) == ESP_OK, "set ECON1.RXEN failed", out, ESP_FAIL);
    
    // Set read pointer to the start of the RX buffer
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTL, 0x00) == ESP_OK, "write ERDPTL failed", out, ESP_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERDPTH, 0x00) == ESP_OK, "write ERDPTH failed", out, ESP_FAIL);

out:
    return ret;
}

static esp_err_t emac_enc28j60_stop(esp_eth_mac_t *mac) {
    esp_err_t ret = ESP_OK;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    
    // Disable interrupt
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, 0xFF) == ESP_OK, "clear EIE failed", out, ESP_FAIL);
    
    // Disable RX logic
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_RXEN) == ESP_OK, "clear ECON1.RXEN failed", out, ESP_FAIL);

out:
    return ret;
}

static esp_err_t emac_enc28j60_set_addr(esp_eth_mac_t *mac, uint8_t *addr) {
    esp_err_t ret = ESP_OK;
    MAC_CHECK(addr, "can't set mac addr to null", out, ESP_ERR_INVALID_ARG);
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    memcpy(emac->addr, addr, 6);
    MAC_CHECK(enc28j60_set_mac_addr(emac) == ESP_OK, "set mac address failed", out, ESP_FAIL);

out:
    return ret;
}

static esp_err_t emac_enc28j60_get_addr(esp_eth_mac_t *mac, uint8_t *addr) {
    esp_err_t ret = ESP_OK;
    MAC_CHECK(addr, "can't set mac addr to null", out, ESP_ERR_INVALID_ARG);
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    memcpy(addr, emac->addr, 6);

out:
    return ret;
}

static inline esp_err_t emac_enc28j60_get_tsv(emac_enc28j60_t *emac, enc28j60_tsv_t *tsv) {
    return enc28j60_read_packet(emac, emac->last_tsv_addr, (uint8_t *)tsv, ENC28J60_TSV_SIZE);
}

static void enc28j60_isr_handler(void *arg) {
    emac_enc28j60_t *emac = (emac_enc28j60_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    // Notify ENC28J60 task
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void enc28j60_poll_timer(void *arg) {
    emac_enc28j60_t *emac = (emac_enc28j60_t *)arg;
    xTaskNotifyGive(emac->rx_task_hdl);
}

static void emac_enc28j60_task(void *arg) {
    emac_enc28j60_t *emac = (emac_enc28j60_t *)arg;
    uint8_t status = 0;
    uint8_t mask = 0;
    uint8_t *buffer = NULL;
    uint32_t length = 0;

    while (1) {
    loop_start:
        if (emac->int_gpio_num >= 0) {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 && gpio_get_level(emac->int_gpio_num) != 0) {
                continue;
            }
        } else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        
        MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, EIE_INTIE) == ESP_OK, "clear EIE_INTIE failed", loop_start);
        MAC_CHECK_NO_RET(enc28j60_do_register_read(emac, true, ENC28J60_EIR, &status) == ESP_OK, "read EIR failed", loop_end);
        MAC_CHECK_NO_RET(enc28j60_do_register_read(emac, true, ENC28J60_EIE, &mask) == ESP_OK, "read EIE failed", loop_end);
        status &= mask;

        if (status == 0) {
            uint8_t pk_counter;
            MAC_CHECK_NO_RET(enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pk_counter) == ESP_OK, "read EPKTCNT failed", loop_end);
            if (pk_counter > 0) {
                status = EIR_PKTIF;
            } else {
                goto loop_end;
            }
        }

        if (status & EIR_PKTIF) {
            do {
                length = ETH_MAX_PACKET_SIZE;
                buffer = heap_caps_malloc(length, MALLOC_CAP_DMA);
                if (!buffer) {
                    ESP_LOGE(TAG, "no mem for receive buffer");
                } else if (emac->parent.receive(&emac->parent, buffer, &length) == ESP_OK) {
                    if (length) {
                        emac->eth->stack_input(emac->eth, buffer, length);
                    } else {
                        free(buffer);
                    }
                } else {
                    free(buffer);
                }
            } while (emac->packets_remain);
        }

        if (status & EIR_TXERIF) {
            MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRST) == ESP_OK, "set TXRST failed", loop_end);
            MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_ECON1, ECON1_TXRST) == ESP_OK, "clear TXRST failed", loop_end);
            MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXERIF) == ESP_OK, "clear TXERIF failed", loop_end);

            if (emac->revision == ENC28J60_REV_B5 || emac->revision == ENC28J60_REV_B7) {
                __attribute__((aligned(4))) enc28j60_tsv_t tx_status;
                MAC_CHECK_NO_RET(emac_enc28j60_get_tsv(emac, &tx_status) == ESP_OK, "get Tx Status Vector failed", loop_end);
                if (tx_status.late_collision) {
                    MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) == ESP_OK, "clear TXIF failed", loop_end);
                    MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_INTIE) == ESP_OK, "set INTIE failed", loop_end);
                    MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRTS) == ESP_OK, "set TXRTS failed", loop_end);
                    continue;
                }
            }
        }

        if (status & EIR_TXIF) {
            MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) == ESP_OK, "clear TXIF failed", loop_end);
            MAC_CHECK_NO_RET(enc28j60_do_bitwise_clr(emac, ENC28J60_EIE, EIE_TXIE) == ESP_OK, "clear TXIE failed", loop_end);

            xSemaphoreGive(emac->tx_ready_sem);
        }
    loop_end:
        MAC_CHECK_NO_RET(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_INTIE) == ESP_OK, "clear INTIE failed", loop_start);
    }
    vTaskDelete(NULL);
}

static esp_err_t emac_enc28j60_set_link(esp_eth_mac_t *mac, eth_link_t link) {
    esp_err_t ret = ESP_OK;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);

    switch (link) {
    case ETH_LINK_UP:
        MAC_CHECK(mac->start(mac) == ESP_OK, "enc28j60 start failed", out, ESP_FAIL);
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_start_periodic(emac->poll_timer, emac->poll_period_ms * 1000), out, TAG, "start poll timer failed");
        }
        break;
    case ETH_LINK_DOWN:
        MAC_CHECK(mac->stop(mac) == ESP_OK, "enc28j60 stop failed", out, ESP_FAIL);
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_stop(emac->poll_timer), out, TAG, "stop poll timer failed");
        }
        break;
    default:
        MAC_CHECK(false, "unknown link status", out, ESP_ERR_INVALID_ARG);
        break;
    }

out:
    return ret;
}


static esp_err_t emac_enc28j60_set_speed(esp_eth_mac_t *mac, eth_speed_t speed) {
    esp_err_t ret = ESP_OK;
    switch (speed) {
    case ETH_SPEED_10M:
        ESP_LOGI(TAG, "working in 10Mbps");
        break;
    default:
        MAC_CHECK(false, "100Mbps unsupported", out, ESP_ERR_NOT_SUPPORTED);
        break;
    }
out:
    return ret;
}

static esp_err_t emac_enc28j60_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex) {
    esp_err_t ret = ESP_OK;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    uint8_t mac3 = 0;
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_MACON3, &mac3) == ESP_OK, "read MACON3 failed", out, ESP_FAIL);

    switch (duplex) {
    case ETH_DUPLEX_HALF:
        mac3 &= ~MACON3_FULDPX;
        MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MABBIPG, 0x12) == ESP_OK, "write MABBIPG failed", out, ESP_FAIL);
        ESP_LOGI(TAG, "working in half duplex");
        break;
    case ETH_DUPLEX_FULL:
        mac3 |= MACON3_FULDPX;
        MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MABBIPG, 0x15) == ESP_OK, "write MABBIPG failed", out, ESP_FAIL);
        ESP_LOGI(TAG, "working in full duplex");
        break;
    default:
        MAC_CHECK(false, "unknown duplex", out, ESP_ERR_INVALID_ARG);
        break;
    }
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_MACON3, mac3) == ESP_OK, "write MACON3 failed", out, ESP_FAIL);
out:
    return ret;
}

static esp_err_t emac_enc28j60_set_promiscuous(esp_eth_mac_t *mac, bool enable) {
    esp_err_t ret = ESP_OK;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    if (enable) {
        MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXFCON, 0x00) == ESP_OK, "write ERXFCON failed", out, ESP_FAIL);
    }
out:
    return ret;
}

static esp_err_t emac_enc28j60_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length) {
    esp_err_t ret = ESP_OK;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    uint8_t econ1 = 0;

    // Check if the ENC28J60 is ready for transmission
    if (xSemaphoreTake(emac->tx_ready_sem, pdMS_TO_TICKS(ENC28J60_TX_READY_TIMEOUT_MS)) == pdFALSE) {
        ESP_LOGW(TAG, "tx_ready_sem expired");
    }
    MAC_CHECK(enc28j60_do_register_read(emac, true, ENC28J60_ECON1, &econ1) == ESP_OK, "read ECON1 failed", out, ESP_FAIL);
    MAC_CHECK(!(econ1 & ECON1_TXRTS), "last transmit still in progress", out, ESP_ERR_INVALID_STATE);

    // Set the write pointer to the start of the transmit buffer area
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_EWRPTL, ENC28J60_BUF_TX_START & 0xFF) == ESP_OK, "write EWRPTL failed", out, ESP_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_EWRPTH, (ENC28J60_BUF_TX_START & 0xFF00) >> 8) == ESP_OK, "write EWRPTH failed", out, ESP_FAIL);

    // Set the end pointer to correspond to the packet size given
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ETXNDL, (ENC28J60_BUF_TX_START + length) & 0xFF) == ESP_OK, "write ETXNDL failed", out, ESP_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ETXNDH, ((ENC28J60_BUF_TX_START + length) & 0xFF00) >> 8) == ESP_OK, "write ETXNDH failed", out, ESP_FAIL);

    // Copy data to transmit memory
    uint8_t per_pkt_control = 0; // MACON3 will be used to determine how the packet will be transmitted
    MAC_CHECK(enc28j60_do_memory_write(emac, &per_pkt_control, 1) == ESP_OK, "write packet control byte failed", out, ESP_FAIL);
    MAC_CHECK(enc28j60_do_memory_write(emac, buf, length) == ESP_OK, "buffer memory write failed", out, ESP_FAIL);
    emac->last_tsv_addr = ENC28J60_BUF_TX_START + length + 1;

    // Enable Tx Interrupt to indicate next Tx ready state
    MAC_CHECK(enc28j60_do_bitwise_clr(emac, ENC28J60_EIR, EIR_TXIF) == ESP_OK, "set EIR_TXIF failed", out, ESP_FAIL);
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_EIE, EIE_TXIE) == ESP_OK, "set EIE_TXIE failed", out, ESP_FAIL);

    // Issue tx polling command
    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON1, ECON1_TXRTS) == ESP_OK, "set ECON1.TXRTS failed", out, ESP_FAIL);
out:
    return ret;
}

static esp_err_t emac_enc28j60_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length) {
    esp_err_t ret = ESP_OK;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    uint8_t pk_counter = 0;
    uint16_t rx_len = 0;
    uint32_t next_packet_addr = 0;
    __attribute__((aligned(4))) enc28j60_rx_header_t header; // SPI driver needs the rx buffer 4 byte align

    // Read packet header
    MAC_CHECK(enc28j60_read_packet(emac, emac->next_packet_ptr, (uint8_t *)&header, sizeof(header)) == ESP_OK, "read header failed", out, ESP_FAIL);

    // Get packet length and address
    rx_len = header.length_low + (header.length_high << 8);
    next_packet_addr = header.next_packet_low + (header.next_packet_high << 8);

    // Read packet content
    MAC_CHECK(enc28j60_read_packet(emac, enc28j60_rx_packet_start(emac->next_packet_ptr, ENC28J60_RSV_SIZE), buf, rx_len) == ESP_OK, "read packet content failed", out, ESP_FAIL);

    // Free receive buffer space
    uint32_t erxrdpt = enc28j60_next_ptr_align_odd(next_packet_addr, ENC28J60_BUF_RX_START, ENC28J60_BUF_RX_END);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTL, (erxrdpt & 0xFF)) == ESP_OK, "write ERXRDPTL failed", out, ESP_FAIL);
    MAC_CHECK(enc28j60_register_write(emac, ENC28J60_ERXRDPTH, (erxrdpt & 0xFF00) >> 8) == ESP_OK, "write ERXRDPTH failed", out, ESP_FAIL);
    emac->next_packet_ptr = next_packet_addr;

    MAC_CHECK(enc28j60_do_bitwise_set(emac, ENC28J60_ECON2, ECON2_PKTDEC) == ESP_OK, "set ECON2.PKTDEC failed", out, ESP_FAIL);
    MAC_CHECK(enc28j60_register_read(emac, ENC28J60_EPKTCNT, &pk_counter) == ESP_OK, "read EPKTCNT failed", out, ESP_FAIL);

    *length = rx_len - 4; // Subtract the CRC length
    emac->packets_remain = pk_counter > 0;
out:
    return ret;
}


/**
 * @brief Get chip info
 */
eth_enc28j60_rev_t emac_enc28j60_get_chip_info(esp_eth_mac_t *mac)
{
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    return emac->revision;
}

static esp_err_t emac_enc28j60_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    esp_eth_mediator_t *eth = emac->eth;

    /* Init GPIO for ENC28J60 interrupt */
    if (emac->int_gpio_num >= 0) {
        gpio_reset_pin(emac->int_gpio_num);
        gpio_set_direction(emac->int_gpio_num, GPIO_MODE_INPUT);
        gpio_set_pull_mode(emac->int_gpio_num, GPIO_PULLUP_ONLY);
        gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_NEGEDGE);
        gpio_intr_enable(emac->int_gpio_num);
        gpio_isr_handler_add(emac->int_gpio_num, enc28j60_isr_handler, emac);
    }

    /* Notify ETH mediator that low-level initialization is starting */
    MAC_CHECK(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL) == ESP_OK,
              "Low-level init failed", out, ESP_FAIL);

    /* Reset ENC28J60 */
    MAC_CHECK(enc28j60_do_reset(emac) == ESP_OK, "Reset ENC28J60 failed", out, ESP_FAIL);
    
    /* Verify chip ID */
    MAC_CHECK(enc28j60_verify_id(emac) == ESP_OK, "Verify chip ID failed", out, ESP_FAIL);

    /* Default setup of internal registers */
    MAC_CHECK(enc28j60_setup_default(emac) == ESP_OK, "ENC28J60 default setup failed", out, ESP_FAIL);

    /* Clear multicast hash table */
    MAC_CHECK(enc28j60_clear_multicast_table(emac) == ESP_OK, "Clear multicast table failed", out, ESP_FAIL);

    return ESP_OK;

out:
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
        gpio_reset_pin(emac->int_gpio_num);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

static esp_err_t emac_enc28j60_deinit(esp_eth_mac_t *mac)
{
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);
    esp_eth_mediator_t *eth = emac->eth;

    mac->stop(mac); // Stop the MAC

    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
        gpio_reset_pin(emac->int_gpio_num);
    }

    if (emac->poll_timer && esp_timer_is_active(emac->poll_timer)) {
        esp_timer_stop(emac->poll_timer);
    }

    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ESP_OK;
}

static esp_err_t emac_enc28j60_del(esp_eth_mac_t *mac)
{
    emac_enc28j60_t *emac = __containerof(mac, emac_enc28j60_t, parent);

    if (emac->poll_timer) {
        esp_timer_delete(emac->poll_timer);
    }

    if (emac->rx_task_hdl) {
        vTaskDelete(emac->rx_task_hdl);
    }

    if (emac->spi.ctx) {
        emac->spi.deinit(emac->spi.ctx);
    }

    if (emac->reg_trans_lock) {
        vSemaphoreDelete(emac->reg_trans_lock);
    }

    if (emac->tx_ready_sem) {
        vSemaphoreDelete(emac->tx_ready_sem);
    }

    free(emac);
    return ESP_OK;
}

esp_eth_mac_t *esp_eth_mac_new_enc28j60(const eth_enc28j60_config_t *enc28j60_config, const eth_mac_config_t *mac_config)
{
    esp_eth_mac_t *ret = NULL;
    emac_enc28j60_t *emac = NULL;

    MAC_CHECK(enc28j60_config, "ENC28J60 config cannot be null", err, NULL);
    MAC_CHECK(mac_config, "MAC config cannot be null", err, NULL);

    emac = calloc(1, sizeof(emac_enc28j60_t));
    MAC_CHECK(emac, "Failed to allocate memory for EMAC", err, NULL);

    /* Check configuration arguments */
    MAC_CHECK((enc28j60_config->int_gpio_num >= 0) != (enc28j60_config->poll_period_ms > 0), "Invalid config arguments", err, NULL);

    /* Initialize ENC28J60 driver instance */
    emac->last_bank = 0xFF;
    emac->next_packet_ptr = ENC28J60_BUF_RX_START;
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->int_gpio_num = enc28j60_config->int_gpio_num;
    emac->poll_period_ms = enc28j60_config->poll_period_ms;

    /* Bind methods and attributes */
    emac->parent.set_mediator = emac_enc28j60_set_mediator;
    emac->parent.init = emac_enc28j60_init;
    emac->parent.deinit = emac_enc28j60_deinit;
    emac->parent.start = emac_enc28j60_start;
    emac->parent.stop = emac_enc28j60_stop;
    emac->parent.del = emac_enc28j60_del;
    emac->parent.write_phy_reg = emac_enc28j60_write_phy_reg;
    emac->parent.read_phy_reg = emac_enc28j60_read_phy_reg;
    emac->parent.set_addr = emac_enc28j60_set_addr;
    emac->parent.get_addr = emac_enc28j60_get_addr;
    emac->parent.set_speed = emac_enc28j60_set_speed;
    emac->parent.set_duplex = emac_enc28j60_set_duplex;
    emac->parent.set_link = emac_enc28j60_set_link;
    emac->parent.set_promiscuous = emac_enc28j60_set_promiscuous;
    emac->parent.transmit = emac_enc28j60_transmit;
    emac->parent.receive = emac_enc28j60_receive;

    /* SPI Driver Initialization */
    if (enc28j60_config->custom_spi_driver.init != NULL && enc28j60_config->custom_spi_driver.deinit != NULL
        && enc28j60_config->custom_spi_driver.read != NULL && enc28j60_config->custom_spi_driver.write != NULL) {
        ESP_LOGD(TAG, "Using custom SPI driver");
        emac->spi.init = enc28j60_config->custom_spi_driver.init;
        emac->spi.deinit = enc28j60_config->custom_spi_driver.deinit;
        emac->spi.read = enc28j60_config->custom_spi_driver.read;
        emac->spi.write = enc28j60_config->custom_spi_driver.write;
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(enc28j60_config->custom_spi_driver.config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    } else {
        ESP_LOGD(TAG, "Using default SPI driver");
        emac->spi.init = enc28j60_spi_init;
        emac->spi.deinit = enc28j60_spi_deinit;
        emac->spi.read = enc28j60_spi_read;
        emac->spi.write = enc28j60_spi_write;
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(enc28j60_config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    }

    /* Create synchronization primitives */
    emac->reg_trans_lock = xSemaphoreCreateMutex();
    MAC_CHECK(emac->reg_trans_lock, "Failed to create register transaction lock", err, NULL);
    emac->tx_ready_sem = xSemaphoreCreateBinary();
    MAC_CHECK(emac->tx_ready_sem, "Failed to create TX ready semaphore", err, NULL);
    xSemaphoreGive(emac->tx_ready_sem); // Ensure the first transmit is performed without waiting

    /* Create ENC28J60 task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_enc28j60_task, "enc28j60_tsk", mac_config->rx_task_stack_size, emac,
                           mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
    MAC_CHECK(xReturned == pdPASS, "Failed to create ENC28J60 task", err, NULL);

    /* Create polling timer if not using interrupts */
    if (emac->int_gpio_num < 0) {
        const esp_timer_create_args_t poll_timer_args = {
            .callback = enc28j60_poll_timer,
            .name = "emac_spi_poll_timer",
            .arg = emac,
            .skip_unhandled_events = true
        };
        ESP_GOTO_ON_FALSE(esp_timer_create(&poll_timer_args, &emac->poll_timer) == ESP_OK, NULL, err, TAG, "Failed to create poll timer");
    }

    return &(emac->parent);

err:
    if (emac) {
        if (emac->poll_timer) {
            esp_timer_delete(emac->poll_timer);
        }
        if (emac->rx_task_hdl) {
            vTaskDelete(emac->rx_task_hdl);
        }
        if (emac->spi.ctx) {
            emac->spi.deinit(emac->spi.ctx);
        }
        if (emac->reg_trans_lock) {
            vSemaphoreDelete(emac->reg_trans_lock);
        }
        if (emac->tx_ready_sem) {
            vSemaphoreDelete(emac->tx_ready_sem);
        }
        free(emac);
    }
    return ret;
}