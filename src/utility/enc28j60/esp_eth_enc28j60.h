// SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ETH_ENC28J60_H
#define ETH_ENC28J60_H

#ifdef __cplusplus
extern "C" {
#endif

#include <SPI.h>

#define CS_HOLD_TIME_MIN_NS 210

/**
 * @brief ENC28J60 specific configuration
 *
 */
typedef struct {
    uint8_t spi_host_id;              /*!< SPI peripheral */
    SPISettings *spi_devcfg;          /*!< SPI device configuration */
    int int_gpio_num;                 /*!< Interrupt GPIO number */
    uint32_t poll_period_ms;          /*!< Period in ms to poll rx status when interrupt mode is not used */
} eth_enc28j60_config_t;

/**
 * @brief ENC28J60 Supported Revisions
 *
 */
typedef enum {
    ENC28J60_REV_B1 = 0b00000010,
    ENC28J60_REV_B4 = 0b00000100,
    ENC28J60_REV_B5 = 0b00000101,
    ENC28J60_REV_B7 = 0b00000110
} eth_enc28j60_rev_t;

/**
 * @brief Default ENC28J60 specific configuration
 *
 */
#define ETH_ENC28J60_DEFAULT_CONFIG(spi_host, spi_devcfg_p) \
    {                                                      \
        .spi_host_id = spi_host,                           \
        .spi_devcfg = spi_devcfg_p,                        \
        .int_gpio_num = 4,                                 \
        .poll_period_ms = 0,                               \
    }

/**
 * @brief Compute amount of SPI bit-cycles the CS should stay active after the transmission
 *        to meet ENC28J60 CS Hold Time specification.
 *
 * @param clock_speed_mhz SPI Clock frequency in MHz (valid range is <1, 20>)
 * @return uint8_t
 */
static inline uint8_t enc28j60_cal_spi_cs_hold_time(int clock_speed_mhz)
{
    if (clock_speed_mhz <= 0 || clock_speed_mhz > 20) {
        return 0;
    }
    int temp = clock_speed_mhz * CS_HOLD_TIME_MIN_NS;
    uint8_t cs_posttrans = temp / 1000;
    if (temp % 1000) {
        cs_posttrans += 1;
    }

    return cs_posttrans;
}

/**
* @brief Create ENC28J60 Ethernet MAC instance
*
* @param[in] enc28j60_config: ENC28J60 specific configuration
* @param[in] mac_config: Ethernet MAC configuration
*
* @return
*      - instance: create MAC instance successfully
*      - NULL: create MAC instance failed because some error occurred
*/
void* esp_eth_mac_new_enc28j60(const eth_enc28j60_config_t *enc28j60_config, const void *mac_config);

/**
* @brief Create a PHY instance of ENC28J60
*
* @param[in] config: configuration of PHY
*
* @return
*      - instance: create PHY instance successfully
*      - NULL: create PHY instance failed because some error occurred
*/
void* esp_eth_phy_new_enc28j60(const void *config);

/**
 * @brief Get ENC28J60 silicon revision ID
 *
 * @param mac ENC28J60 MAC Handle
 * @return eth_enc28j60_rev_t
 *           - returns silicon revision ID read during initialization
 */
eth_enc28j60_rev_t emac_enc28j60_get_chip_info(void* mac);

#ifdef __cplusplus
}
#endif

#endif // ETH_ENC28J60_H
