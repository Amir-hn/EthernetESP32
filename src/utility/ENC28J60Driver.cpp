/*
 This file is part of the EthernetESP32 library for Arduino
 https://github.com/Networking-for-Arduino/EthernetESP32
 Copyright 2024 Juraj Andrassy

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <SPI.h>

class ENC28J60Driver {
public:
  ENC28J60Driver(SPIClass *spi, int pinCS, int pinIRQ, int pinRst, int spiFreq = 8, int phyAddr = 1)
      : spi(spi), pinCS(pinCS), pinIRQ(pinIRQ), pinRst(pinRst), spiFreq(spiFreq), phyAddr(phyAddr) {}

  void initCustomSPI(void *custom_spi_driver) {}

  esp_eth_mac_t *newMAC() {
    pinMode(pinCS, OUTPUT);
    digitalWrite(pinCS, HIGH);

    spi->begin();

    eth_enc28j60_config_t mac_config;
    mac_config.int_gpio_num = pinIRQ;
    mac_config.poll_period_ms = (pinIRQ < 0) ? 10 : 0;
    initCustomSPI(mac_config.custom_spi_driver);

    eth_mac_config_t eth_mac_config = ETH_MAC_DEFAULT_CONFIG();
    return esp_eth_mac_new_enc28j60(&mac_config, &eth_mac_config);
  }

  esp_eth_phy_t *newPHY() {
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = phyAddr;
    phy_config.reset_gpio_num = pinRst;

    return esp_eth_phy_new_enc28j60(&phy_config);
  }

  bool read(uint32_t cmd, uint32_t addr, void *data, uint32_t data_len) {
    spi->beginTransaction(SPISettings(1000000L * spiFreq, MSBFIRST, SPI_MODE0));
    digitalWrite(pinCS, LOW);

    spi->write((cmd << 5) | addr);
    spi->transfer(NULL, (uint8_t *)data, data_len);

    digitalWrite(pinCS, HIGH);
    spi->endTransaction();
    return true;
  }

  bool write(uint32_t cmd, uint32_t addr, const void *data, uint32_t data_len) {
    spi->beginTransaction(SPISettings(1000000L * spiFreq, MSBFIRST, SPI_MODE0));
    digitalWrite(pinCS, LOW);

    spi->write((cmd << 5) | addr);
    spi->transfer((const uint8_t *)data, data_len);

    digitalWrite(pinCS, HIGH);
    spi->endTransaction();
    return true;
  }

private:
  SPIClass *spi;
  int pinCS;
  int pinIRQ;
  int pinRst;
  int spiFreq;
  int phyAddr;
};
