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
#include <Arduino.h>

class EthDriver {
public:
  EthDriver() : mac(NULL), phy(NULL), phyAddr(0) {}

  void begin() {
    if (mac == NULL) {
      mac = newMAC();
      phy = newPHY();
    }
  }

  void end() {
    if (mac != NULL) {
      mac->del(mac);
      mac = NULL;
    }
    if (phy != NULL) {
      phy->del(phy);
      phy = NULL;
    }
  }

  void setPhyAddress(int32_t addr) {
    phyAddr = addr;
  }

  ~EthDriver() {
    end();
  }

protected:
  virtual esp_eth_mac_t *newMAC() = 0;
  virtual esp_eth_phy_t *newPHY() = 0;

  esp_eth_mac_t *mac;
  esp_eth_phy_t *phy;
  int32_t phyAddr;
};

class EthSpiDriver : public EthDriver {
public:
  EthSpiDriver(SPIClass *spi) : spi(spi) {}

  void initCustomSPI(eth_spi_custom_driver_config_t &customSPI) {
    customSPI.config = this;
    customSPI.init = eth_spi_init;
    customSPI.deinit = eth_spi_deinit;
    customSPI.read = eth_spi_read;
    customSPI.write = eth_spi_write;
  }

  bool read(uint32_t cmd, uint32_t addr, void *data, uint32_t data_len) {
    // Implement your SPI read logic here
    return true;
  }

  bool write(uint32_t cmd, uint32_t addr, const void *data, uint32_t data_len) {
    // Implement your SPI write logic here
    return true;
  }

private:
  SPIClass *spi;
};

void *eth_spi_init(const void *ctx) {
  return (void *)ctx;
}

bool eth_spi_deinit(void *ctx) {
  return true;
}

bool eth_spi_read(void *ctx, uint32_t cmd, uint32_t addr, void *data, uint32_t data_len) {
  return ((EthSpiDriver *)ctx)->read(cmd, addr, data, data_len);
}

bool eth_spi_write(void *ctx, uint32_t cmd, uint32_t addr, const void *data, uint32_t data_len) {
  return ((EthSpiDriver *)ctx)->write(cmd, addr, data, data_len);
}
