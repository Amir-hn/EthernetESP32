// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ENC28J60_DEFINITIONS_H
#define ENC28J60_DEFINITIONS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SPI Instruction Set
 *
 */
#define ENC28J60_SPI_CMD_RCR (0x00) // Read Control Register
#define ENC28J60_SPI_CMD_RBM (0x01) // Read Buffer Memory
#define ENC28J60_SPI_CMD_WCR (0x02) // Write Control Register
#define ENC28J60_SPI_CMD_WBM (0x03) // Write Buffer Memory
#define ENC28J60_SPI_CMD_BFS (0x04) // Bit Field Set
#define ENC28J60_SPI_CMD_BFC (0x05) // Bit Field Clear
#define ENC28J60_SPI_CMD_SRC (0x07) // Soft Reset

/**
 * @brief Shared Registers in ENC28J60 (accessible on each bank)
 *
 */
#define ENC28J60_EIE      (0x1B) // Ethernet Interrupt Enable
#define ENC28J60_EIR      (0x1C) // Ethernet Interrupt flags
#define ENC28J60_ESTAT    (0x1D) // Ethernet Status
#define ENC28J60_ECON2    (0x1E) // Ethernet Control Register2
#define ENC28J60_ECON1    (0x1F) // Ethernet Control Register1

/**
 * @brief Per-bank Registers in ENC28J60
 * @note Address[15:12]: Register Type, 0 -> ETH, 1 -> MII/MAC
 *       Address[11:8] : Bank address
 *       Address[7:0]  : Register Index
 */
// Bank 0 Registers
#define ENC28J60_ERDPTL   (0x0000) // Read Pointer Low Byte (ERDPT<7:0>)
#define ENC28J60_ERDPTH   (0x0001) // Read Pointer High Byte (ERDPT<12:8>)
#define ENC28J60_EWRPTL   (0x0002) // Write Pointer Low Byte (EWRPT<7:0>)
#define ENC28J60_EWRPTH   (0x0003) // Write Pointer High Byte (EWRPT<12:8>)
#define ENC28J60_ETXSTL   (0x0004) // TX Start Low Byte (ETXST<7:0>)
#define ENC28J60_ETXSTH   (0x0005) // TX Start High Byte (ETXST<12:8>)
#define ENC28J60_ETXNDL   (0x0006) // TX End Low Byte (ETXND<7:0>)
#define ENC28J60_ETXNDH   (0x0007) // TX End High Byte (ETXND<12:8>)
#define ENC28J60_ERXSTL   (0x0008) // RX Start Low Byte (ERXST<7:0>)
#define ENC28J60_ERXSTH   (0x0009) // RX Start High Byte (ERXST<12:8>)
#define ENC28J60_ERXNDL   (0x000A) // RX End Low Byte (ERXND<7:0>)
#define ENC28J60_ERXNDH   (0x000B) // RX End High Byte (ERXND<12:8>)
#define ENC28J60_ERXRDPTL (0x000C) // RX RD Pointer Low Byte (ERXRDPT<7:0>)
#define ENC28J60_ERXRDPTH (0x000D) // RX RD Pointer High Byte (ERXRDPT<12:8>)
#define ENC28J60_ERXWRPTL (0x000E) // RX WR Pointer Low Byte (ERXWRPT<7:0>)
#define ENC28J60_ERXWRPTH (0x000F) // RX WR Pointer High Byte (ERXWRPT<12:8>)
#define ENC28J60_EDMASTL  (0x0010) // DMA Start Low Byte (EDMAST<7:0>)
#define ENC28J60_EDMASTH  (0x0011) // DMA Start High Byte (EDMAST<12:8>)
#define ENC28J60_EDMANDL  (0x0012) // DMA End Low Byte (EDMAND<7:0>)
#define ENC28J60_EDMANDH  (0x0013) // DMA End High Byte (EDMAND<12:8>)
#define ENC28J60_EDMADSTL (0x0014) // DMA Destination Low Byte (EDMADST<7:0>)
#define ENC28J60_EDMADSTH (0x0015) // DMA Destination High Byte (EDMADST<12:8>)
#define ENC28J60_EDMACSL  (0x0016) // DMA Checksum Low Byte (EDMACS<7:0>)
#define ENC28J60_EDMACSH  (0x0017) // DMA Checksum High Byte (EDMACS<15:8>)

// Bank 1 Registers
#define ENC28J60_EHT0     (0x0100) // Hash Table Byte 0 (EHT<7:0>)
#define ENC28J60_EHT1     (0x0101) // Hash Table Byte 1 (EHT<15:8>)
#define ENC28J60_EHT2     (0x0102) // Hash Table Byte 2 (EHT<23:16>)
#define ENC28J60_EHT3     (0x0103) // Hash Table Byte 3 (EHT<31:24>)
#define ENC28J60_EHT4     (0x0104) // Hash Table Byte 4 (EHT<39:32>)
#define ENC28J60_EHT5     (0x0105) // Hash Table Byte 5 (EHT<47:40>)
#define ENC28J60_EHT6     (0x0106) // Hash Table Byte 6 (EHT<55:48>)
#define ENC28J60_EHT7     (0x0107) // Hash Table Byte 7 (EHT<63:56>)
#define ENC28J60_EPMM0    (0x0108) // Pattern Match Mask Byte 0 (EPMM<7:0>)
#define ENC28J60_EPMM1    (0x0109) // Pattern Match Mask Byte 1 (EPMM<15:8>)
#define ENC28J60_EPMM2    (0x010A) // Pattern Match Mask Byte 2 (EPMM<23:16>)
#define ENC28J60_EPMM3    (0x010B) // Pattern Match Mask Byte 3 (EPMM<31:24>)
#define ENC28J60_EPMM4    (0x010C) // Pattern Match Mask Byte 4 (EPMM<39:32>)
#define ENC28J60_EPMM5    (0x010D) // Pattern Match Mask Byte 5 (EPMM<47:40>)
#define ENC28J60_EPMM6    (0x010E) // Pattern Match Mask Byte 6 (EPMM<55:48>)
#define ENC28J60_EPMM7    (0x010F) // Pattern Match Mask Byte 7 (EPMM<63:56>)
#define ENC28J60_EPMCSL   (0x0110) // Pattern Match Checksum Low Byte (EPMCS<7:0>)
#define ENC28J60_EPMCSH   (0x0111) // Pattern Match Checksum High Byte (EPMCS<15:0>)
#define ENC28J60_EPMOL    (0x0114) // Pattern Match Offset Low Byte (EPMO<7:0>)
#define ENC28J60_EPMOH    (0x0115) // Pattern Match Offset High Byte (EPMO<12:8>)
#define ENC28J60_ERXFCON  (0x0118) // Receive Filter Control
#define ENC28J60_EPKTCNT  (0x0119) // Ethernet Packet Count

// Bank 2 Registers
#define ENC28J60_MACON1   (0x1200) // MAC Control Register 1
#define ENC28J60_MACON2   (0x1201) // MAC Control Register 2
#define ENC28J60_MACON3   (0x1202) // MAC Control Register 3
#define ENC28J60_MACON4   (0x1203) // MAC Control Register 4
#define ENC28J60_MABBIPG  (0x1204) // MAC Back-to-Back Inter-Packet Gap
#define ENC28J60_MAIPGL   (0x1206) // MAC Inter-Packet Gap Low Byte
#define ENC28J60_MAIPGH   (0x1207) // MAC Inter-Packet Gap High Byte
#define ENC28J60_MACLCON  (0x1208) // MAC Link Control
#define ENC28J60_MACTOL   (0x1209) // MAC Time-out Low Byte
#define ENC28J60_MACTOH   (0x120A) // MAC Time-out High Byte
#define ENC28J60_MACTON   (0x120B) // MAC Time-out 2 Low Byte
#define ENC28J60_MACTOH2  (0x120C) // MAC Time-out 2 High Byte

// Bank 3 Registers
#define ENC28J60_MIREGADR (0x1400) // MII Register Address
#define ENC28J60_MIWRL    (0x1402) // MII Write Data Low Byte
#define ENC28J60_MIWRH    (0x1403) // MII Write Data High Byte
#define ENC28J60_MIRD     (0x1404) // MII Read Data

/**
 * @brief Initialization and Reset Commands
 *
 */
#define ENC28J60_SOFT_RESET()    \
  do {                           \
    ENC28J60_WRITE_REG(ENC28J60_ECON1, 0x00); \
    ENC28J60_WRITE_REG(ENC28J60_ECON1, 0x80); \
  } while (0)

/**
 * @brief Function to write data to ENC28J60
 *
 * @param reg Register address
 * @param data Data to be written
 */
void ENC28J60_WRITE_REG(uint8_t reg, uint8_t data);

/**
 * @brief Function to read data from ENC28J60
 *
 * @param reg Register address
 * @return Data read from the register
 */
uint8_t ENC28J60_READ_REG(uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif // ENC28J60_DEFINITIONS_H
