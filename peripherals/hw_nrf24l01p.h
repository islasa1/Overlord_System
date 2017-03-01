/**
 * \file hw_nrf24l01p.h
 * Hardware registers for Nordic Semiconductors nRF24L01+ ultra low power 2.4GHz
 * RF Transceiver
 *
 *  Created on: Feb 21, 2017
 *      Author: Anthony
 */

#ifndef __PERIPHERALS_HW_NRF24L01P_H__
#define __PERIPHERALS_HW_NRF24L01P_H__

//******************************************************************************
//
// The following are defines for the nRF24L01+ register addresses
//
//******************************************************************************
#define NRF24L01P_O_CONFIG      0x00    // Configuration Register
#define NRF24L01P_O_EN_AA       0x01    // Enhanced Shockburst™ Auto Acknowledge
#define NRF24L01P_O_EN_RXADDR   0x02    // Enable RX Addresses
#define NRF24L01P_O_SETUP_AW    0x03    // Setup of Address widths (common for
                                        // all data pipes)
#define NRF24L01P_O_SETUP_RETR  0x04    // Setup of automatic retransmission
#define NRF24L01P_O_RF_CH       0x05    // RF Channel - [6:0] sets the frequency
                                        // channel nRF24L01+ operates on
#define NRF24L01P_O_RF_SETUP    0x06    // RF Setup register
#define NRF24L01P_O_STATUS      0x07    // Status register
#define NRF24L01P_O_OBSERVE_TX  0x08    // Transmit observe register
#define NRF24L01P_O_RPD         0x09    // Receive Power Detector (Carrier
                                        // Detect in nRF24L0)
#define NRF24L01P_O_RX_ADDR_P0  0x0A    // Receive address data pipe 0. 5 Bytes
                                        // maximum length. LSB written first.
#define NRF24L01P_O_RX_ADDR_P1  0x0B    // Receive address data pipe 1. 5 Bytes
                                        // maximum length. LSB written first.
#define NRF24L01P_O_RX_ADDR_P2  0x0C    // Receive address data pipe 2. Only
                                        // LSB. MSBytes are equal to
                                        // RX_ADDR_P1[39:8]
#define NRF24L01P_O_RX_ADDR_P3  0x0D    // Receive address data pipe 3. Only
                                        // LSB. MSBytes are equal to
                                        // RX_ADDR_P1[39:8]
#define NRF24L01P_O_RX_ADDR_P4  0x0E    // Receive address data pipe 4. Only
                                        // LSB. MSBytes are equal to
                                        // RX_ADDR_P1[39:8]
#define NRF24L01P_O_RX_ADDR_P5  0x0F    // Receive address data pipe 5. Only
                                        // LSB. MSBytes are equal to
                                        // RX_ADDR_P1[39:8]
#define NRF24L01P_O_TX_ADDR     0x10    // Transmit address. Used for a PTX
                                        // device only. Set RX_ADDR_P0 for AA
#define NRF24L01P_O_RX_PW_P0    0x11    // Number of bytes in RX payload in data
                                        // pipe 0 [5:0]
#define NRF24L01P_O_RX_PW_P1    0x12    // Number of bytes in RX payload in data
                                        // pipe 0 [5:0]
#define NRF24L01P_O_RX_PW_P2    0x13    // Number of bytes in RX payload in data
                                        // pipe 0 [5:0]
#define NRF24L01P_O_RX_PW_P3    0x14    // Number of bytes in RX payload in data
                                        // pipe 0 [5:0]
#define NRF24L01P_O_RX_PW_P4    0x15    // Number of bytes in RX payload in data
                                        // pipe 0 [5:0]
#define NRF24L01P_O_RX_PW_P5    0x16    // Number of bytes in RX payload in data
                                        // pipe 0 [5:0]
#define NRF24L01P_O_FIFO_STATUS 0x17    // FIFO Status register
#define NRF24L01P_O_DYNPD       0x1C    // Enable Dynamic Payload length
#define NRF24L01P_O_FEATURE     0x1D    // Feature register

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_CONFIG
// register.
//
//*****************************************************************************
#define NRF24L01P_CONFIG_MASK_RX_DR                                           \
                                0x40    // Mask interrupt caused by RX_DR:
                                        // 1: Interrupt not reflected on IRQ pin
                                        // 0: Reflect RX_DR as active low on IRQ
#define NRF24L01P_CONFIG_MASK_TX_DS                                           \
                                0x20    // Mask interrupt caused by TX_DS
                                        // 1: Interrupt not reflected on IRQ pin
                                        // 0: Reflect TX_DS as active low on IRQ
#define NRF24L01P_CONFIG_MASK_MAX_RT                                          \
                                0x10    // Mask interrupt caused by MAX_RT
                                        // 1: Interrupt not reflected on IRQ pin
                                        // 0: Reflect MAX_RT as active low on IRQ
#define NRF24L01P_CONFIG_EN_CRC 0x08    // Enable CRC. Forced high if one of the
                                        // bits in the EN_AA is high
#define NRF24L01P_CONFIG_CRCO   0x04    // CRC encoding scheme
                                        // '0': 1 byte
                                        // '1': 2 bytes
#define NRF24L01P_CONFIG_PWR_UP 0x02    // 1: Power up, 0: Power down
#define NRF24L01P_CONFIG_PRIM_RX                                              \
                                0x01    // RX/TX control - 1: PRX, 0: PTX

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_EN_AA
// register.
//
//*****************************************************************************
#define NRF24L01P_EN_AA_ENAA_P5 0x20    // Enable auto acknowledge data pipe 5
#define NRF24L01P_EN_AA_ENAA_P4 0x10    // Enable auto acknowledge data pipe 4
#define NRF24L01P_EN_AA_ENAA_P3 0x08    // Enable auto acknowledge data pipe 3
#define NRF24L01P_EN_AA_ENAA_P2 0x04    // Enable auto acknowledge data pipe 2
#define NRF24L01P_EN_AA_ENAA_P1 0x02    // Enable auto acknowledge data pipe 1
#define NRF24L01P_EN_AA_ENAA_P0 0x01    // Enable auto acknowledge data pipe 0

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_EN_RXADDR
// register.
//
//*****************************************************************************
#define NRF24L01P_EN_RXADDR_ERX_P5                                            \
                                0x20    // Enable data pipe 5
#define NRF24L01P_EN_RXADDR_ERX_P4                                            \
                                0x10    // Enable data pipe 4
#define NRF24L01P_EN_RXADDR_ERX_P3                                            \
                                0x08    // Enable data pipe 3
#define NRF24L01P_EN_RXADDR_ERX_P2                                            \
                                0x04    // Enable data pipe 2
#define NRF24L01P_EN_RXADDR_ERX_P1                                            \
                                0x02    // Enable data pipe 1
#define NRF24L01P_EN_RXADDR_ERX_P0                                            \
                                0x01    // Enable data pipe 0

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_SETUP_AW
// register.
//
// Note: LSByte is used if address width is below 5 bytes
//
//*****************************************************************************
#define NRF24L01P_SETUP_AW_M    0x03
#define NRF24L01P_SETUP_AW_S       0

#define NRF24L01P_SETUP_AW_3_BYTES                                            \
                                0x01    // RX/TX address field width 3 bytes
#define NRF24L01P_SETUP_AW_4_BYTES                                            \
                                0x02    // RX/TX address field width 4 bytes
#define NRF24L01P_SETUP_AW_5_BYTES                                            \
                                0x03    // RX/TX address field width 5 bytes

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_SETUP_RETR
// register.
//
//*****************************************************************************
#define NRF24L01P_SETUP_RETR_ARD_M                                            \
                                0xF0
#define NRF24L01P_SETUP_RETR_ARD_S                                            \
                                   4
#define NRF24L01P_SETUP_RETR_ARD                                              \
                                0xF0    // Auto Retransmit Delay
                                        // 0000 - Wait 250us
                                        // 0001 - Wait 500us
                                        // ...
                                        // 1111 - Wait 4000us
#define NRF24L01P_SETUP_RETR_ARD_DELAY(us)                                    \
                                (((uint8_t)(us / 250) - 1) << 4)              \
                                & NRF24L01P_SETUP_RETR_ARD)
                                        // Wait us microseconds
#define NRF24L01P_SETUP_RETR_ARC_M                                            \
                                0x0F
#define NRF24L01P_SETUP_RETR_ARC_S                                            \
                                   0
#define NRF24L01P_SETUP_RETR_ARC                                              \
                                0x0F    // Retransmit time, up to 15 on AA fail

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_RF_CH
// register.
//
// Masks available bits to set RF Channel Frequency (RF_CH)
//
//*****************************************************************************
#define NRF24L01P_RF_CH_FREQ_M  0x3F
#define NRF24L01P_RF_CH_FREQ_S     0
#define NRF24L01P_RF_CH_FREQ    0x3F    // Sets the frequency channel nRF24L01+
                                        // operates on

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_RF_SETUP
// register.
//
//*****************************************************************************
#define NRF24L01P_RF_SETUP_CONT_WAVE                                          \
                                0x80    // Enables continuous carrier transmit
                                        // when high
#define NRF24L01P_RF_SETUP_RF_DR_LOW                                          \
                                0x20    // Set RF Data Rate to 250kbps
#define NRF24L01P_RF_SETUP_PLL_LOCK                                           \
                                0x10    // Force PLL lock signal. Only used in
                                        // test
#define NRF24L01P_RF_SETUP_RF_DR_HIGH                                         \
                                0x08    // Select between the high speed data
                                        // rates. Bit is do not care (X) if
                                        // RF_DR_LOW is set
                                        // [RF_DR_LOW, RF_DR_HIGH]
                                        // '00' - 1Mbps
                                        // '01' - 2Mbps
                                        // '1X' - 250Kbps
#define NRF24L01P_RF_SETUP_RF_PWR_M                                           \
                                0x06
#define NRF24L01P_RF_SETUP_RF_PWR_S                                           \
                                   1
#define NRF24L01P_RF_SETUP_RF_PWR                                             \
                                0x06    // Set RF output power in TX mode
                                        // '00' - -18dBm
                                        // '01' - -12dBm
                                        // '10' -  -6dBm
                                        // '11' -   0dBm
#define NRF24L01P_RF_SETUP_RF_PWR_N18DBM                                      \
                                0x00
#define NRF24L01P_RF_SETUP_RF_PWR_N12DBM                                      \
                                0x02
#define NRF24L01P_RF_SETUP_RF_PWR_N6DBM                                       \
                                0x04
#define NRF24L01P_RF_SETUP_RF_PWR_0DBM                                        \
                                0x06

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_STATUS
// register.
//
//*****************************************************************************
#define NRF24L01P_STATUS_RX_DR  0x40    // Data ready RX FIFO interrupt.
                                        // Asserted when new data arrives RX
                                        // FIFO. Write 1 to clear
#define NRF24L01P_STATUS_TX_DS  0x20    // Data sent RX FIFO interrupt.
                                        // Asserted when packet transmitted on
                                        // RX. If AUTO_ACK is activated, this
                                        // bit is set high only when ACK is
                                        // received. Write 1 to clear
#define NRF24L01P_STATUS_MAX_RT 0x10    // Max num of TX retransmits interrupt.
                                        // Write 1 to clear. If MAX_RT is
                                        // asserted it must be cleared to enable
                                        // further communication
#define NRF24L01P_STATUS_RX_P_NO_M                                            \
                                0x0E
#define NRF24L01P_STATUS_RX_P_NO_S                                            \
                                   1    // Data pipe number for the payload
                                        // available for reading from RX_FIFO
                                        // 000-101: Data pipe number (0-5)
                                        // 110    : Unused
                                        // 111    : RX FIFO empty
#define NRF24L01P_STATUS_TX_FULL                                              \
                                0x01    // TX FIFO full flag
                                        // 1: TX FIFO full
                                        // 0: Available locations in TX FIFO

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_OBSERVE_TX
// register.
//
//*****************************************************************************
#define NRF24L01P_OBSERVE_TX_PLOS_CNT_M                                       \
                                0xF0
#define NRF24L01P_OBSERVE_TX_PLOS_CNT_S                                       \
                                   4    // Count lost packets. The counter is
                                        // overflow protected to 15, and
                                        // discontinues at max until reset.
                                        // Counter reset by writing to RF_CH
#define NRF24L01P_OBSERVE_TX_ARC_CNT_M                                        \
                                0x0F
#define NRF24L01P_OBSERVE_TX_ARC_CNT_S                                        \
                                   0    // Count retransmitted packets. The
                                        // counter is reset when transmission of
                                        // new packet starts.

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_RPD
// register.
//
//*****************************************************************************
#define NRF24L01P_RPD_RPD       0x01    // Receiver Power Detector

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_RX_PW_P[X]
// register.
//
//*****************************************************************************
#define NRF24L01P_RX_PW_M       0x3F
#define NRF24L01P_RX_PW_S          0
#define NRF24L01P_RX_PW_BYTES   0x3F    // Number of bytes in RX payload
                                        // 0: Pipe not used
                                        // Common for all pipes

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_FIFO_STATUS
// register.
//
//*****************************************************************************
#define NRF24L01P_FIFO_STATUS_TX_REUSE                                        \
                                0x40    // Used for a PTX device.
                                        // Pulse the rfce high for at least 10us
                                        // to reuse last transmitted payload.
                                        // TX Payload reuse is active until
                                        // W_TX_PAYLOAD is set by the SPI cmd
                                        // REUSE_TX_PL, and is reset by the SPI
                                        // cmds W_TX_PAYLOAD or FLUSH_TX
#define NRF24L01P_FIFO_STATUS_TX_FULL                                         \
                                0x20    // TX FIFO full flag.
                                        // 1: TX FIFO full
                                        // 0: Available locations in TX FIFO
#define NRF24L01P_FIFO_STATUS_TX_EMPTY                                        \
                                0x10    // TX FIFO empty flag
                                        // 1: TX FIFO empty
                                        // 0: TX Data in TX FIFO
#define NRF24L01P_FIFO_STATUS_RX_FULL                                         \
                                0x02    // RX FIFO full flag
                                        // 1: RX FIFO full
                                        // 0: Available locations in RX FIFO
#define NRF24L01P_FIFO_STATUS_RX_EMPTY                                        \
                                0x01    // RX FIFO empty flag
                                        // 1: RX FIFO empty
                                        // 0: Data in RX FIFO

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_DYNPD
// register.
//
//*****************************************************************************
#define NRF24L01P_DYNPD_DPL_P5  0x20    // Enable dynamic payload length data
                                        // pipe 5 (Requires EN_DPL and ENAA_P5)
#define NRF24L01P_DYNPD_DPL_P4  0x10    // Enable dynamic payload length data
                                        // pipe 4 (Requires EN_DPL and ENAA_P4)
#define NRF24L01P_DYNPD_DPL_P3  0x08    // Enable dynamic payload length data
                                        // pipe 3 (Requires EN_DPL and ENAA_P3)
#define NRF24L01P_DYNPD_DPL_P2  0x04    // Enable dynamic payload length data
                                        // pipe 2 (Requires EN_DPL and ENAA_P2)
#define NRF24L01P_DYNPD_DPL_P1  0x02    // Enable dynamic payload length data
                                        // pipe 1 (Requires EN_DPL and ENAA_P1)
#define NRF24L01P_DYNPD_DPL_P0  0x01    // Enable dynamic payload length data
                                        // pipe 0 (Requires EN_DPL and ENAA_P0)

//*****************************************************************************
//
// The following are defines for the bit fields in the NRF24L01P_O_FEATURE
// register.
//
//*****************************************************************************
#define NRF24L01P__FEATURE_EN_DPL                                             \
                                0x04    // Enables Dynamic Payload Length
#define NRF24L01P_FEATURE_EN_ACK_PAY                                          \
                                0x02    // Enables Payload with ACK
#define NRF24L01P_FEATURE_EN_DYN_ACK                                          \
                                0x01    // Enables the W_TX_PAYLOAD_NOACK cmd

//*****************************************************************************
//
// The following are defines for the nRF24L01+ SPI commands
//
//*****************************************************************************
#define NRF24L01P_R_REGISTER    0x00    // Read cmd and status registers.
                                        // [4:0] : 5 bit Register Map Address
#define NRF24L01P_W_REGISTER    0x20    // Write cmd and status registers.
                                        // [4:0] : 5 bit Register Map Address
                                        // Executable in power down or standby
#define NRF24L01P_R_RX_PAYLOAD  0x61    // Read RX-payload: 1-32 bytes. A read
                                        // operation always starts at byte 0.
                                        // Payload deleted from FIFO after it
                                        // is read. Use in RX mode
#define NRF24L01P_W_TX_PAYLOAD  0xA0    // Write TX-payload: 1-32 bytes. A write
                                        // operation always starts at byte 0
                                        // used in TX payload
#define NRF24L01P_FLUSH_TX      0xE1    // Flush TX FIFO, used in TX mode
#define NRF24L01P_FLUSH_RX      0xE2    // Flush RX FIFO, used in RX mode
                                        // Should not be executed during
                                        // transmission of acknowledge, that is,
                                        // ack package will not be completed
#define NRF24L01P_REUSE_TX_PL   0xE3    // Used for a PTX device
                                        // Reuse last transmitted payload
                                        // TX payload reuse is active until
                                        // W_TX_PAYLOAD or FLUSH_TX is executed
                                        // TX payload reuse must not be activated
                                        // or deactivated during package trans.
#define NRF24L01P_R_RX_PL_WID   0x60    // Read RX payload width for the top
                                        // R_RX_PAYLOAD in the RX FIFO
                                        // Note: Flush RX FIFO if > 32 bytes
#define NRF24L01P_W_ACK_PAYLOAD 0xA1    // Used in RX mode
                                        // Write Payload to be transmitted
                                        // together with ACK packet on pipe [2:0]
                                        // Maximum three ACK packet payloads can
                                        // be pending. Payloads with same pipe
                                        // are handled using FIFO principle.
                                        // Write payload: 1-32 bytes, start at 0
#define NRF24L01P_W_TX_PAYLOAD_NOACK                                          \
                                0xB0    // Used in TX mode. Disables AUTOACK on
                                        // this specific packet
#define NRF24L01P_NOP           0xFF    // No Operation. Might be used to read
                                        // the STATUS register

#endif /* __PERIPHERALS_HW_NRF24L01P_H__ */
