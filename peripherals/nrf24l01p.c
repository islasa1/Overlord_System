//*****************************************************************************
//! nrf24l01p.h - Driver for the nRF24L01+ 2.4GHz Transceiver.
//
//
//!  Created on: Mar 1, 2017
//!      Author: islasa1
//
//*****************************************************************************

#include "nrf24l01p.h"

#define NRF24L01P_STATE_IDLE            0
#define NRF24L01P_STATE_LAST            1
#define NRF24L01P_STATE_READ            2
#define NRF24L01P_STATE_WRITE           3
#define NRF24L01P_STATE_RMW             4
#define NRF24L01P_STATE_INIT_RESET      5
#define NRF24L01P_STATE_INIT_USE        6
#define NRF24L01P_STATE_GET_RF_FREQ     7
#define NRF24L01P_STATE_GET_RF_PWR      8
#define NRF24L01P_STATE_GET_ADR         9
#define NRF24L01P_STATE_GET_CRC_WIDTH   10
#define NRF24L01P_STATE_GET_RX_ADDR     11
#define NRF24L01P_STATE_GET_TX_ADDR     12
#define NRF24L01P_STATE_GET_TRANS_SIZE  13
#define NRF24L01P_STATE_POWER_UP        14
#define NRF24L01P_STATE_POWER_DOWN      15
#define NRF24L01P_STATE_ENABLE          16
#define NRF24L01P_STATE_DISABLE         17
#define NRF24L01P_STATE_PIPE_READ       18


#define NRF24L01P_MODE_PWR_DOWN     1
#define NRF24L01P_MODE_STANDBY_I    2
#define NRF24L01P_MODE_STANDBY_II   3
#define NRF24L01P_MODE_RX           4
#define NRF24L01P_MODE_TX           5

#define NRF24L01P_FIFO_CNT      3
#define NRF24L01P_RX_FIFO_SIZE  32
#define NRF24L01P_TX_FIFO_SIZE  32


//*****************************************************************************
//
//! Callback function for SPI transactions with the nRF24L01+.
//!
//! \param pvCallbackData is a pointer to the data used by the callback
//! function. Should point to an nRF24L01+ instance.
//! \param ui8Status is an 8 bit integer that represents the status of the
//! I2C transaction.
//!
//! This function is a state machine that is used to determine the response
//! at the end of an SPI transaction. The state of the nRF24L01+ instance given
//! determines course of action. For initialization, this function is called
//! multiple times, changing the state with every call to progress in the
//! correct order. Otherwise, for reads and writes it makes sure that the
//! status is success and then calls the nRF24L01+ instance callback function.
//!
//! \return None.
//
//*****************************************************************************
static void NRF24L01PCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    tNRF24L01P *psInst;

    //
    // Convert the instance data into a pointer to a tNRF24L01P structure
    //
    psInst = pvCallbackData;

    //
    // Determine the current state of the nRF24L01+
    //
    switch(psInst->ui8State)
    {
        //
        // All the states that trivially transition to IDLE, and all unknown
        // states.
        //
        case NRF24L01P_STATE_READ:
        case NRF24L01P_STATE_LAST:
        default:
        {
            //
            // The state machine is now idle
            //
            psInst->ui8State = NRF24L01P_STATE_IDLE;

            //
            // Done.
            //
            break;
        }

        //
        // Device Reset was issued
        //
        case NRF24L01P_STATE_INIT_RESET:
        {
            //
            // Reset function
            // TODO: do it
            //
            break;
        }

        //
        //
        //
        case NRF24L01P_STATE_WRITE:
        {
            //
            // Done.
            //
            break;
        }

        //
        //
        //
        case NRF24L01P_STATE_RMW:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_INIT_USE:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_GET_RF_FREQ:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_GET_RF_PWR:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_GET_ADR:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_GET_CRC_WIDTH:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_GET_RX_ADDR:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_GET_TX_ADDR:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_GET_TRANS_SIZE:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_POWER_UP:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_POWER_DOWN:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_ENABLE:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_DISABLE:
        {
            //
            // Done.
            //
            break;
        }
        //
        //
        //
        case NRF24L01P_STATE_PIPE_READ:
        {
            //
            // Done.
            //
            break;
        }
    }

    //
    // See if the state machine is now idle and there is a callback function.
    //
    if((psInst->ui8State == NRF24L01P_STATE_IDLE) && psInst->pfnCallback)
    {
        //
        // Call the application-supplied callback function.
        //
        psInst->pfnCallback(psInst->pvCallbackData, ui8Status);
    }
}

//*****************************************************************************
//! Initialize the NRF24L01P driver.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param psSPIInst is a pointer to the SPI master driver instance data.
//! \param ui32CSPort is the base port of the Chip Select.
//! \param ui8CSPin is the pin of the Chip Select.
//! \param ui32CEPort is the base port of the Chip Enable (RX/TX mode).
//! \param ui8CEPin is the pin of the Chip Enable (RX/TX mode).
//! \param pfnCallback is the function to be called when the initialization has
//! completed (can be \b NULL if a callback is not required).
//! \param pvCallbackData is a pointer that is passed to the callback function.
//!
//! This function initializes the NRF24L01P driver, preparing it for operation.
//!
//! \return Returns 1 if the NRF24L01P driver was successfully initialized and 0
//! if it was not.
//
//*****************************************************************************
uint_fast8_t NRF24L01PInit(tNRF24L01P *psInst, tSPIMInstance *psSPIInst,
                           uint32_t ui32CSPort, uint8_t ui8CSPin,
                           uint32_t ui32CEPort, uint8_t ui8CEPin,
                           tSPICallback *pfnCallback,
                           void *pvCallbackData)
{

}

//*****************************************************************************
//! Sets the operating frequency of the NRF24L01P driver.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui16Freq is the frequency in MHz (2400..2525).
//!
//! This function prepares a frequency into the \e pui8Data buffer with the
//! appropriate register to write to as well. The \e ui16Freq is used to update
//! the \e psInst internal frequency stored.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetFrequency(tNRF24L01P *psInst, uint16_t ui16Freq)
{
    // Assert frequency restrictions
    if(ui16Freq <= NRF24L01P_RF_CH_FREQ_MAX &&
       ui16Freq >= NRF24L01P_RF_CH_FREQ_BASE)
    {
        //
        // Set register
        //
        psInst->pui8Data[0] = NRF24L01P_O_RF_CH;

        //
        // Set frequency
        //
        psInst->pui8Data[1] = ((ui16Freq - NRF24L01P_RF_CH_FREQ_BASE) &
                NRF24L01P_RF_CH_FREQ_M) << NRF24L01P_RF_CH_FREQ_S;

        //
        // Update psInst
        //

    }
}

//*****************************************************************************
//! Returns the operating frequency of the NRF24L01P driver.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function returns the operating frequency of the NRF24L01P driver, and
//! auto-updates the value if necessary
//!
//! \return Returns the operating frequency stored in the NRF24L01P instance
//! data.
//
//*****************************************************************************
uint16_t NRF24L01PGetFrequency(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Sets the radio frequency output power.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Power is the predefined bit pattern encoding for different levels
//! of output power.
//!
//! This function sets the RF output power to one of four predefined levels for
//! use when transmitting. Use the \e NRF24L01P_RF_SETUP_RF_PWR dBm defines to
//! select a power level.
//!
//! \b DO \b NOT pass a numeric dBm value as \e ui8Power.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetRFOutputPower(tNRF24L01P *psInst, uint8_t ui8Power)
{
}

//*****************************************************************************
//! \param psInst is a pointer to the NRF24L01P instance data.
//! @return
//
//*****************************************************************************
int8_t NRF24L01PGetRFOutputPower(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! \param psInst is a pointer to the NRF24L01P instance data.
//! @param ui16AirDataRate
//
//*****************************************************************************
void NRF24L01PSetAirDataRate(tNRF24L01P *psInst, uint16_t ui16AirDataRate)
{
}

//*****************************************************************************
//! Get the Air data rate.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return the air data rate in kbps (250, 1M or 2M).
//
//*****************************************************************************
int32_t NRF24L01PGetAirDataRate(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Set the CRC width.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Width the number of bits for the CRC (0, 8 or 16).
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetCrcWidth(tNRF24L01P *psInst, uint8_t ui8Width)
{
}

//*****************************************************************************
//! Get the CRC width.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return the number of bits for the CRC (0, 8 or 16).
//
//*****************************************************************************
int32_t NRF24L01PGetCrcWidth(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Set the Receive address.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui64Address address associated with the particular pipe.
//! \param ui8Width width of the address in bytes (3..5).
//! \param ui8Pipe pipe to associate the address with (0..5, default 0).
//!
//! Note that Pipes 0 &amp; 1 have 3, 4 or 5 byte addresses,
//!  while Pipes 2..5 only use the lowest byte (bits 7..0) of the
//!  address provided here, and use 2, 3 or 4 bytes from Pipe 1's address.
//!  The width parameter is ignored for Pipes 2..5.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetRxAddress(tNRF24L01P *psInst, uint64_t ui64Address, uint8_t ui8Width, uint8_t ui8Pipe)
{
}

//*****************************************************************************
//! Set the Transmit address.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui64Address address for transmission
//! \param ui8Width width of the address in bytes (3..5).
//!
//! Note that the address width is shared with the Receive pipes,
//!  so a change to that address width affect transmissions.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetTxAddress(tNRF24L01P *psInst, uint64_t ui64Address, uint8_t ui8Width)
{
}

//*****************************************************************************
//! Get the Receive address.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Pipe pipe to get the address from (0..5, default 0).
//!
//!
//!
//! \return the address associated with the particular pipe.
//
//*****************************************************************************
uint64_t NRF24L01PGetRxAddress(tNRF24L01P *psInst, uint8_t ui8Pipe)
{
}

//*****************************************************************************
//! Get the Transmit address.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return address address for transmission
//
//*****************************************************************************
uint64_t NRF24L01PGetTxAddress(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Set the transfer size.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Size the size of the transfer, in bytes (1..32).
//! \param ui8Pipe pipe for the transfer (0..5, default 0).
//!
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetTransferSize(tNRF24L01P *psInst, uint8_t ui8Size, uint8_t ui8Pipe )
{
}

//*****************************************************************************
//! Get the transfer size.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return the size of the transfer, in bytes (1..32).
//
//*****************************************************************************
int32_t NRF24L01PGetTransferSize(tNRF24L01P *psInst, uint8_t ui8Pipe)
{
}


//*****************************************************************************
//! Get the RPD (Received Power Detector) state.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return true if the received power exceeded -64dBm
//*****************************************************************************
bool NRF24L01PGetRPD(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Put the nRF24L01+ into Receive mode
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetReceiveMode(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Put the nRF24L01+ into Transmit mode
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetTransmitMode(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Power up the nRF24L01+ into Standby mode
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PPowerUp(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Power down the nRF24L01+ into Power Down mode
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return None.
//!
//
//*****************************************************************************
void NRF24L01PPowerDown(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Enable the nRF24L01+ to Receive or Transmit (using the CE pin)
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PEnable(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Disable the nRF24L01+ to Receive or Transmit (using the CE pin)
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PDisable(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Transmit data
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Data pointer to an array of bytes to write.
//! \param ui8Count the number of bytes to send (1..32).
//! \return the number of bytes actually written, or -1 for an error.
//
//*****************************************************************************
int32_t NRF24L01PTransmit(tNRF24L01P *psInst, uint8_t *ui8Data,
                          uint8_t ui8Count)
{
}

//*****************************************************************************
//! Receive data
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Pipe the receive pipe to get data from
//! \param ui8Data pointer to an array of bytes to store the received data
//! \param ui8Count the number of bytes to receive (1..32)
//! \return the number of bytes actually received, 0 if none are received, or -1 for an error
//
//*****************************************************************************
int32_t NRF24L01PReceive(tNRF24L01P *psInst, uint8_t ui8Pipe,
                         uint8_t *ui8Data, uint8_t ui8Count)
{
}

//*****************************************************************************
//! Determine if there is data available to read
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Pipe the receive pipe to check for data.
//!
//!
//!
//! \return true if the is data waiting in the given pipe
//
//*****************************************************************************
bool NRF24L01PReadable(tNRF24L01P *psInst, uint8_t ui8Pipe)
{
}

//*****************************************************************************
//! Disable all receive pipes
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! Note: receive pipes are enabled when their address is set.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PDisableAllRxPipes(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Disable AutoAcknowledge function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PDisableAutoAcknowledge(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Enable AutoAcknowledge function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Pipe the receive pipe
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PEnableAutoAcknowledge(tNRF24L01P *psInst, uint8_t ui8Pipe)
{
}

//*****************************************************************************
//! Disable AutoRetransmit function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PDisableAutoRetransmit(tNRF24L01P *psInst)
{
}

//*****************************************************************************
//! Enable AutoRetransmit function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui16Delay the delay between retransmits, in uS (250uS..4000uS).
//! \param ui8Count number of retransmits before generating an error (1..15).
//!
//!
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PEnableAutoRetransmit(tNRF24L01P *psInst, uint16_t ui16Delay, uint8_t ui8Count)
{
}
