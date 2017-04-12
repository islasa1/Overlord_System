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
#define NRF24L01P_STATE_GET_ARC         7
#define NRF24L01P_STATE_GET_ARD         8
#define NRF24L01P_STATE_GET_AW          9
#define NRF24L01P_STATE_GET_CONFIG      10
#define NRF24L01P_STATE_GET_CRC_WIDTH   11
#define NRF24L01P_STATE_GET_RF_FREQ     12
#define NRF24L01P_STATE_GET_RF_PWR      13
#define NRF24L01P_STATE_GET_RF_SETUP    14
#define NRF24L01P_STATE_GET_RPD         15
#define NRF24L01P_STATE_GET_RX_ADDR0    16
#define NRF24L01P_STATE_GET_RX_ADDR1    17
#define NRF24L01P_STATE_GET_RX_ADDR2    18
#define NRF24L01P_STATE_GET_RX_ADDR3    19
#define NRF24L01P_STATE_GET_RX_ADDR4    20
#define NRF24L01P_STATE_GET_RX_ADDR5    21
#define NRF24L01P_STATE_GET_TX_ADDR     22
#define NRF24L01P_STATE_GET_TRANS_SIZE  23
#define NRF24L01P_STATE_POWER_UP        24
#define NRF24L01P_STATE_POWER_DOWN      25
#define NRF24L01P_STATE_ENABLE          26
#define NRF24L01P_STATE_DISABLE         27
#define NRF24L01P_STATE_PIPE_READ       28


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
// Internal Use
//
//*****************************************************************************
void StateGetSetupRetr(tNRF24L01P *psInst)
{
    psInst->ui8ARD = ((psInst->pui8Data[0] & NRF24L01P_SETUP_RETR_ARD_M) >>
                      NRF24L01P_SETUP_RETR_ARD_S);
    psInst->ui8ARC = ((psInst->pui8Data[0] & NRF24L01P_SETUP_RETR_ARC_M) >>
                      NRF24L01P_SETUP_RETR_ARC_S);
}

void StateGetAW(tNRF24L01P *psInst)
{
    if(psInst->pui8Data[0] == NRF24L01P_SETUP_AW_3_BYTES)
    {
        psInst->ui8AddrWidth = 3;
    }
    else if(psInst->pui8Data[0] == NRF24L01P_SETUP_AW_4_BYTES)
    {
        psInst->ui8AddrWidth = 4;
    }
    else if(psInst->pui8Data[0] == NRF24L01P_SETUP_AW_5_BYTES)
    {
        psInst->ui8AddrWidth = 5;
    }
    else
    {
        psInst->ui8AddrWidth = 0;
    }
}

void StateGetConfig(tNRF24L01P *psInst)
{
    psInst->uConfig.ui8Config = psInst->pui8Data[0];
}

void StateGetRFFreq(tNRF24L01P *psInst)
{
    psInst->ui16Freq = ((psInst->pui8Data[0] & NRF24L01P_RF_CH_FREQ_M) >>
                        NRF24L01P_RF_CH_FREQ_S) + NRF24L01P_RF_CH_FREQ_BASE;
}

void StateGetRFSetup(tNRF24L01P *psInst)
{
    uint8_t ui8DataRate = psInst->pui8Data[0] & (NRF24L01P_RF_SETUP_RF_DR_LOW |
                          NRF24L01P_RF_SETUP_RF_DR_HIGH);
    uint8_t ui8Power = psInst->pui8Data[0] & NRF24L01P_RF_SETUP_RF_PWR_M;

    if(ui8DataRate == NRF24L01P_RF_SETUP_RF_DR_250KBPS)
    {
        psInst->ui16AirDataRate = 250;
    }
    else if(ui8DataRate == NRF24L01P_RF_SETUP_RF_DR_1MBPS)
    {
        psInst->ui16AirDataRate = 1000;
    }
    else if(ui8DataRate == NRF24L01P_RF_SETUP_RF_DR_2MBPS)
    {
        psInst->ui16AirDataRate = 2000;
    }
    else
    {
        psInst->ui16AirDataRate = 0;
    }

    if(ui8Power == NRF24L01P_RF_SETUP_RF_PWR_N18DBM)
    {
        psInst->i8Power = -18;
    }
    else if(ui8Power == NRF24L01P_RF_SETUP_RF_PWR_N12DBM)
    {
        psInst->i8Power = -12;
    }
    else if(ui8Power == NRF24L01P_RF_SETUP_RF_PWR_N6DBM)
    {
        psInst->i8Power = -6;
    }
    else if(ui8Power == NRF24L01P_RF_SETUP_RF_PWR_0DBM)
    {
        psInst->i8Power = 0;
    }
    else
    {
        psInst->i8Power = -1;
    }
}

void StateGetRxAddr(tNRF24L01P *psInst, uint8_t ui8Pipe)
{
    switch(ui8Pipe)
    {
        case NRF24L01P_PIPE_0:
            psInst->sPipeAddr.ui64RxAddrP0 = *((uint64_t *)psInst->pui8Data);
            break;
        case NRF24L01P_PIPE_1:
            psInst->sPipeAddr.ui64RxAddrP1 = *((uint64_t *)psInst->pui8Data);
            break;
        case NRF24L01P_PIPE_2:
            psInst->sPipeAddr.ui8RxAddrP2 = psInst->pui8Data[0];
            break;
        case NRF24L01P_PIPE_3:
            psInst->sPipeAddr.ui8RxAddrP3 = psInst->pui8Data[0];
            break;
        case NRF24L01P_PIPE_4:
            psInst->sPipeAddr.ui8RxAddrP4 = psInst->pui8Data[0];
            break;
        case NRF24L01P_PIPE_5:
            psInst->sPipeAddr.ui8RxAddrP5 = psInst->pui8Data[0];
            break;
    }
}

void StateGetTxAddr(tNRF24L01P *psInst)
{
    psInst->sPipeAddr.ui64TxAddr = *((uint64_t *)psInst->pui8Data);
}

void StateGetTransSize(tNRF24L01P *psInst)
{
    uint8_t ui8TransSize, ui8Pipe;
    ui8Pipe = psInst->pui8Data[0] - NRF24L01P_O_RX_PW_P0;
    ui8TransSize = psInst->pui8Data[1];

    psInst->pui8RxSPLSize[ui8Pipe] = ui8TransSize;
}

void StateInitReset(psInst)
{
    //
    // Set up nRF24L01+ device for normal default operation
    //

    //
    // Set CRC Configuration
    //
    NRF24L01PSetCrcWidth(psInst, 1);

    //
    // Default to Primary Transceiver
    //
    NRF24L01PSetTransmitMode(psInst);

    //
    // Set Automatic Retransmit Count
    //
    NRF24L01PSetARC(psInst, 3);

    //
    // Set Automatic Retransmit Delay, refer to nRF24L01+ Spec Rev. 1.0 Pg 43
    // Table 18
    //
    NRF24L01PSetARD(psInst, 1500);

    //
    // Set Address Width
    //
    NRF24L01PSetAddrWidth(psInst, NRF24L01P_SETUP_AW_5_BYTES);

    //
    // Set Operational Radio Frequency
    //
    NRF24L01PSetFrequency(psInst, NRF24L01P_RF_CH_FREQ_BASE);

    //
    // Set Air Data Rate
    //
    NRF24L01PSetAirDataRate(psInst, NRF24L01P_RF_SETUP_RF_DR_250KBPS);

    //
    // Set Payload Widths for Static Payload Size
    //
    NRF24L01PSetTransferSize(psInst, NRF24L01P_RX_FIFO_SIZE, NRF24L01P_PIPE_0);
    NRF24L01PSetTransferSize(psInst, NRF24L01P_RX_FIFO_SIZE, NRF24L01P_PIPE_1);
    NRF24L01PSetTransferSize(psInst, NRF24L01P_RX_FIFO_SIZE, NRF24L01P_PIPE_2);
    NRF24L01PSetTransferSize(psInst, NRF24L01P_RX_FIFO_SIZE, NRF24L01P_PIPE_3);
    NRF24L01PSetTransferSize(psInst, NRF24L01P_RX_FIFO_SIZE, NRF24L01P_PIPE_4);
    NRF24L01PSetTransferSize(psInst, NRF24L01P_RX_FIFO_SIZE, NRF24L01P_PIPE_5);

    //
    // Enable Enhanced Shockburst™ Auto Acknowledge
    //
    NRF24L01PEnableAutoAcknowledge(psInst, NRF24L01P_PIPE_0);
    NRF24L01PEnableAutoAcknowledge(psInst, NRF24L01P_PIPE_1);
    NRF24L01PEnableAutoAcknowledge(psInst, NRF24L01P_PIPE_2);
    NRF24L01PEnableAutoAcknowledge(psInst, NRF24L01P_PIPE_3);
    NRF24L01PEnableAutoAcknowledge(psInst, NRF24L01P_PIPE_4);
    NRF24L01PEnableAutoAcknowledge(psInst, NRF24L01P_PIPE_5);


    //
    // Power up
    //
    NRF24L01PSet(psInst);
}

//*****************************************************************************
//
//! Callback function for SPI transactions with the nRF24L01+.
//!
//! \param pvCallbackData is a pointer to the data used by the callback
//! function. Should point to an nRF24L01+ instance.
//! \param ui8Status is an 8 bit integer that represents the status of the
//! SPI transaction.
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
        case NRF24L01P_STATE_GET_RPD:
        case NRF24L01P_STATE_READ:  // Not used
        case NRF24L01P_STATE_WRITE: // Not used
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
        case NRF24L01P_STATE_INIT_USE:
        case NRF24L01P_STATE_INIT_RESET:
        {
            //
            // The state machine is now idle
            //
            psInst->ui8State = NRF24L01P_STATE_IDLE;
            //
            // Reset function
            //
            StateInitReset(psInst);
            break;
        }

        //
        // Pass NRF24L01P Instance to appropriate function to parse return data
        //
        case NRF24L01P_STATE_GET_ARC:
        case NRF24L01P_STATE_GET_ARD:
        {
            StateGetSetupRetr(psInst);
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
        // Pass NRF24L01P Instance to appropriate function to parse return data
        //
        case NRF24L01P_STATE_GET_AW:
        {
            StateGetAW(psInst);
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
        // Pass NRF24L01P Instance to appropriate function to parse return data
        //
        case NRF24L01P_STATE_GET_CONFIG:
        {
            StateGetConfig(psInst);
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
        // Pass NRF24L01P Instance to appropriate function to parse return data
        //
        case NRF24L01P_STATE_GET_RF_FREQ:
        {
            StateGetRFFreq(psInst);
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
        // Pass NRF24L01P Instance to appropriate function to parse return data
        //
        case NRF24L01P_STATE_GET_RF_SETUP:
        {
            StateGetRFSetup(psInst);
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
        // Pass NRF24L01P Instance to appropriate function to parse return data
        //
        case NRF24L01P_STATE_GET_RX_ADDR5:
        case NRF24L01P_STATE_GET_RX_ADDR4:
        case NRF24L01P_STATE_GET_RX_ADDR3:
        case NRF24L01P_STATE_GET_RX_ADDR2:
        case NRF24L01P_STATE_GET_RX_ADDR1:
        case NRF24L01P_STATE_GET_RX_ADDR0:
        {
            StateGetRxAddr(psInst, psInst->ui8State - NRF24L01P_STATE_GET_RX_ADDR0);
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
        // Pass NRF24L01P Instance to appropriate function to parse return data
        //
        case NRF24L01P_STATE_GET_TX_ADDR:
        {
            StateGetTxAddr(psInst);
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
        // Pass NRF24L01P Instance to appropriate function to parse return data
        //
        case NRF24L01P_STATE_GET_TRANS_SIZE:
        {
            StateGetTransSize(psInst);
            //
            // The state machine is now idle
            //
            psInst->ui8State = NRF24L01P_STATE_IDLE;
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
//! \param ui32CSBase is the base port of the Chip Select.
//! \param ui8CSPin is the pin of the Chip Select.
//! \param ui32CEBase is the base port of the Chip Enable (RX/TX mode).
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
    psInst->psSPIInst = psSPIInst;
    psInst->ui32CSBase = ui32CSPort;
    psInst->ui8CSPin = ui8CSPin;
    psInst->ui32CEBase = ui32CEPort;
    psInst->ui8CEPin = ui8CEPin;
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    //
    // Set to Init Reset
    //
    psInst->ui8State = NRF24L01P_STATE_INIT_RESET;

    //
    // Call device-supplied callback to initialize
    //
    NRF24L01PCallback(psInst, 0);

    return (1);
}

//*****************************************************************************
//! Put the nRF24L01+ into Receive mode.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function puts the nRF24L01+ device into Transmit Mode by setting the
//! PRIM_RX = 1.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetReceiveMode(tNRF24L01P *psInst)
{
    psInst->uConfig.ui1PrimaryRx = 1;

    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_CONFIG;

    //
    // Pack data
    //
    psInst->pui8Data[1] = psInst->uConfig.ui8Config;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Put the nRF24L01+ into Transmit mode
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function puts the nRF24L01+ device into Transmit Mode by setting the
//! PRIM_RX = 0.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetTransmitMode(tNRF24L01P *psInst)
{
    psInst->uConfig.ui1PrimaryRx = 0;

    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_CONFIG;

    //
    // Pack data
    //
    psInst->pui8Data[1] = psInst->uConfig.ui8Config;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
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
        // SPI command write register and address
        //
        psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_RF_CH;

        //
        // Set frequency
        //
        psInst->pui8Data[1] = ((ui16Freq - NRF24L01P_RF_CH_FREQ_BASE) &
                NRF24L01P_RF_CH_FREQ_M) << NRF24L01P_RF_CH_FREQ_S;

        //
        // Update psInst
        //
        psInst->ui16Freq = ui16Freq;

        //
        // Start SPI transfer
        //
        SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                     psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
    }
}

//*****************************************************************************
//! Returns the operating frequency of the NRF24L01P driver.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function returns the operating frequency of the NRF24L01P driver, and
//! auto-updates the value.
//!
//! \return Returns the operating frequency stored in the NRF24L01P instance
//! data.
//
//*****************************************************************************
uint16_t NRF24L01PGetFrequency(tNRF24L01P *psInst)
{
    //
    // Set state machine state so we know how to parse data from callback
    //
    psInst->ui8State = NRF24L01P_STATE_GET_RF_FREQ;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_RF_CH;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);


    //
    // Return internal structure
    //
    return (psInst->ui16Freq);
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
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_RF_SETUP;

    //
    // Default, clear old values
    //
    psInst->pui8Data[1] = 0x00;

    //
    // Set RF Setup options, taking from current device since its bit fields
    //
    if(psInst->ui16AirDataRate == 250)
    {
        psInst->pui8Data[1] |= NRF24L01P_RF_SETUP_RF_DR_250KBPS;
    }
    else if(psInst->ui16AirDataRate == 1000)
    {
        psInst->pui8Data[1] |= NRF24L01P_RF_SETUP_RF_DR_1MBPS;
    }
    else if(psInst->ui16AirDataRate == 2000)
    {
        psInst->pui8Data[1] |= NRF24L01P_RF_SETUP_RF_DR_2MBPS;
    }
    //
    // Else something is very wrong, leave at 0, defaults to 1Mbps
    //

    //
    // Do not use the other 2 bit fields, check if valid input
    //
    if(ui8Power == NRF24L01P_RF_SETUP_RF_PWR_0DBM   ||
       ui8Power == NRF24L01P_RF_SETUP_RF_PWR_N6DBM  ||
       ui8Power == NRF24L01P_RF_SETUP_RF_PWR_N12DBM ||
       ui8Power == NRF24L01P_RF_SETUP_RF_PWR_N18DBM)
    {
        psInst->pui8Data[1] |= ui8Power;

        //
        // Set internal structure, increases by 6 dBm each step
        //
        psInst->i8Power = -18 + (ui8Power >> 1) * (6);

    }
    //
    // If invalid input, default to -18 dBm
    //

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Returns the operating radio frequency output power of the NRF24L01P driver.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function returns the operating radio frequency output power used when
//! transmitting, and auto-updates.
//!
//! \return Returns the operating output power as dBm.
//
//*****************************************************************************
int8_t NRF24L01PGetRFOutputPower(tNRF24L01P *psInst)
{
    //
    // Set state machine state so we know how to parse data from callback
    //
    psInst->ui8State = NRF24L01P_STATE_GET_RF_SETUP;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_RF_SETUP;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    //
    // Return internal structure
    //
    return (psInst->i8Power);
}

//*****************************************************************************
//! Sets the operating data rate of the NRF24L01P driver.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8AirDataRate is the predefined bit pattern encoding for different
//! data rates.
//!
//! This function sets the data rate to one of the 3 available (250 kbps,
//! 1 mbps, 2 mbps) for device operation. use the \e NRF24L01P_RF_SETUP_RF_DR
//! data rate defines to select a data rate.
//!
//! \b DO \b NOT pass a numeric data rate value as \e ui8AirDataRate
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetAirDataRate(tNRF24L01P *psInst, uint8_t ui8AirDataRate)
{
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_RF_SETUP;

    //
    // Default, clear old values
    //
    psInst->pui8Data[1] = 0x00;

    //
    // Set RF Setup options, taking from current device since its bit fields
    //
    if(psInst->i8Power % 6 == 0)
    {
        psInst->pui8Data[1] |= ((3 - (psInst->i8Power / (-6))) <<
                NRF24L01P_RF_SETUP_RF_PWR_S) & NRF24L01P_RF_SETUP_RF_PWR_M;
    }

    //
    // Check for valid input
    //
    if(ui8AirDataRate == NRF24L01P_RF_SETUP_RF_DR_250KBPS)
    {
        psInst->pui8Data[1] |= NRF24L01P_RF_SETUP_RF_DR_250KBPS;
    }
    else if(ui8AirDataRate == NRF24L01P_RF_SETUP_RF_DR_1MBPS)
    {
        psInst->pui8Data[1] |= NRF24L01P_RF_SETUP_RF_DR_1MBPS;
    }
    else if(ui8AirDataRate == NRF24L01P_RF_SETUP_RF_DR_2MBPS)
    {
        psInst->pui8Data[1] |= NRF24L01P_RF_SETUP_RF_DR_2MBPS;
    }
    //
    // Else if invalid input, default to 1 Mbps
    //

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Returns the operating data rate.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function returns the device operating data rate.
//!
//! \return Returns the air data rate in kbps (250, 1000 or 2000).
//
//*****************************************************************************
uint16_t NRF24L01PGetAirDataRate(tNRF24L01P *psInst)
{
    //
    // Set state machine state so we know how to parse data from callback
    //
    psInst->ui8State = NRF24L01P_STATE_GET_RF_SETUP;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_RF_SETUP;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    //
    // Return internal structure
    //
    return (psInst->ui16AirDataRate);
}

//*****************************************************************************
//! Set the CRC width.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Width the number of bytes for the CRC (1 or 2). Anything greater
//! than 2 is ignored. Setting 0 disables use of CRC (not recommended).
//!
//! This function sets the device CRC encoding scheme.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetCrcWidth(tNRF24L01P *psInst, uint8_t ui8Width)
{
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_CONFIG;

    //
    // Default, clear old values
    //
    psInst->pui8Data[1] = 0x00;

    //
    // Set CRC scheme
    //
    if(ui8Width == 0)
    {
        //
        // Disable CRC
        //
        psInst->uConfig.ui1EnableCRC = 0;
    }
    else
    {
        //
        // Default to enable CRC
        //
        psInst->uConfig.ui1EnableCRC = 1;

        if(ui8Width == 1)
        {
            psInst->uConfig.ui1CRCScheme = 0;
        }
        else // Greater than 1
        {
            psInst->uConfig.ui1CRCScheme = 1;
        }
    }

    //
    // Pack data
    //
    psInst->pui8Data[1] = psInst->uConfig.ui8Config;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Returns the CRC width.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function returns the device CRC encoding scheme.
//!
//! \return Returns the number of bytes for the CRC (1 or 2, or 0 if disabled).
//
//*****************************************************************************
uint8_t NRF24L01PGetCrcWidth(tNRF24L01P *psInst)
{
    //
    // Set state machine state so we know how to parse data from callback
    //
    psInst->ui8State = NRF24L01P_STATE_GET_CONFIG;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_CONFIG;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    if(!psInst->uConfig.ui1EnableCRC)
    {
        return (0);
    }
    else
    {
        if(psInst->uConfig.ui1CRCScheme == 0)
        {
            return (1);
        }
        else
        {
            return (2);
        }
    }
}

//*****************************************************************************
//! Sets the address width for all pipes (RX/TX).
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Width width of the address.
//!
//! This function sets the pipe address widths for all pipes. Parameter
//! \e ui8Width must be an option from the NRF24L01P_SETUP_AW_X_BYTES.
//!
//! Note that Pipes 0 & 1 have 3, 4 or 5 byte addresses, while Pipes 2..5 only
//! use the lowest byte (bits 7..0) of the address provided here, and use 2, 3
//! or 4 bytes from Pipe 1's address.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetAddrWidth(tNRF24L01P *psInst, uint8_t ui8Width)
{
    //
    // Set internal structure.
    //
    psInst->ui8AddrWidth = ui8Width + 2;

    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_SETUP_AW;

    //
    // Default, clear old values
    //
    psInst->pui8Data[1] = NRF24L01P_SETUP_AW_3_BYTES;

    //
    // Check for valid input
    //
    if(ui8Width == NRF24L01P_SETUP_AW_3_BYTES ||
       ui8Width == NRF24L01P_SETUP_AW_4_BYTES ||
       ui8Width == NRF24L01P_SETUP_AW_5_BYTES)
    {
        psInst->pui8Data[1] = (ui8Width << NRF24L01P_SETUP_AW_S) &
                NRF24L01P_SETUP_AW_M;
    }
    //
    // Else default to 3 bytes.
    //

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Returns the address width for all pipes (RX/TX)
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! Note that Pipes 0 & 1 have 3, 4 or 5 byte addresses, while Pipes 2..5 only
//! use the lowest byte (bits 7..0) of the address provided here, and use 2, 3
//! or 4 bytes from Pipe 1's address. The width parameter is ignored for
//! Pipes 2..5.
//!
//! \return Returns the address width in bytes of pipes.
//
//*****************************************************************************
uint8_t NFR24L01PGetAddrWidth(tNRF24L01P *psInst)
{
    //
    // Set state.
    //
    psInst->ui8State = NRF24L01P_STATE_GET_AW;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_SETUP_AW;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    return (psInst->ui8AddrWidth);
}

//*****************************************************************************
//! Set the Receive address.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui64Address address associated with the particular pipe. The 5
//! LSBytes are used for addressing.
//! \param ui8Pipe pipe to associate the address with (0..5).
//!
//! This function sets the receive address for a given pipe. Must comply with
//! set address width.
//!
//! Note Pipe 0 and Pipe 1 have address widths of up to 5 bytes, and all other
//! pipes have 1 byte addresses, with the remaining MSBytes to achieve address
//! width (set by NRF24L01P_O_SETUP_AW register) taken from Pipe 1[39:8].
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetRxAddress(tNRF24L01P *psInst, uint64_t ui64Address,
                           uint8_t ui8Pipe)
{
    //
    // Make sure Pipe Number is within bounds.
    //
    if(ui8Pipe > NRF24L01P_PIPE_5)
    {
        return;
    }

    uint8_t pui8Address[8], ui8TransferSize, ui64AddrMask;

    NRF24L01P_UI64_TO_PUI8(ui64Address, pui8Address);

    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER |
            (NRF24L01P_O_RX_ADDR_P0 + ui8Pipe);

    if(ui8Pipe > NRF24L01P_PIPE_1)
    {
        //
        // Only take LSByte
        //
        psInst->pui8Data[1] = pui8Address[7];

        ui8TransferSize = 2;
    }
    else
    {
        int i;
        for(i = 0; i < psInst->ui8AddrWidth; i++)
        {
            //
            // LSByte written first.
            //
            psInst->pui8Data[1 + i] = pui8Address[7 - i];
        }

        ui8TransferSize = 1 + psInst->ui8AddrWidth;
    }

    if(psInst->ui8AddrWidth == 3)
    {
        ui64AddrMask = ui8Pipe < 2 ? NRF24L01P_3BYTE_ADDR_M : NRF24L01P_1BYTE_ADDR_M;
    }
    else if(psInst->ui8AddrWidth == 4)
    {
        ui64AddrMask = ui8Pipe < 2 ? NRF24L01P_4BYTE_ADDR_M : NRF24L01P_1BYTE_ADDR_M;
    }
    else if(psInst->ui8AddrWidth == 5)
    {
        ui64AddrMask = ui8Pipe < 2 ? NRF24L01P_5BYTE_ADDR_M : NRF24L01P_1BYTE_ADDR_M;
    }

    //
    // Save Address
    //
    switch(ui8Pipe)
    {
        case NRF24L01P_PIPE_0:
            psInst->sPipeAddr.ui64RxAddrP0 = ui64Address & ui64AddrMask;
            break;
        case NRF24L01P_PIPE_1:
            psInst->sPipeAddr.ui64RxAddrP1 = ui64Address & ui64AddrMask;
            break;
        case NRF24L01P_PIPE_2:
            psInst->sPipeAddr.ui8RxAddrP2 = (uint8_t) (ui64Address & ui64AddrMask);
            break;
        case NRF24L01P_PIPE_3:
            psInst->sPipeAddr.ui8RxAddrP3 = (uint8_t) (ui64Address & ui64AddrMask);
            break;
        case NRF24L01P_PIPE_4:
            psInst->sPipeAddr.ui8RxAddrP4 = (uint8_t) (ui64Address & ui64AddrMask);
            break;
        case NRF24L01P_PIPE_5:
            psInst->sPipeAddr.ui8RxAddrP5 = (uint8_t) (ui64Address & ui64AddrMask);
            break;
    }


    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, ui8TransferSize,
                 NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Set the Transmit address.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui64Address address for transmission.
//!
//! This function sets the destination address for transmitting. Must comply
//! with set address width.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetTxAddress(tNRF24L01P *psInst, uint64_t ui64Address)
{
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_TX_ADDR;


    uint8_t pui8Address[8];
    uint64_t ui64AddrMask;

    NRF24L01P_UI64_TO_PUI8(ui64Address, pui8Address);

    //
    // Use assigned address width
    //
    int i;
    for(i = 0; i < psInst->ui8AddrWidth; i++)
    {
        //
        // LSByte written first.
        //
        psInst->pui8Data[1 + i] = pui8Address[7 - i];
    }

    if(psInst->ui8AddrWidth == 3)
    {
        ui64AddrMask = NRF24L01P_3BYTE_ADDR_M;
    }
    else if(psInst->ui8AddrWidth == 4)
    {
        ui64AddrMask = NRF24L01P_4BYTE_ADDR_M;
    }
    else if(psInst->ui8AddrWidth == 5)
    {
        ui64AddrMask = NRF24L01P_5BYTE_ADDR_M;
    }

    //
    // Save address
    //
    psInst->sPipeAddr.ui64TxAddr = ui64Address & ui64AddrMask;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 1 + psInst->ui8AddrWidth,
                 NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Get the Receive address.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Pipe pipe to get the address from (0..5, default 0).
//!
//! This function returns the receiving address for a given pipe. Will be the
//! size of the set address width.
//!
//! \return Returns the address associated with the particular pipe.
//
//*****************************************************************************
uint64_t NRF24L01PGetRxAddress(tNRF24L01P *psInst, uint8_t ui8Pipe)
{
    //
    // Set state.
    //
    psInst->ui8State = NRF24L01P_STATE_GET_RX_ADDR0 + ui8Pipe;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER |
                          (NRF24L01P_O_RX_ADDR_P0 + ui8Pipe);

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    switch(ui8Pipe)
    {
        case NRF24L01P_PIPE_0:
            return (psInst->sPipeAddr.ui64RxAddrP0);
        case NRF24L01P_PIPE_1:
            return (psInst->sPipeAddr.ui64RxAddrP1);
        case NRF24L01P_PIPE_2:
            return ((uint64_t) psInst->sPipeAddr.ui8RxAddrP2 |
                    (psInst->sPipeAddr.ui64RxAddrP1 & NRF24L01P_MSBYTES_ADDR_M));
        case NRF24L01P_PIPE_3:
            return ((uint64_t) psInst->sPipeAddr.ui8RxAddrP2 |
                    (psInst->sPipeAddr.ui64RxAddrP1 & NRF24L01P_MSBYTES_ADDR_M));
        case NRF24L01P_PIPE_4:
            return ((uint64_t) psInst->sPipeAddr.ui8RxAddrP2 |
                    (psInst->sPipeAddr.ui64RxAddrP1 & NRF24L01P_MSBYTES_ADDR_M));
        case NRF24L01P_PIPE_5:
            return ((uint64_t) psInst->sPipeAddr.ui8RxAddrP2 |
                    (psInst->sPipeAddr.ui64RxAddrP1 & NRF24L01P_MSBYTES_ADDR_M));
    }

    return (0);
}

//*****************************************************************************
//! Return the Transmit address.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//!  This function returns the destination address for transmitting. Will be
//! the size of the set address width.
//!
//! \return Returns destination address for transmission.
//
//*****************************************************************************
uint64_t NRF24L01PGetTxAddress(tNRF24L01P *psInst)
{
    //
    // Set state.
    //
    psInst->ui8State = NRF24L01P_STATE_GET_TX_ADDR;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_TX_ADDR;

    //
    // Start SPI transfer, save data after our current data written out
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    return (psInst->sPipeAddr.ui64TxAddr);
}

//*****************************************************************************
//! Set the transfer size.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Size the size of the transfer, in bytes (1..32).
//! \param ui8Pipe pipe for the transfer (0..5, default 0).
//!
//! This function sets the static transfer size for the RX payload. This is
//! used for each pipe only if Dynamic Payload isn't used. With static payload,
//! the payload length on the transmitter side is set by the number of bytes
//! clocked into the TX FIFO and must be equal to RX_PW_Px on the receiver side.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetTransferSize(tNRF24L01P *psInst, uint8_t ui8Size, uint8_t ui8Pipe )
{
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_RX_PW_P0 + ui8Pipe;

    //
    // Pack data
    //
    psInst->pui8Data[1] = (ui8Size << NRF24L01P_RX_PW_S) & NRF24L01P_RX_PW_M;

    //
    // Save size
    //
    psInst->pui8RxSPLSize[ui8Pipe] = psInst->pui8Data[1];

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Return the transfer size.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Pipe is the pipe to receive the transfer size for (0..5)
//!
//! This function returns the defined payload size to be used when utilizing
//! static payload transfer sizes.
//!
//! \return Returns the size of the transfer, in bytes (1..32).
//
//*****************************************************************************
uint8_t NRF24L01PGetTransferSize(tNRF24L01P *psInst, uint8_t ui8Pipe)
{
    //
    // Set state.
    //
    psInst->ui8State = NRF24L01P_STATE_GET_TRANS_SIZE;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_RX_PW_P0 + ui8Pipe;

    //
    // Start SPI transfer, save data after our current data written out
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data + 1,
                 1, NRF24L01PCallback, psInst);

    return (psInst->pui8RxSPLSize[ui8Pipe]);
}


//*****************************************************************************
//! Returns the RPD (Received Power Detector) state.
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function returns the current received power detector.
//!
//! \return Returns true if the received power exceeded -64dBm
//*****************************************************************************
bool NRF24L01PGetRPD(tNRF24L01P *psInst)
{
    //
    // Set state
    //
    psInst->ui8State = NRF24L01P_STATE_GET_RPD;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_RPD;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    //
    // Check value inside pui8Data
    //
    return (psInst->pui8Data[0] > 0);
}

//*****************************************************************************
//! Power up the nRF24L01+ into Standby mode
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function sets the internal bit-field PWR_UP high and puts the device
//! into Standby-I after the crystal oscillator is partially active. This is
//! meant to minimize average current consumption while maintaining short start
//! up times.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PPowerUp(tNRF24L01P *psInst)
{
    psInst->uConfig.ui1PowerUp = 1;

    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_CONFIG;

    //
    // Pack data
    //
    psInst->pui8Data[1] = psInst->uConfig.ui8Config;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Power down the nRF24L01+ into Power Down mode
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function puts the device into Power Down mode, disabling it. All
//! register values available are maintained and the SPI is kept active,
//! enabling change of configuration and the uploading/downloading of data
//! registers.
//!
//! \return None.
//!
//
//*****************************************************************************
void NRF24L01PPowerDown(tNRF24L01P *psInst)
{
    psInst->uConfig.ui1PowerUp = 0;

    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_CONFIG;

    //
    // Pack data
    //
    psInst->pui8Data[1] = psInst->uConfig.ui8Config;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Enable the nRF24L01+ to Receive or Transmit Mode(using the CE pin)
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function pulls the CE GPIO Pin high, setting the device into its
//! respective primary operating mode (PRX / PTX) depending on the PRIM_RX
//! bit-field in the CONFIG register.
//!
//! In RX mode is in an active mode where the nRF24L01+ radio acts as a primary
//! receiver. To be in this mode, the nRF24L01+ must have \e PWR_UP = 1,
//! \e PRIM_RX = 1, and the \e CE pin set high. The device remains in RX mode
//! until the MCU configures it to Standby-I mode or Power Down mode. However,
//! if utilizing Enhanced Shockburst™ automatic protocol features, the device
//! can enter other modes in order to execute the protocol.
//!
//! While in RX mode a Received Power Detector (RPD) signal is available.
//!
//! In TX mode is in an active mode where the nRF24L01+ radio acts as a primary
//! transmitter. To enter this mode, the nRF24L01+  must have \e PWR_UP = 1,
//! \e PRIM_RX = 0, a payload in the TX FIFO and a high pulse on the \e CE pin
//! for more than 10 us. If there is no TX payload available, the device moves
//! into Standby-II mode.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PEnableMode(tNRF24L01P *psInst)
{
    MAP_GPIOPinWrite(psInst->ui32CEBase, psInst->ui8CEPin, psInst->ui8CEPin);
}

//*****************************************************************************
//! Disable the nRF24L01+ to Receive or Transmit Mode(using the CE pin)
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function pulls the CE GPIO Pin low, setting the device into Standby-I
//! mode, a low power semi-active state.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PDisableMode(tNRF24L01P *psInst)
{
    MAP_GPIOPinWrite(psInst->ui32CEBase, psInst->ui8CEPin, 0);
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
//! \return the number of bytes actually received, 0 if none are received,
//! or -1 for an error
//
//*****************************************************************************
int32_t NRF24L01PReceive(tNRF24L01P *psInst, uint8_t ui8Pipe,
                         uint8_t *ui8Data, uint8_t ui8Count)
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
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_EN_RXADDR;

    //
    // Disable
    //
    psInst->uEnabledRxAddr.ui8Pipes = 0x00;

    psInst->pui8Data[1] = psInst->uEnabledRxAddr.ui8Pipes;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Disable AutoAcknowledge function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//!
//! This function disables all Auto Acknowledge for all pipes.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PDisableAutoAcknowledge(tNRF24L01P *psInst)
{
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_EN_AA;

    //
    // Disable
    //
    psInst->uEnabledAA.ui8Pipes = 0x00;

    psInst->pui8Data[1] = psInst->uEnabledAA.ui8Pipes;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Enable AutoAcknowledge function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Pipe the receive pipe
//!
//! This function enables Auto Acknowledge on a per pipe basis.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PEnableAutoAcknowledge(tNRF24L01P *psInst, uint8_t ui8Pipe)
{
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_EN_AA;

    //
    // Enable
    //
    psInst->uEnabledAA.ui8Pipes = 0xFF;

    psInst->pui8Data[1] = psInst->uEnabledAA.ui8Pipes;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Set Auto Retransmit Count function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Count number of retransmits before generating an error (1..15).
//!
//! This function sets the delay between Auto Retransmits and the number of
//! times to perform retransmission before generating an error.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetARC(tNRF24L01P *psInst, uint8_t ui8Count)
{
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_SETUP_RETR;

    //
    // Save
    //
    psInst->ui8ARC = ui8Count;

    //
    // Default, clear old values
    //
    psInst->pui8Data[1] = ((psInst->ui8ARC <<
                            NRF24L01P_SETUP_RETR_ARC_S) &
                            NRF24L01P_SETUP_RETR_ARC_M) |
                          ((psInst->ui8ARD <<
                            NRF24L01P_SETUP_RETR_ARC_S) &
                            NRF24L01P_SETUP_RETR_ARC_M);

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Set Auto Retransmit Count function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui8Count number of retransmits before generating an error (1..15).
//!
//! This function sets the number of times to perform retransmission before
//! generating an error.
//!
//! \return None.
//
//*****************************************************************************
uint8_t NRF24L01PGetARC(tNRF24L01P *psInst)
{
    //
    // Set state
    //
    psInst->ui8State = NRF24L01P_STATE_GET_ARC;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_SETUP_RETR;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    return (psInst->ui8ARC);
}

//*****************************************************************************
//! Set Auto Retransmit Delay function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui16Delay the delay between retransmits, in uS (250uS..4000uS).
//!
//! This function sets the delay between Auto Retransmits.
//!
//! \return None.
//
//*****************************************************************************
void NRF24L01PSetARD(tNRF24L01P *psInst, uint16_t ui16Delay)
{
    //
    // SPI command write register and address
    //
    psInst->pui8Data[0] = NRF24L01P_W_REGISTER | NRF24L01P_O_SETUP_RETR;

    //
    // Save
    //
    psInst->ui8ARD = ((NRF24L01P_SETUP_RETR_ARD_DELAY(ui16Delay) >>
                       NRF24L01P_SETUP_RETR_ARD_S) &
                       NRF24L01P_SETUP_RETR_ARD_M;

    //
    // Default, clear old values
    //
    psInst->pui8Data[1] = ((psInst->ui8ARC <<
                            NRF24L01P_SETUP_RETR_ARC_S) &
                            NRF24L01P_SETUP_RETR_ARC_M) |
                          ((psInst->ui8ARD <<
                            NRF24L01P_SETUP_RETR_ARC_S) &
                            NRF24L01P_SETUP_RETR_ARC_M);

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, NULL, 2, NRF24L01PCallback, psInst);
}

//*****************************************************************************
//! Set Auto Retransmit Delay function
//!
//! \param psInst is a pointer to the NRF24L01P instance data.
//! \param ui16Delay the delay between retransmits, in uS (250uS..4000uS).
//! \param ui8Count number of retransmits before generating an error (1..15).
//!
//! This function sets the delay between Auto Retransmits and the number of
//! times to perform retransmission before generating an error.
//!
//! \return None.
//
//*****************************************************************************
uint16_t NRF24L01PGetARD(tNRF24L01P *psInst)
{
    //
    // Set state
    //
    psInst->ui8State = NRF24L01P_STATE_GET_ARD;

    //
    // SPI command read register and address
    //
    psInst->pui8Data[0] = NRF24L01P_R_REGISTER | NRF24L01P_O_SETUP_RETR;

    //
    // Start SPI transfer
    //
    SPIMTransfer(psInst->psSPIInst, psInst->ui32CSBase, psInst->ui8CSPin,
                 psInst->pui8Data, psInst->pui8Data, 1, NRF24L01PCallback, psInst);

    return ((psInst->ui8ARD + 1) * 250);
}
