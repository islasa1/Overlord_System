//*****************************************************************************
//
// ssim_drv.h - Prototypes for the interrupt-driven SPI master driver.
//
//*****************************************************************************

#ifndef __SPIM_DRV_H__
#define __SPIM_DRV_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// A prototype for the callback function used by the SPI master driver.
//
//*****************************************************************************
typedef void (tSPICallback)(void *pvData, uint_fast8_t ui8Status);

//*****************************************************************************
//
// The possible status values that can be returned by the SPI command callback.
//
//*****************************************************************************
#define SPIM_STATUS_SUCCESS     0
#define SPIM_STATUS_BATCH_DONE  1
#define SPIM_STATUS_BATCH_READY 2

//*****************************************************************************
//
// The maximum number of outstanding commands for each SPI master instance.
//
//*****************************************************************************
#define NUM_SPIM_COMMANDS       10

//*****************************************************************************
//
// The structure that defines an SPI master command.
//
//*****************************************************************************
typedef struct
{
    //
    // The SPI GPIO CS Base
    //
    uint32_t ui32SPICSBase;
    
    //
    // The SPI GPIO CS Pin
    //
    uint8_t ui8SPICSPin;

    //
    // The data buffer containing the data to be written.
    //
    const uint8_t *pui8WriteData;

    //
    // The data buffer to store data that has been read.
    //
    uint8_t *pui8ReadData;

    //
    // The total number of bytes to be read by the command.
    //
    uint16_t ui16TransferCount;

    //
    // The number of bytes to be read in each chuck.
    //
    uint16_t ui16TransferBatchSize;

    //
    // The function that is called when this command has been transferred.
    //
    tSPICallback *pfnCallback;

    //
    // The pointer provided to the callback function.
    //
    void *pvCallbackData;
}
tSPIMCommand;

//*****************************************************************************
//
// The structure that contains the state of an SPI master instance.
//
//*****************************************************************************
typedef struct
{
    //
    // The base address of the SPI module.
    //
    uint32_t ui32Base;

    //
    // The interrupt number associated with the SPI module.
    //
    uint8_t ui8Int;

    //
    // The uDMA channel used to write data to the SPI module.
    //
    uint8_t ui8TxDMA;

    //
    // The uDMA channel used to read data from the SPI module.
    //
    uint8_t ui8RxDMA;

    //
    // The current state of the SPI master driver.
    //
    uint8_t ui8State;

    //
    // The offset of the next command to be read.  The buffer is empty when
    // this value is equal to the write pointer.
    //
    uint8_t ui8ReadPtr;

    //
    // The offset of the next space in the buffer to write a command.  The
    // buffer is full if this value is one less than the read pointer.
    //
    uint8_t ui8WritePtr;

    //
    // The index into the data buffer of the next byte to be transferred.
    //
    uint16_t ui16TxIndex;
    uint16_t ui16RxIndex;

    //
    // An array of commands queued up to be sent via the SPI module.
    //
    tSPIMCommand pCommands[NUM_SPIM_COMMANDS];
}
tSPIMInstance;

//*****************************************************************************
//
// Prototypes.
//
//*****************************************************************************
extern void SPIMIntHandler(tSPIMInstance *psInst);
extern void SPIMInit(tSPIMInstance *psInst, uint32_t ui32Base,
                     uint_fast8_t ui8Int, uint_fast8_t ui8TxDMA,
                     uint_fast8_t ui8RxDMA, uint32_t ui32Clock, uint32_t ui32BitRate);
extern uint_fast8_t SPIMCommand(tSPIMInstance *psInst,
                                uint32_t ui32SPICSBase,
                                uint8_t ui8SPICSPin,
                                const uint8_t *pui8WriteData,
                                uint8_t *pui8ReadData,
                                uint_fast16_t ui16TransferCount,
                                uint_fast16_t ui16TransferBatchSize,
                                tSPICallback *pfnCallback,
                                void *pvCallbackData);
extern uint_fast8_t SPIMTransferResume(tSPIMInstance *psInst,
                                       const uint8_t *pui8WriteData,
                                       uint8_t *pui8ReadData);

//*****************************************************************************
//
// A convenience wrapper around SPIMCommand to perform a read.
//
//*****************************************************************************
inline uint_fast8_t
SPIMTransfer(tSPIMInstance *psInst, uint32_t ui32SPICSBase, uint8_t ui8SPICSPin,
             const uint8_t *pui8WriteData, uint8_t *pui8ReadData,
             uint_fast16_t ui16TransferCount, tSPICallback *pfnCallback,
             void *pvCallbackData)
{
    return(SPIMCommand(psInst, ui32SPICSBase, ui8SPICSPin,  pui8WriteData, pui8ReadData,
                       ui16TransferCount, ui16TransferCount, pfnCallback, pvCallbackData));
}

//*****************************************************************************
//
// A convenience wrapper around SPIMCommand to perform a batched read.
//
//*****************************************************************************
inline uint_fast8_t
SPIMTransferBatched(tSPIMInstance *psInst, uint32_t ui32SPICSBase, uint8_t ui8SPICSPin,
                    const uint8_t *pui8WriteData, uint8_t *pui8ReadData,
                    uint_fast16_t ui16TransferCount, uint_fast16_t ui16TransferBatchSize,
                    tSPICallback *pfnCallback, void *pvCallbackData)
{
    return(SPIMCommand(psInst, ui32SPICSBase, ui8SPICSPin, pui8WriteData,  pui8ReadData,
                       ui16TransferCount, ui16TransferBatchSize, pfnCallback, pvCallbackData));
}

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __SENSORLIB_SPIM_DRV_H__
