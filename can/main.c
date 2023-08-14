#include <stdint.h>                     // Library of Standard Integer Types
#include <stdbool.h>                    // Library of Standard Boolean Types
#include "inc/hw_memmap.h"              // Macros defining the memory map of the Tiva C Series device
#include "inc/hw_types.h"               // Defines common types and macros
#include "driverlib/sysctl.h"           // Defines and macros for System Control API of DriverLib
#include "driverlib/gpio.h"             // Defines and macros for GPIO API of DriverLib
#include "inc/can.h"                    // Defines and macros for CAN API of DriverLib

tCANBitClkParms CANBitClk;
tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx;
uint8_t pui8BufferIn[8u];
uint8_t pui8BufferOut[8u];

/*
Sends out data from CAN controller 0 to be received by CAN controller 1. In
order to actually receive the data, an external cable must be connected between the two ports. In
this example, both controllers are configured for 1 Mbit operation.
*/
void main(){
    //
    // Enable the CAN0 module.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    //
    // Wait for the CAN0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0))
    { }
    //
    // Enable the CAN1 module.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
    //
    // Wait for the CAN1 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN1))
    { }
    //
    // Reset the state of all the message objects and the state of the CAN
    // module to a known state.
    //
    CANInit(CAN0_BASE);
    CANInit(CAN1_BASE);
    //
    // Configure the controller for 1 Mbit operation.
    //
    CANBitRateSet(CAN0_BASE, ui32SysClock, 1000000u);
    CANBitRateSet(CAN1_BASE, ui32SysClock, 1000000u);
    //
    // Take the CAN0 and CAN1 device out of INIT state.
    //
    CANEnable(CAN0_BASE);
    CANEnable(CAN1_BASE);
    //
    // Configure a receive object.
    //
    sMsgObjectRx.ui32MsgID = 0x400u;
    sMsgObjectRx.ui32MsgIDMask = 0x7f8u;
    sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER;
    sMsgObjectRx.pui8MsgData = pui8BufferIn;
    CANMessageSet(CAN1_BASE, 4u, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
    
    //
    // Configure and start transmit of message object.
    //
    sMsgObjectTx.ui32MsgID = 0x400u;
    sMsgObjectTx.ui32Flags = 0u;
    sMsgObjectTx.ui32MsgLen = 8u;
    sMsgObjectTx.pui8MsgData = pui8BufferOut;
    CANMessageSet(CAN0_BASE, 2u, &sMsgObjectTx, MSG_OBJ_TYPE_TX);
    //
    // Wait for new data to become available by checking bit 4 (0x08u).
    //
    while((CANStatusGet(CAN1_BASE, CAN_STS_NEWDAT) & 0x08u) == 0u)
    { }
    //
    // Read the message out of the message object 4.
    //
    CANMessageGet(CAN1_BASE, 4u, &sMsgObjectRx, true);
    //
    // Process new data in sMsgObjectRx.pucMsgData.
    //

}

void CAN_Config_Mess_Ojb_FIFO_Mode()
{
    //
    // Configure the controller for 1 Mbit operation.
    //
    CANBitRateSet(CAN0_BASE, ui32SysClock, 1000000u);
    //
    // Take the CAN0 device out of INIT state.
    //
    CANEnable(CAN0_BASE);
    //
    // Configure a receive object as a CAN FIFO to receive message objects with
    // message ID 0x400-0x407.
    //
    
    sMsgObjectRx.ui32MsgID = 0x400u;
    sMsgObjectRx.ui32MsgIDMask = 0x7f8u;
    sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO | MSG_OBJ_RX_INT_ENABLE;
    //
    // The first three message objects have the MSG_OBJ_FIFO set to indicate
    // that they are part of a FIFO.
    //
    CANMessageSet(CAN0_BASE, 1u, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
    CANMessageSet(CAN0_BASE, 2u, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
    CANMessageSet(CAN0_BASE, 3u, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
    //
    // Last message object does not have the MSG_OBJ_FIFO set to indicate that
    // this is the last message.
    //
    sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_RX_INT_ENABLE;
    CANMessageSet(CAN0_BASE, 4u, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
    //
    // Enable the CAN interrupt for the peripheral.
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR );
    //
    // Enable the CAN interrupt in the NVIC.
    //
    IntEnable(INT_CAN0);
    //
    // Enable global interrupts.
    //
    IntMasterEnable();
}
void ISR_Read_4_Mess_FIFO_Buff()
{
    // Circular buffer for receive CAN frames
    #define BUFFER_SIZE 1024u
    volatile uint64_t g_pui64Array[BUFFER_SIZE];
    volatile bool g_bErrFlag;
    volatile bool g_bRXFlag;
    volatile uint32_t g_ui32RxMsgCount;
    volatile uint32_t g_ui32ArrayHead;
    static uint32_t g_ui32ArrayTail;
    void CAN0IntHandler(void)
    {
    uint32_t ui32Status, i;
    tCANMsgObject sMsgObjectRx;
    //
    // Read the CAN interrupt status to find the cause of the interrupt.
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    //
    // If the cause is a controller status interrupt, then get the status.
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
    //
    // Read the controller status. This will return a field of status
    
    // error bits that can indicate various errors. Error processing
    // is not done in this example for simplicity. Refer to the
    // API documentation for details about the error status bits.
    // The act of reading this status will clear the interrupt. If the
    // CAN peripheral is not connected to a CAN bus with other CAN devices
    // present, then errors will occur and will be indicated in the
    // controller status.
    //
    ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
    //
    // Set a flag to indicate some errors may have occurred.
    //
    g_bErrFlag = 1u;
    }
    //
    // Check if the cause is a receive message object.
    //
    else if((ui32Status > 0u) & (ui32Status < 5u))
    {
    for(i = ui32Status; i < 5u; i++)
    {
    //
    // Getting to this point means that the RX interrupt occurred.
    // Read the message, clear the interrupt.
    //
    sMsgObjectRx.ui32Flags = MSG_OBJ_NO_FLAGS;
    sMsgObjectRx.pui8MsgData = (void *)&(g_pui64Array[g_ui32ArrayHead]);
    CANMessageGet(CAN0_BASE, i, &sMsgObjectRx, true);
    if((sMsgObjectRx.ui32Flags & MSG_OBJ_NEW_DATA) == MSG_OBJ_NEW_DATA)
    {
    // Increment head of circular buffer
    g_ui32ArrayHead++;
    g_ui32ArrayHead = g_ui32ArrayHead % BUFFER_SIZE;
    //
    // Increment a counter to keep track of how many messages have been
    // received. In a real application this could be used to set flags to
    // indicate when a message is received.
    //
    g_ui32RxMsgCount++;
    }
    else
    {
    break;
    }
    }
    }
    //
    // Otherwise, something unexpected caused the interrupt. This should
    // never happen.
    //
    else
    {
    //
    // Spurious interrupt handling can go here.
    //
    }
    }
    }    