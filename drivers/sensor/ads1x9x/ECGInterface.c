/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//#############################################################################
//
//! \file   ECGInterface.c
//!
//! \brief  ECGInterface.h
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################

//*****************************************************************************
// the includes
//*****************************************************************************
#include <sensor.h>
#include <misc/byteorder.h>
#include <kernel.h>
#include <misc/__assert.h>
#include "ECGInterface.h"
#include "ADS1x9xTI.h"
#include "ADS1x9x_Version.h"

uint8_t ECGRxPacket[64],ECGRxCount, dumy ;

uint8_t ECGTxPacket[64],ECGTxCount,ECGTxPacketRdy;

unsigned regval, Live_Streaming_flag;
unsigned Req_Dwnld_occured;
extern unsigned char Filter_Option;

void ECGInterface_Init(void)
{

    volatile unsigned short Init_i, j;

    ADS1x9x_Reset();
    for (j = 0; j < DELAY_COUNT; j++)
    {
        for ( Init_i =0; Init_i < 20000; Init_i++);
        for ( Init_i =0; Init_i < 20000; Init_i++);
        for ( Init_i =0; Init_i < 20000; Init_i++);
    }
    Init_ADS1x9x_DRDY_Interrupt();
    ADS1x9x_Clock_Select(1);        // Set internal clock
    for ( Init_i =0; Init_i < 20000; Init_i++);
    for ( Init_i =0; Init_i < 20000; Init_i++);
    for ( Init_i =0; Init_i < 20000; Init_i++);

    //The START pin must be set high to begin conversions.
    ADS1x9x_Disable_Start();// Set START pin to LOW
    ADS1x9x_Enable_Start();// Set START pin to High

    /* Tie the START pin low to control conversions by command.*/
    Hard_Stop_ADS1x9x();// Set START pin to Low

    //Why send a start command just to send a stop command.
    /* * If the START command is immediately followed by a STOP command then
     *   have a gap of 4 tCLK cycles between them.
     *   --
     *   When the START opcode is sent to the device, keep the START pin low
     *   until the STOP command is issued.
     *
     *   */
    Start_Data_Conv_Command();
    Soft_Stop_ADS1x9x();

    for (j = 0; j < DELAY_COUNT; j++)
    {
        for ( Init_i =0; Init_i < 20000; Init_i++);
    }

    /*The read data continuous mode is the device default mode;
     * the device defaults to this mode on power-up.*/
    Stop_Read_Data_Continuous();                    // SDATAC command
    for (j = 0; j < DELAY_COUNT; j++)
    {
       for ( Init_i =0; Init_i < 35000; Init_i++);
    }
    for (j = 0; j < DELAY_COUNT; j++)
    {
       for ( Init_i =0; Init_i < 35000; Init_i++);
    }
    ADS1x9x_Read_All_Regs(ADS1x9xRegVal);
    ADS1x9x_Default_Reg_Init();
    ADS1x9x_Read_All_Regs(ADS1x9xRegVal);

    //Put_ADS1x9x_In_Sleep();
    ADS1x9x_PowerDown_Enable();
}

void ECGInterface_powerMode0(void)
{

}

void ECGInterface_powerMode1(void)
{

}
#if 0
void ECGInterface_RemoteWriteRegister(void)
{

#if 0
    unsigned short strcpy_i;

    if ( (ECGRxPacket[2] < 12))
    {

        ADS1x9x_Disable_Start();

        Disable_ADS1x9x_DRDY_Interrupt();
        Stop_Read_Data_Continuous();                // SDATAC command

        ADS1x9x_Reg_Write (ECGRxPacket[2], ECGRxPacket[3]);
        ADS1x9xRegVal[ECGRxPacket[2]] = ECGRxPacket[3];

        //                        k_busy_wait(30);
        //                        Start_Read_Data_Continuous();         //RDATAC command
        //                        k_busy_wait(30);
        //                        Enable_ADS1x9x_DRDY_Interrupt();

        ECG_Data_rdy = 0;
        Set_Device_out_bytes();
    }
    else
    {
        ECGRxPacket[2]  =0;
        ECGRxPacket[3]  =0;
    }

    for (strcpy_i = 0; strcpy_i < 7; strcpy_i++)
    {
        ECGTxPacket[strcpy_i]=ECGRxPacket[strcpy_i];  // Prepare the outgoing string
    }

    ECGTxCount = 7;
    ECGTxPacketRdy = true;

#endif
}

void ECGInterface_RemoteReadRegister(void)
{

#if 0
    unsigned short strcpy_i;

    if ( (FileTextE105[5] < 12))
    {

        ADS1x9x_Disable_Start();                    // Disable Start

        Disable_ADS1x9x_DRDY_Interrupt();
        Stop_Read_Data_Continuous();                // SDATAC command

        FileTextE105[6] = ADS1x9x_Reg_Read (ECGRxPacket[2]);
        ADS1x9xRegVal[FileTextE105[5]] = FileTextE105[6];
        k_busy_wait(300);
        ECG_Data_rdy = 0;
        //ECG_Recoder_state.state = IDLE_STATE;
    }
    else
    {
        FileTextE105[5]  =0;
        FileTextE105[6]  =0;
    }
    for (strcpy_i = 0; strcpy_i < 7; strcpy_i++)
    {
        ECGTxPacket[strcpy_i]=ECGRxPacket[strcpy_i];                                      // Prepare the outgoing string
    }

    ECGTxCount = 7;

    ECGTxPacketRdy = true;
#endif
}

#endif
void ECGInterface_StreamData(void)
{
    ADS1x9x_Disable_Start();            //set ADC_START pin to LOW

    Clear_ADS1x9x_Chip_Enable();        //set SPI_CS pin to High
    k_busy_wait(300);
    Set_ADS1x9x_Chip_Enable();          //set SPI_CS pin to LOW

    Start_Read_Data_Continuous();       //RDATAC command
    k_busy_wait(30);
    Enable_ADS1x9x_DRDY_Interrupt();    // Enable DRDY interrupt

    ADS1x9x_Enable_Start();             //Set ADC_START pin to High

    //ECG_Recoder_state.state = DATA_STREAMING_STATE;   // Set to Live Streaming state
    Live_Streaming_flag = true;             // Set Live Streaming Flag
    ECG_Data_rdy = 0;
    //Sleep
}

#if 0
void ECGInterface_AcquireData(void)
{
    short i_acq;
    unsigned short strcpy_i;

    if ( (ECGRxPacket[2] | ECGRxPacket[3]) && ((ADS1x9xRegVal[1] & 0x80) != 0x80) )
    {

        ADS1x9x_Disable_Start();                // Disable START (SET START to high)

        Set_ADS1x9x_Chip_Enable();              // CS = 0
        k_busy_wait(300);
        Clear_ADS1x9x_Chip_Enable();            // CS = 1

#if 0
        UCA1CTL1 |= UCSWRST;                    // Enable SW reset
        UCA1CTL0 |= UCMSB+UCMST+UCSYNC;         //[b0]   1 -  Synchronous mode
                                                //[b2-1] 00-  3-pin SPI
                                                //[b3]   1 -  Master mode
                                                //[b4]   0 - 8-bit data
                                                //[b5]   1 - MSB first
                                                //[b6]   0 - Clock polarity low.
                                                //[b7]   1 - Clock phase - Data is captured on the first UCLK edge and changed on the following edge.

        UCA1CTL1 |= UCSSEL__ACLK;               // ACLK
        UCA1BR0 = 2;                            // 12 MHz
        UCA1BR1 = 0;                            //
        UCA1CTL1 &= ~UCSWRST;                   // Clear SW reset, resume operation

#endif
        NumPackets = ECGRxPacket[2];            // Parse the 16 bit sample count
        NumPackets = NumPackets << 8;
        NumPackets |= ECGRxPacket[3];
        ReqSamples = NumPackets;
        NumFrames = 0;
        ECGTxCount = 7;
        ECGTxPacketRdy = true;
        BlockNum =5;
        //if ((ADS1x9xRegVal[1] & 0x07) == 6)
        if ((ADS1x9xRegVal[1] & 0x07) == 6)
        {
//          NAND_Init();
//          NAND_Reset();
            k_busy_wait(300);
            k_busy_wait(3000);
            for ( i_acq= 1; i_acq < 4096; i_acq++)
            {
//              NAND_EraseBlock(i_acq);         // Erase blank
            }

            ECGTxPacketRdy = false;
            Store_data_rdy =0;
        }
//      Recorder_head = 0;
//      Recorder_tail =0;
//      Acquire_NANDAddress.usBlockNum = BlockNum;
//      Acquire_NANDAddress.ucPageNum = 0;
//      Acquire_NANDAddress.usColNum = 0;
        AcqpacketCounter = 0;

        Set_ADS1x9x_Chip_Enable();              // CS =0
        k_busy_wait(300);
        Start_Read_Data_Continuous();           //RDATAC command
        k_busy_wait(300);
            //ECG_Recoder_state.state = ACQUIRE_DATA_STATE; // state = ACQUIRE_DATA_STATE
        ECG_Data_rdy = 0;
        Enable_ADS1x9x_DRDY_Interrupt();        // Enable DRDY interrupt
        ADS1x9x_Enable_Start();                 // Enable START (SET START to high)

        for (strcpy_i = 0; strcpy_i < 7; strcpy_i++)
        {
            ECGTxPacket[strcpy_i]=ECGRxPacket[strcpy_i];  // Prepare the outgoing string
        }
        //TA1CTL |= MC_1;                          // Turn ON Timer


    }
    else
    {
        ECGRxPacket[2] = 0;
        ECGRxPacket[3] = 0;

        for (strcpy_i = 0; strcpy_i < 7; strcpy_i++)
        {
            ECGTxPacket[strcpy_i]=ECGRxPacket[strcpy_i];  // Prepare the outgoing string
        }
       ECGTxCount = 7;
       ECGTxPacketRdy = true;

    }

}

#endif
void ECGInterface_DownloadData(void)
{
    Req_Dwnld_occured = true;
}

void ECGInterface_FirmwareUpgrade(void)
{
    Disable_ADS1x9x_DRDY_Interrupt();           // Disable interrupt
    k_busy_wait(200);                       // Delay
    ((void (*)())0x1000)();                     // Go to BSL
}

void ECGInterface_FirmwareVersion(void)
{

    unsigned short strcpy_i;

    for (strcpy_i = 0; strcpy_i < 7; strcpy_i++)
    {
        ECGTxPacket[strcpy_i]=ECGRxPacket[strcpy_i];
                                                // Prepare the outgoing string
    }

    ECGTxPacket[2] = ADS1x9x_Major_Number;      // Firmware Major number
    ECGTxPacket[3] = ADS1x9x_Minor_Number;      // Firmware Minor number

    ECGTxCount = 7;                             // number of bytes to send
    ECGTxPacketRdy = true;
}

void ECGInterface_SelectFilter(void)
{

    unsigned short strcpy_i;

    if ( (ECGRxPacket[2] < 4) && (ECGRxPacket[2] != 1) )
    {
        Filter_Option =  ECGRxPacket[3];            // Filter option from user
    }
    else
    {
        ECGRxPacket[2]  =0;
        ECGRxPacket[3]  =0;
    }

    for (strcpy_i = 0; strcpy_i < 7; strcpy_i++)
    {
        ECGTxPacket[strcpy_i]=ECGRxPacket[strcpy_i];  // Prepare the outgoing string
    }

    ECGTxCount = 7;
    ECGTxPacketRdy = true;

}

void ECGInterface_KeyPressed(void)
{

    ADS1x9x_Disable_Start();
    ADS1x9x_Enable_Start();
    ADS1x9x_Disable_Start();            // Disable START (SET START to high)

    Set_ADS1x9x_Chip_Enable();          // CS = 0
    k_busy_wait(300);
    Clear_ADS1x9x_Chip_Enable();        // CS = 1
    k_busy_wait(30000);
    k_busy_wait(30000);
    Set_ADS1x9x_Chip_Enable();          // CS =0

    k_busy_wait(300);
    Start_Read_Data_Continuous();       //RDATAC command
    k_busy_wait(300);
    Enable_ADS1x9x_DRDY_Interrupt();    // Enable DRDY interrupt
    ADS1x9x_Enable_Start();             // Enable START (SET START to high)

    //ECG_Recoder_state.state = ECG_RECORDING_STATE;

}

void ECGInterface_KeyNotPressed(void)
{

    //ECG_Recoder_state.state = IDLE_STATE;
    Disable_ADS1x9x_DRDY_Interrupt();       // Disable DRDY interrupt
    Stop_Read_Data_Continuous();            // SDATAC command

}


