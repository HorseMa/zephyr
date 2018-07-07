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
//! \file   ECGInterface.h
//!
//! \brief  This is the IC interface file for ECG ICs. This IC interface
//! 		file contains function calls which are only found in the ADS1292R IC
//!			driver file. If its corresponding ADS1292R IC driver files changes,
//!			then this IC interface file should also change as well. This IC
//!			interface file has generic function calls which should be found in
//!			similar ECG IC’s. For example power up IC, power down IC, get ECG.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef ECGINTERFACE_H_
#define ECGINTERFACE_H_

//! \brief This #define variable defines the number of delay cycles in a delay
//! 		subroutine. An update to this file would be adding a state machine and
//! 		incorporating a timer.
#define DELAY_COUNT 2

//*****************************************************************************
//
//! \brief   This function initializes the AD1292R IC by calling functions
//!			 found in the AD1292R IC driver file.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_Init(void);

//*****************************************************************************
//
//! \brief   This function puts the AD1292R IC in Normal mode. According to the
//!			 datasheet the current consumption in this mode is 670 uW.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_powerMode0(void);

//*****************************************************************************
//
//! \brief   This function puts the AD1292R IC in Standby mode. According to the
//!			 datasheet the current consumption in this mode is 160 uW.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_powerMode1(void);

//*****************************************************************************
//
//! \brief   This function handles everything needed to write to the ADS1292
//!			 registers. Because the functions call is remote, it has the
//!			 possibility to hang. Caution should be used when calling this function.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_RemoteWriteRegister(void);

//*****************************************************************************
//
//! \brief   This function handles everything needed to read from the ADS1292
//!			 registers. Because the functions call is remote, it has the
//!			 possibility to hang. Caution should be used when calling this function.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_RemoteReadRegister(void);

//*****************************************************************************
//
//! \brief   This functions sets the ECG Interface in streaming mode. Streaming
//!			 mode basically transferred the ECG data directly to a file.
//!			 Preprocessing is done before the transfer to extract the respiration
//!			 rate, heart rate, and lead status.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_StreamData(void);

//*****************************************************************************
//
//! \brief   This function sets the ECG Interface to acquire data while the phone
//!			 is not close to the MPBSM system. Another way of calling this mode
//!			 is data logging mode. This function should be used in conjunction
//!			 with ECGInterface_DownloadData().
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_AcquireData(void);

//*****************************************************************************
//
//! \brief   This function downloads the data which was gathered by the
//!			 ECGInterface_AcquireData() function call and displays it in the
//!			 Android app.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_DownloadData(void);

//*****************************************************************************
//
//! \brief   This function enables the MSP430 boot loader. It allows the MPS430
//!			 to upgrade its firmware. This function should be moved.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_FirmwareUpgrade(void);

//*****************************************************************************
//
//! \brief   This function sends the Firmware version to the Android app.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_FirmwareVersion(void);

//*****************************************************************************
//
//! \brief   This function sets up the filter used by the respiration rate and
//!			 heart rate algorithms.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_SelectFilter(void);

//*****************************************************************************
//
//! \brief   This function works similar if not identical to  the
//!			 ECGInterface_AcquireData function. This is a residual function
//!			 from the ADS1x9x EVM software.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_KeyPressed(void);

//*****************************************************************************
//
//! \brief   This function disables whatever settings where done by the
//!		     ECGInterface_KeyPressed() function. This function is also a residual
//!			 function from the ADS1x9x EVM software.
//
//! \param none     none
//
//! \return  none
//
//*****************************************************************************
void ECGInterface_KeyNotPressed(void);

#endif /* ECGINTERFACE_H_ */










