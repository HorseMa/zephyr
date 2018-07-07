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
//! \file   ADS1x9x_RESP_Processing.h
//!
//! \brief  The software module is responsible extracting the respiration rate
//!			from the ECG signals. Knowledge if signal processing are necessary
//!			to understand this file. Originally the algorithm was developed in
//!			MATLAB and eventually ported over to the MSP430.
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#ifndef ADS1x9x_RESP_PROCESSING_H_
#define ADS1x9x_RESP_PROCESSING_H_

#define RESP_MAX_PEAK_TO_SEARCH 			5
#define RESP_MAXIMA_SEARCH_WINDOW			8
#define RESP_MINIMUM_SKIP_WINDOW			80

#define RESP_SAMPLING_RATE				100
#define RESP_TWO_SEC_SAMPLES  			2 * RESP_SAMPLING_RATE

/*threshold = 0.7 * maxima*/
#define QRS_THRESHOLD_FRACTION	0.7					

#define MAXCHAN						2
#define FILTERORDER 				161

#define TRUE	1
#define FALSE	0

/* DC Removal Numerator Coeff*/
#define NRCOEFF (0.992)

//*****************************************************************************
//
//! \brief   The function process one sample of data at a time and which stores
//!			 the filtered out sample in the Leadinfobuff. The function does the
//!			 following
//!			 	- DC Removal of the current sample                          							**
//!				- Multi band FIR LPF with Notch at 50Hz filtering
//
//! \param RESP_WorkingBuff     In - input sample buffer
//! \param FilterOut     		 Out - Filtered output
//
//! \return  None
//
//*****************************************************************************
void Resp_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut);

//*****************************************************************************
//
//! \brief   This function is called by the main acquisition thread at every
//!			 samples read. Before calling the process_buffer() the below check
//!	  		 has to be done. i.e. We have always received +2 samples before
//!			 starting the processing  for each samples. This function basically
//!			 checks the difference between the current  and  previous ECG Samples
//!			 using 1st & 2nd differentiation calculations.
//!			 	- DC Removal of the current sample
//!				- Multi band FIR LPF with Notch at 50Hz filtering
//
//! \param Respiration CurrSample
//
//! \return  None
//
//*****************************************************************************
void RESP_Algorithm_Interface(short CurrSample);

//*****************************************************************************
//
//! \brief   The function process one sample filtering with 161 ORDER FIR low
//!			 pass filter with 2Hz .
//
//! \param RESP_WorkingBuff In - input sample buffer
//! \param CoeffBuf  In - Co-eficients for FIR filter.
//! \param FilterOut Out - Filtered output
//
//! \return  None
//
//*****************************************************************************
//void Resp_FilterProcess(short * RESP_WorkingBuff, short * CoeffBuf, short* FilterOut);


//*****************************************************************************
//
//! \brief   The function detects the respiration rate.
//
//! \param Resp_wave ECG samples
//
//! \return  None
//
//*****************************************************************************
//void Respiration_Rate_Detection(short Resp_wave);

#endif /*ADS1x9x_RESP_PROCESSING_H_*/
