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
//! \file   ADS1x9x_ECG_Processing.h
//!
//! \brief  The software module is responsible extracting the heart rate from
//!			the ECG signals. Knowledge if signal processing are necessary to
//!			understand this file. Originally the algorithm was developed in
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
#ifndef ADS1x9x_ECG_PROCESSING_H_
#define ADS1x9x_ECG_PROCESSING_H_

//! \brief MAX_PEAK_TO_SEARCH
//!
#define MAX_PEAK_TO_SEARCH 				5

//! \brief MAXIMA_SEARCH_WINDOW
//!
#define MAXIMA_SEARCH_WINDOW			40

//! \brief MINIMUM_SKIP_WINDOW
//!
#define MINIMUM_SKIP_WINDOW				50

//! \brief SAMPLING_RATE
//!
#define SAMPLING_RATE					500

//! \brief TWO_SEC_SAMPLES
//!
#define TWO_SEC_SAMPLES  				2 * SAMPLING_RATE

/*threshold = 0.7 * maxima*/
#define QRS_THRESHOLD_FRACTION	0.7					

//! \brief MAXCHAN
//!
#define MAXCHAN						2

//! \brief FILTERORDER
//!
#define FILTERORDER 				161

#define TRUE	1
#define FALSE	0

//! \brief MUSCLE_ARTIFACT_FILTER
//!
#define MUSCLE_ARTIFACT_FILTER		1

//! \brief NOTCHFILTERSEL
//!
#define NOTCHFILTERSEL				1		// 0 - 50 Hz Notch filter
											// 1 - 60 Hz Notch filter

//! \brief DC Removal Numerator Coeff
//!
#define NRCOEFF (0.992)

//*****************************************************************************
//
//! \brief   This function is called by the main acquisition thread at every
//!			 samples read. Before calling the process_buffer() the below check
//!			 has to be done. i.e. We have always received +2 samples before
//!			 starting the processing  for each samples. This function basically
//!			 checks the difference between the current  and  previous ECG Samples
//!			 using 1st & 2nd differentiation calculations.
//
//! \param Lead     II sample CurrSample
//
//! \return  None
//
//*****************************************************************************
void QRS_Algorithm_Interface(short CurrSample);

//*****************************************************************************
//
//! \brief   The function process one sample filtering with 161 ORDER FIR
//!			 multiband filter 0.5 t0 150 Hz and 50/60Hz line nose. The
//!			 function supports compile time 50/60 Hz option
//
//! \param WorkingBuff     In - input sample buffer
//! \param CoeffBuf        In - Co-eficients for FIR filter.
//! \param FilterOut       Out - Filtered output
//
//! \return  None
//
//*****************************************************************************
void ECG_FilterProcess(short * WorkingBuff, short * CoeffBuf, short* FilterOut);

//*****************************************************************************
//
//! \brief   This function is the one that calls the Respiration detect and the
//!			 heart rate detect functions. Before calling each function, the function
//!			 makes sure that the ECG IC supports the mode. For instance not all
//!			 ADS1x9x support respiration rate calculation.
//
//! \param none     none
//
//! \return  None
//
//*****************************************************************************
void ADS1x9x_Filtered_ECG(void);

//*****************************************************************************
//
//! \brief   The function process one sample of data at a time and which stores
//!			 the filtered out sample in the Leadinfobuff. The function does the
//!			 following.
//!				- DC Removal of the current sample
//!				- Multi band 161 Tab FIR Filter with Notch at 50Hz/60Hz.
//
//! \param ECG_WorkingBuff     In - ECG. input sample buffer
//! \param FilterOut     	   Out - Filtered output
//
//! \return  None
//
//*****************************************************************************
//void ECG_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut);

//*****************************************************************************
//
//! \brief   The function process one sample of data at a time and which stores
//!			 the filtered out sample in the Leadinfobuff. The function does the
//!			 following.
//!				- DC Removal of the current sample
//!				- Multi band 161 Tab FIR Filter with Notch at 50Hz/60Hz.
//
//! \param ECG_WorkingBuff     In - ECG. input sample buffer
//! \param FilterOut     	   Out - Filtered output
//
//! \return  None
//
//*****************************************************************************
//void ECG_ProcessCurrSample_ch0(short *CurrAqsSample, short *FilteredOut);

//*****************************************************************************
//
//! \brief   This function computes duration of QRS peaks using order
//!			 differentiated input sample and computes QRS_Current_Sample.
//!			 After we process the data we can Heart rate. It mutes comptation
//!			 in case of leads off.
//
//! \param Scaled     Result
//! \param QRS_Heart_Rate and HR_flag
//
//! \return  None
//
//*****************************************************************************
//static void QRS_check_sample_crossing_threshold( unsigned short scaled_result );

//*****************************************************************************
//
//! \brief   This function will be doing the first and second order differentiation
//!			 for input sample, QRS_Current_Sample.After we process the data we can
//! 		 fill the QRS_Proc_Data_Buffer which is the input for HR calculation
//!			 Algorithm. This function is called for each n sample.Once we have
//!			 received 6s of processed 	data(i.e.Sampling rate*6) in the B4 buffer
//!			 we will start the heart rate calculation for first time and later we
//!			 will do heart rate calculations once we receive the defined number
//!			 of samples for the expected number of refresh seconds.
//
//! \param none     none
//
//! \return  None
//
//*****************************************************************************
//static void QRS_process_buffer( void );

#endif /*ADS1x9x_ECG_PROCESSING_H_*/
