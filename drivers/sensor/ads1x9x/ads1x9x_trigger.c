/* Bosch ADS1X9X inertial measurement unit driver, trigger implementation
 *
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <sensor.h>
#include <gpio.h>
#include <misc/byteorder.h>
#include <misc/__assert.h>
#ifndef __KERNEL__
#include <stdlib.h>
#endif
#include "ads1x9x.h"
#include "ADS1x9x_ECG_Processing.h"
#include "ADS1x9x_RESP_Processing.h"

unsigned short Respiration_Rate = 0 ;
//unsigned char RR_flag;

/* Variables to hold the sample data for calculating the 1st and 2nd */
/* differentiation                                                   */
int RESP_Second_Prev_Sample = 0 ;
int RESP_Prev_Sample = 0 ;
int RESP_Current_Sample = 0 ;
int RESP_Next_Sample = 0 ;
int RESP_Second_Next_Sample = 0 ;

short RESP_WorkingBuff[2 * FILTERORDER];
//extern unsigned short Resp_Rr_val;
static unsigned char LeadStatus;
#if (FILTERORDER == 161)

short RespCoeffBuf[FILTERORDER] = {

/* Coeff for lowpass Fc=2Hz @ 500 SPS*/

       15,     16,     16,     17,     18,     19,     20,     22,     23,
       25,     27,     29,     32,     34,     37,     41,     44,     48,
       51,     56,     60,     64,     69,     74,     80,     85,     91,
       97,    103,    109,    116,    123,    130,    137,    144,    152,
      159,    167,    175,    183,    191,    199,    207,    216,    224,
      232,    241,    249,    257,    266,    274,    282,    290,    298,
      306,    313,    321,    328,    336,    343,    349,    356,    362,
      368,    374,    379,    385,    389,    394,    398,    402,    406,
      409,    412,    414,    416,    418,    419,    420,    421,    421,
      421,    420,    419,    418,    416,    414,    412,    409,    406,
      402,    398,    394,    389,    385,    379,    374,    368,    362,
      356,    349,    343,    336,    328,    321,    313,    306,    298,
      290,    282,    274,    266,    257,    249,    241,    232,    224,
      216,    207,    199,    191,    183,    175,    167,    159,    152,
      144,    137,    130,    123,    116,    109,    103,     97,     91,
       85,     80,     74,     69,     64,     60,     56,     51,     48,
       44,     41,     37,     34,     32,     29,     27,     25,     23,
       22,     20,     19,     18,     17,     16,     16,     15

};
#endif

#if (FILTERORDER == 161)
short CoeffBuf_40Hz_LowPass[FILTERORDER] = {

      -33,     19,     48,     44,      9,    -49,   -118,   -179,   -217,
     -222,   -191,   -131,    -57,     13,     61,     74,     48,    -12,
      -92,   -173,   -233,   -257,   -237,   -178,    -92,     -1,     73,
      110,     99,     40,    -52,   -156,   -246,   -299,   -299,   -244,
     -145,    -26,     83,    155,    170,    119,     14,   -123,   -257,
     -355,   -389,   -347,   -234,    -75,     91,    224,    286,    256,
      135,    -53,   -266,   -449,   -555,   -547,   -417,   -186,     97,
      365,    547,    584,    447,    145,   -271,   -711,  -1067,  -1230,
    -1111,   -664,    100,   1114,   2259,   3386,   4334,   4967,   5189,
     4967,   4334,   3386,   2259,   1114,    100,   -664,  -1111,  -1230,
    -1067,   -711,   -271,    145,    447,    584,    547,    365,     97,
     -186,   -417,   -547,   -555,   -449,   -266,    -53,    135,    256,
      286,    224,     91,    -75,   -234,   -347,   -389,   -355,   -257,
     -123,     14,    119,    170,    155,     83,    -26,   -145,   -244,
     -299,   -299,   -246,   -156,    -52,     40,     99,    110,     73,
       -1,    -92,   -178,   -237,   -257,   -233,   -173,    -92,    -12,
       48,     74,     61,     13,    -57,   -131,   -191,   -222,   -217,
     -179,   -118,    -49,      9,     44,     48,     19,    -33
};

const short CoeffBuf_60Hz_Notch[FILTERORDER] = {
/* Coeff for Notch @ 60Hz for 500SPS/60Hz Notch coeff13102008*/
      131,    -16,     85,     97,   -192,   -210,      9,    -37,    -11,
      277,    213,   -105,    -94,   -100,   -324,   -142,    257,    211,
      121,    242,    -38,   -447,   -275,    -39,    -79,    221,    543,
      181,   -187,   -138,   -351,   -515,     34,    446,    260,    303,
      312,   -344,   -667,   -234,    -98,    -46,    585,    702,    -17,
     -241,   -197,   -683,   -552,    394,    540,    239,    543,    230,
     -811,   -700,    -44,   -254,     81,   1089,    594,   -416,    -81,
     -249,  -1195,   -282,   1012,    223,     80,   1170,   -156,  -1742,
       21,    543,  -1503,    505,   3202,  -1539,  -3169,   9372,  19006,
     9372,  -3169,  -1539,   3202,    505,  -1503,    543,     21,  -1742,
     -156,   1170,     80,    223,   1012,   -282,  -1195,   -249,    -81,
     -416,    594,   1089,     81,   -254,    -44,   -700,   -811,    230,
      543,    239,    540,    394,   -552,   -683,   -197,   -241,    -17,
      702,    585,    -46,    -98,   -234,   -667,   -344,    312,    303,
      260,    446,     34,   -515,   -351,   -138,   -187,    181,    543,
      221,    -79,    -39,   -275,   -447,    -38,    242,    121,    211,
      257,   -142,   -324,   -100,    -94,   -105,    213,    277,    -11,
      -37,      9,   -210,   -192,     97,     85,    -16,    131
};

const short CoeffBuf_50Hz_Notch[FILTERORDER] = {
/* Coeff for Notch @ 50Hz @ 500 SPS*/
      -47,   -210,    -25,    144,     17,     84,    249,     24,   -177,
      -58,   -144,   -312,    -44,    191,     78,    185,    357,     42,
     -226,   -118,   -248,   -426,    -61,    243,    134,    290,    476,
       56,   -282,   -169,   -352,   -549,    -70,    301,    177,    392,
      604,     60,   -344,   -200,   -450,   -684,    -66,    369,    191,
      484,    749,     44,   -420,   -189,   -535,   -843,    -32,    458,
      146,    560,    934,    -16,   -532,    -89,   -600,  -1079,     72,
      613,    -50,    614,   1275,   -208,   -781,    308,   -642,  -1694,
      488,   1141,  -1062,    642,   3070,  -1775,  -3344,   9315,  19005,
     9315,  -3344,  -1775,   3070,    642,  -1062,   1141,    488,  -1694,
     -642,    308,   -781,   -208,   1275,    614,    -50,    613,     72,
    -1079,   -600,    -89,   -532,    -16,    934,    560,    146,    458,
      -32,   -843,   -535,   -189,   -420,     44,    749,    484,    191,
      369,    -66,   -684,   -450,   -200,   -344,     60,    604,    392,
      177,    301,    -70,   -549,   -352,   -169,   -282,     56,    476,
      290,    134,    243,    -61,   -426,   -248,   -118,   -226,     42,
      357,    185,     78,    191,    -44,   -312,   -144,    -58,   -177,
       24,    249,     84,     17,    144,    -25,   -210,    -47
};
#endif

unsigned int DL_ADC[2][125]=    //缓冲区
   {
      {0},
	  {0},
   };

void ADC_DLLB(short in, u8_t index,short *out)    //队列滚动滤波, DL_long: 滤波队列长度
{
    unsigned char i=0;
 	long temp=0;
 	static u8_t len = 16;
	for(i = 0;i < (len-1);i++)
	{
		DL_ADC[index][(len - 1) - i] = DL_ADC[index][(len - 2) - i];
	}
	DL_ADC[index][0] = in;
	for(i = 0;i < len;i++)
	{
		temp += DL_ADC[index][i];
	}
	*out = temp / len;
}
static  uint8_t SPI_Rx_Data_Flag = 0,  SPI_Rx_buf[12];
extern u8_t ads1x9xregval[];
static long ADS1x9x_ECG_Data_buf[6];
static  short ECGRawData[4],ECGFilteredData[4] ;
static  unsigned short QRS_Heart_Rate;
static unsigned char Filter_Option = 0;
static short ECG_WorkingBuff[2 * FILTERORDER];
static short ECG_ch0_WorkingBuff[2 * FILTERORDER];
static short ECG_WorkingBuff[2 * FILTERORDER];
static unsigned char HR_flag;
static unsigned short QRS_B4_Buffer_ptr = 0 ;

/* 	Variable which holds the threshold value to calculate the maxima */
short QRS_Threshold_Old = 0;
short QRS_Threshold_New = 0;
/* Variables to hold the sample data for calculating the 1st and 2nd */
/* differentiation                                                   */
int QRS_Second_Prev_Sample = 0 ;
int QRS_Prev_Sample = 0 ;
int QRS_Current_Sample = 0 ;
int QRS_Next_Sample = 0 ;
int QRS_Second_Next_Sample = 0 ;

/* Flag which identifies the duration for which sample count has to be incremented */
unsigned char Start_Sample_Count_Flag = 0;
unsigned char first_peak_detect = FALSE ;
unsigned int sample_count = 0 ;
unsigned int sample_index[MAX_PEAK_TO_SEARCH+2] ;

void Resp_FilterProcess(short * RESP_WorkingBuff, short * CoeffBuf, short* FilterOut)
{
  int32_t acc = 0;   // accumulator for MACs
  int  k;

  // perform the multiply-accumulate
  for ( k = 0; k < 161; k++ )
  {
    acc += (int32_t)(*CoeffBuf++) * (int32_t)(*RESP_WorkingBuff--);
  }
  // saturate the result
  if ( acc > 0x3fffffff )
  {
    acc = 0x3fffffff;
  } else if ( acc < -0x40000000 )
  {
    acc = -0x40000000;
  }
  // convert from Q30 to Q15
  *FilterOut = (int16_t)(acc >> 15);
  //*FilterOut = *WorkingBuff;
#if 0
	RESHI = 0;
	RESLO = 0;
	MPYS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	OP2 = *CoeffBuf++;                             // Load second operand

	for ( i = 0; i < FILTERORDER/10; i++)
	{
	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	  MACS = *RESP_WorkingBuff--;                             // Load first operand -unsigned mult
	  OP2 = *CoeffBuf++;                             // Load second operand

	}

	 Val_Hi = RESHI << 1;                       // Q15 result
	 Val_Lo = RESLO >> 15;
	 Val_Lo &= 0x01;
	 *FilterOut = Val_Hi | Val_Lo;
	#endif
}

void Resp_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut)
{

 	static unsigned short bufStart=0, bufCur = FILTERORDER-1, FirstFlag = 1;
 	static short Pvev_DC_Sample, Pvev_Sample;
 	short temp1, temp2, RESPData;

	/* Count variable*/
	unsigned short Cur_Chan;
	short FiltOut;

	if  ( FirstFlag )
	{
		for ( Cur_Chan =0 ; Cur_Chan < FILTERORDER; Cur_Chan++)
		{
			RESP_WorkingBuff[Cur_Chan] = 0;
		}

		Pvev_DC_Sample = 0;
		Pvev_Sample = 0;
		FirstFlag = 0;
	}
	temp1 = NRCOEFF * Pvev_DC_Sample;
	Pvev_DC_Sample = (CurrAqsSample[0]  - Pvev_Sample) + temp1;
	Pvev_Sample = CurrAqsSample[0];
	temp2 = Pvev_DC_Sample >> 2;
	RESPData = (short) temp2;

	/* Store the DC removed value in RESP_WorkingBuff buffer in millivolts range*/
	RESP_WorkingBuff[bufCur] = RESPData;
	//ADC_DLLB(RESPData,1,(short*)&FiltOut);
	ECG_FilterProcess(&RESP_WorkingBuff[bufCur],RespCoeffBuf,(short*)&FiltOut);
	/* Store the DC removed value in Working buffer in millivolts range*/
	RESP_WorkingBuff[bufStart] = RESPData;


	//FiltOut = RESPData[Cur_Chan];

	/* Store the filtered out sample to the LeadInfo buffer*/
	FilteredOut[0] = FiltOut ;//(CurrOut);

	bufCur++;
	bufStart++;
	if ( bufStart  == (FILTERORDER-1))
	{
		bufStart=0;
		bufCur = FILTERORDER-1;
	}

	return ;
}

void Respiration_Rate_Detection(short Resp_wave)
{

	static unsigned short skipCount = 0, SampleCount = 0,TimeCnt=0, SampleCountNtve=0, PtiveCnt =0,NtiveCnt=0 ;
	static short MinThreshold = 0x7FFF, MaxThreshold = 0x8000, PrevSample = 0, PrevPrevSample = 0, PrevPrevPrevSample =0;
	static short MinThresholdNew = 0x7FFF, MaxThresholdNew = 0x8000, AvgThreshold = 0;
	static unsigned char startCalc=0, PtiveEdgeDetected=0, NtiveEdgeDetected=0, peakCount = 0;
	static unsigned short PeakCount[8];

	//printk("%s\r\n",__func__);
	SampleCount++;
	SampleCountNtve++;
	TimeCnt++;
	if (Resp_wave < MinThresholdNew) MinThresholdNew = Resp_wave;
	if (Resp_wave > MaxThresholdNew) MaxThresholdNew = Resp_wave;

	if (SampleCount > 800)
	{
		SampleCount =0;
	}
	if (SampleCountNtve > 800)
	{
		SampleCountNtve =0;
	}


	if ( startCalc == 1)
	{
		if (TimeCnt >= 500)
		{
			TimeCnt =0;
			if ( (MaxThresholdNew - MinThresholdNew) > 400)
			{
				MaxThreshold = MaxThresholdNew;
				MinThreshold =  MinThresholdNew;
				AvgThreshold = MaxThreshold + MinThreshold;
				AvgThreshold = AvgThreshold >> 1;
			}
			else
			{
				startCalc = 0;
				Respiration_Rate = 0;
			}
		}

		PrevPrevPrevSample = PrevPrevSample;
		PrevPrevSample = PrevSample;
		PrevSample = Resp_wave;
		if ( skipCount == 0)
		{
			if (PrevPrevPrevSample < AvgThreshold && Resp_wave > AvgThreshold)
			{
				if ( SampleCount > 40 &&  SampleCount < 700)
				{
//						Respiration_Rate = 6000/SampleCount;	// 60 * 100/SampleCount;
					PtiveEdgeDetected = 1;
					PtiveCnt = SampleCount;
					skipCount = 4;
				}
				SampleCount = 0;
			}
			if (PrevPrevPrevSample < AvgThreshold && Resp_wave > AvgThreshold)
			{
				if ( SampleCountNtve > 40 &&  SampleCountNtve < 700)
				{
					NtiveEdgeDetected = 1;
					NtiveCnt = SampleCountNtve;
					skipCount = 4;
				}
				SampleCountNtve = 0;
			}

			if (PtiveEdgeDetected ==1 && NtiveEdgeDetected ==1)
			{
				PtiveEdgeDetected = 0;
				NtiveEdgeDetected =0;

				if (abs(PtiveCnt - NtiveCnt) < 5)
				{
					PeakCount[peakCount++] = PtiveCnt;
					PeakCount[peakCount++] = NtiveCnt;
					if( peakCount == 8)
					{
						peakCount = 0;
						PtiveCnt = PeakCount[0] + PeakCount[1] + PeakCount[2] + PeakCount[3] +
								PeakCount[4] + PeakCount[5] + PeakCount[6] + PeakCount[7];
						PtiveCnt = PtiveCnt >> 3;
						Respiration_Rate = 6000/PtiveCnt;	// 60 * 100/SampleCount;
						//printk("Respiration_Rate = %d\r\n",Respiration_Rate);
					}
				}
			}
		}
		else
		{
			skipCount--;
		}

	}
	else
	{
		TimeCnt++;
		if (TimeCnt >= 500)
		{
			TimeCnt = 0;
			if ( (MaxThresholdNew - MinThresholdNew) > 400)
			{
				startCalc = 1;
				MaxThreshold = MaxThresholdNew;
				MinThreshold =  MinThresholdNew;
				AvgThreshold = MaxThreshold + MinThreshold;
				AvgThreshold = AvgThreshold >> 1;
				PrevPrevPrevSample = Resp_wave;
				PrevPrevSample = Resp_wave;
				PrevSample = Resp_wave;

			}
		}
	}
}

void RESP_Algorithm_Interface(short CurrSample)
{
//	static FILE *fp = fopen("RESPData.txt", "w");
	static short prev_data[64] ={0};
	static unsigned char Decimeter = 0;
	char i;
	long Mac=0;
	prev_data[0] = CurrSample;
	for ( i=63; i > 0; i--)
	{
		Mac += prev_data[i];
		prev_data[i] = prev_data[i-1];

	}
	Mac += CurrSample;
//	Mac = Mac;
	CurrSample = (short) Mac >> 1;
	RESP_Second_Prev_Sample = RESP_Prev_Sample ;
	RESP_Prev_Sample = RESP_Current_Sample ;
	RESP_Current_Sample = RESP_Next_Sample ;
	RESP_Next_Sample = RESP_Second_Next_Sample ;
	RESP_Second_Next_Sample = CurrSample;// << 3 ;
//	fprintf(fp,"%d\n", CurrSample);
	Decimeter++;
	//Resp_Rr_val = RESP_Second_Next_Sample;
	if ( Decimeter == 5)
	{
		Decimeter = 0;
//		RESP_process_buffer();
		Respiration_Rate_Detection(RESP_Second_Next_Sample);
	}
}

void ECG_FilterProcess(short * WorkingBuff, short * CoeffBuf, short* FilterOut)
{
  int32_t acc = 0;   // accumulator for MACs
  int  k;

  // perform the multiply-accumulate
  for ( k = 0; k < 161; k++ )
  {
    acc += (int32_t)(*CoeffBuf++) * (int32_t)(*WorkingBuff--);
  }
  // saturate the result
  if ( acc > 0x3fffffff )
  {
    acc = 0x3fffffff;
  } else if ( acc < -0x40000000 )
  {
    acc = -0x40000000;
  }
  // convert from Q30 to Q15
  *FilterOut = (int16_t)(acc >> 15);
  //*FilterOut = *WorkingBuff;
#if 0
	RESHI = 0;
	RESLO = 0;
	MPYS = *WorkingBuff--;				// Load first operand -unsigned mult
	OP2 = *CoeffBuf++;					// Load second operand

	for ( i = 0; i < FILTERORDER/10; i++)
	{

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

		MACS = *WorkingBuff--;			// Load first operand -unsigned mult
		OP2 = *CoeffBuf++;				// Load second operand

	}

	 Val_Hi = RESHI << 1;				// Q15 result
	 Val_Lo = RESLO >> 15;
	 Val_Lo &= 0x01;
	 *FilterOut = Val_Hi | Val_Lo;
	#endif
}

void ECG_ProcessCurrSample(short *CurrAqsSample, short *FilteredOut)
{

	static unsigned short ECG_bufStart=0, ECG_bufCur = FILTERORDER-1, ECGFirstFlag = 1;
 	static short ECG_Pvev_DC_Sample, ECG_Pvev_Sample;/* Working Buffer Used for Filtering*/
//	static short ECG_WorkingBuff[2 * FILTERORDER];
 	short *CoeffBuf;

 	short temp1, temp2, ECGData;


	/* Count variable*/
	unsigned short Cur_Chan;
	short FiltOut;
//	short FilterOut[2];
	CoeffBuf = CoeffBuf_40Hz_LowPass;					// Default filter option is 40Hz LowPass
	if ( Filter_Option == 2)
	{
		CoeffBuf = CoeffBuf_50Hz_Notch;					// filter option is 50Hz Notch & 0.5-150 Hz Band
	}
	else if ( Filter_Option == 3)
	{
		CoeffBuf = CoeffBuf_60Hz_Notch;					// filter option is 60Hz Notch & 0.5-150 Hz Band
	}
	if  ( ECGFirstFlag )								// First Time initialize static variables.
	{
		for ( Cur_Chan =0 ; Cur_Chan < FILTERORDER; Cur_Chan++)
		{
			ECG_WorkingBuff[Cur_Chan] = 0;
		}
		ECG_Pvev_DC_Sample = 0;
		ECG_Pvev_Sample = 0;
		ECGFirstFlag = 0;
	}
	temp1 = NRCOEFF * ECG_Pvev_DC_Sample;				//First order IIR
	ECG_Pvev_DC_Sample = (CurrAqsSample[0]  - ECG_Pvev_Sample) + temp1;
	ECG_Pvev_Sample = CurrAqsSample[0];
	temp2 = ECG_Pvev_DC_Sample >> 2;
	ECGData = (short) temp2;

	/* Store the DC removed value in Working buffer in millivolts range*/
	ECG_WorkingBuff[ECG_bufCur] = ECGData;
	//ADC_DLLB(ECGData,0,(short*)&FiltOut);
	ECG_FilterProcess(&ECG_WorkingBuff[ECG_bufCur],CoeffBuf,(short*)&FiltOut);
	/* Store the DC removed value in ECG_WorkingBuff buffer in millivolts range*/
	ECG_WorkingBuff[ECG_bufStart] = ECGData;

	//FiltOut = ECGData[Cur_Chan];

	/* Store the filtered out sample to the LeadInfo buffer*/
	FilteredOut[0] = FiltOut ;//(CurrOut);

	ECG_bufCur++;
	ECG_bufStart++;
	if ( ECG_bufStart  == (FILTERORDER-1))
	{
		ECG_bufStart=0;
		ECG_bufCur = FILTERORDER-1;
	}

	return ;
}

void ECG_ProcessCurrSample_ch0(short *CurrAqsSample, short *FilteredOut)
{

 	static unsigned short ECG_ch0_bufStart=0, ECG_ch0_bufCur = FILTERORDER-1, ECG_ch0FirstFlag = 1;
 	static short ECG_ch0_Pvev_DC_Sample, ECG_ch0_Pvev_Sample;
 	//static short ECG_ch0_WorkingBuff[2 * FILTERORDER];
 	short *CoeffBuf;

 	short temp1_ch0, temp2_ch0, ECGData_ch0;


	/* Count variable*/
	unsigned short Cur_Chan_ch0;
	short FiltOut_ch0;

	CoeffBuf = CoeffBuf_40Hz_LowPass;					// Default filter option is 40Hz LowPass
	if ( Filter_Option == 2)
	{
		CoeffBuf = CoeffBuf_50Hz_Notch;					// filter option is 50Hz Notch & 0.5-150 Hz Band
	}
	else if ( Filter_Option == 3)
	{
		CoeffBuf = CoeffBuf_60Hz_Notch;					// filter option is 60Hz Notch & 0.5-150 Hz Band
	}
	if  ( ECG_ch0FirstFlag )							// First time initialize static variables
	{
		for ( Cur_Chan_ch0 =0 ; Cur_Chan_ch0 < FILTERORDER; Cur_Chan_ch0++)
		{
			ECG_ch0_WorkingBuff[Cur_Chan_ch0] = 0;
		}
		ECG_ch0_Pvev_DC_Sample = 0;
		ECG_ch0_Pvev_Sample = 0;
		ECG_ch0FirstFlag = 0;
	}
	temp1_ch0 = NRCOEFF * ECG_ch0_Pvev_DC_Sample;			// First order IIR
	ECG_ch0_Pvev_DC_Sample = (CurrAqsSample[0]  - ECG_ch0_Pvev_Sample) + temp1_ch0;
	ECG_ch0_Pvev_Sample = CurrAqsSample[0];
	temp2_ch0 = ECG_ch0_Pvev_DC_Sample >> 2;
	ECGData_ch0 = (short) temp2_ch0;

	/* Store the DC removed value in Working buffer in millivolts range*/
	ECG_ch0_WorkingBuff[ECG_ch0_bufCur] = ECGData_ch0;
	/* */
	ECG_FilterProcess(&ECG_ch0_WorkingBuff[ECG_ch0_bufCur],CoeffBuf,(short*)&FiltOut_ch0);
	/* Store the DC removed value in ECG_WorkingBuff buffer in millivolts range*/
	ECG_ch0_WorkingBuff[ECG_ch0_bufStart] = ECGData_ch0;

	//FiltOut = ECGData[Cur_Chan];

	/* Store the filtered out sample to the LeadInfo buffer*/
	FilteredOut[0] = FiltOut_ch0 ;//(CurrOut);

	ECG_ch0_bufCur++;
	ECG_ch0_bufStart++;
	if ( ECG_ch0_bufStart  == (FILTERORDER-1))
	{
		ECG_ch0_bufStart=0;
		ECG_ch0_bufCur = FILTERORDER-1;
	}

	return ;
}

static void QRS_check_sample_crossing_threshold( unsigned short scaled_result )
{
    /* array to hold the sample indexes S1,S2,S3 etc */

    static unsigned short s_array_index = 0 ;
    static unsigned short m_array_index = 0 ;

    static unsigned char threshold_crossed = false ;
    static unsigned short maxima_search = 0 ;
    static unsigned char peak_detected = false ;
    static unsigned short skip_window = 0 ;
    static long maxima_sum = 0 ;
    static unsigned int peak = 0;
    static unsigned int sample_sum = 0;
    static unsigned int nopeak=0;
    unsigned short max = 0 ;
    unsigned short HRAvg;

    //printk("%s\r\n",__func__);
    if( true == threshold_crossed  )
    {
        /*
        Once the sample value crosses the threshold check for the
        maxima value till MAXIMA_SEARCH_WINDOW samples are received
        */
        sample_count ++ ;
        maxima_search ++ ;

        if( scaled_result > peak )
        {
            peak = scaled_result ;
        }

        if( maxima_search >= MAXIMA_SEARCH_WINDOW )
        {
            // Store the maxima values for each peak
            maxima_sum += peak ;
            maxima_search = 0 ;

            threshold_crossed = false ;
            peak_detected = true ;
        }

    }
    else if( true == peak_detected )
    {
        /*
        Once the sample value goes below the threshold
        skip the samples untill the SKIP WINDOW criteria is meet
        */
        sample_count ++ ;
        skip_window ++ ;

        if( skip_window >= MINIMUM_SKIP_WINDOW )
        {
            skip_window = 0 ;
            peak_detected = false ;
        }

        if( m_array_index == MAX_PEAK_TO_SEARCH )
        {
            sample_sum = sample_sum / (MAX_PEAK_TO_SEARCH - 1);
            HRAvg =  (unsigned short) sample_sum  ;
#if 0
            if((LeadStatus & 0x0005)== 0x0000)
            {

            QRS_Heart_Rate = (unsigned short) 60 *  SAMPLING_RATE;
            QRS_Heart_Rate =  QRS_Heart_Rate/ HRAvg ;
                if(QRS_Heart_Rate > 250)
                    QRS_Heart_Rate = 250 ;
            }
            else
            {
                QRS_Heart_Rate = 0;
            }
#else
            // Compute HR without checking LeadOffStatus
            QRS_Heart_Rate = (unsigned short) 60 *  SAMPLING_RATE;
            QRS_Heart_Rate =  QRS_Heart_Rate/ HRAvg ;
            //printk("QRS_Heart_Rate = %d\r\n",QRS_Heart_Rate);
            if(QRS_Heart_Rate > 250)
                QRS_Heart_Rate = 250 ;
#endif

            /* Setting the Current HR value in the ECG_Info structure*/

            HR_flag = 1;

            maxima_sum =  maxima_sum / MAX_PEAK_TO_SEARCH;
            max = (short) maxima_sum ;
            /*  calculating the new QRS_Threshold based on the maxima obtained in 4 peaks */
            maxima_sum = max * 7;
            maxima_sum = maxima_sum/10;
            QRS_Threshold_New = (short)maxima_sum;

            /* Limiting the QRS Threshold to be in the permissible range*/
            if(QRS_Threshold_New > (4 * QRS_Threshold_Old))
            {
                QRS_Threshold_New = QRS_Threshold_Old;
            }

            sample_count = 0 ;
            s_array_index = 0 ;
            m_array_index = 0 ;
            maxima_sum = 0 ;
            sample_index[0] = 0 ;
            sample_index[1] = 0 ;
            sample_index[2] = 0 ;
            sample_index[3] = 0 ;
            Start_Sample_Count_Flag = 0;

            sample_sum = 0;
        }
    }
    else if( scaled_result > QRS_Threshold_New )
    {
        /*
            If the sample value crosses the threshold then store the sample index
        */
        Start_Sample_Count_Flag = 1;
        sample_count ++ ;
        m_array_index++;
        threshold_crossed = true ;
        peak = scaled_result ;
        nopeak = 0;

        /*  storing sample index*/
        sample_index[ s_array_index ] = sample_count ;
        if( s_array_index >= 1 )
        {
            sample_sum += sample_index[ s_array_index ] - sample_index[ s_array_index - 1 ] ;
        }
        s_array_index ++ ;
    }

    else if(( scaled_result < QRS_Threshold_New ) && (Start_Sample_Count_Flag == 1))
    {
        sample_count ++ ;
        nopeak++;
        if (nopeak > (3 * SAMPLING_RATE))
        {
            sample_count = 0 ;
            s_array_index = 0 ;
            m_array_index = 0 ;
            maxima_sum = 0 ;
            sample_index[0] = 0 ;
            sample_index[1] = 0 ;
            sample_index[2] = 0 ;
            sample_index[3] = 0 ;
            Start_Sample_Count_Flag = 0;
            peak_detected = false ;
            sample_sum = 0;

            first_peak_detect = false;
            nopeak=0;

            QRS_Heart_Rate = 0;
            HR_flag = 1;
        }
    }
   else
   {
    nopeak++;
    if (nopeak > (3 * SAMPLING_RATE))
     {
        /* Reset heart rate computation sate variable in case of no peak found in 3 seconds */
        sample_count = 0 ;
        s_array_index = 0 ;
        m_array_index = 0 ;
        maxima_sum = 0 ;
        sample_index[0] = 0 ;
        sample_index[1] = 0 ;
        sample_index[2] = 0 ;
        sample_index[3] = 0 ;
        Start_Sample_Count_Flag = 0;
        peak_detected = false ;
        sample_sum = 0;
        first_peak_detect = false;
        nopeak = 0;
        QRS_Heart_Rate = 0;
        HR_flag = 1;

     }
   }

}

static void QRS_process_buffer( void )
{

    short first_derivative = 0 ;
    short scaled_result = 0 ;

    static short max = 0 ;

    /* calculating first derivative*/
    first_derivative = QRS_Next_Sample - QRS_Prev_Sample  ;

    /*taking the absolute value*/

    if(first_derivative < 0)
    {
        first_derivative = -(first_derivative);
    }

    scaled_result = first_derivative;

    if( scaled_result > max )
    {
        max = scaled_result ;
    }

    QRS_B4_Buffer_ptr++;
    if (QRS_B4_Buffer_ptr ==  TWO_SEC_SAMPLES)
    {
        QRS_Threshold_Old = ((max *7) /10 ) ;
        QRS_Threshold_New = QRS_Threshold_Old ;
        if(max > 70)
        first_peak_detect = true ;
        max = 0;
        QRS_B4_Buffer_ptr = 0;
    }


    if( true == first_peak_detect )
    {
        QRS_check_sample_crossing_threshold( scaled_result ) ;
    }
}

void QRS_Algorithm_Interface(short CurrSample)
{
//  static FILE *fp = fopen("ecgData.txt", "w");
    static short prev_data[32] ={0};
    short i;
    long Mac=0;
    prev_data[0] = CurrSample;
    for ( i=31; i > 0; i--)
    {
        Mac += prev_data[i];
        prev_data[i] = prev_data[i-1];

    }
    Mac += CurrSample;
    Mac = Mac >> 2;
    CurrSample = (short) Mac;
    QRS_Second_Prev_Sample = QRS_Prev_Sample ;
    QRS_Prev_Sample = QRS_Current_Sample ;
    QRS_Current_Sample = QRS_Next_Sample ;
    QRS_Next_Sample = QRS_Second_Next_Sample ;
    QRS_Second_Next_Sample = CurrSample ;
    QRS_process_buffer();
}

void ADS1x9x_Filtered_ECG(void)
{
	//ADS1x9x_ECG_Data_buf[1] = 0-ADS1x9x_ECG_Data_buf[1];
	//printk("%s\r\n",__func__);
	static int loop = 0;
	switch( ads1x9xregval[0] & 0x03)
	{

		case ADS1191_16BIT:
		{
		   ADS1x9x_ECG_Data_buf[1] = ADS1x9x_ECG_Data_buf[1] << 4;
		   ADS1x9x_ECG_Data_buf[1] &= 0xFFFF;

		   ECGRawData[0] = (short)ADS1x9x_ECG_Data_buf[1];

		   ECG_ProcessCurrSample(&ECGRawData[0],&ECGFilteredData[0]);
		   QRS_Algorithm_Interface(ECGFilteredData[0]);
		   ECGFilteredData[1] = ECGFilteredData[0];
		}
		break;

		case ADS1192_16BIT:
		{
		   ADS1x9x_ECG_Data_buf[1] = ADS1x9x_ECG_Data_buf[1] << 4;
		   ADS1x9x_ECG_Data_buf[2] = ADS1x9x_ECG_Data_buf[2] << 4;

		   ADS1x9x_ECG_Data_buf[1] &= 0xFFFF;
		   ADS1x9x_ECG_Data_buf[2] &= 0xFFFF;
		   ECGRawData[0] = (short)ADS1x9x_ECG_Data_buf[1];
		   ECGRawData[1] = (short)ADS1x9x_ECG_Data_buf[2];

		   ECG_ProcessCurrSample_ch0(&ECGRawData[0],&ECGFilteredData[0]);
		   ECG_ProcessCurrSample(&ECGRawData[1],&ECGFilteredData[1]);
		   QRS_Algorithm_Interface(ECGFilteredData[1]);
		}
		break;

		case ADS1291_24BIT:
		{
		   ADS1x9x_ECG_Data_buf[1] = ADS1x9x_ECG_Data_buf[1] >> 4;

		   ADS1x9x_ECG_Data_buf[1] &= 0xFFFF;

		   ECGRawData[0] = (short)ADS1x9x_ECG_Data_buf[1];

		   ECG_ProcessCurrSample(&ECGRawData[0],&ECGFilteredData[0]);
		   QRS_Algorithm_Interface(ECGFilteredData[0]);
		   ECGFilteredData[1] = ECGFilteredData[0];
		}
		break;

		case ADS1292_24BIT:
		{
			if ((ads1x9xregval[0]& 0x20) == 0x20)
			{
			   ADS1x9x_ECG_Data_buf[1] = ADS1x9x_ECG_Data_buf[1];//3 bytes ch1 data
			   ADS1x9x_ECG_Data_buf[2] = ADS1x9x_ECG_Data_buf[2] >> 4;//3 bytes CH0 data

			   ADS1x9x_ECG_Data_buf[1] &= 0xFFFF;//3 bytes ch1 data
			   ADS1x9x_ECG_Data_buf[2] &= 0xFFFF;//3 bytes ch0 data

			   ECGRawData[0] = (short)ADS1x9x_ECG_Data_buf[1];//3 bytes ch1 data
			   ECGRawData[1] = (short)ADS1x9x_ECG_Data_buf[2];//3 bytes ch0 data

			   Resp_ProcessCurrSample(&ECGRawData[0],&ECGFilteredData[0]);
			   ECG_ProcessCurrSample(&ECGRawData[1],&ECGFilteredData[1]);
			   if(loop % 10 == 0)
			   {
			   		//printk("%d,%d,%d\n",loop/5,ECGRawData[0],ECGRawData[1]);
			   		printk("%d,%d,%d\n",loop/10,ECGFilteredData[0],ECGFilteredData[1]);
			   	}
			   	loop ++;
			   //printk("%d,%d\n",ECGRawData[0]/0xff,ECGRawData[1]/0xff);
			   //printk("%d,%d,%d\n",loop++,ECGFilteredData[0],ECGFilteredData[1]);
			   RESP_Algorithm_Interface(ECGFilteredData[0]);
			   QRS_Algorithm_Interface(ECGFilteredData[1]);
			}
			else
			{
			   ADS1x9x_ECG_Data_buf[1] = ADS1x9x_ECG_Data_buf[1] >> 4;//3 bytes ch1 data
			   ADS1x9x_ECG_Data_buf[2] = ADS1x9x_ECG_Data_buf[2] >> 4;//3 bytes CH0 data

			   ADS1x9x_ECG_Data_buf[1] &= 0xFFFF;
			   ADS1x9x_ECG_Data_buf[2] &= 0xFFFF;

			   ECGRawData[0] = (short)ADS1x9x_ECG_Data_buf[1];
			   ECGRawData[1] = (short)ADS1x9x_ECG_Data_buf[2];
			   ECG_ProcessCurrSample_ch0(&ECGRawData[0],&ECGFilteredData[0]);
			   ECG_ProcessCurrSample(&ECGRawData[1],&ECGFilteredData[1]);
			   QRS_Algorithm_Interface(ECGFilteredData[1]);

			}
		}
		break;

	}
}

void ADS1191_Parse_data_packet(void)
{
  	uint8_t ECG_Chan_num;

	for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
	{
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num];   // Get MSB 8 bits
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];       // Get LSB 8 bits
	}
	ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;               // to make compatable with 24 bit devices
}

void ADS1192_Parse_data_packet(void)
{
	uint8_t ECG_Chan_num;

	for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
	{
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num]; // Get MSB Bits15-bits8
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];       // Get LSB Bits7-bits0
    }
    ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;       // to make compatable with 24 bit devices
}

void ADS1291_Parse_data_packet(void)
{
	uint8_t ECG_Chan_num;

	for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
	{
    	ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[3*ECG_Chan_num]; // Get Bits23-bits16

		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+1];       // Get Bits15-bits8

		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+2];       // Get Bits7-bits0
    }
}

void ADS1292x_Parse_data_packet(void)
{
	uint8_t ECG_Chan_num;
	//static int loop = 0;
	for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
	{
		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[3*ECG_Chan_num];

		ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+1];

        ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
        ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+2];
    }
    //ADS1x9x_ECG_Data_buf[0] = 3 byte status (24 bits)
    //ADS1x9x_ECG_Data_buf[1] = 3 bytes ch1 data
    //ADS1x9x_ECG_Data_buf[2] = 3 bytes CH0 data
    //printk("%c,%c",ADS1x9x_ECG_Data_buf[1]>>16,ADS1x9x_ECG_Data_buf[2]>>16);
    //printk("%d,%d,%d\n",loop++,ADS1x9x_ECG_Data_buf[1],ADS1x9x_ECG_Data_buf[2]);
}

void ADS1x9x_Parse_data_packet(void)
{

	switch( ads1x9xregval[0] & 0x03)
	{
		case ADS1191_16BIT:
		{
			ADS1191_Parse_data_packet();
		}

		break;

		case ADS1192_16BIT:
		{
			ADS1192_Parse_data_packet();
		}

		break;

		case ADS1291_24BIT:
		{
			ADS1291_Parse_data_packet();
		}

		break;

		case ADS1292_24BIT:
		{
			ADS1292x_Parse_data_packet();
		}
		break;
	}
	ADS1x9x_Filtered_ECG();
}

static void ads1x9x_handle_interrupts(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;
	static int level = 0;
	//level = level ^ 1;
	//printk("ads1x9x data ready!!!\r\n");
	//gpio_pin_write(ads1x9x->gpio, 20,level);
	ads1x9x_read_data(dev,SPI_Rx_buf);
	/*for(u8_t i = 0;i < 9;i++)
	{
		printk("%02X ",SPI_Rx_buf[i]);
	}
	//printk("\r\n");*/
	ADS1x9x_Parse_data_packet();
	if((level ++) % 2)
	{
		//ads1x9x_write_reg(dev,11,0x03);
		//gpio_pin_write(ads1x9x->gpio, 20,1);
	}
	else
	{
		//ads1x9x_write_reg(dev,11,0x00);
		//gpio_pin_write(ads1x9x->gpio, 20,0);
	}
	//printk("QRS_Heart_Rate = %d\r\n",QRS_Heart_Rate);
	//printk("Respiration_Rate = %d\r\n",Respiration_Rate);
}

#ifdef CONFIG_ADS1X9X_TRIGGER_OWN_THREAD
static K_THREAD_STACK_DEFINE(ads1x9x_thread_stack, CONFIG_ADS1X9X_THREAD_STACK_SIZE);
static struct k_thread ads1x9x_thread;

static void ads1x9x_thread_main(void *arg1, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	struct device *dev = (struct device *)arg1;
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	while (1) {
		k_sem_take(&ads1x9x->sem, K_FOREVER);
		ads1x9x_handle_interrupts(dev);
	}
}
#endif

#ifdef CONFIG_ADS1X9X_TRIGGER_GLOBAL_THREAD
static void ads1x9x_work_handler(struct k_work *work)
{
	struct ads1x9x_device_data *ads1x9x =
		CONTAINER_OF(work, struct ads1x9x_device_data, work);

	ads1x9x_handle_interrupts(ads1x9x->dev);
}
#endif

extern struct ads1x9x_device_data ads1x9x_data;

static void ads1x9x_gpio_callback(struct device *port,
				 struct gpio_callback *cb, u32_t pin)
{
	struct ads1x9x_device_data *ads1x9x =
		CONTAINER_OF(cb, struct ads1x9x_device_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pin);
#if 1
#if defined(CONFIG_ADS1X9X_TRIGGER_OWN_THREAD)
	k_sem_give(&ads1x9x->sem);
#elif defined(CONFIG_ADS1X9X_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&ads1x9x->work);
#endif
#endif
//ads1x9x_handle_interrupts(ads1x9x->dev);
}

int ads1x9x_trigger_set(struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	return -ENOTSUP;
}

int ads1x9x_trigger_mode_init(struct device *dev)
{
	struct ads1x9x_device_data *ads1x9x = dev->driver_data;

	const struct ads1x9x_device_config *cfg = dev->config->config_info;
	printk("%s\r\n",__func__);
#if defined(CONFIG_ADS1X9X_TRIGGER_OWN_THREAD)
	k_sem_init(&ads1x9x->sem, 0, UINT_MAX);

	k_thread_create(&ads1x9x_thread, ads1x9x_thread_stack,
			CONFIG_ADS1X9X_THREAD_STACK_SIZE,
			ads1x9x_thread_main, dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_ADS1X9X_THREAD_PRIORITY), 0, 0);
#elif defined(CONFIG_ADS1X9X_TRIGGER_GLOBAL_THREAD)
	ads1x9x->work.handler = ads1x9x_work_handler;
	ads1x9x->dev = dev;
#endif

	gpio_pin_configure(ads1x9x->gpio, cfg->int_pin,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE);

	gpio_init_callback(&ads1x9x->gpio_cb,
			   ads1x9x_gpio_callback,
			   BIT(cfg->int_pin));

	gpio_add_callback(ads1x9x->gpio, &ads1x9x->gpio_cb);
	gpio_pin_enable_callback(ads1x9x->gpio, cfg->int_pin);

	return 0;
}
