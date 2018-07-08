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
//! \file   ADS1x9x.c
//!
//! \brief  Please see ADS1x9x.h
//
//  Group:          MSP430
//  Target Device:  MSP430FR5989
//
//  (C) Copyright 2015, Texas Instruments, Inc.
//#############################################################################
// $TI Release: PACKAGE NAME $
// $Release Date: PACKAGE RELEASE DATE $
//#############################################################################
#include "ADS1x9x.h"
#include "driverlib.h"
#include "ADS1x9x_USB_Communication.h"
#include "eUSCIA1_SPI.h"
#include "StateMachine.h"
#include "Executive.h"
/**************************************************************************************************************************************************
*         Prototypes                                                                                          *
**************************************************************************************************************************************************/

/**************************************************************************************************************************************************
*         Global Variables                                                                              *
**************************************************************************************************************************************************/
uint8_t ADC_Read_data[16];
uint8_t ADS129x_SPI_cmd_Flag=0, ADS129x_SPI_data_Flag=0,  SPI_Send_count=0, SPI_Tx_Count = 0,SPI_Tx_buf[10];
#pragma NOINIT(SPI_Rx_buf)
uint8_t SPI_Rx_Data_Flag = 0,  SPI_Rx_buf[12], SPI_Rx_Count=0, SPI_Rx_exp_Count=0 ;
uint8_t ECG_Data_rdy;
long ADS1x9x_ECG_Data_buf[6];

extern struct ADS1x9x_state ECG_Recoder_state;
//extern uint8_t ECGRecorder_data_Buf[256], Recorder_head,Recorder_tail;
extern uint8_t ECGRecorder_data_Buf[80], Recorder_head,Recorder_tail;
//uint8_t ECGRecorder_data_Buf[256], Recorder_head,Recorder_tail;
//extern struct ECGPage ECGPageBuf2_data;
//extern uint8_t ECGRecorder_ACQdata_Buf[128];
extern uint8_t Store_data_rdy;
//uint8_t bankFlag = 0;
//uint8_t ECG_recorderReadyFlag = 0;
//unsigned short sampleCount = 0;

#define DELAY_COUNT 2

/* ADS1x9x Register values*/

extern uint8_t ADS1x9xRegVal[16] = {

  //Device ID read Ony
  0x00,
    //CONFIG1
   0x02,
    //CONFIG2
     0xE0,
    //LOFF
     0xF0,
   //CH1SET (PGA gain = 6)
     0x00,
   //CH2SET (PGA gain = 6)
     0x00,
   //RLD_SENS (default)
   0x2C,
   //LOFF_SENS (default)
   0x0F,
    //LOFF_STAT
     0x00,
    //RESP1
     0xEA,
  //RESP2
   0x03,
  //GPIO
     0x0C
};
extern uint8_t ADS1x9xR_Default_Register_Settings[15] = {

  //Device ID read Ony
  0x00,
    //CONFIG1
   0x02,
    //CONFIG2
     0xE0,
    //LOFF
     0xF0,
   //CH1SET (PGA gain = 6)
     0x00,
   //CH2SET (PGA gain = 6)
     0x00,
   //RLD_SENS (default)
   0x2C,
   //LOFF_SENS (default)
   0x0F,
    //LOFF_STAT
     0x00,
    //RESP1
     0xEA,
  //RESP2
   0x03,
  //GPIO
     0x0C
};
extern uint8_t ADS1x9x_Default_Register_Settings[15] = {

  //Device ID read Ony
  0x00,
    //CONFIG1
   0x02,
    //CONFIG2
     0xE0,
    //LOFF
     0xF0,
   //CH1SET (PGA gain = 6)
     0x00,
   //CH2SET (PGA gain = 6)
     0x00,
   //RLD_SENS (default)
   0x2C,
   //LOFF_SENS (default)
   0x0F,
    //LOFF_STAT
     0x00,
    //RESP1
     0x02,
  //RESP2
   0x03,
  //GPIO
     0x0C
};


void ADS1x9x_Clock_Select(uint8_t clock_in)
{

  if (clock_in == 1)
    {
      P2OUT |= (enum PORT2_ADC_CONTROL)ADC_CLK_SEL; // Choose internal clock input
    }
    else
    {
      P2OUT &= ~(enum PORT2_ADC_CONTROL)ADC_CLK_SEL;  // Choose external clock input
    }

}

void ADS1x9x_Reset(void)
{
  unsigned short i;
  P2OUT |= (enum PORT2_ADC_CONTROL)ADC_RESET;   // Set High
  /* Provide suficient dealy*/
  for(i= 0; i < 5000; i++);           // Wait 1 mSec
  P2OUT &= ~(enum PORT2_ADC_CONTROL)ADC_RESET;  // Set to low
  for(i= 0; i < 5000; i++);             // Wait 1 mSec
  P2OUT |= (enum PORT2_ADC_CONTROL)ADC_RESET;   // Set High
  for(i= 0; i < 35000; i++);
}

void ADS1x9x_Disable_Start(void)
{
  unsigned short i;
    P2OUT &= ~(enum PORT2_ADC_CONTROL)ADC_START;  // Set to LOW
    for(i=0; i<35000; i++);                 // Small Delay to settle
}

void ADS1x9x_Enable_Start(void)
{
  unsigned short i;
    P2OUT |= (enum PORT2_ADC_CONTROL)ADC_START;   // Set to High
    for(i=0; i<50000; i++);                 // Small Delay to settle
}

void Set_ADS1x9x_Chip_Enable (void)
{
  /* ADS1x9x CS is Active low*/
  P3OUT &= ~(enum PORT3_ADC_CONTROL)SPI_CS;   // Set to LOW
}

void Clear_ADS1x9x_Chip_Enable (void)
{
    uint8_t CsDelay;

    for ( CsDelay = 0;  CsDelay < 100 ;CsDelay++);
  /* ADS1x9x CS is Active low*/
  P3OUT |= (enum PORT3_ADC_CONTROL)SPI_CS;    // Set to High
}

void Init_ADS1x9x_DRDY_Interrupt (void)
{

    P3DIR &= ~(enum PORT3_ADC_CONTROL)ADC_DRDY;
    P3REN |= (enum PORT3_ADC_CONTROL)ADC_DRDY;                              // Enable P1.1 internal resistance
    P3OUT |= (enum PORT3_ADC_CONTROL)ADC_DRDY;                              // Set P1.1 as pull-Up resistance
    P3IES |= (enum PORT3_ADC_CONTROL)ADC_DRDY;                            // P1.1 Lo/Hi edge
    P3IFG &= ~(enum PORT3_ADC_CONTROL)ADC_DRDY;                             // P1.1 IFG cleared
    P3IE &= ~(enum PORT3_ADC_CONTROL)ADC_DRDY;                              // P1.1 interrupt disabled

}

void Enable_ADS1x9x_DRDY_Interrupt (void)
{
    P3IFG &= ~(enum PORT3_ADC_CONTROL)ADC_DRDY;                             // P1.1 IFG cleared
    P3IE |= (enum PORT3_ADC_CONTROL)ADC_DRDY;                               // P1.1 interrupt enabled
}

void Disable_ADS1x9x_DRDY_Interrupt (void)
{
    P3IFG &= ~(enum PORT3_ADC_CONTROL)ADC_DRDY;                             // P1.1 IFG cleared
    P3IE &= ~(enum PORT3_ADC_CONTROL)ADC_DRDY;                              // P1.1 interrupt disabled
}

void Set_DMA_SPI(void)
{
  DMACTL0 = DMA0TSEL_12;                      // USCI_B0 Transmit Ready Trigger
  //DMA0SA = (void (*)())&UCB0RXBUF;            // Source block address
  //DMA0DA = (void (*)())ADC_Read_data;         // Destination single address
  DMA0SZ = 16;                                // Block size
  DMA0CTL = DMADT_4 + DMADSTINCR_3 + DMADSTBYTE + DMASRCBYTE;
  /* DMADT_4 - DMA transfer mode 4: Repeated Single transfer */
  /* DMADSTINCR_3 - 11b = Destination address is incremented. */
  /* DMADSTBYTE - DMA destination byte. This bit selects the destination as a byte or */
  /* DMASRCBYTE - DMA source byte. This bit selects the source as a byte */

                                              // Rpt, inc src, byte-byte
  DMA0CTL |= DMAEN;                           // Enable DMA for consecutive Xfers


}

void ADS1x9x_SPI_Command_Data(uint8_t Data)
{
  uint8_t delayVar;
  Set_ADS1x9x_Chip_Enable();
  for (delayVar = 0; delayVar < 50; delayVar++);
  Clear_ADS1x9x_Chip_Enable();
  Set_ADS1x9x_Chip_Enable();

  UCA1TXBUF = Data;                                     // Send the data sitting at the pointer DATA to the TX Buffer
  while ( (UCA1STATW & UCBUSY) );

  delayVar = UCA1RXBUF;

  for (delayVar = 0; delayVar < 150; delayVar++);

}

void Wake_Up_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (WAKEUP);                   // Send 0x02 to the ADS1x9x
}

void Put_ADS1x9x_In_Sleep (void)
{
    ADS1x9x_SPI_Command_Data (STANDBY);                 // Send 0x04 to the ADS1x9x
}

void Soft_Reset_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (RESET);                   // Send 0x06 to the ADS1x9x
}


/**********************************************************************************************************
* ADS1x9x_PowerDown_Enable                                                                  *
**********************************************************************************************************/
void ADS1x9x_PowerDown_Enable(void)
{
  unsigned short i, j;
  P2OUT &= ~(enum PORT2_ADC_CONTROL)ADC_RESET;  // Set to low
    for (j = 0; j < DELAY_COUNT; j++)
    {
      for ( i=0; i < 35000; i++);
    }
}

void ADS1x9x_PowerDown_Disable(void)
{
  unsigned short i, j;
  P2OUT |= (enum PORT2_ADC_CONTROL)ADC_RESET;   // Set High
    for (j = 0; j < DELAY_COUNT; j++)
    {
      for ( i=0; i < 35000; i++);
    }
}

void Soft_Start_ReStart_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (START_);                  // Send 0x08 to the ADS1x9x
    Clear_ADS1x9x_Chip_Enable ();
}

void Hard_Start_ReStart_ADS1x9x(void)
{
  P2OUT |= (enum PORT2_ADC_CONTROL)ADC_START;     // Set Start pin to High
}

void Soft_Start_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (START_);                   // Send 0x0A to the ADS1x9x
}

void Soft_Stop_ADS1x9x (void)
{
    ADS1x9x_SPI_Command_Data (STOP);                   // Send 0x0A to the ADS1x9x
}

void Hard_Stop_ADS1x9x (void)
{
    unsigned short i, j;
    P2OUT &= ~(enum PORT2_ADC_CONTROL)ADC_START;    // Set Start pin to Low
    for (j = 0; j < DELAY_COUNT; j++)
    {
      for ( i=0; i < 35000; i++);
    }
}

void Stop_Read_Data_Continuous (void)
{
    ADS1x9x_SPI_Command_Data(SDATAC);         // Send 0x11 to the ADS1x9x
}

void Start_Read_Data_Continuous (void)
{
    ADS1x9x_SPI_Command_Data (RDATAC);          // Send 0x10 to the ADS1x9x
}

void Start_Data_Conv_Command (void)
{
    ADS1x9x_SPI_Command_Data (START_);          // Send 0x08 to the ADS1x9x
}

void Init_ADS1x9x (void)
{
  ADS1x9x_Reset();
  ADS1x9x_Disable_Start();
  ADS1x9x_Enable_Start();
}

void enable_ADS1x9x_Conversion (void)
{
    Start_Read_Data_Continuous ();    //RDATAC command

    Hard_Start_ReStart_ADS1x9x();

}

void ADS1x9x_Reg_Write (uint8_t READ_WRITE_ADDRESS, uint8_t DATA)
  {
    short i;
    switch (READ_WRITE_ADDRESS)
    {
      case 1:
        DATA = DATA & 0x87;
      break;
      case 2:
        DATA = DATA & 0xFB;
        DATA |= 0x80;

      break;
      case 3:
        DATA = DATA & 0xFD;
        DATA |= 0x10;

      break;
      case 7:
        DATA = DATA & 0x3F;
      break;
      case 8:
        DATA = DATA & 0x5F;
      break;
      case 9:
        DATA |= 0x02;
      break;
      case 10:
        DATA = DATA & 0x87;
        DATA |= 0x01;
      break;
      case 11:
        DATA = DATA & 0x0F;
      break;

      default:

      break;

    }
  SPI_Tx_buf[0] = READ_WRITE_ADDRESS | WREG;
  SPI_Tx_buf[1] = 0;            // Write Single byte
  SPI_Tx_buf[2] = DATA;         // Write Single byte
  Set_ADS1x9x_Chip_Enable();

  for ( i =0; i < 50;i++);

  UCA1TXBUF = SPI_Tx_buf[0];              // Send the first data to the TX Buffer
  while ( (UCA1STATW & UCBUSY) );     // USCI_B0 TX buffer ready?
  i = UCA1RXBUF;              // Read Rx buf

  UCA1TXBUF = SPI_Tx_buf[1];              // Send the first data to the TX Buffer
  while ( (UCA1STATW & UCBUSY) );     // USCI_B0 TX buffer ready?
  i = UCA1RXBUF;
  UCA1TXBUF = SPI_Tx_buf[2];              // Send the first data to the TX Buffer
  while ( (UCA1STATW & UCBUSY) );     // USCI_B0 TX buffer ready?
  i = UCA1RXBUF;

}

uint8_t ADS1x9x_Reg_Read(uint8_t Reg_address)
{

  uint8_t retVal;

  SPI_Tx_buf[0] = Reg_address | RREG;
  SPI_Tx_buf[1] = 0;              // Read number of bytes - 1

  Set_ADS1x9x_Chip_Enable();          // Set chip select to low

  UCA1TXBUF = SPI_Tx_buf[0];                  // Send the first data to the TX Buffer
  while ( (UCA1STATW & UCBUSY) );       // USCI_B0 TX buffer ready?
  UCA1TXBUF = SPI_Tx_buf[1];                  // Send the first data to the TX Buffer
  while ( (UCA1STATW & UCBUSY) );       // USCI_B0 TX buffer ready?
  retVal = UCA1RXBUF;             // Read RX buff
  UCA1TXBUF = 0x00;                           // Send the first data to the TX Buffer
  while ( (UCA1STATW & UCBUSY) );       // USCI_B0 TX buffer ready?
  retVal = UCA1RXBUF;             // Read RX buff

  Clear_ADS1x9x_Chip_Enable();        // Disable chip select
  return  retVal;

}

void ADS1x9x_Default_Reg_Init(void)
{

  uint8_t Reg_Init_i;
  Set_ADS1x9x_Chip_Enable();
  for ( Reg_Init_i =0; Reg_Init_i <100;Reg_Init_i++);
  Clear_ADS1x9x_Chip_Enable();

  if ((ADS1x9xRegVal[0] & 0X20) == 0x20)
  {
    for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
    {
      ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9xR_Default_Register_Settings[Reg_Init_i]);
    }
  }
  else
  {
    for ( Reg_Init_i = 1; Reg_Init_i < 12; Reg_Init_i++)
    {
      ADS1x9x_Reg_Write(Reg_Init_i,ADS1x9x_Default_Register_Settings[Reg_Init_i]);
    }
  }

}

void ADS1x9x_Read_All_Regs(uint8_t ADS1x9xeg_buf[])
{
  uint8_t Regs_i;
  Set_ADS1x9x_Chip_Enable();
  for ( Regs_i =0; Regs_i <200;Regs_i++);
  Clear_ADS1x9x_Chip_Enable();

  for ( Regs_i = 0; Regs_i < 12; Regs_i++)
  {
    ADS1x9xeg_buf[Regs_i] = ADS1x9x_Reg_Read(Regs_i);

  }

}

void ADS1x9x_PowerOn_Init(void)
{
   volatile unsigned short Init_i, j;
   //Init_ADS1x9x_Resource();
   ADS1x9x_Reset();
  for (j = 0; j < DELAY_COUNT; j++)
    {
        for ( Init_i =0; Init_i < 20000; Init_i++);
      for ( Init_i =0; Init_i < 20000; Init_i++);
      for ( Init_i =0; Init_i < 20000; Init_i++);
    }
   Init_ADS1x9x_DRDY_Interrupt();
   ADS1x9x_Clock_Select(1);   // Set internal clock
   for ( Init_i =0; Init_i < 20000; Init_i++);
   for ( Init_i =0; Init_i < 20000; Init_i++);
   for ( Init_i =0; Init_i < 20000; Init_i++);

   //The START pin must be set high to begin conversions.
   ADS1x9x_Disable_Start();// Set to LOW
   ADS1x9x_Enable_Start();// Set to High

   Hard_Stop_ADS1x9x();

   Start_Data_Conv_Command();

   Soft_Stop_ADS1x9x();

   for (j = 0; j < DELAY_COUNT; j++)
    {
      for ( Init_i =0; Init_i < 20000; Init_i++);
    }
   Stop_Read_Data_Continuous();         // SDATAC command
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

   ADS1x9x_PowerDown_Enable();

}

void ADS1191_Parse_data_packet(void)
{
  uint8_t ECG_Chan_num;

  switch (ECG_Recoder_state.state)
  {

       case IDLE_STATE:
       break;
       case DATA_STREAMING_STATE:
       {
          for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
          {
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num];   // Get MSB 8 bits
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];       // Get LSB 8 bits
          }
          ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;               // to make compatable with 24 bit devices
       }
       break;


       case ACQUIRE_DATA_STATE:

       case ECG_RECORDING_STATE:
    {
        uint8_t *ptr;
      ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
      *ptr++ = SPI_Rx_buf[0];       // Store status
      *ptr++ = SPI_Rx_buf[1];       // Store status
//      if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
//      *ptr++ = 0xFF;            // CH0[23-16] = 0xFF ( 16Bit device) sign
//      else
//      *ptr++ = 0;             // CH0[23-16] = 0 ( 16Bit device)
      *ptr++ = SPI_Rx_buf[2];       // CH0[15-8] = MSB ( 16Bit device)
      *ptr++ = SPI_Rx_buf[3];       // CH0[7-0] = LSB ( 16Bit device)
      *ptr++ = 0;
//      if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
//      *ptr++ = 0xFF;            // CH0[23-16] = 0xFF ( 16Bit device) sign
//      else
//      *ptr++ = 0;             // CH0[23-16] = 0 ( 16Bit device)
      *ptr++ = SPI_Rx_buf[2];       // CH0[15-8] = MSB ( 16Bit device)
      *ptr++ = SPI_Rx_buf[3];       // CH0[7-0] = LSB ( 16Bit device)
      *ptr++ = 0;
      Recorder_head ++;         // Increment Circuler buffer pointer

      if (Recorder_head == 32)      // Check for circuler buffer depth.
        Recorder_head = 0;        // Rest once it reach to MAX
    }
            break;

       default:
            break;

  }

}

void ADS1192_Parse_data_packet(void)
{
  uint8_t ECG_Chan_num;

  switch (ECG_Recoder_state.state)
  {

       case IDLE_STATE:
       break;
       case DATA_STREAMING_STATE:
       {
          for (ECG_Chan_num = 0; ECG_Chan_num < 3; ECG_Chan_num++)
          {
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[2*ECG_Chan_num]; // Get MSB Bits15-bits8
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[2*ECG_Chan_num+1];       // Get LSB Bits7-bits0
          }
          ADS1x9x_ECG_Data_buf[0] = ADS1x9x_ECG_Data_buf[0] << 8;       // to make compatable with 24 bit devices
       }
       break;


       case ACQUIRE_DATA_STATE:

       case ECG_RECORDING_STATE:
    {
        uint8_t *ptr;

      ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
      *ptr++ = SPI_Rx_buf[0];       // Store status
      *ptr++ = SPI_Rx_buf[1];       // Store status

//      if ((SPI_Rx_buf[2] & 0x80 ) == 0x80)// CH0[15-8] = MSB ( 16Bit device)
//      *ptr++ = 0xFF;            // CH0[23-16] = 0xFF ( 16Bit device) sign
//      else
//      *ptr++ = 0;             // CH0[23-16] = 0 ( 16Bit device)
      *ptr++ = SPI_Rx_buf[2];       // CH0[15-8] = MSB ( 16Bit device)
      *ptr++ = SPI_Rx_buf[3];       // CH0[7-0] = LSB ( 16Bit device)
      *ptr++ = 0;

//      if ((SPI_Rx_buf[4] & 0x80 ) == 0x80)// CH1[15-8] = MSB ( 16Bit device)
//      *ptr++ = 0xFF;            // CH1[23-16] = 0xFF ( 16Bit device) sign
//      else
//      *ptr++ = 0;             // CH1[23-16] = 0 ( 16Bit device)
      *ptr++ = SPI_Rx_buf[4];       // CH1[15-8] = MSB ( 16Bit device)
      *ptr++ = SPI_Rx_buf[5];       // CH1[7-0] = LSB ( 16Bit device)
      *ptr++ = 0;
      Recorder_head ++;         // Increment Circuler buffer pointer

      if (Recorder_head == 32)      // Check for circuler buffer depth.
        Recorder_head = 0;        // Rest once it reach to MAX
    }
            break;

       default:
            break;

  }

}

void ADS1291_Parse_data_packet(void)
{
  uint8_t ECG_Chan_num;

  switch (ECG_Recoder_state.state)
  {
       case DATA_STREAMING_STATE:
       {
          for (ECG_Chan_num = 0; ECG_Chan_num < 2; ECG_Chan_num++)
          {
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] = (signed long)SPI_Rx_buf[3*ECG_Chan_num]; // Get Bits23-bits16

            ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+1];       // Get Bits15-bits8

            ADS1x9x_ECG_Data_buf[ECG_Chan_num] = ADS1x9x_ECG_Data_buf[ECG_Chan_num] << 8;
            ADS1x9x_ECG_Data_buf[ECG_Chan_num] |= SPI_Rx_buf[3*ECG_Chan_num+2];       // Get Bits7-bits0
          }
       }
       break;


       case ACQUIRE_DATA_STATE:

       case ECG_RECORDING_STATE:
      {
        uint8_t *ptr;

      ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;

      *ptr++ = SPI_Rx_buf[0];       // Store status
      *ptr++ = SPI_Rx_buf[1];       // Store status
      //SPI_Rx_buf[2] is always 0x00 so it is discarded

      *ptr++ = SPI_Rx_buf[3];       // CH0[23-16] = MSB ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[4];       // CH0[15-8] = MID ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[5];       // CH0[7-0] = LSB ( 24 Bit device)

      *ptr++ = SPI_Rx_buf[3];       // CH1[23-16] = Ch0 to mentain uniformality
      *ptr++ = SPI_Rx_buf[4];       // CH1[15-8] =  Ch0 to mentain uniformality
      *ptr++ = SPI_Rx_buf[5];       // CH1[7-0] =  Ch0 to mentain uniformality

      Recorder_head++;          // Increment Circuler buffer pointer
      if (Recorder_head == 32)      // Check for circuler buffer depth.
        Recorder_head = 0;        // Rest once it reach to MAX
      }

        break;

       default:
            break;

  }

}

void ADS1292x_Parse_data_packet(void)
{
  uint8_t ECG_Chan_num;
  switch (ECG_Recoder_state.state)
  {
       case DATA_STREAMING_STATE:
       {
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

       }
       break;


     case ACQUIRE_DATA_STATE:

       case ECG_RECORDING_STATE:
      {
        uint8_t *ptr;

      ptr = &ECGRecorder_data_Buf[Recorder_head << 3]; // Point to Circular buffer at head*8;
      *ptr++ = SPI_Rx_buf[0];       // Store status
      *ptr++ = SPI_Rx_buf[1];       // Store status
      //SPI_Rx_buf[2] is always 0x00 so it is discarded

      *ptr++ = SPI_Rx_buf[3];       // CH0[23-16] = MSB ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[4];       // CH0[15-8] = MID ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[5];       // CH0[7-0] = LSB ( 24 Bit device)

      *ptr++ = SPI_Rx_buf[6];       // CH1[23-16] = MSB ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[7];       // CH1[15-8] = MID ( 24 Bit device)
      *ptr++ = SPI_Rx_buf[8];       // CH1[7-0] = LSB ( 24 Bit device)

      Recorder_head ++;         // Increment Circuler buffer pointer
      if (Recorder_head == 32)      // Check for circuler buffer depth.
        Recorder_head = 0;        // Rest once it reach to MAX

      }

        break;

       default:
       break;
  }
}

void ADS1x9x_Parse_data_packet(void)
{

  switch( ADS1x9xRegVal[0] & 0x03)
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
  ECG_Data_rdy = 1;

  //Stream_ECG_data_packets();
  //SPI_Rx_exp_Count = 1;

}


#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  switch(__even_in_range(UCA1IV,4))
  {
    case 0:break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG

      //while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?

        SPI_Rx_buf[SPI_Rx_Count] = UCA1RXBUF;
        SPI_Rx_Count++;
        if ( SPI_Rx_Count == SPI_Rx_exp_Count)
        {
      UCA1IE &= ~UCRXIE;                 // Disable USCI_A0 RX interrupt
      ADS1x9x_Parse_data_packet();

      if(ECG_Recoder_state.state == DATA_STREAMING_STATE)
      PUBLISH_EVENT(EV_ECG_DATA_READY);
      else if(ECG_Recoder_state.state == ACQUIRE_DATA_STATE)
      PUBLISH_EVENT(EV_ECG_DATA_READY);

      __bic_SR_register_on_exit(LPM3_bits + GIE); //wake up to handle INTO
        }
        else
        {
      UCA1TXBUF = 0;          // To get Next byte.
        }
      break;
    case 4:break;                             // Vector 4 - TXIFG

    default: break;
  }
}

#pragma vector=PORT3_VECTOR
__interrupt void Port_3(void)
{
  if ( P3IFG &= BIT2)
  {
    P3IFG &= ~BIT2;                 // Clear P3.2 IFG i.e Data RDY interrupt status
    SPI_Rx_Count = UCA1RXBUF;     // Dummy Read
    SPI_Rx_Count=0;

    UCA1TXBUF = 0;
    UCA1IE |= UCRXIE;               // Enable USCI_B0 RX interrupt
  }
}

void Set_Device_out_bytes(void)
{
  switch( ADS1x9xRegVal[0] & 0x03)
  {
    case ADS1191_16BIT:
      SPI_Rx_exp_Count=4;   // 2 byte status + 2 bytes CH0 data
    break;

    case ADS1192_16BIT:
      SPI_Rx_exp_Count=6;   // 2 byte status + 2 bytes ch1 data + 2 bytes CH0 data
    break;

    case ADS1291_24BIT:
      SPI_Rx_exp_Count=6;   // 3 byte status + 3 bytes CH0 data
    break;

    case ADS1292_24BIT:
      SPI_Rx_exp_Count=9;   // 3 byte status + 3 bytes ch1 data + 3 bytes CH0 data
    break;
  }
}
// End of file
