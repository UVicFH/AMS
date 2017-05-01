/*!
  ltc6811-1 Multicell Battery Monitor
@verbatim
  The ltc6811 is a 3rd generation multicell battery stack
  monitor that measures up to 12 series connected battery
  cells with a total measurement error of less than 1.2mV. The
  cell measurement range of 0V to 5V makes the ltc6811
  suitable for most battery chemistries. All 12 cell voltages
  can be captured in 290uS, and lower data acquisition rates
  can be selected for high noise reduction.

  Using the ltc6811-1, multiple devices are connected in
  a daisy-chain with one host processor connection for all
  devices.
@endverbatim
REVISION HISTORY
$Revision: 1200 $
$Date: 2016-3-9

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2013 Linear Technology Corp. (LTC)
***********************************************************/
//! @defgroup ltc68111 ltc6811-1: Multicell Battery Monitor

/*! @file
    @ingroup ltc68111
    Library for ltc6811-1 Multicell Battery Monitor
*/

#include <stdint.h>
#include <Arduino.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LTC6811_daisy.h"
//#include <SPI.h>




/*
Starts cell voltage conversion
*/
void ltc6811_adcv(
  uint8_t MD, //ADC Mode
  uint8_t DCP, //Discharge Permit
  uint8_t CH //Cell Channels to be measured
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + 0x60 + (DCP<<4) + CH;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);


  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}


//Starts cell voltage and SOC conversion
void ltc6811_adcvsc(
  uint8_t MD, //ADC Mode
  uint8_t DCP //Discharge Permit
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  //1
  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + 0x60 + (DCP<<4) +0x07;

  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  //4
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

// Starts cell voltage  and GPIO 1&2 conversion
void ltc6811_adcvax(
  uint8_t MD, //ADC Mode
  uint8_t DCP //Discharge Permit
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;


  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((DCP&0x01)<<4) + 0x6F;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

//Starts cell voltage overlap conversion
void ltc6811_adol(
  uint8_t MD, //ADC Mode
  uint8_t DCP //Discharge Permit
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + (DCP<<4) +0x01;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

//Starts cell voltage self test conversion
void ltc6811_cvst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST)<<5) +0x07;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

//Start an Auxiliary Register Self Test Conversion
void ltc6811_axst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST&0x03)<<5) +0x07;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

//Start a Status Register Self Test Conversion
void ltc6811_statst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + ((ST&0x03)<<5) +0x0F;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

//Sends the poll adc command
uint8_t ltc6811_pladc()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t adc_state = 0xFF;

  cmd[0] = 0x07;
  cmd[1] = 0x14;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  adc_state = spi_read(0xFF);

  output_high(LTC6811_CS);
  return(adc_state);
}

//This function will block operation until the ADC has finished it's conversion
uint32_t ltc6811_pollAdc()
{
  uint32_t counter = 0;
  uint8_t finished = 0;
  uint8_t current_time = 0;
  uint8_t cmd[4];
  uint16_t cmd_pec;


  cmd[0] = 0x07;
  cmd[1] = 0x14;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_array(4,cmd);

  while ((counter<200000)&&(finished == 0))
  {
    current_time = spi_read(0xFF);
    if (current_time>0)
    {
      finished = 1;
    }
    else
    {
      counter = counter + 10;
    }
  }

  output_high(LTC6811_CS);


  return(counter);
}

//Start a GPIO and Vref2 Conversion
void ltc6811_adax(
  uint8_t MD, //ADC Mode
  uint8_t CHG //GPIO Channels to be measured)
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x60 + CHG ;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

//Start an GPIO Redundancy test
void ltc6811_adaxd(
  uint8_t MD, //ADC Mode
  uint8_t CHG //GPIO Channels to be measured)
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + CHG ;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

//Start a Status ADC Conversion
void ltc6811_adstat(
  uint8_t MD, //ADC Mode
  uint8_t CHST //GPIO Channels to be measured
)
{


  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x68 + CHST ;


  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

// Start a Status register redundancy test Conversion
void ltc6811_adstatd(
  uint8_t MD, //ADC Mode
  uint8_t CHST //GPIO Channels to be measured
)
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x04;
  md_bits = (MD & 0x01) << 7;
  cmd[1] = md_bits + 0x08 + CHST ;


  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}


// Start an open wire Conversion
void ltc6811_adow(
  uint8_t MD, //ADC Mode
  uint8_t PUP //Discharge Permit
)
{

  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] = md_bits + 0x02;
  md_bits = (MD & 0x01) << 7;
  cmd[1] =  md_bits + 0x28 + (PUP<<6) ;//+ CH;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);
  output_high(LTC6811_CS);

}

// Reads and parses the ltc6811 cell voltage registers.
uint8_t ltc6811_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // the number of ICs in the system
                     uint16_t cell_codes[][CELL_CHANNELS] // Array of the parsed cell codes
                    )
{

  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t CELL_IN_REG = 3;
  const uint8_t NUM_CV_REG = 4;

  uint8_t *cell_data;
  uint8_t pec_error = 0;
  uint16_t parsed_cell;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter=0; //data counter
  cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

  if (reg == 0)
  {

    for (uint8_t cell_reg = 1; cell_reg<NUM_CV_REG+1; cell_reg++)                   //executes once for each of the ltc6811 cell voltage registers
    {
      data_counter = 0;
      ltc6811_rdcv_reg(cell_reg, total_ic,cell_data );                //Reads a single Cell voltage register

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every ltc6811 in the daisy chain
      {
        // current_ic is used as the IC counter

        for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
        {
          // loops once for each of the 3 cell voltage codes in the register

          parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
          // create the parsed cell voltage code

          cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
          data_counter = data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
          //must increment by two for each parsed cell code
        }

        received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 cell voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the serial data
        }
        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs cell voltage data
      }
    }
  }

  else
  {

    ltc6811_rdcv_reg(reg, total_ic,cell_data);
    for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)        // executes for every ltc6811 in the daisy chain
    {
      // current_ic is used as the IC counter

      for (uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
      {
        // loops once for each of the 3 cell voltage codes in the register

        parsed_cell = cell_data[data_counter] + (cell_data[data_counter+1]<<8); //Each cell code is received as two bytes and is combined to
        // create the parsed cell voltage code

        cell_codes[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] = 0x0000FFFF & parsed_cell;
        data_counter= data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
        //must increment by two for each parsed cell code
      }

      received_pec = (cell_data[data_counter] << 8 )+ cell_data[data_counter + 1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 cell voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the serial data
      }
      data_counter= data_counter + 2;                       //Because the transmitted PEC code is 2 bytes long the data_counter
      //must be incremented by 2 bytes to point to the next ICs cell voltage data
    }
  }


  free(cell_data);
  return(pec_error);
}


//Read the raw data from the ltc6811 cell voltage register
void ltc6811_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                      uint8_t total_ic, //the number of ICs in the
                      uint8_t *data //An array of the unparsed cell codes
                     )
{
  const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;

  if (reg == 1)     //1: RDCVA
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if (reg == 2) //2: RDCVB
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  }
  else if (reg == 3) //3: RDCVC
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  }
  else if (reg == 4) //4: RDCVD
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  }
  else if (reg == 5) //4: RDCVE
  {
    cmd[1] = 0x09;
    cmd[0] = 0x00;
  }
  else if (reg == 6) //4: RDCVF
  {
    cmd[1] = 0x0B;
    cmd[0] = 0x00;
  }


  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  output_high(LTC6811_CS);

}

/*
 The function is used
 to read the  parsed GPIO codes of the ltc6811. This function will send the requested
 read commands parse the data and store the gpio voltages in aux_codes variable
*/
int8_t ltc6811_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
                     uint8_t total_ic,//the number of ICs in the system
                     uint16_t aux_codes[][AUX_CHANNELS]//A two dimensional array of the gpio voltage codes.
                    )
{


  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t GPIO_IN_REG = 3;
  const uint8_t NUM_GPIO_REG = 2;
  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t parsed_aux;
  uint16_t received_pec;
  uint16_t data_pec;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

  if (reg == 0)
  {

    for (uint8_t gpio_reg = 1; gpio_reg<NUM_GPIO_REG+1; gpio_reg++)                 //executes once for each of the ltc6811 aux voltage registers
    {
      data_counter = 0;
      ltc6811_rdaux_reg(gpio_reg, total_ic,data);                 //Reads the raw auxiliary register data into the data[] array

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every ltc6811 in the daisy chain
      {
        // current_ic is used as the IC counter


        for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
        {
          // loops once for each of the 3 gpio voltage codes in the register

          parsed_aux = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
          // create the parsed gpio voltage code

          aux_codes[current_ic][current_gpio +((gpio_reg-1)*GPIO_IN_REG)] = parsed_aux;
          data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter
          //must increment by two for each parsed gpio voltage code

        }

        received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 gpio voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the received serial data
        }

        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs gpio voltage data
      }


    }

  }
  else
  {

    ltc6811_rdaux_reg(reg, total_ic, data);
    for (int current_ic = 0 ; current_ic < total_ic; current_ic++)            // executes for every ltc6811 in the daisy chain
    {
      // current_ic is used as an IC counter


      for (int current_gpio = 0; current_gpio<GPIO_IN_REG; current_gpio++)    // This loop parses the read back data. Loops
      {
        // once for each aux voltage in the register

        parsed_aux = (data[data_counter] + (data[data_counter+1]<<8));        //Each gpio codes is received as two bytes and is combined to
        // create the parsed gpio voltage code
        aux_codes[current_ic][current_gpio +((reg-1)*GPIO_IN_REG)] = parsed_aux;
        data_counter=data_counter+2;                      //Because gpio voltage codes are two bytes the data counter
        //must increment by two for each parsed gpio voltage code
      }

      received_pec = (data[data_counter]<<8) + data[data_counter+1];         //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 gpio voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                               //The pec_error variable is simply set negative if any PEC errors
        //are detected in the received serial data
      }

      data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
      //must be incremented by 2 bytes to point to the next ICs gpio voltage data
    }
  }
  free(data);
  return (pec_error);
}


/*
 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the ltc6811_rdaux() command.
 */
void ltc6811_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
                       uint8_t total_ic, //The number of ICs in the system
                       uint8_t *data //Array of the unparsed auxiliary codes
                      )
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back auxiliary group B
  {
    cmd[1] = 0x0e;
    cmd[0] = 0x00;
  }
  else if (reg == 3)  //Read back auxiliary group B
  {
    cmd[1] = 0x0D;
    cmd[0] = 0x00;
  }
  else if (reg == 4)  //Read back auxiliary group B
  {
    cmd[1] = 0x0F;
    cmd[0] = 0x00;
  }
  else          //Read back auxiliary group A
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);


  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake, this command can be removed.

  output_low(LTC6811_CS);
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  output_high(LTC6811_CS);

}


/*
 Reads and parses the ltc6811 stat registers.

 The function is used
 to read the  parsed stat codes of the ltc6811. This function will send the requested
 read commands parse the data and store the stat voltages in stat_codes variable
*/
int8_t ltc6811_rdstat(uint8_t reg, //Determines which Stat  register is read back.
                      uint8_t total_ic,//the number of ICs in the system
                      uint16_t stat_codes[][4],//A two dimensional array of the stat voltage codes.
                      uint8_t flags[][3],
                      uint8_t mux_fail[][1],
                      uint8_t thsd[][1]
                     )
{


  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t GPIO_IN_REG = 3;

  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t parsed_stat;
  uint16_t received_pec;
  uint16_t data_pec;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

  if (reg == 0)
  {

    for (uint8_t gpio_reg = 1; gpio_reg<3; gpio_reg++)                //executes once for each of the ltc6811 stat voltage registers
    {
      data_counter = 0;
      ltc6811_rdstat_reg(gpio_reg, total_ic,data);                  //Reads the raw statiliary register data into the data[] array

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every ltc6811 in the daisy chain
      {
        // current_ic is used as the IC counter


        if (gpio_reg ==1)
        {
          for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
          {
            // loops once for each of the 3 gpio voltage codes in the register

            parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
            stat_codes[current_ic][current_gpio] = parsed_stat;
            data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter

          }
        }
        else if (gpio_reg == 2)
        {
          parsed_stat = data[data_counter++] + (data[data_counter++]<<8);              //Each gpio codes is received as two bytes and is combined to
          stat_codes[current_ic][3] = parsed_stat;
          flags[current_ic][0] = data[data_counter++];
          flags[current_ic][1] = data[data_counter++];
          flags[current_ic][2] = data[data_counter++];
          mux_fail[current_ic][0] = (data[data_counter] & 0x02)>>1;
          thsd[current_ic][0] = data[data_counter++] & 0x01;
        }


        received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 gpio voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);

        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the received serial data
        }

        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs gpio voltage data
      }


    }

  }
  else
  {

    ltc6811_rdstat_reg(reg, total_ic, data);
    for (int current_ic = 0 ; current_ic < total_ic; current_ic++)            // executes for every ltc6811 in the daisy chain
    {
      // current_ic is used as an IC counter

      if (reg ==1)
      {
        for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
        {
          // loops once for each of the 3 gpio voltage codes in the register

          parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
          // create the parsed gpio voltage code

          stat_codes[current_ic][current_gpio] = parsed_stat;
          data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter
          //must increment by two for each parsed gpio voltage code

        }
      }
      else if (reg == 2)
      {
        parsed_stat = data[data_counter++] + (data[data_counter++]<<8);              //Each gpio codes is received as two bytes and is combined to
        stat_codes[current_ic][3] = parsed_stat;
        flags[current_ic][0] = data[data_counter++];
        flags[current_ic][1] = data[data_counter++];
        flags[current_ic][2] = data[data_counter++];
        mux_fail[current_ic][0] = (data[data_counter] & 0x02)>>1;
        thsd[current_ic][0] = data[data_counter] & 0x01;
      }


      received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 gpio voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the received serial data
      }

      data_counter=data_counter+2;
    }
  }
  free(data);
  return (pec_error);
}



/*
 The function reads a single stat  register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the ltc6811_rdstat() command.
*/
void ltc6811_rdstat_reg(uint8_t reg, //Determines which stat register is read back
                        uint8_t total_ic, //The number of ICs in the system
                        uint8_t *data //Array of the unparsed stat codes
                       )
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //Read back statiliary group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back statiliary group B
  {
    cmd[1] = 0x12;
    cmd[0] = 0x00;
  }

  else          //Read back statiliary group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);


  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake, this command can be removed.
  output_low(LTC6811_CS);
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  output_high(LTC6811_CS);

}


/*
 The command clears the cell voltage registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
*/
void ltc6811_clrcell()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x11;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec );

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6811_CS);
}


/*
 The command clears the Auxiliary registers and initializes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
*/
void ltc6811_clraux()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x12;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.
  output_low(LTC6811_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6811_CS);
}


/*
 The command clears the Stat registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.

*/
void ltc6811_clrstat()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x13;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.
  output_low(LTC6811_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6811_CS);
}


/*
 The command clears the Sctrl registers and initializes
 all values to 0. The register will read back hexadecimal 0x00
 after the command is sent.
 */
void ltc6811_clrsctrl()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x00;
  cmd[1] = 0x18;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec );

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6811_CS);
}

//Starts the Mux Decoder diagnostic self test
void ltc6811_diagn()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x15;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec );

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6811_CS);
}


/*
 This command will write the configuration registers of the ltc6811-1s
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.
*/
void ltc6811_wrcfg(uint8_t total_ic, //The number of ICs being written to
                   uint8_t config[][6] //A two dimensional array of the configuration data that will be written
                  )
{
  const uint8_t BYTES_IN_REG = 6;
  const uint8_t CMD_LEN = 4+(8*total_ic);
  uint8_t *cmd;
  uint16_t cfg_pec;
  uint8_t cmd_index; //command counter

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));


  cmd[0] = 0x00;
  cmd[1] = 0x01;
  cmd[2] = 0x3d;
  cmd[3] = 0x6e;


  cmd_index = 4;
  for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each ltc6811 in daisy chain, this loops starts with
  {
    // the last IC on the stack. The first configuration written is
    // received by the last IC in the daisy chain

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
    {
      // current_byte is the byte counter

      cmd[cmd_index] = config[current_ic-1][current_byte];            //adding the config data to the array to be sent
      cmd_index = cmd_index + 1;
    }

    cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);   // calculating the PEC for each ICs configuration register data
    cmd[cmd_index] = (uint8_t)(cfg_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)cfg_pec;
    cmd_index = cmd_index + 2;
  }


  wakeup_idle ();                                 //This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.

  output_low(LTC6811_CS);
  spi_write_array(CMD_LEN, cmd);
  output_high(LTC6811_CS);
  free(cmd);
}


/*
Reads configuration registers of a ltc6811 daisy chain
*/
int8_t ltc6811_rdcfg(uint8_t total_ic, //Number of ICs in the system
                     uint8_t r_config[][8] //A two dimensional array that the function stores the read configuration data.
                    )
{
  const uint8_t BYTES_IN_REG = 8;

  uint8_t cmd[4];
  uint8_t *rx_data;
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t received_pec;

  rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));

  cmd[0] = 0x00;
  cmd[1] = 0x02;
  cmd[2] = 0x2b;
  cmd[3] = 0x0A;

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

  output_low(LTC6811_CS);
  spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into
  output_high(LTC6811_CS);                          //rx_data[] array

  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each ltc6811 in the daisy chain and packs the data
  {
    //into the r_config array as well as check the received Config data
    //for any bit errors

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      r_config[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
    }

    received_pec = (r_config[current_ic][6]<<8) + r_config[current_ic][7];
    data_pec = pec15_calc(6, &r_config[current_ic][0]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }
  }

  free(rx_data);
  return(pec_error);
}


/*
Writes the pwm registers of a ltc6811 daisy chain
*/
void ltc6811_wrpwm(uint8_t total_ic,
                   uint8_t pwmReg,  //The number of ICs being written to
                   uint8_t pwm[][6] //A two dimensional array of the configuration data that will be written
                  )
{
  const uint8_t BYTES_IN_REG = 6;
  const uint8_t CMD_LEN = 4+(8*total_ic);
  uint8_t *cmd;
  uint16_t pwm_pec;
  uint16_t cmd_pec;
  uint8_t cmd_index; //command counter

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

  if (pwmReg == 0)
  {
    cmd[0] = 0x00;
    cmd[1] = 0x20;
  }
  else
  {
    cmd[0] = 0x00;
    cmd[1] = 0x1C;
  }
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);


  cmd_index = 4;
  for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each ltc6811 in daisy chain, this loops starts with
  {
    // the last IC on the stack. The first configuration written is
    // received by the last IC in the daisy chain

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
    {
      // current_byte is the byte counter

      cmd[cmd_index] = pwm[current_ic-1][current_byte];             //adding the pwm data to the array to be sent
      cmd_index = cmd_index + 1;
    }

    pwm_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &pwm[current_ic-1][0]);    // calculating the PEC for each ICs configuration register data
    cmd[cmd_index] = (uint8_t)(pwm_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)pwm_pec;
    cmd_index = cmd_index + 2;
  }


  wakeup_idle ();                                 //This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.

  output_low(LTC6811_CS);
  spi_write_array(CMD_LEN, cmd);
  output_high(LTC6811_CS);
  free(cmd);
}


/*
Reads pwm registers of a ltc6811 daisy chain
*/
int8_t ltc6811_rdpwm(uint8_t total_ic, //Number of ICs in the system
                     uint8_t pwmReg,
                     uint8_t r_pwm[][8] //A two dimensional array that the function stores the read configuration data.
                    )
{
  const uint8_t BYTES_IN_REG = 8;

  uint8_t cmd[4];
  uint8_t *rx_PwmData;
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t cmd_pec;
  uint16_t received_pec;

  rx_PwmData = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));

  if (pwmReg == 0)
  {
    cmd[0] = 0x00;
    cmd[1] = 0x22;
  }
  else
  {
    cmd[0] = 0x00;
    cmd[1] = 0x1E;
  }
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_read(cmd, 4, rx_PwmData, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into
  output_high(LTC6811_CS);                          //rx_PwmData[] array

  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each ltc6811 in the daisy chain and packs the data
  {
    //into the r_config array as well as check the received Config data
    //for any bit errors
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      r_pwm[current_ic][current_byte] = rx_PwmData[current_byte + (current_ic*BYTES_IN_REG)];
    }

    received_pec = (r_pwm[current_ic][6]<<8) + r_pwm[current_ic][7];
    data_pec = pec15_calc(6, &r_pwm[current_ic][0]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }
  }

  free(rx_PwmData);
  return(pec_error);
}

/*
Writes the COMM registers of a ltc6811 daisy chain
*/
void ltc6811_wrcomm(uint8_t total_ic, //The number of ICs being written to
                    uint8_t comm[][6] //A two dimensional array of the comm data that will be written
                   )
{
  const uint8_t BYTES_IN_REG = 6;
  const uint8_t CMD_LEN = 4+(8*total_ic);
  uint8_t *cmd;
  uint16_t comm_pec;
  uint16_t cmd_pec;
  uint8_t cmd_index; //command counter

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

  cmd[0] = 0x07;
  cmd[1] = 0x21;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  cmd_index = 4;
  for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each ltc6811 in daisy chain, this loops starts with
  {
    // the last IC on the stack. The first configuration written is
    // received by the last IC in the daisy chain

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
    {
      // current_byte is the byte counter

      cmd[cmd_index] = comm[current_ic-1][current_byte];            //adding the config data to the array to be sent
      cmd_index = cmd_index + 1;
    }
    comm_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &comm[current_ic-1][0]);    // calculating the PEC for each ICs configuration register data
    cmd[cmd_index] = (uint8_t)(comm_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)comm_pec;
    cmd_index = cmd_index + 2;
  }

  wakeup_idle ();                                 //This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(CMD_LEN, cmd);
  output_high(LTC6811_CS);
  free(cmd);
}

/*
Reads COMM registers of a ltc6811 daisy chain
*/
int8_t ltc6811_rdcomm(uint8_t total_ic, //Number of ICs in the system
                      uint8_t r_comm[][8] //A two dimensional array that the function stores the read configuration data.
                     )
{
  const uint8_t BYTES_IN_REG = 8;

  uint8_t cmd[4];
  uint8_t *rx_data;
  int8_t pec_error = 0;
  uint16_t cmd_pec;
  uint16_t data_pec;
  uint16_t received_pec;

  rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));

  cmd[0] = 0x07;
  cmd[1] = 0x22;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into
  output_high(LTC6811_CS);                          //rx_data[] array

  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each ltc6811 in the daisy chain and packs the data
  {
    //into the r_comm array as well as check the received Config data
    //for any bit errors
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      r_comm[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
    }
    received_pec = (r_comm[current_ic][6]<<8) + r_comm[current_ic][7];
    data_pec = pec15_calc(6, &r_comm[current_ic][0]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }
  }

  free(rx_data);
  return(pec_error);
}

/*
Shifts data in COMM register out over ltc6811 SPI/I2C port
*/
void ltc6811_stcomm()
{

  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x07;
  cmd[1] = 0x23;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
  output_low(LTC6811_CS);
  spi_write_array(4,cmd);         //Read the configuration data of all ICs on the daisy chain into
  for (int i = 0; i<9; i++)
  {
    spi_write(0xFF);
  }
  output_high(LTC6811_CS);

}

uint16_t ltc6811_st_lookup(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint16_t test_pattern = 0;
  if (MD == 1)
  {
    if (ST == 1)
    {
      test_pattern = 0x9565;
    }
    else
    {
      test_pattern = 0x6A9A;
    }
  }
  else
  {
    if (ST == 1)
    {
      test_pattern = 0x9555;
    }
    else
    {
      test_pattern = 0x6AAA;
    }
  }
  return(test_pattern);
}

/*
 Generic wakeup command to wake isoSPI up out of idle
*/
void wakeup_idle()
{
  output_low(LTC6811_CS);
  spi_write(0xFF); //Guarantees the isoSPI will be in ready mode
  output_high(LTC6811_CS);
}

/*
 Generic wakeup command to wake the ltc6811 from sleep
*/
void wakeup_sleep()
{
  output_low(LTC6811_CS);
  for (int i = 0; i<220; i++)
  {
    spi_write(0xFF);
  }   // delays ~2.2mS Guarantees the up to 5 ltc6811 will be in standby
  output_high(LTC6811_CS);
}





/*
Calculates  and returns the CRC15
*/
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
  uint16_t remainder,addr;

  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    remainder = (remainder<<8)^pgm_read_word_near(crc15Table+addr);
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}


/*
Writes an array of bytes out of the SPI port
*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
  for (uint8_t i = 0; i < len; i++)
  {
    spi_write((int8_t)data[i]);
  }
}

/*
 Writes and read a set number of bytes using the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
  for (uint8_t i = 0; i < tx_len; i++)
  {
    spi_write(tx_Data[i]);

  }

  for (uint8_t i = 0; i < rx_len; i++)
  {
    rx_data[i] = (uint8_t)spi_read(0xFF);
  }

}



