/************************************
REVISION HISTORY
$Revision: 1200 $
$Date: 2016-3-9

Copyright (c) 2015, Linear Technology Corp.(LTC)
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

Copyright 2015 Linear Technology Corp. (LTC)
***********************************************************/

/*! @file
    @ingroup ltc68111
    Header for ltc6811-1 Multicell Battery Monitor
*/

#ifndef LTC68111_H
#define LTC68111_H


#ifndef LTC6811_CS
#define LTC6811_CS QUIKEVAL_CS
#endif

#define MD_422HZ_1KHZ 0
#define MD_27KHZ_14KHZ 1
#define MD_7KHZ_3KHZ 2
#define MD_26HZ_2KHZ 3

#define ADC_OPT_ENABLED 1
#define ADC_OPT_DISABLED 0

#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6

#define SELFTEST_1 1
#define SELFTEST_2 2

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

#define STAT_CH_ALL 0
#define STAT_CH_SOC 1
#define STAT_CH_ITEMP 2
#define STAT_CH_VREGA 3
#define STAT_CH_VREGD 4

#define DCP_DISABLED 0
#define DCP_ENABLED 1

#define PULL_UP_CURRENT 1
#define PULL_DOWN_CURRENT 0

#define CELL_CHANNELS 12
#define AUX_CHANNELS 6
#define STAT_CHANNELS 4
#define CELL 1
#define AUX 2
#define STAT 3



/*! Starts the Mux Decoder diagnostic self test

 Running this command will start the Mux Decoder Diagnostic Self Test
 This test takes roughly 1mS to complete. The MUXFAIL bit will be updated,
 the bit will be set to 1 for a failure and 0 if the test has been passed.

*/
void ltc6811_diagn();


//! Sends the poll adc command
//! @returns 1 byte read back after a pladc command. If the byte is not 0xFF ADC conversion has completed
uint8_t ltc6811_pladc();


//! This function will block operation until the ADC has finished it's conversion
//! @returns the approximate time it took for the ADC function to complete.
uint32_t ltc6811_pollAdc();




/*! Starts cell voltage conversion


  Starts ADC conversions of the ltc6811 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the following parameters:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted       |
 | ST     | Determines which Self Test is executed       |

 Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |

*/
void ltc6811_adcv(uint8_t MD, //!< ADC Conversion Mode
                  uint8_t DCP, //!< Controls if Discharge is permitted during conversion
                  uint8_t CH //!< Sets which Cell channels are converted
                 );

/*!  Starts cell voltage  and GPIO 1&2 conversion

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCVAX:    |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   1   |   1   |   1   |   1   |
*/
void ltc6811_adcvax(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t DCP //!< Controls if Discharge is permitted during conversion
);


/*!  Starts cell voltage self test conversion

  Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CVST:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] | ST[1] | ST[0] |  0    |   0   |   1   |   1   |   1   |
*/
void ltc6811_cvst(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t ST //!< Self Test Mode
);

/*!  Starts cell voltage and SOC conversion

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCVSC:    |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   |   1   |   1   |   1   |
*/
void ltc6811_adcvsc(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t DCP //!< Controls if Discharge is permitted during conversion
);
/*!  Starts cell voltage overlap conversion

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   0   |   0   |  DCP  |   0   |   0   |   0   |   1   |
*/
void ltc6811_adol(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t DCP //!< Discharge permitted during conversion
);

/*!  Start an open wire Conversion

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADOW:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |  PUP  |   1   |  DCP  |   1   | CH[2] | CH[1] | CH[0] |
*/
void ltc6811_adow(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t PUP //!< Controls if Discharge is permitted during conversion
);


/*!  Start GPIO and Vref2 Conversion


 The type of ADC conversion executed can be changed by passing the following variables to the conversion functions:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |
 | ST     | Determines which Self Test is executed       |

Starts an ADC conversions of the ltc6811 GPIO inputs.
Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADAX:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  0    |   0   | CHG[2]| CHG[1]| CHG[0]|
*/
void ltc6811_adax(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t CHG //!< Sets which GPIO channels are converted
);

/*!  Start an GPIO Redundancy self test

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADAXD:     |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   0   |   0   |   0   |   0   | CHG[2]| CHG[1]| CHG[0]|
*/
void ltc6811_adaxd(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t CHG //!< Sets which GPIO channels are converted
);

/*!  Start an Auxiliary Register Self Test Conversion

 Starts an ADC conversions for testing the Auxiliary register memory.
Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|AXST:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] | ST[1] | ST[0] |  0    |   0   |   1   |   1   |   1   |
*/
void ltc6811_axst(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t ST //!< Sets if self test 1 or 2 is run
);



/*!  Start a Status ADC Conversion

Starts an ADC conversions of the ltc6811 status inputs.
The type of ADC conversion executed can be changed by passing the following variables to the conversion functions:
 |Variable|Function                                       |
 |--------|-----------------------------------------------|
 | MD     | Determines the filter corner of the ADC       |
 | CHST   | Determines which status channels are converted|
 | ST     | Determines which Self Test is executed        |


Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2    |   1    |   0    |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|--------|--------|--------|
|ADSTAT:  |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  0    |   1   | CHST[2]| CHST[1]| CHST[0]|
*/
void ltc6811_adstat(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t CHST //!< Sets which Stat channels are converted
);

/*!   Start a Status register redundancy test Conversion

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2    |   1    |   0    |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|--------|--------|--------|
|ASTATD:    |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   0   |   0   |  0    |   1   | CHST[2]| CHST[1]| CHST[0]|
*/
void ltc6811_adstatd(
  uint8_t MD, //!< ADC Mode
  uint8_t CHST //!< Sets which Status channels are converted
);


/*!  Start a Status Register Self Test Conversion

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2    |   1    |   0    |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|--------|--------|--------|
|STATST:    |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] | ST[1] | ST[0] |  0    |   1   |    1   |    1   |   1    |
*/
void ltc6811_statst(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t ST //!< Sets if self test 1 or 2 is run
);

/*!  Reads and parses the ltc6811 cell voltage registers.

 The function is used to read the cell codes of the ltc6811.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.

 uint8_t reg; This controls which cell voltage register is read back.
          0: Read back all Cell registers
          1: Read back cell group A
          2: Read back cell group B
          3: Read back cell group C
          4: Read back cell group D

  uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

  uint16_t cell_codes[]; An array of the parsed cell codes from lowest to highest. The cell codes will
  be stored in the cell_codes[] array in the following format:
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |

  @return int8_t, PEC Status.

    0: No PEC error detected

    -1: PEC error detected, retry read


*/
uint8_t ltc6811_rdcv(uint8_t reg, //!< controls which cell voltage register is read back.
                     uint8_t total_ic, //!< the number of ICs in the daisy chain(-1 only)
                     uint16_t cell_codes[][CELL_CHANNELS] //!< array of the parsed cell codes from lowest to highest.
                    );

/*!  Read the raw data from the ltc6811 cell voltage register

 The function reads a single cell voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the ltc6811_rdcv() command.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.
          1: Read back cell group A
          2: Read back cell group B
          3: Read back cell group C
          4: Read back cell group D

Command Code:
-------------
|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCVA:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |
|RDCVB:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |
|RDCVC:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |
|RDCVD:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   0   |

*/
void ltc6811_rdcv_reg(uint8_t reg, //!< Controls which cell voltage register is read back.
                      uint8_t nIC, //!<  The number of ICs in the daisy chain
                      uint8_t *data //!< An array of the unparsed cell codes
                     );

/*!  Reads and parses the ltc6811 auxiliary registers.

 The function is used to read the  parsed GPIO codes of the ltc6811. This function will send the requested
 read commands parse the data and store the gpio voltages in aux_codes variable


 uint16_t aux_codes[][6]; A two dimensional array of the gpio voltage codes. The GPIO codes will
 be stored in the aux_codes[][6] array in the following format:
 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |

@return  int8_t, PEC Status
  0: No PEC error detected
 -1: PEC error detected, retry read

*/
int8_t ltc6811_rdaux(uint8_t reg,        //!< controls which GPIO voltage register is read back
                     uint8_t nIC,        //!< the number of ICs in the daisy chain
                     uint16_t aux_codes[][AUX_CHANNELS] //!< A two dimensional array of the parsed gpio voltage codes
                    );



/*!  Read the raw data from the ltc6811 auxiliary register

 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the ltc6811_rdaux() command.

Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDAUXA:      |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |   0   |
|RDAUXB:      |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |

*/
void ltc6811_rdaux_reg(uint8_t reg, //!< controls which GPIO voltage register is read back
                       uint8_t nIC, //!< the number of ICs in the daisy chain
                       uint8_t *data //!< An array of the unparsed aux codes
                      );

/*!  Read the raw data from the ltc6811 stat register

 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the ltc6811_rdstat() command.

 @param[in] uint8_t reg; This controls which GPIO voltage register is read back.
          1: Read back stat group A
          2: Read back stat group B

Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDstatA:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |   0   |
|RDstatB:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   0   |

*/
void ltc6811_rdstat_reg(uint8_t reg, //!< Determines which GPIO voltage register is read back
                        uint8_t total_ic, //!< the number of ICs in the daisy chain
                        uint8_t *data //!< Array of the unparsed statiliary codes
                       );

/*!  Reads and parses the ltc6811 stat registers.

 The function is used to read the  parsed status codes of the ltc6811. This function will send the requested
 read commands parse the data and store the status voltages in stat_codes variable

@param[in] uint8_t reg; This controls which GPIO voltage register is read back.
          0: Read back all stat registers
          1: Read back stat group A
          2: Read back stat group B

 uint16_t stat_codes[][6]; A two dimensional array of the gpio voltage codes. The GPIO codes will
 be stored in the stat_codes[][6] array in the following format:
 |  stat_codes[0][0]| stat_codes[0][1] |  stat_codes[0][2]|  stat_codes[0][3]| stat_codes[1][0] |stat_codes[1][1]|  .....    |
 |------------------|------------------|------------------|------------------|------------------|----------------|-----------|
 |IC1 SOC           |IC1 ITMP          |IC1 VregA         |IC1 VregD         |IC2 SOC           |IC2 ITMP        |  .....    |

@return  int8_t, PEC Status
  0: No PEC error detected
 -1: PEC error detected, retry read
*/
int8_t ltc6811_rdstat(uint8_t reg, //!< Determines which GPIO voltage register is read back.
                      uint8_t total_ic,//!< the number of ICs in the system
                      uint16_t stat_codes[][4],//!< A two dimensional array of the stat voltage codes.
                      uint8_t flags[][3], //!< byte array that contains the uv/ov flag data
                      uint8_t mux_fail[][1], //!< Mux self test status flag
                      uint8_t thsd[][1] //!< Thermal shutdown status
                     );

/*!  Clears the ltc6811 cell voltage registers

 The command clears the cell voltage registers and initializes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.

Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRCELL:     |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |   1   |
*/
void ltc6811_clrcell();

/*! Clears the ltc6811 Auxiliary registers

 The command clears the Auxiliary registers and initializes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.

Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRAUX:      |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   0   |
*/
void ltc6811_clraux();

/*!  Clears the ltc6811 Stat registers

 The command clears the Stat registers and initializes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.

Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRSTAT:     |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   1   |
*/
void ltc6811_clrstat();

/*!  Clears the ltc6811 Sctrl registers

 The command clears the Sctrl registers and initializes
 all values to 0. The register will read back hexadecimal 0x00
 after the command is sent.

Command Code:
-------------
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRSCTRL:      |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |   0   |   0   |
*/
void ltc6811_clrsctrl();

/*!  Write the ltc6811 configuration register

 This command will write the configuration registers of the ltc6811-1s
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.

 uint8_t config[][6] is a two dimensional array of the configuration data that will be written, the array should contain the 6 bytes for each

 IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array. The array should
 have the following format:
 |  config[0][0]| config[0][1] |  config[0][2]|  config[0][3]|  config[0][4]|  config[0][5]| config[1][0] |  config[1][1]|  config[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.


Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRCFG:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
*/
void ltc6811_wrcfg(uint8_t nIC, //!< The number of ICs being written
                   uint8_t config[][6] //!< a two dimensional array of the configuration data that will be written
                  );

/*!  Reads configuration registers of a ltc6811 daisy chain

uint8_t r_config[][8] is a two dimensional array that the function stores the read configuration data. The configuration data for each IC
is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes
block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_config[0][0]|r_config[0][1]|r_config[0][2]|r_config[0][3]|r_config[0][4]|r_config[0][5]|r_config[0][6]  |r_config[0][7] |r_config[1][0]|r_config[1][1]|  .....    |
|--------------|--------------|--------------|--------------|--------------|--------------|----------------|---------------|--------------|--------------|-----------|
|IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC1 PEC High    |IC1 PEC Low    |IC2 CFGR0     |IC2 CFGR1     |  .....    |


@return int8_t, PEC Status.
  0: Data read back has matching PEC
   -1: Data read back has incorrect PEC

Command Code:
-------------

|CMD[0:1]   |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCFG:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |
*/
int8_t ltc6811_rdcfg(uint8_t nIC, //!< number of ICs in the daisy chain
                     uint8_t r_config[][8] //!< a two dimensional array that the function stores the read configuration data
                    );


/*!  Write the ltc6811 PWM register

 This command will write the pwm registers of the ltc6811-1s
 connected in a daisy chain stack. The pwm is written in descending
 order so the last device's pwm is written first.

 uint8_t pwm[][6] is a two dimensional array of the pwm data that will be written, the array should contain the 6 bytes for each

 IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array. The array should
 have the following format:
 |  pwm[0][0]| pwm[0][1] |  pwm[0][2]|  pwm[0][3]|  pwm[0][4]|  pwm[0][5]| pwm[1][0] |  pwm[1][1]|  pwm[1][2]|  .....    |
 |-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
 |IC1 pwm0   |IC1 pwm1   |IC1 pwm2   |IC1 pwm3   |IC1 pwm4   |IC1 pwm5   |IC2 pwm0   |IC2 pwm1   | IC2 pwm2  |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.

Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRPWM:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
*/
void ltc6811_wrpwm(uint8_t nIC, //!< number of ICs in the daisy chain
                   uint8_t pwmReg,
                   uint8_t pwm[][6]
                  );

/*!  Reads pwm registers of a ltc6811 daisy chain

uint8_t r_pwm[][8] is a two dimensional array that the function stores the read pwm data. The pwm data for each IC
is stored in blocks of 8 bytes with the pwm data of the lowest IC on the stack in the first 8 bytes
block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_pwm[0][0]|r_pwm[0][1]|r_pwm[0][2]|r_pwm[0][3]|r_pwm[0][4]|r_pwm[0][5]|r_pwm[0][6]  |r_pwm[0][7] |r_pwm[1][0]|r_pwm[1][1]|  .....    |
|-----------|-----------|-----------|-----------|-----------|-----------|-------------|------------|-----------|-----------|-----------|
|IC1 pwm0   |IC1 pwm1   |IC1 pwm2   |IC1 pwm3   |IC1 pwm4   |IC1 pwm5   |IC1 PEC High |IC1 PEC Low |IC2 CFGR0  |IC2 CFGR1  |  .....    |

@return int8_t, PEC Status.
  0: Data read back has matching PEC
  -1: Data read back has incorrect PEC

Command Code:
-------------

|CMD[0:1]   |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDPWM:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |   1   |   0   |
*/
int8_t ltc6811_rdpwm(uint8_t nIC, //!< number of ICs in the daisy chain
                     uint8_t pwmReg,
                     uint8_t r_pwm[][8] //!< a two dimensional array that the function stores the read pwm data
                    );




/*!  Write the ltc6811 Sctrl register

 This command will write the pwm registers of the ltc6811-1s
 connected in a daisy chain stack. The pwm is written in descending
 order so the last device's pwm is written first.

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.

Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRSCTRL:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   0   |   0   |
*/
void ltc6811_wrsctrl(uint8_t nIC, //!< number of ICs in the daisy chain
                     uint8_t sctrl_reg,
                     uint8_t sctrl[][6]
                    );


/*!  Reads sctrl registers of a ltc6811 daisy chain

uint8_t r_sctrl[][8] is a two dimensional array that the function stores the read pwm data. The sctrl data for each IC
is stored in blocks of 8 bytes with the pwm data of the lowest IC on the stack in the first 8 bytes
block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

@return int8_t, PEC Status.
  0: Data read back has matching PEC
  -1: Data read back has incorrect PEC

Command Code:
-------------

|CMD[0:1]   |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|rdsctrl:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   1   |   0   |
*/
int8_t ltc6811_rdsctrl(uint8_t nIC, //!< number of ICs in the daisy chain
                       uint8_t sctrl_reg,
                       uint8_t r_sctrl[][8] //!< a two dimensional array that the function stores the read pwm data
                      );


/*!  Start Sctrl data communication

This command will start the sctrl pulse communication over the spins

Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|stsctrl:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   0   |   0   |
*/
void ltc6811_stsctrl();


/*!  Write the ltc6811 COMM register

 This command will write the comm registers of the ltc6811-1s
 connected in a daisy chain stack. The comm is written in descending
 order so the last device's configuration is written first.

 uint8_t comm[][6] is a two dimensional array of the configuration data that will be written, the array should contain the 6 bytes for each

 IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array. The array should
 have the following format:
 |  COMM[0][0]| COMM[0][1] |  COMM[0][2]|  COMM[0][3]|  COMM[0][4]|  COMM[0][5]| COMM[1][0] |  COMM[1][1]|  COMM[1][2]|  .....    |
 |------------|------------|------------|------------|------------|------------|------------|------------|------------|-----------|
 |IC1 COMM0   |IC1 COMM1   |IC1 COMM2   |IC1 COMM3   |IC1 COMM4   |IC1 COMM5   |IC2 COMM0   |IC2 COMM1   | IC2 COMM2  |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.

Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRCOMM:      |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
*/
void ltc6811_wrcomm(uint8_t total_ic, //!< Number of ICs in the daisy chain
                    uint8_t comm[][6] //!< A two dimensional array of the comm data that will be written
                   );

/*!  Reads comm registers of a ltc6811 daisy chain

uint8_t r_comm[][8] is a two dimensional array that the function stores the read comm data. The comm data for each IC
is stored in blocks of 8 bytes with the comm data of the lowest IC on the stack in the first 8 bytes
block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_comm[0][0]|r_comm[0][1]|r_comm[0][2]|r_comm[0][3]|r_comm[0][4]|r_comm[0][5]|r_comm[0][6] |r_comm[0][7] |r_comm[1][0]|r_comm[1][1]|  .....    |
|------------|------------|------------|------------|------------|------------|-------------|-------------|------------|------------|-----------|
|IC1 comm0   |IC1 comm1   |IC1 comm2   |IC1 comm3   |IC1 comm4   |IC1 comm5   |IC1 PEC High |IC1 PEC Low  |IC2 CFGR0   |IC2 CFGR1   |  .....    |


@return int8_t, PEC Status.

  0: Data read back has matching PEC

  -1: Data read back has incorrect PEC


Command Code:
-------------

|CMD[0:1]   |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCCOMM:     |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   1   |   0   |   0   |   0   |   1   |   0   |
*/
int8_t ltc6811_rdcomm(uint8_t total_ic, //!< number of ICs in the daisy chain
                      uint8_t r_comm[][8] //!< Two dimensional array that the function stores the read comm data.
                     );

/*!  issues a stcomm command and clocks data out of the COMM register */
void ltc6811_stcomm();

/*!  Wake isoSPI up from idle state */
void wakeup_idle();

/*!  Wake the ltc6811 from the sleep state */
void wakeup_sleep();

/*!   calculates  and returns the CRC15
  @returns The calculated pec15 as an unsigned int
*/
uint16_t pec15_calc(uint8_t len, //!< the length of the data array being passed to the function
                    uint8_t *data //!<  the array of data that the PEC will be generated from
                   );



/*! @returns returns the register data pattern for a given ADC MD and Self test */
uint16_t ltc6811_st_lookup(
  uint8_t MD, //!< ADC Conversion Mode
  uint8_t ST //!< Self test number
);

/*!  Writes an array of bytes out of the SPI port*/
void spi_write_array(uint8_t length, //!< length of the data array being written on the SPI port
                     uint8_t *data); //!< the data array to be written on the SPI port

/*!  Writes and read a set number of bytes using the SPI port. */
void spi_write_read(uint8_t *TxData, //!< array of data to be written on the SPI port
                    uint8_t TXlen,  //!< tx_len length of the tx_data array
                    uint8_t *rx_data, //!< rx_data array that read data will be written too.
                    uint8_t RXlen); //!< number of bytes to be read from the SPI port.


const uint16_t crc15Table[256] PROGMEM = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
                               0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
                               0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
                               0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
                               0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
                               0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
                               0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
                               0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
                               0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
                               0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
                               0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
                               0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
                               0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
                               0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
                               0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
                               0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
                               0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
                               0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
                               0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
                               0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
                               0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
                               0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
                               0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
                                         };
#endif
