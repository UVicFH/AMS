#include <mcp_can.h>
#include <mcp_can_dfs.h>

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC6811_daisy.h"
#include <SPI.h>
#include <avr/wdt.h>

#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/
const uint8_t TOTAL_IC = 3;//!<number of ICs in the daisy chain

//ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See ltc6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //MD_26HZ_2KHZ;//MD_7KHZ_3KHZ; // See ltc6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See ltc6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See ltc6811_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; // See ltc6811_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; // See ltc6811_daisy.h for Options

const uint16_t MEASUREMENT_LOOP_TIME = 500;//milliseconds(mS)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 27000; // Over voltage threshold ADC Code. LSB = 0.0001
const uint16_t UV_THRESHOLD = 10000; // Under voltage threshold ADC Code. LSB = 0.0001

//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_STAT = DISABLED; //This is ENABLED or DISABLED

/************************************
  END SETUP
*************************************/

/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the ltc6811
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
uint16_t cell_codes[TOTAL_IC][CELL_CHANNELS];
/*!<
  The cell codes will be stored in the cell_codes[][12] array in the following format:

  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

uint16_t aux_codes[TOTAL_IC][AUX_CHANNELS];
/*!<
 The GPIO codes will be stored in the aux_codes[][6] array in the following format:

 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/

uint16_t stat_codes[TOTAL_IC][4];
/*!<
 The GPIO codes will be stored in the aux_codes[][6] array in the following format:

 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/

uint8_t flags_uvov[TOTAL_IC][3];

uint8_t system_thsd[TOTAL_IC][1];

uint8_t system_muxfail[TOTAL_IC][1];

long system_open_wire[TOTAL_IC];

uint8_t tx_cfg[TOTAL_IC][6];
/*!<
  The tx_cfg[][6] stores the ltc6811 configuration data that is going to be written
  to the ltc6811 ICs on the daisy chain. The ltc6811 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:

 |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

*/

uint8_t rx_cfg[TOTAL_IC][8];
/*!<
  the rx_cfg[][8] array stores the data that is read back from a ltc6811-1 daisy chain.
  The configuration data for each IC  is stored in blocks of 8 bytes. Below is an table illustrating the array organization:

|rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
|---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
|IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/

#define SPI_CS_PIN 10
#define MOSFET_STATUS_PIN 7

#define KELLY_KDHE_SEND_CAN_ID 0x6B
#define KELLY_KDHE_RESPONSE_CAN_ID 0x73
#define AMS_CAN_ID 0x69
#define CCP_A2D_BATCH_READ1 0x1b

MCP_CAN CAN(SPI_CS_PIN);

// Precharge 10 seconds TODO

/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
  delay(5000);
  Serial.begin(115200);
//  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock

  init_cfg();        //initialize the 681x configuration array to be written
  pinMode(MOSFET_STATUS_PIN, OUTPUT);
  digitalWrite(MOSFET_STATUS_PIN, HIGH);
//  pinMode(9,OUTPUT);
//  digitalWrite(9,HIGH);

  for(;;)
  {
    if(CAN_OK == CAN.begin(CAN_500KBPS))
    {
      Serial.println("CAN BUS INIT GOOD");
      break;
    }
    else
    {
      Serial.println("CAN BUS INIT FAIL, RETRY");
      delay(100);
    }
  }

  Serial.println("Starting the precharge!");
  // CCP_A2D_BATCH_READ1 is what we are checking
  
  bool preCharging = true;
  unsigned char preChargeSendMsg[1] = { CCP_A2D_BATCH_READ1 };
  unsigned char preChargeRcvMsg[5];
  unsigned char len = 0;
  int8_t error = 0;
  
  while (preCharging)
  {

    CAN.sendMsgBuf(KELLY_KDHE_SEND_CAN_ID, 0, 1, preChargeSendMsg);
    
    // We may need a delay here
    
    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
      CAN.readMsgBuf(&len, preChargeRcvMsg);
      unsigned long canID = CAN.getCanId();  
      if(canID == KELLY_KDHE_RESPONSE_CAN_ID)
      {
        float kellyVoltage = (float) preChargeRcvMsg[4]/1.39;
        Serial.print("Float voltage is: ");
        Serial.print(preChargeRcvMsg[4]);
        Serial.print(" | ");
        Serial.println(kellyVoltage);
        
        ltc6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
        ltc6811_pollAdc();
        wakeup_idle();
        error = ltc6811_rdcv(0, TOTAL_IC,cell_codes);
        check_error(error);

        float total = 0.00;
        for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
        {
          for (int i=0; i<CELL_CHANNELS; i++)
          {
            total = total + (cell_codes[current_ic][i]*0.0001);
          }
        }

        Serial.print("Measured Pack Voltage is ");
        Serial.println(total);

        float thresholdVoltage = total*(.90);

        if (kellyVoltage >= thresholdVoltage)
        {
          Serial.println("Yeoooow greater");
          preCharging = false;
          break;
        }
      }
    }
    delay(MEASUREMENT_LOOP_TIME);
  }

  
}


void loop() {
    unsigned char len = 0;
    unsigned char buf[8];
    int8_t error = 0;
    wakeup_sleep();
    ltc6811_wrcfg(TOTAL_IC,tx_cfg);

    // Check if we got a CAN message for charging
    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
       CAN.readMsgBuf(&len, buf);

       unsigned char canId = CAN.getCanId();

       if (canId == AMS_CAN_ID)
       {
        // TODO If we get a charge signal
       }
    }

    ltc6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
    ltc6811_pollAdc();
    wakeup_idle();
    error = ltc6811_rdcv(0, TOTAL_IC,cell_codes);
    check_error(error);

    float total = 0.00;
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
    {
      Serial.print("Cells, ");
      for (int i=0; i<CELL_CHANNELS; i++)
      {
        if (cell_codes[current_ic][i]*0.0001 < 1)
        {
          digitalWrite(MOSFET_STATUS_PIN, LOW);
          Serial.print("UNDER VOLTAGE FOR CELL: ");
          Serial.print(current_ic);
          Serial.print(" | ");
          Serial.println(i);
        }
        Serial.print(cell_codes[current_ic][i]*0.0001,4);
        total = total + (cell_codes[current_ic][i]*0.0001);
        Serial.print(",");
      }
    }

    


    delay(MEASUREMENT_LOOP_TIME);
      
//        measurement_loop(DATALOG_DISABLED);
}

void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  if (WRITE_CONFIG == ENABLED)
  {
    wakeup_sleep();
    ltc6811_wrcfg(TOTAL_IC,tx_cfg);
    print_config();
  }

  if (READ_CONFIG == ENABLED)
  {
    wakeup_sleep();
    error = ltc6811_rdcfg(TOTAL_IC,rx_cfg);
    check_error(error);
    print_rxconfig();
  }

  if (MEASURE_CELL == ENABLED)
  {

    ltc6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
    ltc6811_pollAdc();
    wakeup_idle();
    error = ltc6811_rdcv(0, TOTAL_IC,cell_codes);
    check_error(error);
    print_cells(datalog_en);

  }

  if (MEASURE_AUX == ENABLED)
  {
    wakeup_idle();
    ltc6811_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
    ltc6811_pollAdc();
    wakeup_idle();
    error = ltc6811_rdaux(0,TOTAL_IC,aux_codes); // Set to read back all aux registers
    check_error(error);
    print_aux(datalog_en);
  }

  if (MEASURE_STAT == ENABLED)
  {
    wakeup_idle();
    ltc6811_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
    ltc6811_pollAdc();
    wakeup_idle();
    error = ltc6811_rdstat(0,TOTAL_IC,stat_codes,flags_uvov,system_muxfail,system_thsd); // Set to read back all aux registers
    check_error(error);
    print_stat();
  }
}

void init_cfg()
{
  uint16_t uv_val = (UV_THRESHOLD/16)-1;
  uint16_t ov_val = (OV_THRESHOLD/16);
  for (int i = 0; i<TOTAL_IC; i++)
  {
    tx_cfg[i][0] = 0xFC | ADC_OPT;
    tx_cfg[i][1] = (uint8_t)(uv_val&0xFF) ;
    tx_cfg[i][2] = (uint8_t)((ov_val&0x00F)|((uv_val&0xF00)>>8)) ;
    tx_cfg[i][3] = (uint8_t)((ov_val&0xFF0)>>4);
    tx_cfg[i][4] = 0x00 ;
    tx_cfg[i][5] = 0x00 ;

  }

}

void print_cells(uint8_t datalog_en)
{

  float total = 0.00;
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      Serial.print(", ");
      for (int i=0; i<CELL_CHANNELS; i++)
      {

        Serial.print(" C");
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(cell_codes[current_ic][i]*0.0001,4);
        total = total + (cell_codes[current_ic][i]*0.0001);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {
      Serial.print("Cells, ");
      for (int i=0; i<CELL_CHANNELS; i++)
      {
        Serial.print(cell_codes[current_ic][i]*0.0001,4);
        Serial.print(",");
      }

    }
  }
  Serial.println();
  Serial.print("Pack Voltage: ");
  Serial.print(total);
  Serial.println();
}

void print_config()
{
  int cfg_pec;

  Serial.println(F("Written Configuration: "));
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(tx_cfg[current_ic][0]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][1]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][2]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][3]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][4]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][5]);
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6,&tx_cfg[current_ic][0]);
    serial_print_hex((uint8_t)(cfg_pec>>8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
  }
  Serial.println();
}

//Function to check error flag and print PEC error message
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 ltc6811 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
  Serial.println(F("Received Configuration "));
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(rx_cfg[current_ic][0]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][1]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][2]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][3]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][4]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(rx_cfg[current_ic][6]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][7]);
    Serial.println();
  }
  Serial.println();
}


/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic+1,DEC);
      for (int i=0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i+1,DEC);
        Serial.print(":");
        Serial.print(aux_codes[current_ic][i]*0.0001,4);
        Serial.print(",");
      }
      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(aux_codes[current_ic][5]*0.0001,4);
      Serial.println();
    }
    else
    {
      Serial.print("AUX, ");

      for (int i=0; i < 6; i++)
      {
        Serial.print(aux_codes[current_ic][i]*0.0001,4);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_stat()
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(" SOC:"));
    Serial.print(stat_codes[current_ic][0]*0.0001*20,4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    Serial.print(stat_codes[current_ic][1]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(stat_codes[current_ic][2]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(stat_codes[current_ic][3]*0.0001,4);
    Serial.println();
  }

  Serial.println();
}

void serial_print_hex(uint8_t data)
{
  if (data< 16)
  {
    Serial.print("0");
    Serial.print((byte)data,HEX);
  }
  else
    Serial.print((byte)data,HEX);
}

