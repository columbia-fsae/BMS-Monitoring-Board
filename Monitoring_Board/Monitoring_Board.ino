#include <SPI.h>
#include "mcp_can/mcp_can.h"
#include "Linduino/Linduino.h"
#include "LTC68041/LTC68041.h"
#include "LT_SPI/LT_SPI.h"

//The arduino itself does not have the capability to use the CAN communication protocol
//A separate module is used to enable CAN messageing, and the arduino is connected to this module 
// via SPI communication. The clock pin of the SPI hardware must be selected

MCP_CAN CAN(53);          //set SPI Chip Select to pin 53



//add 0,4,7 back in after testing
//Arrays that correspond to the ADC pins of the arduino.
static const int analog_pin[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};   
int hold_array[16];
uint8_t temp_array[16];


//initializing variables that will be used to calculate the crc, average, high, and low temperatures
int hold_crc;       
int hold_high = hold_array[0];
int hold_low = hold_array[0];
int hold_avg = 0;
int hold_sum = 0;
float stdev_t;


// Only bytes can be sent via CAN. The previous int variables will be converted to uint8_t           
uint8_t* crc;
uint8_t* high;
uint8_t* low;
uint8_t* avg;
uint8_t* t_stdev;

// These variables are utilized to store information from the LTC6804
uint16_t cell_codes[12]; //Cell codes
uint8_t tx_cfg[6]; //Configuration data written to ICs

// These variables are used for voltage-related calculations.
int minCell_i, maxCell_i;
float minCell_v, maxCell_v, stdev_v;

// Bytes to send voltages via CAN
uint8_t* v_crc;
uint8_t* v_high;
uint8_t* v_low;
uint8_t* v_avg;
uint8_t* v_stdev;

// Cell balancing threshold
float CELL_BALANCE_THRESHOLD = 3.9;

// Fan enable threshold
float FAN_THRESHOLD = 40.0;

//Variable used when creating array, needed to use pow() function
float hold_float = 0;


//In order to send temperatures to the BMS, the following CAN messages must be sent.
//Message 2 contains the data that will be calculated from the ADC readings, and
//will be initialized later.
byte message1[8] = {0xF3, 0x00, 0x80, 0xF3, 0x00, 0x40, 0x1E, 0x90};

byte message3[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0F, 0x00};
byte message4[4];

//In order to send voltages to the BMS, the following CAN messages must be sent.
//Message 6 contains the data that will be calculated from the ADC readings, and
//will be initialized later.
byte message5[8] = {0xF3, 0x00, 0x80, 0xF3, 0x00, 0x40, 0x1E, 0x90};

byte message7[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0F, 0x00};
byte message8[4];

//LTC6804 Configuration Functions
void init_cfg() { //Initialize LTC6804 configuration
  //Values for CFGR registers
  tx_cfg[0] = 0xFE; //Keep reference voltage on (speeds up readings)
  tx_cfg[1] = 0x4E1; //2V
  tx_cfg[2] = 0x8CA; //3.6V
  tx_cfg[3] = 0x00;
  tx_cfg[4] = 0x00; //Discharge switches initialially deactivated
  tx_cfg[5] = 0x20; //One minute software timer
}
void balance_cfg(int cell) {
  //Clear discharge switches
  tx_cfg[4] = 0x00;
  tx_cfg[5] = tx_cfg[5] & 0xF0;
  if (cell >= 0 && cell <= 7) {
    tx_cfg[4] = tx_cfg[4] | 1 << cell;
  }
  else if (cell > 7) { //Check if greater than 7, as -1 is used to stop all discharging
    tx_cfg[5] = tx_cfg[5] | (1 << (cell - 8));
  }
}

//Fan control
void setFan(bool on) {
  return; //TODO
}


//Initialization of SPI and CAN communication protocols.
void setup() {
  // put your setup code here, to run once:
    SPI.begin();
 
    Serial.begin(115200);

    while (CAN_OK != CAN.begin(CAN_500KBPS))
    {
        Serial.println("CAN BUS init Failed");
        delay(100);
    }
    Serial.println("CAN BUS Shield Init OK!"); 

    LTC6804_initialize();
    init_cfg();
}

void loop() {
  /*
  sensorInput3 = analogRead(A3);
  delay(20);
  sensorInput = analogRead(A3); //read the analog sensor and store it
  temp = (double)sensorInput / 1024;   //find percentage of input reading
  temp = temp * 5;
  temp = 60 + (60/-0.66) * (temp-1.51);                                    
   Serial.print("Current Temperature: "); 
  Serial.println(sensorInput);
  Serial.println(temp); 
 */

 
/*
  sensorInput = analogRead(A0);
  delay(20);
  sensorInput = analogRead(A0); //read the analog sensor and store it
  Serial.println("sensor input");
  Serial.println(sensorInput);
  hold_high = sensorInput*5L;   //find percentage of input reading
  Serial.println("/ 1024");
  Serial.println(hold_high);
  hold_high = (float)hold_high / 1024.00L;
  Serial.println("*5");
  Serial.println(hold_high);
  hold_high = 60.00L + (60.00L/-0.66L) * (hold_high-1.51L);
 Serial.println("temp");
 Serial.println(hold_high);

 high = (int)hold_high;
 Serial.println(high);
 delay(20000);
*/
 


 
 
// Setup pins for input and convert from ADC bit value to temperature in celsius
for (int i = 0; i < 16; i++) { //or i <= 4
  
  /*
  
  int hold = 0;
  for(int j = 0; j< 10; j++){
   hold += analogRead(analog_pin[i]);
    delay(1);
  //hold = analogRead(analog_pin[i]);
  }
  //Serial.println(hold);
  hold = hold / 10;
  //Serial.println(hold_float);

  */
  hold_float = analogRead(analog_pin[i]);
  //Serial.println(hold);
  hold_float = hold_float / 1024.00L;
  //Serial.println(hold_float);

    hold_float = 5.00L * hold_float;
  //Serial.println(hold_float);
  hold_array[i] = 60L + (60L/-.66L) * (hold_float-1.51L);
 // hold_array[i] = hold_float;
  //hold_array[i] = (int)(1768L - (hold_float * 2595L) + (1311L * pow(hold_float,2)) - (226 * pow(hold_float,3)));
 //Serial.println(hold_float);
  Serial.print("Temp reading number:");
   Serial.println(i);
   Serial.println(hold_array[i]);
   delay(500);

}

  //Read voltages
  wakeup_idle();
  LTC6804_adcv(); //Do A->D conversion and fill registers
  delay(10);
  wakeup_idle();
  error = LTC6804_rdcv(0, 1, cell_codes); // Read cell voltages from registers
  if (error == -1) {
    Serial.println("PEC error detected!");
  }

  //Calculate maximum, minimum, and average cell voltages
  minCell_i = -1;
  maxCell_i = -1;
  minCell_v = 500.0; //This is much higher than and voltage will be
  maxCell_v = 0.0;
  avgCell_v = 0.0;
  for (int i = 0; i < 12; i++) {
    float v = cell_codes[i] * 0.0001; //Convert voltage to reading
    if (v < minCell_v) {
      minCell_v = v;
    }
    if (v > maxCell_v) {
      maxCell_v = v;
    }
    avgCell_v += v;
  }
  avgCell_v /= 12;

  //Calculate standard deviation of voltages
  stdev_v = 0.0;
  for (int i = 0; i < 12; i++) {
    stdev_v += pow(cell_codes[i] * 0.0001 - avgCell_v, 2);
  }
  stdev_v /= 12;
  stdev_v = pow(stdev_v, 0.5);

  //Enable passive balancing for highest-voltage cell if it exceeds the threshold
  if (maxCell_v > CELL_BALANCE_THRESHOLD) {
    balance_cfg(maxCell_i);
  }
  else {
    balance_cfg(-1);
  }
  LTC6804_wrcfg(0, tx_cfg); //Write balancing configuration to LTC6804
  


  //Find highest temperature using hold_array and hold_high

  for (int i = 1; i < 16; i++) { //or i <= 4
      if (hold_array[i] > hold_high) {
          hold_high = hold_array[i];
          
      }
  }  

  
  //Find lowest temperature using hold_array and hold_low
 
  for (int i = 1; i < 16; i++) {
    if (hold_array[i] < hold_low) {
        hold_low = hold_array[i];
    }
} 

  
  //Find average temperature using hold_array and hold_avg
   for (int i = 0; i < 16; i++) {
    hold_sum = hold_sum + hold_array[i];
   }
   hold_avg = hold_sum/13;

  //Calculate standard deviation of temperatures
  stdev_t = 0.0;
  for (int i = 0; i < 16; i++) {
    stdev_t += pow(hold_array[i] - hold_avg, 2);
  }
  stdev_t /= 16;
  stdev_t = pow(stdev_t, 0.5);

  //Set fan state
   if (hold_high > FAN_THRESHOLD) {
    setFan(1);
   }
   else {
    setFan(0);
   }


//Testing code

//Serial.println(hold_high);
//Serial.println(hold_low);
//Serial.println(hold_avg);
//delay(1000);
 
 //hold_high = hold_array[1];
 //hold_low = hold_array[2];
   
   
  // Compute temperature crc
  hold_crc = 57 + hold_low + hold_high  + hold_avg + 16 + 15+8;
  uint8_t* crc = (uint8_t*)hold_crc; 

  // convert crc, high, low, and avg from int to uint8_t
  high = (uint8_t*)hold_high;
  crc = (uint8_t*)hold_crc; 
  low = (uint8_t*)hold_low;
  avg = (uint8_t*)hold_avg;

  //Compute voltage crc
  v_crc = (uint8_t*) (57 + minCell_v + maxCell_v + avgCell_v + 16+15+8);

  //Convert cell voltages to uint8_t
  v_low = (uint8_t*) minCell_v;
  v_high = (uint8_t*) maxCell_v;
  v_avg = (uint8_t*) avgCell_v;

  //Convert standard deviations
  t_stdev = (uint8_t*) stdev_t;
  v_stdev = (uint8_t*) stdev_v;
  
  //Set CAN message 2 
  
  byte message2[8] = {0x00, low, high, avg, 0x10, 0x0F, 0x00, crc};

  //Set CAN message 5
  byte message6[8] = {0x00, v_low, v_high, v_avg, 0x10, 0x0F, 0x00, v_crc};

  //send CAN messages
  //messages 2-4 are sent every 100ms, and message 1 is sent every 200ms

  delay(100);
  
  CAN.sendMsgBuf(0x1839F380, 1, 8, message2);
  CAN.sendMsgBuf(0x1838F380,1, 8, message3);
  CAN.sendMsgBuf(0x80, 0, 4, message4);
  
  delay(100);
   
  CAN.sendMsgBuf(0x18EEFF80, 1, 8, message1);

  delay(100);
  
  CAN.sendMsgBuf(0x1839F380, 1, 8, message6);
  CAN.sendMsgBuf(0x1838F380,1, 8, message7);
  CAN.sendMsgBuf(0x80, 0, 4, message8);
  
  delay(100);
   
  CAN.sendMsgBuf(0x18EEFF80, 1, 8, message5);

 
}
