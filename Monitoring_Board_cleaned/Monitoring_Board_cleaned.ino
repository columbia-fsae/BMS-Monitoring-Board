#include <SPI.h>
#include "src/mcp_can/mcp_can.h"
#include "src/LTC68041/LTC68041.h"

//The arduino itself does not have the capability to use the CAN communication protocol
//A separate module is used to enable CAN messageing, and the arduino is connected to this module
// via SPI communication. The clock pin of the SPI hardware must be selected

MCP_CAN CAN(53);          //set SPI Chip Select to pin 53

//add 0,4,7 back in after testing
//Arrays that correspond to the ADC pins of the arduino.
static const int analog_pin[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
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
uint8_t crc;
uint8_t high;
uint8_t low;
uint8_t avg;
uint8_t t_stdev;

// These variables are utilized to store information from the LTC6804
uint16_t cell_codes[1][12]; //Cell codes (in this format for rdcv)
uint8_t tx_cfg[1][6]; //Configuration data written to ICs
int error;

// These variables are used for voltage-related calculations.
int minCell_i, maxCell_i;
float minCell_v, maxCell_v, avgCell_v, stdev_v;

// Bytes to send voltages via CAN
uint8_t v_crc;
uint8_t v_high;
uint8_t v_low;
uint8_t v_avg;
uint8_t v_stdev;

// Cell balancing threshold
float CELL_BALANCE_THRESHOLD = 2.5;

// Fan enable threshold
float FAN_THRESHOLD = 40.0; //In degrees Celsius

//Variable used when creating array, needed to use pow() function
float hold_float = 0;

const float TEMP_MAX = 250;
const float VOLTAGE_MAX = 5;
const float TEMP_STDEV_MAX = 25;
const float VOLTAGE_STDEV_MAX = 5;

//LTC6804 Configuration Functions
void init_cfg() { //Initialize LTC6804 configuration
  //Values for CFGR registers
  tx_cfg[0][0] = 0xFE; //Keep reference voltage on (speeds up readings)
  tx_cfg[0][1] = 0x00;
  tx_cfg[0][2] = 0x00;
  tx_cfg[0][3] = 0x00;
  tx_cfg[0][4] = 0x00; //Discharge switches initialially deactivated
  tx_cfg[0][5] = 0x20; //One minute software timer
}
void balance_cfg(int cell) {
  //Clear discharge switches
  tx_cfg[0][4] = 0x00;
  tx_cfg[0][5] = tx_cfg[0][5] & 0xF0;
  if (cell >= 0 && cell <= 7) {
    tx_cfg[0][4] = tx_cfg[0][4] | 1 << cell;
  }
  else if (cell > 7) { //Check if greater than 7, as -1 is used to stop all discharging
    tx_cfg[0][5] = tx_cfg[0][5] | (1 << (cell - 8));
  }
}

//Fan control
void setFan(bool on) {
  return; //TODO
}

uint8_t encodeInt(int number, float maxVal) {
  return (uint8_t) floor(((number) / maxVal) * 0xFF);
}

uint8_t encodeFloat(float number, float maxVal) {
  return (uint8_t) floor(((number) / maxVal) * 0xFF);
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


  // Setup pins for input and convert from ADC bit value to temperature in celsius
  for (int i = 0; i < 16; i++) { //or i <= 4
    hold_float = analogRead(analog_pin[i]);
    hold_float = hold_float / 1024.00L;

    hold_float = 5.00L * hold_float;
    hold_array[i] = 60L + (60L / -.66L) * (hold_float - 1.51L);
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
    float v = cell_codes[0][i] * 0.0001; //Convert voltage to reading
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
    stdev_v += pow(cell_codes[0][i] * 0.0001 - avgCell_v, 2);
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
  hold_avg = hold_sum / 16;

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

  // convert crc, high, low, and avg from int to uint8_t
  high = encodeInt(hold_high, TEMP_MAX);
  //crc = (uint8_t*)hold_crc;
  low = encodeInt(hold_low, TEMP_MAX);
  avg = encodeInt(hold_avg, TEMP_MAX);

  //Compute voltage crc
  //v_crc = (uint8_t*) (57 + minCell_v + maxCell_v + avgCell_v + 16+15+8);

  //Convert cell voltages to uint8_t
  v_low = encodeFloat(minCell_v, VOLTAGE_MAX);
  v_high = encodeFloat(maxCell_v, VOLTAGE_MAX);
  v_avg = encodeFloat(avgCell_v, VOLTAGE_MAX);

  //Convert standard deviations
  t_stdev = encodeFloat(stdev_t, TEMP_STDEV_MAX);
  v_stdev = encodeFloat(stdev_v, VOLTAGE_STDEV_MAX);

  byte dataMessage[8] = {low, high, avg, t_stdev, v_low, v_high, v_avg, v_stdev};//{(byte)low, (byte)high, (byte)avg, (byte)t_stdev, (byte)v_low, (byte)v_high, (byte)v_avg, (byte)v_stdev};

  //send CAN message
  CAN.sendMsgBuf(0x99, 0, 8, dataMessage);

}
