//Stephen K. "TheRCGuy"
//Tank Controller REV 1.0
//Uses VESC over CAN, SRXL reciever, and LCD for status

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <FlexCAN_T4.h>

#include "can.h"
#include "spm_srxl.h"
#include "myLCD.h"
#include "vesc.h"


const int DEBUG_LED_1 = 35;
const int DEBUG_LED_2 = 36;

const uint8_t VESC_ID_LEFT = 0x0B;    // shown as 11 DEC in VESC tool
const uint8_t VESC_ID_RIGHT = 0x16;   // shown as 22 DEC in VESC tool

const unsigned long HEARTBEAT_INTERVAL = 1000;  // ms
const unsigned long VESC_UPDATE_INTERVAL = 250; // ms
const unsigned long LCD_UPDATE_INTERVAL = 250;  // ms


uint8_t vesc_index_left;
uint8_t vesc_index_right;



boolean failsafeState = false;
boolean armState = false;

unsigned long currentMillis = 0;
unsigned long heartbeat_millis = 0;
unsigned long vesc_update_millis = 0;
unsigned long lcd_update_millis = 0;

uint8_t uartHandle = 1; //useless handler for UART port

uint32_t serialNum = random(10, 2147483640); //Makes random ID for reciver

// UART receive buffer
byte rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
uint8_t rxBufferIndex = 0;

int channel[9];

int x = 7;

// INPUTS
int     nJoyX = 0;              // Joystick X input
int     nJoyY = 0;              // Joystick Y input
// OUTPUTS
int     nMotMixL;           // Motor (left)  mixed output
int     nMotMixR;           // Motor (right) mixed output
// CONFIG
// - fPivYLimt  : The threshold at which the pivot action starts
//                This threshold is measured in units on the Y-axis
//                away from the X-axis (Y=0). A greater value will assign
//                more of the joystick's range to pivot actions.
float fPivYLimit = 417.0;
// TEMP VARIABLES
float   nMotPremixL;    // Motor (left)  premixed output
float   nMotPremixR;    // Motor (right) premixed output
int     nPivSpeed;      // Pivot Speed
float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

int incomingByte = 0;    // for incoming serial data

myLCD BongoLCD;


// LED controller state variables
boolean ModeToggle = false;
boolean LightsEnable = false;

int RGBMode = 0;//0: OFF 1: Solid Color 2: Rainbow
int RGBColor = 0;
boolean RGBenable = false;
struct RGBstruct{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
RGBstruct StripColors[8] = {{0,0,0},{240,0,0},{0,240,0},{0,0,240},{0,240,240},{240,0,240},{240,240,0},{240,240,240}};//OFF,RED,GREEN,BLUE,CYAN,PURPLE,YELLOW,WHITE


// Forward definitions of app-specific handling of telemetry and channel data -- see examples below
//void userProvidedFillSrxlTelemetry(SrxlTelemetryData* pTelemetry);
void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t lengthData);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup()
{
  Serial.begin(9600);
  Serial1.begin(115200, SERIAL_8N1);
  Serial1.setTimeout(5);
  Serial2.begin(115200);

  delay(500);

  pinMode(DEBUG_LED_1, OUTPUT);
  pinMode(DEBUG_LED_2, OUTPUT);


  delay(100);
  BongoLCD.lcdInit();
  delay(100);


  serial1_enable_TX_tristate(true);

  uint32_t uniqueID = serialNum;  // or hashFunction(serialNum) or rand() -- just needs to be likely unique

  // Init the local SRXL device
  if (!srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, uniqueID)) {
    Serial.println("failed init device");
  }

  // Init the SRXL bus: The bus index must always be < SRXL_NUM_OF_BUSES -- in this case, it can only be 0
  if (!srxlInitBus(0, uartHandle, SRXL_SUPPORTED_BAUD_RATES)) {
    Serial.println("failed init bus");
  }


  can_init();

  vesc_index_left = vesc_register(VESC_ID_LEFT);
  vesc_index_right = vesc_register(VESC_ID_RIGHT);
  vesc_init();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop()
{
  currentMillis = millis();

  // serial receiver stuff
  getSrxlPacket();
  rxBufferIndex = 0;

  // print the channels to the serial console
  //channelDebug();

  // arm switch checking
  if (channel[4] < 700 && channel[4] > 600 && !failsafeState)
  {
    digitalWrite(DEBUG_LED_1, HIGH);
    calcTankMix();
    armState = true;
    vesc_set_duty(VESC_ID_LEFT, nMotMixL);
    vesc_set_duty(VESC_ID_RIGHT, nMotMixR);
  }
  else
  {
    if(!failsafeState) armState = false;
    digitalWrite(DEBUG_LED_1, LOW);
    vesc_set_duty(VESC_ID_LEFT, 0);
    vesc_set_duty(VESC_ID_RIGHT, 0);
  }

  // Heartbeat blink task
  if(currentMillis >= (heartbeat_millis + HEARTBEAT_INTERVAL))
  {
    heartbeat_millis = currentMillis;
    digitalWrite(DEBUG_LED_2, digitalRead(DEBUG_LED_2) == HIGH ? LOW : HIGH);
  }

  // LCD update task
  if(currentMillis >= (lcd_update_millis + LCD_UPDATE_INTERVAL))
  {
    lcd_update_millis = currentMillis;
    update_lcd();
  }

  // VESC values update task
  if(currentMillis >= (vesc_update_millis + VESC_UPDATE_INTERVAL))
  {
    vesc_update_millis = currentMillis;
    vesc_update_values(VESC_ID_LEFT);
    vesc_update_values(VESC_ID_RIGHT);
    //print_vesc_values();
  }

  // led and rgb strip stuff
  if (((channel[5] < 725) && (channel[5] > 625)) && !LightsEnable) {//Both RGB and Lights
    //RGBMode on
    Serial.println("All lights on");
    RGBenable = true;
    RGBMode = 1;
    //Lights on
    LightsEnable = true;

    uint8_t payload_led[1] = {0xFF}; // turn all leds on
    can_send_cmd(0xA3, 1, payload_led, 1);

    uint8_t payload_rgb[4] = {RGBMode, StripColors[RGBColor].r, StripColors[RGBColor].g, StripColors[RGBColor].b}; // turn on rgb strip
    can_send_cmd(0xA3, 5, payload_rgb, 4);
    can_send_cmd(0xA3, 6, payload_rgb, 4);
  }
  else if(((channel[5] < 2075) && (channel[5] > 2023)) && (!RGBenable || LightsEnable)) {//Only RGB

    //RGB Mode on
    Serial.println("RGB on");
    RGBenable = true;
    RGBMode = 1;
    //Lights off
    LightsEnable = false;

    uint8_t payload_led[1] = {0x00}; // turn all leds off
    can_send_cmd(0xA3, 1, payload_led, 1);

    uint8_t payload_rgb[4] = {RGBMode, StripColors[RGBColor].r, StripColors[RGBColor].g, StripColors[RGBColor].b}; // turn on rgb strip
    can_send_cmd(0xA3, 5, payload_rgb, 4);
    can_send_cmd(0xA3, 6, payload_rgb, 4);
  }
  else if ( ((channel[5] < 3450) && (channel[5] > 3375)) && (RGBenable || LightsEnable)){

    //RGB Mode 0
    Serial.println("Lights off");
    RGBenable = false;
    RGBMode = 0;
    //Lights off
    LightsEnable = false;

    uint8_t payload_led[1] = {0x00}; // turn all leds off
    can_send_cmd(0xA3, 1, payload_led, 1);

    uint8_t payload_rgb[4] = {RGBMode, StripColors[RGBColor].r, StripColors[RGBColor].g, StripColors[RGBColor].b}; // turn off rgb strip
    can_send_cmd(0xA3, 5, payload_rgb, 4);
    can_send_cmd(0xA3, 6, payload_rgb, 4);
  }

  //RGB mode control (toggle)
  if (((channel[6] < 725) && (channel[6] > 625)) && !ModeToggle)
  {
    Serial.println("RGB toggle");
    ModeToggle = true;

    if(RGBMode == 1){
      if(RGBColor < 7){
        RGBColor++;
      }
      else{
        RGBColor = 0;
        RGBMode = 2;
      }
    }
    else{
      RGBMode = 1;
    }

    uint8_t payload_rgb[4] = {RGBMode, StripColors[RGBColor].r, StripColors[RGBColor].g, StripColors[RGBColor].b}; // set rgb strip
    can_send_cmd(0xA3, 5, payload_rgb, 4);
    can_send_cmd(0xA3, 6, payload_rgb, 4);
  }
  else if(!((channel[6] < 725) && (channel[6] > 625)))
  {
    ModeToggle = false;
  }

  // software triggered estop
  if (((channel[7] < 725) && (channel[7] > 625)) || (armState && failsafeState))
  {
    Serial.println("E-Stopped");

    uint8_t payload[1] = {0x00};  // dummy payload
    can_send_cmd(0xB3, CAN_ID, payload, 1);
  }

  can_task();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// User-defined routine to use the provided channel data from the SRXL bus master
void userProvidedReceivedChannelData(bool isFailsafe)
{
  // The recommended method is to access all channel data through the global srxlChData var:
  channel[0] = srxlChData.values[0] >> 4; //Throttle
  channel[1] = srxlChData.values[1] >> 4; //Roll
  channel[2] = srxlChData.values[2] >> 4; //Pitch
  channel[3] = srxlChData.values[3] >> 4; //Yaw
  channel[4] = srxlChData.values[4] >> 4; //Switch H
  channel[5] = srxlChData.values[5] >> 4; //Switch B
  channel[6] = srxlChData.values[6] >> 4; //Switch A
  channel[7] = srxlChData.values[7] >> 4; //Switch E
  channel[8] = srxlChData.values[8] >> 4; //Right slider

  failsafeState = isFailsafe;

  // RSSI and frame loss data are also available:
  if (srxlChData.rssi < -85 || (srxlChData.rssi > 0 && srxlChData.rssi < 10)) {
    Serial.println("bad rssi");
  }
  if (isFailsafe == true) {
    Serial.println("Failsafe");
    digitalWrite(DEBUG_LED_2, HIGH);
  }
  else {
    digitalWrite(DEBUG_LED_2, LOW);
  }

}

void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t lengthData)
{

  serial1_enable_TX_tristate(false);

  Serial1.write(pBuffer, lengthData);

  Serial1.flush(); // waits until the last data byte has been sent
  serial1_enable_TX_tristate(true);
}

// User-defined routine to populate telemetry data
void userProvidedFillSrxlTelemetry(SrxlTelemetryData * pTelemetry)
{
  //Serial.println("here");
  // Copy in whatever telemetry data you wish to send back this cycle
  // ... or directly access the global value srxlTelemData
  //memset(pTelemetry->raw, 0, 16);

  //srxlTelemData.sensorID = 1;
}

void serial1_enable_TX_tristate(bool bEnable)
{
  if (bEnable) CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(3); // set Open Drain Enabled
  else         CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // set Slew Rate Enabled
}

void getSrxlPacket()
{

  if (Serial1.available())
  {
    Serial1.readBytes(&rxBuffer[rxBufferIndex], SRXL_MAX_BUFFER_SIZE);

    rxBufferIndex += sizeof(rxBuffer);

    if (rxBuffer[0] == SPEKTRUM_SRXL_ID)
    {
      //Serial.println("ID correct");
      uint8_t packetLength = rxBuffer[2];
      if (rxBufferIndex > packetLength)
      {
        // Try to parse SRXL packet -- this internally calls srxlRun() after packet is parsed and reset timeout
        if (srxlParsePacket(0, rxBuffer, packetLength))
        {
          // Move any remaining bytes to beginning of buffer (usually 0)
          //Serial.println("Parsed?");
          rxBufferIndex -= packetLength;
          memmove(rxBuffer, &rxBuffer[packetLength], rxBufferIndex);
        }
        else
        {
          rxBufferIndex = 0;
        }
      }
    }
  }
}

void channelDebug(void)
{

  Serial.print("channel ");
  Serial.print(x);
  Serial.print(": ");
  Serial.println(channel[x]);

  /*  if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (x == 8) x = 0;
      else x++;
    }*/

}

void calcTankMix(void)
{

  //1340 +-

  nJoyX = userMap(channel[1], 704.0, 3398.0, -1304.0, 1304.0);
  nJoyY = userMap(channel[2], 696.0, 3408.0, -1304.0, 1304.0);

  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX >= 0) ? 1340.0 : (1340.0 + nJoyX);
    nMotPremixR = (nJoyX >= 0) ? (1340.0 - nJoyX) : 1340.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX >= 0) ? (1340.0 - nJoyX) : 1340.0;
    nMotPremixR = (nJoyX >= 0) ? 1340.0 : (1340.0 + nJoyX);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY / 1340.0;
  nMotPremixR = nMotPremixR * nJoyY / 1340.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * ( nPivSpeed);
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

  nMotMixL = userMap(nMotMixL, -1304.0, 1304.0, -10000.0, 10000.0);
  nMotMixR = userMap(nMotMixR, -1304.0, 1304.0, -10000.0, 10000.0);

  /*Serial.print("Mix Left: ");
    Serial.println(nMotMixL);
    Serial.print("Mix Right: ");
    Serial.println(nMotMixR);
  */
}

long userMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void print_vesc_values(void)
{
  Serial.println("");
  Serial.println("Left Motor:");
  Serial.print("FET Temp: ");
  Serial.print(vesc_values[vesc_index_left].tempFET, 1);
  Serial.print("°C\tCurrent: ");
  Serial.print(vesc_values[vesc_index_left].avgInputCurrent, 3);
  Serial.print(" A\t\tDuty: ");
  Serial.print(vesc_values[vesc_index_left].dutyCycle, 1);
  Serial.print("\tRPM: ");
  Serial.print(vesc_values[vesc_index_left].rpm, 0);
  Serial.print("\t\tVoltage: ");
  Serial.println(vesc_values[vesc_index_left].inputVoltage, 3);
  Serial.println("Right Motor:");
  Serial.print("FET Temp: ");
  Serial.print(vesc_values[vesc_index_right].tempFET, 1);
  Serial.print("°C\tCurrent: ");
  Serial.print(vesc_values[vesc_index_right].avgInputCurrent, 3);
  Serial.print(" A\t\tDuty: ");
  Serial.print(vesc_values[vesc_index_right].dutyCycle, 1);
  Serial.print("\tRPM: ");
  Serial.print(vesc_values[vesc_index_right].rpm, 0);
  Serial.print("\t\tVoltage: ");
  Serial.println(vesc_values[vesc_index_right].inputVoltage, 3);
  Serial.println("");
}

void update_lcd(void)
{
  float lcd_data[25];

  /*
    LCD Data:
      armStatus
      failsafeStatus
      average volt
      average amp
      tempLeftMotor
      dutyLeft
      rpmLeft
      tempRightMotor
      dutyRight
      rpmRight
      mode
  */

  lcd_data[0] = armState;
  lcd_data[1] = failsafeState;
  lcd_data[2] = vesc_values[vesc_index_left].inputVoltage;
  lcd_data[3] = 0;  //vesc_values[vesc_index_left].avgInputCurrent;
  lcd_data[4] = vesc_values[vesc_index_left].tempMotor;
  lcd_data[5] = vesc_values[vesc_index_left].tempFET;
  lcd_data[6] = vesc_values[vesc_index_left].dutyCycle;
  lcd_data[7] = 0;  //vesc_values[vesc_index_left].rpm;
  lcd_data[8] = vesc_values[vesc_index_right].tempMotor;
  lcd_data[9] = vesc_values[vesc_index_right].tempFET;
  lcd_data[10] = vesc_values[vesc_index_right].dutyCycle;
  lcd_data[11] = 0; //vesc_values[vesc_index_right].rpm;
  lcd_data[12] = 0;

  // too many variables causes the screen to stop working
  /*
  lcd_data[0] = armState;
  lcd_data[1] = failsafeState;
  lcd_data[2] = ((vesc_values[vesc_values_left].inputVoltage + vesc_values[vesc_values_right].inputVoltage) / 2);
  lcd_data[3] = ((vesc_values[vesc_values_left].avgInputCurrent + vesc_values[vesc_values_right].avgInputCurrent) / 2);
  lcd_data[4] = vesc_values[vesc_values_left].tempMotor;
  lcd_data[5] = vesc_values[vesc_values_left].tempFET;
  lcd_data[6] = vesc_values[vesc_values_left].dutyCycle;
  lcd_data[7] = vesc_values[vesc_values_left].rpm;
  lcd_data[8] = vesc_values[vesc_values_right].tempMotor;
  lcd_data[9] = vesc_values[vesc_values_right].tempFET;
  lcd_data[10] = vesc_values[vesc_values_right].dutyCycle;
  lcd_data[11] = vesc_values[vesc_values_right].rpm;
  lcd_data[12] = 0;
  */

  BongoLCD.lcdUpdate(lcd_data);
}
