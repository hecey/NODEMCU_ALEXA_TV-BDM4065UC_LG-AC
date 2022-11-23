#include <Arduino.h>
#include "SoftwareSerial.h"
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#define SERIAL_BAUDRATE 9600

// Codes RS232 for Monitor Phillips
byte msgTurnOff[] = {0xA6, 0x01, 0x00, 0x00, 0x00, 0x04, 0x01, 0x18, 0x01, 0xBB}; // 10
byte msgTurnOn[] = {0xA6, 0x01, 0x00, 0x00, 0x00, 0x04, 0x01, 0x18, 0x02, 0xB8};  // 10
byte getVolume[] = {0xA6, 0x01, 0x00, 0x00, 0x00, 0x03, 0x01, 0x45, 0xE0};        // 9
byte setVolume[] = {0xA6, 0x01, 0x00, 0x00, 0x00, 0x04, 0x01, 0x44, 0x01, 0xE7};  // 10

byte inputSourceHDMI[2][13] = {{0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0xAC, 0x06, 0x02, 0x00, 0x00, 0x09},  // hdmi
                               {0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0xAC, 0x06, 0x03, 0x00, 0x00, 0x08}}; // MHL-hdmi

byte PIP[2][13] = {{0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x9D},  // off
                   {0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0x3C, 0x01, 0x00, 0x00, 0x00, 0x9C}}; // on

byte PIPPosition[4][13] = {
    {0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0x3C, 0x01, 0x00, 0x00, 0x00, 0x9C}, // bottom left
    {0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0x3C, 0x01, 0x01, 0x00, 0x00, 0x9D}, // top left
    {0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0x3C, 0x01, 0x02, 0x00, 0x00, 0x9E}, // topright
    {0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0x3C, 0x01, 0x03, 0x00, 0x00, 0x9F}  // bottom roght
};

byte PIPSource[2][13] = {{0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0x84, 0xFD, 0x02, 0x00, 0x00, 0xDA},  // hdmi
                         {0xA6, 0x01, 0x00, 0x00, 0x00, 0x07, 0x01, 0x84, 0xFD, 0x03, 0x00, 0x00, 0xDB}}; // mhd-hdmi

// code for LG Air Cond.

uint32_t lgTurnOn = 0x8800B0B;
uint32_t lgTurnOff = 0x88C0051;

// val for monitor
byte incomingByte;
int increment = 5;

int currentInputSource = 0;
int PIPstate = 0;
int PIPPositionState = 0;
int PIPSourceState = 0;

bool powerState = true;
bool acState = true;
int volume;
int eIndex;

#define MAX_MILLIS_TO_WAIT 1000 // or whatever
unsigned long starttime;
byte read_bytes[9];

// Define IR Receiver and Results
const int RECV_PIN = 2;
const int EMIT_PIN = 4;
IRrecv irrecv(RECV_PIN);
IRsend irsend(EMIT_PIN);

decode_results results;

SoftwareSerial gtSerial(12, 13, false, 256);
; // Arduino RX, Arduino TX

void setup()
{
  // Serial Monitor @ 9600 baud
  Serial.begin(9600, SERIAL_8N1);
  gtSerial.begin(9600);
  // Enable the IR Receiver
  irrecv.enableIRIn();
  irsend.begin();
  volume = 10;
}

byte CheckSum(byte msgData[], int indexSize)
{
  byte hashValue = 0;
  for (int i = 0; i < indexSize - 1; i++)
  {
    hashValue ^= msgData[i];
  }
  return hashValue;
}

void loop()
{

  if (irrecv.decode(&results))
  {

    serialPrintUint64(results.value, 16);
    Serial.print("");
    switch (results.value)
    {
    case 0x34347887: // power
      tvpower();
      break;
    case 0x3434E817: // volume up
      volup();
      break;
    case 0x34346897: // volume down
      voldown();
      break;
    case 0xC: // power
      tvpower();
      break;
    case 0x10: // volume up
      volup();
      break;
    case 0x11: // volume down
      voldown();
      break;
    case 0x343459A6: // inputsource
      // Turn on LED for 2 Seconds
      //  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      inputtv();
      break;
    case 0x3434D52A: // PIP On Off
      gtSerial.write(PIP[PIPstate], sizeof(PIP[PIPstate]));
      PIPstate++;
      if (PIPstate == 2)
        PIPstate = 0;

      delay(500); // wait for a second
      break;
    case 0x343445BA: // PIP Position
      gtSerial.write(PIPPosition[PIPPositionState], sizeof(PIPPosition[PIPPositionState]));
      PIPPositionState++;
      if (PIPPositionState == 4)
        PIPPositionState = 0;

      delay(500); // wait for a second
      break;
    case 0x3434FD02: // PIP source
      gtSerial.write(PIPSource[PIPSourceState], sizeof(PIPSource[PIPSourceState]));
      PIPSourceState++;
      if (PIPSourceState == 2)
        PIPSourceState = 0;

      delay(500); // wait for a second
      break;
    case 0xD: // PIP source
      // acpower();
      inputtv();
      break;
    }
    irrecv.resume();
  }
}

void volup()
{
  // Turn on LED for 2 Seconds
  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  if (eIndex != -1)
  {
    // Serial.println(e);

    if (volume < 96)
      volume = volume + increment;
    Serial.println(volume);
    // Serial.println("volume "+volume);

    setVolume[8] = (byte)volume;
    // Serial.println(setVolume[8]);

    // printArray(setVolume,10);

    setVolume[10 - 1] = CheckSum(setVolume, 10);

    gtSerial.write(setVolume, sizeof(setVolume));

    // printArray(setVolume,10);
  }
  delay(500); // wait for a second
              // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}
void voldown()
{
  // Turn on LED for 2 Seconds
  //  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  if (volume >= increment)
    volume = volume - increment;
  setVolume[8] = (byte)volume;
  Serial.println(volume);
  setVolume[10 - 1] = CheckSum(setVolume, 10);
  gtSerial.write(setVolume, sizeof(setVolume));

  delay(500); // wait for a second
              //  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}
void tvpower()
{
  // Turn on LED for 2 Seconds
  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  if (powerState)
  {
    msgTurnOff[10 - 1] = CheckSum(msgTurnOff, 10);
    gtSerial.write(msgTurnOff, sizeof(msgTurnOff));
    Serial.print("off");
    powerState = false;
  }
  else
  {
    msgTurnOn[10 - 1] = CheckSum(msgTurnOn, 10);
    gtSerial.write(msgTurnOn, sizeof(msgTurnOn));
    Serial.print("on");
    powerState = true;
  }

  delay(500); // wait for a second
              // digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}
void acpower()
{
  if (acState)
  {
    irsend.sendLG(lgTurnOff, 28);
    Serial.print("offac");
    acState = false;
  }
  else
  {
    irsend.sendLG(lgTurnOn, 28);

    Serial.print("onac");
    acState = true;
  }
}
void inputtv()
{

  gtSerial.write(inputSourceHDMI[currentInputSource], sizeof(inputSourceHDMI[currentInputSource]));
  if (currentInputSource == 0)
    currentInputSource = 1;
  else
    currentInputSource = 0;
  delay(500); // wait for a second
              //  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}
