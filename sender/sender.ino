// ***********************************************************************
// *****************************   LIBRARY   *****************************
// ***********************************************************************

#include <Arduino.h>
#include <SoftwareSerial.h>
// ***********************************************************************
// ************************     CONSTANTES    ****************************
// ***********************************************************************

#define SERIAL_RX_PIN 10
#define SERIAL_TX_PIN 11
#define START_FRAME 0xABCD

// ***********************************************************************
// ********************     VARIABLES GLOBALES     ***********************
// ***********************************************************************

SoftwareSerial Soft_serial(SERIAL_RX_PIN, SERIAL_TX_PIN);

struct consigne_triac //  uint16_t -->65535
{
  uint16_t start;
  uint16_t triac_1;
  uint16_t triac_2;
  //uint16_t triac_3;
  //uint16_t triac_4;
  uint16_t checksum;
} SerialCommand;

// ***********************************************************************
// ****************   Inclusion des sous programmes   ********************
// ***********************************************************************


// ***********************************************************************
// ***********************     FUNCTION SETUP     ************************
// ***********************************************************************

void setup()
{
  Serial.begin(115200);
  //Soft_serial.begin(115200);
  /*Serial.println(F("Serial communication between 2 arduino"));
  Serial.println(F("Sender"));*/

  SerialCommand.triac_1=80;
  SerialCommand.triac_2=40;
  //SerialCommand.triac_3=3;
  //SerialCommand.triac_4=4;

}

// ***********************************************************************
// ***********************     FUNCTION LOOP     *************************
// ***********************************************************************

void loop()
{

  serial_send();

  //SerialCommand.triac_3++;
  //SerialCommand.triac_4++;
}

// ***********************************************************************
// ************************     SERIAL SEND     **************************
// ***********************************************************************

void serial_send()
{
  // Create command
  SerialCommand.start = (uint16_t)START_FRAME;
  //SerialCommand.checksum = (uint16_t)(SerialCommand.start ^ SerialCommand.triac_1 ^ SerialCommand.triac_2 ^ SerialCommand.triac_3 ^ SerialCommand.triac_4);
  SerialCommand.checksum = (uint16_t)(SerialCommand.start ^ SerialCommand.triac_1 ^ SerialCommand.triac_2);

  // Write to Serial
  //Soft_serial.write((uint8_t *)&SerialCommand, sizeof(SerialCommand));
  Serial.write((uint8_t *)&SerialCommand, sizeof(SerialCommand));
  delay(10);
}
