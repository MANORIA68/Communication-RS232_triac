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
#define DEBUG_RX 0
#define START_FRAME 0xABCD

// ***********************************************************************
// ********************     VARIABLES GLOBALES     ***********************
// ***********************************************************************

SoftwareSerial Soft_serial(SERIAL_RX_PIN, SERIAL_TX_PIN);

typedef struct //  uint16_t -->65535
{
  uint16_t start;
  uint16_t triac_1;
  uint16_t triac_2;
  uint16_t triac_3;
  uint16_t triac_4;
  uint16_t checksum;
} SerialCommand;

SerialCommand Consigne_triac;
SerialCommand New_consigne_triac;

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

// ***********************************************************************
// ***********************     FUNCTION SETUP     ************************
// ***********************************************************************

void setup()
{
  Serial.begin(115200);
  Soft_serial.begin(115200);
  Serial.println(F("Serial communication between 2 arduino"));
  Serial.println(F("Receiver"));
}

// ***********************************************************************
// ***********************     FUNCTION LOOP     *************************
// ***********************************************************************

void loop()
{
  serial_receive();
}

void serial_receive()
{
  // Check for new data availability in the Serial buffer
  if (!Soft_serial.available())
    return;

  incomingByte = Soft_serial.read();                                  // Read the incoming byte
  bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame

  if (DEBUG_RX)
  {
    Serial.print(incomingByte, HEX);
    Serial.print(" ");
    return;
  }

  if (idx > sizeof(SerialCommand))
    idx = 0;

  // Copy received data
  if (bufStartFrame == START_FRAME && idx <= 2)
  { // Initialize if new data is detected
    p = (byte *)&New_consigne_triac;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
    // Serial.println("START_FRAME");
  }
  else if (idx >= 2 && idx < sizeof(SerialCommand))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialCommand))
  {
    uint16_t checksum;
    checksum = (uint16_t)(New_consigne_triac.start ^ New_consigne_triac.triac_1 ^ New_consigne_triac.triac_2 ^ New_consigne_triac.triac_3 ^ New_consigne_triac.triac_4);

    // Check validity of the new data
    if (New_consigne_triac.start == START_FRAME && checksum == New_consigne_triac.checksum)
    {
      // Copy the new data
      memcpy(&Consigne_triac, &New_consigne_triac, sizeof(SerialCommand));

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(Consigne_triac.triac_1);
      Serial.print(" 2: ");
      Serial.print(Consigne_triac.triac_2);
      Serial.print(" 3: ");
      Serial.print(Consigne_triac.triac_3);
      Serial.print(" 4: ");
      Serial.println(Consigne_triac.triac_4);
    }
    else
    {
      Serial.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}