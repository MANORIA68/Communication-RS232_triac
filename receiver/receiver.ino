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

#define P_MAX 2400 // puissance du récepteur

#define ZERO_CROSS_PIN 2
#define GACHETTE_1_PIN 12

#define GACHETTE_2_PIN 4

#define VENTILO_PIN 8

// ***********************************************************************
// ********************     VARIABLES GLOBALES     ***********************
// ***********************************************************************

SoftwareSerial Soft_serial(SERIAL_RX_PIN, SERIAL_TX_PIN);

typedef struct //  uint16_t -->65535
{
  uint16_t start;
  uint16_t triac_1;
  uint16_t triac_2;
  // uint16_t triac_3;
  // uint16_t triac_4;
  uint16_t checksum;
} SerialCommand;

SerialCommand Consigne_triac;
SerialCommand New_consigne_triac;

uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
unsigned int msg_succes = 0;
unsigned int msg_echec = 0;

// millis
unsigned int millis_loop_start = 0;
unsigned int millis_loop = 0;
unsigned int millis_loop_min = 0;
unsigned int millis_loop_max = 0;

bool zero_crossMem = 0; // variable pour l'état précédent du zero cross
bool zero_cross = 0;    // variable pour l'état actuel du zero cross
bool triac_start = 0;   // variable pour l'état de la led
bool gachettemem = 0;
unsigned int delai_triac = 0;

class Triac
{

private:
  int GACHETTE_PIN;

public:
  Triac(int GACH_PIN)
  {
    GACHETTE_PIN = GACH_PIN;
  }

  void pulse()
  {
    digitalWrite(GACHETTE_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(GACHETTE_PIN, LOW);
  }
};

Triac triac_1(GACHETTE_1_PIN);
Triac triac_2(GACHETTE_2_PIN);

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

  millis_loop_start = micros();

  zero_cross = digitalRead(ZERO_CROSS_PIN);

  if (zero_cross != zero_crossMem)
  {
    zero_crossMem = zero_cross;
    if (zero_cross)
    {
      triac_start = LOW;
    }
    else
    {
      triac_start = HIGH;
    }
  }

  if (triac_start == HIGH)
  {
    if (serial_receive())
    {
      millis_min_max(millis_loop_start, millis_loop, millis_loop_min, millis_loop_max);

      /*Serial.print(F("millis:"));
      Serial.print(millis_loop);
      Serial.print(F("\t  min :"));
      Serial.print(millis_loop_min);
      Serial.print(F("\t max :"));
      Serial.print(millis_loop_max);
      Serial.print(F("\t sucess:"));
      Serial.print(map(msg_succes, 0, msg_succes + msg_echec, 0, 100));
      Serial.print(F("%"));
      Serial.println("");*/
    }

    // if (Consigne_triac.triac_1 > Consigne_triac.triac_2)
    //{
    unsigned int delai_triac = map(Consigne_triac.triac_1, 1, P_MAX, 8400, 2); // valeur voulue , mini, maxi(de la valeur voulue), délai maxi pour avoir 0 et délai mini pour avoir toute la sinusoide)
    delayMicroseconds(delai_triac);
    triac_1.pulse();

    delai_triac = map(Consigne_triac.triac_1 - Consigne_triac.triac_2, 1, P_MAX, 8400, 2);   
    delayMicroseconds(delai_triac);
    triac_2.pulse();
    //}

    triac_start = LOW;
  }
}

// ***********************************************************************
// *******************    SERIAL COMMUNICATION    ************************
// ***********************************************************************

bool serial_receive()
{
  // Check for new data availability in the Serial buffer
  //if (!Soft_serial.available())
  if (!Serial.available())
    return LOW;

  //incomingByte = Soft_serial.read();  
  incomingByte = Serial.read();                                  // Read the incoming byte
  bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame

  if (DEBUG_RX)
  {
    Serial.print(incomingByte, HEX);
    Serial.print(" ");
    return LOW;
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
    // checksum = (uint16_t)(New_consigne_triac.start ^ New_consigne_triac.triac_1 ^ New_consigne_triac.triac_2 ^ New_consigne_triac.triac_3 ^ New_consigne_triac.triac_4);
    checksum = (uint16_t)(New_consigne_triac.start ^ New_consigne_triac.triac_1 ^ New_consigne_triac.triac_2);

    // Check validity of the new data
    if (New_consigne_triac.start == START_FRAME && checksum == New_consigne_triac.checksum)
    {
      // Copy the new data
      memcpy(&Consigne_triac, &New_consigne_triac, sizeof(SerialCommand));

      // Print data to built-in Serial
     /* Serial.print("1: ");
      Serial.print(Consigne_triac.triac_1);
      Serial.print(" 2: ");
      Serial.println(Consigne_triac.triac_2);
      /*Serial.print(" 3: ");
      Serial.print(Consigne_triac.triac_3);
      Serial.print(" 4: ");
      Serial.println(Consigne_triac.triac_4);*/
      msg_succes++;
    }
    else
    {
      //Serial.println("Non-valid data");
      msg_echec++;
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  else
  {
    incomingBytePrev = incomingByte;
    return LOW;
  }

  // Update previous states
  incomingBytePrev = incomingByte;
  return HIGH;
}

// ***********************************************************************
// ************************     MILLIS DEBUG    **************************
// ***********************************************************************

void millis_min_max(unsigned int value_depart, unsigned int &value_actuel, unsigned int &value_min, unsigned int &value_max)
{
  if (value_depart == 0)
    return; // valeur pas encore initialisé avec millis

  value_actuel = micros() - value_depart;

  if (value_actuel < value_min)
    value_min = value_actuel;

  if (value_actuel > value_max)
    value_max = value_actuel;
}

// ***********************************************************************
// *********************     MANAGEMENT TRIAC    *************************
// ***********************************************************************
