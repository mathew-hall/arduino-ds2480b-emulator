#include <Arduino.h>


#define BYTE uint8_t
//#define DEBUG 1
#ifdef DEBUG
char debugBuffer[128]; // Buffer for debug messages
#define DEBUGF(fmt, ...) \
  snprintf(debugBuffer, sizeof(debugBuffer), "%s:%d: " fmt, __FILE__, __LINE__, ##__VA_ARGS__); \
  Serial.println(debugBuffer);
#else
#define DEBUGF(fmt, ...) \
  do {} while (0) // No-op if DEBUG is not defined
#endif

bool onewire_reset(int pin){
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW); // Pull the bus low
    delayMicroseconds(480);         // Hold low for 480us
    pinMode(pin, INPUT);    // Release the bus

    byte noShort = digitalRead(pin); // Read the bus state
    delayMicroseconds(70);
    byte noPresence = digitalRead(pin); // Read the presence pulse
    delayMicroseconds(410);
    DEBUGF("Presence pulse: %d, No short: %d", noPresence, noShort);
    return !noPresence;
}


bool onewire_send_bit(int pin, bool bitValue)
{
  // Write a bit to the bus then read. If we write a 1 there is time for the slave to change the line.
  pinMode(pin, OUTPUT);
  digitalWrite(pin, 0);
  bool value = 0;
  if (bitValue)
  {
    delayMicroseconds(7); // Simulate the time it takes to write a bit
    pinMode(pin, INPUT);
    delayMicroseconds(4); // Wait for the slave to respond
    value = digitalRead(pin);
    delayMicroseconds(49); // Wait for the slave to respond
  }
  else
  {
    delayMicroseconds(61);
    pinMode(pin, INPUT); // Set the pin to input to read the bus
    delayMicroseconds(3);        // Wait for the slave to respond
    value = 0;
  }
  delayMicroseconds(2); // Wait for the bus to stabilize
  return value; // Return the read value
}


BYTE onewire_send_byte(int pin, BYTE byteIn)
{
  BYTE response = 0;
  for (int i = 0; i < 8; i++)
  {
    bool bitValue = (byteIn & (1 << i)) != 0;    // Extract the bit
    bool readValue = onewire_send_bit(pin, bitValue); // Send the bit and read the response
    if (readValue)
    {
      response |= (1 << i); // Set the corresponding bit in the response
    }
  }

  return response; // Return the read byte
}