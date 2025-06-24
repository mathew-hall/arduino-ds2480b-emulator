#include <Arduino.h>

#define RX_PIN 0 // Arduino's hardware RX pin
#define ONEWIRE_PIN 2

// Mode Commands
#define MODE_DATA 0xE1
#define MODE_COMMAND 0xE3
#define MODE_STOP_PULSE 0xF1

// Masks for all bit ranges
#define CMD_MASK 0x80
#define FUNCTSEL_MASK 0x60
#define BITPOL_MASK 0x10
#define SPEEDSEL_MASK 0x0C
#define MODSEL_MASK 0x02
#define PARMSEL_MASK 0x70
#define PARMSET_MASK 0x0E
#define CMD_MODE_OP_MASK 0x81

// Command or config bit
#define CMD_COMM 0x81
#define CMD_COMM_RESPONSE 0x80
#define CMD_CONFIG 0x01
#define CMD_CONFIG_RESPONSE 0x00

// Function select bits
#define FUNCTSEL_BIT 0x00
#define FUNCTSEL_SEARCHON 0x30
#define FUNCTSEL_SEARCHOFF 0x20
#define FUNCTSEL_RESET 0x40
#define FUNCTSEL_CHMOD 0x60

// One Wire speed bits
#define SPEEDSEL_STD 0x00
#define SPEEDSEL_FLEX 0x04
#define SPEEDSEL_OD 0x08
#define SPEEDSEL_PULSE 0x0C

// Bit polarity/Pulse voltage bits
#define BITPOL_ONE 0x10
#define BITPOL_ZERO 0x00
#define BITPOL_5V 0x00
#define BITPOL_12V 0x10

// Parameter select bits
#define PARMSEL_PARMREAD 0x00
#define PARMSEL_SLEW 0x10
#define PARMSEL_12VPULSE 0x20
#define PARMSEL_5VPULSE 0x30
#define PARMSEL_WRITE1LOW 0x40
#define PARMSEL_SAMPLEOFFSET 0x50
#define PARMSEL_ACTIVEPULLUPTIME 0x60
#define PARMSEL_BAUDRATE 0x70

// 5V Follow Pulse select bits (If 5V pulse
// will be following the next byte or bit.)
#define PRIME5V_TRUE 0x02
#define PRIME5V_FALSE 0x00

#define PARM_BAUDRATE 0b111

// Baud rate bits
#define PARMSET_9600 0x00
#define PARMSET_19200 0x02
#define PARMSET_57600 0x04
#define PARMSET_115200 0x06
#define PARMSET_REVERSE_POLARITY 0x08

#define BYTE uint8_t

BYTE speedsel_config = SPEEDSEL_STD; // Default speed selection
int baudRate = 9600;                 // Default baud rate

enum mode
{
  COMMAND,
  DATA,
};

boolean isSearchOn = false;
boolean dataModePullupAfterEveryByte = false;

enum mode currentMode = COMMAND;

bool onewire_send_bit(bool bitValue)
{
  // Write a bit to the bus then read. If we write a 1 there is time for the slave to change the line.
  pinMode(ONEWIRE_PIN, OUTPUT);
  digitalWrite(ONEWIRE_PIN, 0);
  bool value = 0;
  if (bitValue)
  {
    delayMicroseconds(8); // Simulate the time it takes to write a bit
    pinMode(ONEWIRE_PIN, INPUT);
    delayMicroseconds(3); // Wait for the slave to respond
    value = digitalRead(ONEWIRE_PIN);
    delayMicroseconds(49); // Wait for the slave to respond
  }
  else
  {
    delayMicroseconds(57);
    pinMode(ONEWIRE_PIN, INPUT); // Set the pin to input to read the bus
    delayMicroseconds(3);        // Wait for the slave to respond
    value = 0;
  }
  return value; // Return the read value
}

BYTE onewire_send_byte(BYTE byteIn)
{
  BYTE response = 0;
  for (int i = 0; i < 8; i++)
  {
    bool bitValue = (byteIn & (1 << i)) != 0;    // Extract the bit
    bool readValue = onewire_send_bit(bitValue); // Send the bit and read the response
    if (readValue)
    {
      response |= (1 << i); // Set the corresponding bit in the response
    }
    if (dataModePullupAfterEveryByte)
    {
      pinMode(ONEWIRE_PIN, OUTPUT);
      digitalWrite(ONEWIRE_PIN, HIGH); // Pull up after every byte
      delayMicroseconds(60);           // Wait for the pull-up to take effect
      pinMode(ONEWIRE_PIN, INPUT);     // Set the pin to input to read the bus
    }
  }

  return response; // Return the read byte
}

void handle_config(byte byteIn)
{
  BYTE parameter = (byteIn & PARMSEL_MASK) >> 1 + 3;
  BYTE value = (byteIn & PARMSET_MASK) >> 1;

  if (parameter == PARMSEL_PARMREAD)
  {
    byte response = 0; // response is 3 bits, we'll shift out later
    if (parameter == PARM_BAUDRATE)
    {
      if (baudRate == 9600)
      {
        response = PARMSET_9600;
      }
      else if (baudRate == 19200)
      {
        response = PARMSET_19200;
      }
      else if (baudRate == 57600)
      {
        response = PARMSET_57600;
      }
      else if (baudRate == 115200)
      {
        response = PARMSET_115200;
      }
      response >>= 1;
    }
    // TODO: read back other params here
    Serial.write((0b01110000 & byteIn) | response << 1);
    return;
  }
  else
  {
    if (parameter == PARM_BAUDRATE)
    {
      // baud:
      switch (value << 1)
      {
      default:
      case PARMSET_9600:
        baudRate = 9600;
        break;
      case PARMSET_19200:
        baudRate = 19200;
        break;
      case PARMSET_57600:
        baudRate = 57600;
        break;
      case PARMSET_115200:
        baudRate = 115200;
        break;
      }
      Serial.end();           // End the current serial connection
      Serial.begin(baudRate); // Reinitialize serial with the new baud rate
      while (!Serial)
        ; // Wait for the serial port to be ready
    }
    Serial.write(byteIn & 0b01111110);
  }
  return;
}

void handle_command_mode(byte byteIn)
{
  if (byteIn == 0xE1)
  {
    // handle mode shift
    currentMode = DATA;
  }

  currentMode = COMMAND;

  if (byteIn & CMD_MODE_OP_MASK == CMD_CONFIG)
  {
    handle_config(byteIn);
    return;
  }

  if (byteIn & CMD_MODE_OP_MASK != CMD_MODE_OP_MASK)
  {
    // This is an invalid command
    return;
  }

  byte func = byteIn & FUNCTSEL_MASK;

  switch (func)
  {
  case FUNCTSEL_BIT:
    // get bit value:
    byte bitValue = (byteIn & BITPOL_MASK) == BITPOL_ONE;
    speedsel_config = byteIn & SPEEDSEL_MASK;
    break;

  // Speed selection (ignored for now)
  case FUNCTSEL_RESET:
    speedsel_config = byteIn & SPEEDSEL_MASK;
    // Reset the bus:
    pinMode(ONEWIRE_PIN, OUTPUT);
    digitalWrite(ONEWIRE_PIN, LOW); // Pull the bus low
    delayMicroseconds(480);         // Hold low for 480us
    pinMode(ONEWIRE_PIN, INPUT);    // Release the bus
    Serial.write(0b11100001);
    break;
  case FUNCTSEL_SEARCHOFF:
    if (byteIn & FUNCTSEL_SEARCHON == FUNCTSEL_SEARCHON)
    {
      isSearchOn = true;
    }
    else
    {
      isSearchOn = false;
    }
    speedsel_config = byteIn & SPEEDSEL_MASK;
    break;
  case FUNCTSEL_CHMOD:
    // pulse is not implemented
    if (byteIn & PRIME5V_TRUE == PRIME5V_TRUE)
    {
      dataModePullupAfterEveryByte = true;
    }
    else
    {
      dataModePullupAfterEveryByte = false;
    }
    Serial.write(byteIn);
    break;
  }
}

void handle_data_mode(BYTE byteIn)
{

  if (isSearchOn)
  {
    BYTE searchIn[16];
    searchIn[0] = byteIn;
    int i = 1;
    while (Serial.available() && i < 16)
    {
      searchIn[i++] = Serial.read();
    }
  }
  else
  {
    onewire_send_byte(byteIn);
  }
}

void setup()
{

  pinMode(RX_PIN, INPUT);
  byte byteIn;

  // Wait for a break condition: line LOW for >12ms
  waitForBreak();

  // Start serial after detecting break
  Serial.begin(9600);
  while (!Serial)
    ;

  while (true)
  {
    while (!Serial.available())
      ;
    byteIn = Serial.read();
    switch (currentMode)
    {
    case COMMAND:
      handle_command_mode(byteIn);
      break;
    case DATA:
      handle_data_mode(byteIn);
      break;
    }
  }
}

void loop()
{
  // You could add further DS2480B emulation here
}

void waitForBreak()
{
  unsigned long lowStart = 0;

  while (true)
  {
    if (digitalRead(RX_PIN) == LOW)
    {
      if (lowStart == 0)
      {
        lowStart = millis();
      }
      else if (millis() - lowStart > 12)
      {
        // Break detected (>12ms low)
        return;
      }
    }
    else
    {
      lowStart = 0; // Reset timer
    }
  }
}
