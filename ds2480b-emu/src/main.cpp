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

void waitForBreak();
void doSearch(BYTE byteIn);
void sendByte(BYTE byteIn);

BYTE speedsel_config = SPEEDSEL_STD; // Default speed selection
unsigned int baudRate = 9600;                 // Default baud rate


#define PDSRC_BITS 0b001
#define PPD_BITS   0b010
#define SPUD_BITS  0b011
#define W1LT_BITS  0b100
#define DS0_BITS   0b101
#define LOAD_BITS  0b110
#define RBR_BITS   0b111

byte config[8] = {
  0,
  0, //PPD
  0b100, //SPUD
  0b100, //W1LT
  0, //DS0
  0, //LOAD
  0, //RBR
};

//config bits:

enum mode
{
  COMMAND,
  DATA,
  CHECK,
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
    delayMicroseconds(7); // Simulate the time it takes to write a bit
    pinMode(ONEWIRE_PIN, INPUT);
    delayMicroseconds(4); // Wait for the slave to respond
    value = digitalRead(ONEWIRE_PIN);
    delayMicroseconds(49); // Wait for the slave to respond
  }
  else
  {
    delayMicroseconds(61);
    pinMode(ONEWIRE_PIN, INPUT); // Set the pin to input to read the bus
    delayMicroseconds(3);        // Wait for the slave to respond
    value = 0;
  }
  delayMicroseconds(2); // Wait for the bus to stabilize
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

void handle_config(BYTE byteIn)
{
  BYTE parameter = (byteIn & PARMSEL_MASK) >> (1 + 3);
  BYTE value = (byteIn & PARMSET_MASK) >> 1;

  if (parameter == PARMSEL_PARMREAD)
  {
    
    BYTE response = config[value];
    DEBUGF("Reading parameter: 0x%02X: %02x", value, response);
    // TODO: read back other params here
    Serial.write((0b01110000 & byteIn) | response << 1);
    return;
  }
  else
  {
    config[parameter] = value <<1; // Store the baud rate setting
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
      DEBUGF("Changing baud rate to: %d (%dbd)", value << 1, baudRate);
      Serial.end();           // End the current serial connection
      Serial.begin(baudRate); // Reinitialize serial with the new baud rate
      while (!Serial)
        ; // Wait for the serial port to be ready
    }
    Serial.write(byteIn & 0b01111110);
  }
  return;
}

bool onewire_reset(){
    pinMode(ONEWIRE_PIN, OUTPUT);
    digitalWrite(ONEWIRE_PIN, LOW); // Pull the bus low
    delayMicroseconds(480);         // Hold low for 480us
    pinMode(ONEWIRE_PIN, INPUT);    // Release the bus

    byte noShort = digitalRead(ONEWIRE_PIN); // Read the bus state
    delayMicroseconds(70);
    byte noPresence = digitalRead(ONEWIRE_PIN); // Read the presence pulse
    delayMicroseconds(410);
    DEBUGF("Presence pulse: %d, No short: %d", noPresence, noShort);
    return !noPresence;
}

void handle_command_mode(BYTE byteIn)
{
  DEBUGF("Handling command mode with byte: 0x%02X", byteIn);
  if (byteIn == 0xE1)
  {
    DEBUGF("Switching to DATA mode");
    // handle mode shift
    currentMode = DATA;
    return;
  }

  currentMode = COMMAND;

  if ((byteIn & CMD_MODE_OP_MASK) == CMD_CONFIG)
  {
    DEBUGF("Handling configuration command: 0x%02X", byteIn);
    handle_config(byteIn);
    return;
  }
  
  if ((byteIn & CMD_MODE_OP_MASK) != CMD_MODE_OP_MASK)
  {
    DEBUGF("Invalid command in COMMAND mode: 0x%02X", byteIn);
    // This is an invalid command
    return;
  }

  BYTE func = byteIn & FUNCTSEL_MASK;
  DEBUGF("Function select: 0x%02X", func);

  if(func == FUNCTSEL_BIT){
    DEBUGF("Handling bit function with byte: 0x%02X", byteIn);
    // get bit value:
    BYTE bitValue = (byteIn & BITPOL_MASK) == BITPOL_ONE;
    speedsel_config = byteIn & SPEEDSEL_MASK;

    BYTE result = onewire_send_bit(bitValue);
    BYTE responseByte = 0b10000000 | (byteIn & 0b11100) | (result ? 0b00000011 : 0b00000000);

    DEBUGF("Sending bit: %d with speed: 0x%02X; result %d response %02x", bitValue, speedsel_config, result, responseByte);
    Serial.write(responseByte);
   }else if( func == FUNCTSEL_RESET){
    DEBUGF("Reset command received: 0x%02X", byteIn);
    speedsel_config = byteIn & SPEEDSEL_MASK;
    // Reset the bus:

    bool presence = onewire_reset();
    if(presence){
      Serial.write(0b11001100);
    }else{
      Serial.write(0b11001101);
    }
    DEBUGF("Reset command processed. Presence pulse: %d", presence);
   }else if(func == FUNCTSEL_SEARCHOFF){
    DEBUGF("Search mode change");
    if ((byteIn & FUNCTSEL_SEARCHON) == FUNCTSEL_SEARCHON)
    {
      isSearchOn = true;
    }
    else
    {
      isSearchOn = false;
    }
    DEBUGF("Search mode is now %s", isSearchOn ? "ON" : "OFF");
    speedsel_config = byteIn & SPEEDSEL_MASK;
   }else if(func == FUNCTSEL_CHMOD){
    DEBUGF("Changing to command mode with speed: 0x%02X", byteIn & SPEEDSEL_MASK);
    // pulse is not implemented
    if ((byteIn & PRIME5V_TRUE) == PRIME5V_TRUE)
    {
      dataModePullupAfterEveryByte = true;
    }
    else
    {
      dataModePullupAfterEveryByte = false;
    }
    Serial.write(byteIn);
    DEBUGF("Changed to command mode with pull-up: %s", dataModePullupAfterEveryByte ? "ON" : "OFF");
  }else {
    DEBUGF("Unknown function select: 0x%02X", func);
  }
  DEBUGF("Completed handling command mode with byte: 0x%02X", byteIn);
}

void handle_check_mode(BYTE byteIn)
{

  if (byteIn == 0xe3)
  {
      if(isSearchOn){
        doSearch(byteIn);
      }else{
        sendByte(byteIn);
      }
    currentMode = DATA;
  }
  else
  {
    // Return a dummy response for other commands
    currentMode = COMMAND;
    handle_command_mode(byteIn);
  }
}


void sendByte(BYTE byteIn){
  DEBUGF("Sending byte in DATA mode: 0x%02X", byteIn);  
      BYTE res = onewire_send_byte(byteIn);
      DEBUGF("Received byte: 0x%02X", res);
      Serial.write(res); // Echo the response back
}
void doSearch(BYTE byteIn){
  BYTE searchIn[16];
    BYTE searchOut[16] = {0}; // Initialize output array to zero
    searchIn[0] = byteIn;
    int i = 1;
    while (i < 16)
    {
      while(!Serial.available())
        ;
      searchIn[i++] = Serial.read();
    }
    DEBUGF("Search inputs:");
    for (int j = 0; j < i; j++)
    {
      DEBUGF("0x%02X", searchIn[j]);
    }

    onewire_reset();

    onewire_send_byte(0xF0);

    //onewire_send_byte(0xf0); //rom search
    // Process the search command
    for(int i = 0; i < 16; i++){
      for(int j = 0; j < 8; j+=2){
        boolean b0 = onewire_send_bit(1);
        boolean b1 = onewire_send_bit(1);
        boolean rn = (searchIn[i] >> (j+1)) & 0x1;
        boolean b2;
        boolean d;
        if(b0 == b1){
          //conflict:
          d=1;
          b2 = rn;
        }else{
          d=0;
          b2 = b0;
        }
        onewire_send_bit(b2);

        BYTE ret = d | (b2 << 1);
        searchOut[i] |= ret << j;
      }
    }
    DEBUGF("Search outputs");
    for (int j = 0; j < i; j++)
    {
      DEBUGF("0x%02X", searchOut[j]);
      Serial.write(searchOut[j]);
    }
}
void handle_data_mode(BYTE byteIn)
{
  DEBUGF("Handling data mode with byte: 0x%02X", byteIn);
  if(byteIn == 0xe3){
      currentMode = CHECK;
      return;
  }

  if (isSearchOn)
  {
    doSearch(byteIn);
  }
  else
  { 
      sendByte(byteIn);
  }
}

void setup()
{

  //pinMode(RX_PIN, INPUT);
  

  #ifndef DEBUG
  // Wait for a break condition: line LOW for >12ms
  //waitForBreak();
  #endif

  // Start serial after detecting break
  Serial.begin(9600);
  while (!Serial); 
  DEBUGF("Serial started at 9600 baud");
  while(!Serial.available());


}

void debugLoop(){
   
  onewire_reset();
  onewire_send_byte(0x33);
  BYTE romInfo[8];
  for(int i = 0; i < 8; i++){
    romInfo[i] = onewire_send_byte(0xff);
  }
  DEBUGF("ROM Info:");
  DEBUGF("Family Code: 0x%02X", romInfo[0]);
  DEBUGF("Serial Number: 0x%02X%02X%02X%02X%02X%02X", romInfo[1], romInfo[2], romInfo[3], romInfo[4], romInfo[5], romInfo[6]);
  DEBUGF("CRC: 0x%02X", romInfo[7]);

  onewire_send_byte(0x44);
  while(true){
  delay(100);
  bool done = onewire_send_bit(1);
  if(done){
    break;
  }
  }

  onewire_reset();
  onewire_send_byte(0x33);
  for(int i = 0; i < 8; i++){
    romInfo[i] = onewire_send_byte(0xff);
  }
  onewire_send_byte(0xbe);
  
  BYTE result[8];
  for(int i = 0; i < 8; i++){
    result[i] = onewire_send_byte(0xff);
  }
  DEBUGF("Temperature: %d.%d C", result[0], result[1]);


  return;
}
void loop()
{

  DEBUGF("Starting main loop");
  #ifdef DEBUG
  //debugLoop();
  //return;
  #endif
 
 while (true)
  {
    while (!Serial.available())
      ;
    byte byteIn;
    byteIn = Serial.read();
    DEBUGF("Received byte: 0x%02X. Current mode is %d", byteIn, currentMode);
    switch (currentMode)
    {
    case COMMAND:
      handle_command_mode(byteIn);
      break;
    case DATA:
      handle_data_mode(byteIn);
      break;
    case CHECK:
      handle_check_mode(byteIn);
      break;
    }
  }
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
