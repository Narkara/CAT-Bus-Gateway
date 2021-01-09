#include <Narkara_Core.h>
#include <avr/boot.h>
#include <CAT_Bus.h>

#define VERSION_MAJOR 1
#define VERSION_MINOR 0

#define DEBUG_ENABLE_DEFAULT false

//CAT Bus Commands
#define READ_SERIAL_NUMBER 0xA0
#define WRITE_RELAY_0 0xA1    
#define WRITE_RELAY_1 0xA2    
#define WRITE_RELAY_2 0xA3    
#define WRITE_RELAY_3 0xA4    
#define GET_USB_CURRENT_LIMIT 0xA5
#define READ_FIRMWARE_VERSION 0xAF

#define SET_DEBUG_PORT_ENABLE 0xCE
#define GET_DEBUG_PORT_ENABLE 0xCF

//Pin definitions
pin ss = PB0;
pin sck = PB1;
pin mosi = PB2;
pin miso = PB3;
pin rf_int = PB4;
pin r0 = PB5;
pin r1 = PB6;

pin r2 = PC6;
pin r3 = PC7;

pin rx = PD2;
pin tx = PD3;
pin debug_led = PD4;
pin dir = PD5;

pin boot_led = PE2;
pin nss = PF4;
pin chg_det = PF5;
pin cc2_det = PF6;
pin cc1_det = PF7;

//Global Variables
CAT_Bus_1 bus = CAT_Bus_1(rx, tx, dir);
bool debugEn = DEBUG_ENABLE_DEFAULT;
uint8_t address = 0x40; 

//RS-485 interface
ISR(USART1_RX_vect){
  if(bus.receiveCommand()) bus.sendResponse(processBusData());
}

//Processes the data in the bus data buffer.
uint8_t processBusData(){
  //Standard return packet includes 2 data bytes
  uint8_t returnDataBytes = 2;
  
  if(debugEn){
    Serial.print(F("Received packet: "));
    Serial.print(bus.address, HEX);
    Serial.print(' ');
    Serial.print(bus.command, HEX);
    Serial.print(' ');
    Serial.print(bus.data[0], HEX);
    Serial.print(' ');
    Serial.print(bus.data[1], HEX);
    Serial.print(' ');
    Serial.println(bus.crc, HEX);
  }
  
  switch(bus.command){
    case READ_SERIAL_NUMBER:{
      if(debugEn) Serial.println(F("READ_SERIAL_NUMBER"));
    
      for(uint8_t i = 0; i < 10; i++)
        bus.data[i] = boot_signature_byte_get(i+14);
      
      returnDataBytes = 10;
      break;
    }
    case READ_FIRMWARE_VERSION:{
      if(debugEn) Serial.println(F("READ_FIRMWARE_VERSION"));

      bus.data[0] = VERSION_MAJOR;
      bus.data[1] = VERSION_MINOR;

      break;
    }
    case WRITE_RELAY_0:{
      if(debugEn) Serial.println(F("WRITE_RELAY_0"));

      if(bus.data[0] || bus.data[1])
        setState(r0, HIGH);
      else
        setState(r0, LOW);

      break;
    }
    case WRITE_RELAY_1:{
      if(debugEn) Serial.println(F("WRITE_RELAY_1"));

      if(bus.data[0] || bus.data[1])
        setState(r1, HIGH);
      else
        setState(r1, LOW);

      break;
    }
    case WRITE_RELAY_2:{
      if(debugEn) Serial.println(F("WRITE_RELAY_2"));

      if(bus.data[0] || bus.data[1])
        setState(r2, HIGH);
      else
        setState(r2, LOW);

      break;
    }
    case WRITE_RELAY_3:{
      if(debugEn) Serial.println(F("WRITE_RELAY_3"));

      if(bus.data[0] || bus.data[1])
        setState(r3, HIGH);
      else
        setState(r3, LOW);

      break;
    }
    case GET_USB_CURRENT_LIMIT:{
      if(debugEn) Serial.println(F("GET_USB_CURRENT_LIMIT"));

      bus.data[0] = (getState(cc1_det))|(getState(cc2_det) << 4);
      bus.data[1] = getState(chg_det);

      break;
    }
    case SET_DEBUG_PORT_ENABLE:{
      if(debugEn) Serial.println(F("SET_DEBUG_PORT_ENABLE"));
      
      debugEn = bus.data[0];

      returnDataBytes = 1;
      break;
    }
    case GET_DEBUG_PORT_ENABLE:{
      if(debugEn) Serial.println(F("GET_DEBUG_PORT_ENABLE"));
      
      bus.data[0] = debugEn;

      returnDataBytes = 1;
      break;
    }
    default:
      return 0;
  }
  
  //Calculate CRC
  bus.crc = calculateCRC(bus.data, returnDataBytes);
  
  if(debugEn){
    Serial.print(F("Return packet: "));
    for(uint8_t i = 0; i < returnDataBytes; i++){
      Serial.print(bus.data[i], HEX);
      Serial.print(' ');
    }
    Serial.println(bus.crc, HEX);
  }

  //Toggle LED
  setState(debug_led, !getState(debug_led));
  
  return returnDataBytes;
}

void setupSPI(){
  setDirection(ss, OUTPUT);
  setDirection(sck, OUTPUT);
  setDirection(mosi, OUTPUT);
  setDirection(miso, INPUT);

   SPCR = (0 << SPIE)    //Disable interrupt
        |(1 << SPE)     //Enable SPI
        |(0 << DORD)    //MSB first
        |(1 << MSTR)    //Master mode
        |(1 << CPOL)    //Idle high
        |(1 << CPHA)    //Sample trailing edge
        |(0 << SPR1)    //f_cpu/4 (4Mbps)
        |(0 << SPR0);   //f_cpu/4 (4Mbps)
  SPSR = (0 << SPI2X);  //f_cpu/4 (4Mbps)
}

void usbDataRx(){
  //If there is no data to be recieved, exit now
  if(!Serial.available()) return;

  //Recieve a packet from USB
  bus.address = Serial.read();
  bus.command = Serial.read();
  bus.data[0] = Serial.read();
  bus.data[1] = Serial.read();
  bus.crc = Serial.read();
  
  //Check if the CRC is valid. If it's not, exit now
  uint8_t expectedCRC = calculateCRC(bus.address, bus.command, bus.data[0], bus.data[1]);
  if(bus.crc != expectedCRC){
    Serial.print(F("CRC invalid. expected 0x"));
    Serial.println(expectedCRC, HEX);
    return;
  }
  
  //If packet addressed to this device, process the data
  if(bus.address == address){
    uint8_t returnDataBytes = processBusData();

    //If command is unknown, exit now
    if(returnDataBytes == 0) return;

    //Transmit response to USB
    Serial.write(bus.data, returnDataBytes);
    Serial.write(bus.crc);
  }
  else{
    //Packet not addressed to me, transmit on RS485 Bus and read response
    uint8_t returnDataBytes = bus.sendCommand();
    
    //If transmission failed, exit now
    if(returnDataBytes == 0) return;
    
    //Transmit response to USB
    Serial.write(bus.data, returnDataBytes);
    Serial.write(bus.crc);
  }
}

void setup() {
  
  setupSPI();

  setDirection(debug_led, OUTPUT);
  setState(debug_led, HIGH);

  //Setup relays
  setState(r0, LOW);
  setDirection(r0, OUTPUT);
  setState(r1, LOW);
  setDirection(r1, OUTPUT);
  setState(r2, LOW);
  setDirection(r2, OUTPUT);
  setState(r3, LOW);
  setDirection(r3, OUTPUT);
  
  bus.initSlave(address, 10);

  SerialUSB.setTimeout(2);

  //Todo RFM69

}

void loop() {
  usbDataRx();

}
