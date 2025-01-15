//ISOBUS Section Control for ESP32:
//Rauch Axis 30.2 with Mueller Basic Terminal
//Version 01.2025
//by Andreas Orth 

//Based on:
//-Machine Control by Brian Tee - Cut and paste from everywhere
//-changes by Franz Husch for ESP32 with WW5500
//-ESP-TWAI-CAN by https://github.com/handmade0octopus/ESP32-TWAI-CAN.git


//-----------------------------------------------------------------------------------------------
// Change this number to reset and reload default parameters To EEPROM
#define EEP_Ident 0x5422

//the default network address
struct ConfigIP {
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 1;  //5
};  ConfigIP networkAddress;   //3 bytes
//-----------------------------------------------------------------------------------------------

#include <EEPROM.h>
#include <Wire.h>
#include <IPAddress.h>
#include <EthernetUdp.h>
#include <Ethernet.h>
#include "ESP32-TWAI-CAN.hpp"

EthernetUDP EthUDPToAOG;
EthernetUDP EthUDPFromAOG;

IPAddress Eth_ipDestination;



// CAN
#define CAN_TX    25
#define CAN_RX    26



// Ethernet
#define Eth_CS_PIN 5
byte Eth_myip[4] = { 192, 168, 1, 123 };//IP address to send UDP data to
//byte Eth_myip[4] = { 10, 0, 0, 22 };//IP address to send UDP data via router to tablett
//byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xB3, 0x1B}; // original
byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x00, 0x9A };
byte Eth_ipDest_ending = 100; //ending of IP address to send UDP data to router
unsigned int portMy_SC = 5123;          //port maschin/SC
unsigned int portDestination = 9999;    //Port of AOG that listens
unsigned int localPort = 8888;       // local port to listen for UDP packets
bool Ethernet_running = false;
byte ReplyBufferSC[200] = "";
int m;

//Variables for config - 0 is false
struct Config {
  uint8_t raiseTime = 2;
  uint8_t lowerTime = 4;
  uint8_t enableToolLift = 0;
  uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

  uint8_t user1 = 1; //user defined values set in machine tab
  uint8_t user2 = 0;
  uint8_t user3 = 0;
  uint8_t user4 = 0;

};  Config aogConfig;   //4 bytes

//Program counter reset
void(*resetFunc) (void) = 0;

/*
  Functions as below assigned to pins
  0: -
  1 thru 16: Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section 7,Section 8,
            Section 9, Section 10, Section 11, Section 12, Section 13, Section 14, Section 15, Section 16,
  17,18    Hyd Up, Hyd Down,
  19 Tramline,
  20: Geo Stop
  21,22,23 - unused so far*/
uint8_t pin[] = { 2, 25, 13, 17, 16, 27, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

//read value from Machine data and set 1 or zero according to list
uint8_t relayState[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };



//hello from AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };
//uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

const uint8_t LOOP_TIME = 200; //5hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;
uint32_t fifthTime = 0;
uint16_t count = 0;

//Comm checks
uint8_t watchdogTimer = 20; //make sure we are talking to AOG
uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it

bool isRaise = false, isLower = false;

//Communication with AgOpenGPS
int16_t temp, EEread = 0;

//Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0, idx = 0;
int16_t tempHeader = 0;

//settings pgn
//uint8_t PGN_237[] = { 0x80, 0x81, 0x7f, 237, 8, 1, 2, 3, 4, 0, 0, 0, 0, 0xCC };
uint8_t PGN_237[] = { 0x80, 0x81, 0x7f, 237, 8, 1, 2, 3, 4, 0, 0, 0, 0, 0xCC };
int8_t PGN_237_Size = sizeof(PGN_237) - 1;

//The variables used for storage
uint8_t relayHi = 0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0, geoStop = 0;
float gpsSpeed;
uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;


//The variables used for ISOBUS Section Control
uint8_t PGN_234[] = { 0x80, 0x81, 0x7b, 234, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };
int8_t PGN_234_Size = sizeof(PGN_234) - 1;

uint32_t CAN_ID_RX = 0x1CE726AA;
uint32_t CAN_ID_RX2 = 0x0CCBF7AA;
uint32_t CAN_ID_TX = 0x1CE6AA26;

uint32_t FilterMask;

uint8_t speedFactor = 50;
uint8_t switchFactor = 20;

uint32_t BootDelay = 30000;      //Delays the booting of the Machine-Control Sketch to give the GPS Device a head sart

uint8_t buffSectOn = 0;
uint8_t sectionOldAOG = 0;
uint8_t sectionAOG = 0;
uint8_t sectionOldISOBUS = 0;
uint8_t sectionISOBUS = 0;

bool MainSwitchCAN = false;
bool ConfirmAGPmessage = false;

uint8_t SectionBorderLeft = 8;
uint8_t SectionBorderRight = 0;
uint8_t PositionLeft = 0;
uint8_t PositionRight = 0;
uint8_t SectionLeftISOBUS = 0;
uint8_t SectionRightISOBUS = 0;
uint8_t SectionLeftAOG = 0;
uint8_t SectionRightAOG = 0;

int8_t SectionRight = 0;
int8_t SectionLeft = 0;

uint8_t Section98 = 0;
uint8_t SectionA1 = 0;
uint8_t SectionA4 = 0;
uint8_t SectionA6 = 0;

TaskHandle_t Core1;
TaskHandle_t Core2;




void setup()
{
delay(BootDelay);

  //set the baud rate
  Serial.begin(230400);
  //while (!Serial) { ; } // wait for serial port to connect. Needed for native USB

  Serial.println("Start Ethernet");
  Eth_Start();

  delay(2000);
  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(6, aogConfig);
    EEPROM.put(20, pin);
  }
  else
  {
    EEPROM.get(6, aogConfig);
    EEPROM.get(20, pin);
  }


  //set the pins to be outputs (pin numbers)
  //pin[] = { 2, 25, 13, 17, 16, 27, 12,  }
  //pinMode(2, OUTPUT);
  //pinMode(25, OUTPUT);
  //pinMode(13, OUTPUT);
  //pinMode(17, OUTPUT);
  //pinMode(16, OUTPUT);
  //pinMode(27, OUTPUT);
  //pinMode(12, OUTPUT);

  Serial.println("Setup complete, waiting for AgOpenGPS");



  if(ESP32Can.begin(ESP32Can.convertSpeed(250), CAN_TX, CAN_RX, 10, 10)) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }

xTaskCreatePinnedToCore(
                Core1code,
                "Core1",
                10000,
                NULL,
                1,
                &Core1,
                0);
delay(100);

xTaskCreatePinnedToCore(
                Core2code,
                "Core2",
                10000,
                NULL,
                1,
                &Core2,
                1);
delay(100);


}

void loop()
{
  if (Ethernet_running)
  {
    read_Eth();   //for HydLift
    delay(10);
  }

  //Loop triggers every 200 msec and sends back gyro heading, and roll, steer angle etc

  currentTime = millis();

  if (currentTime - lastTime >= LOOP_TIME)
  {
    lastTime = currentTime;

    //If connection lost to AgOpenGPS, the watchdog will count up
    if (watchdogTimer++ > 250) watchdogTimer = 20;

    //clean out serial buffer to prevent buffer overflow
    if (serialResetTimer++ > 20)
    {
      while (Serial.available() > 0) Serial.read();
      serialResetTimer = 0;
    }

    if (watchdogTimer > 20)
    {
      if (aogConfig.isRelayActiveHigh) {
        relayLo = 255;
        relayHi = 255;
      }
      else {
        relayLo = 0;
        relayHi = 0;
      }
    }

    //hydraulic lift

    if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2))
    {
      lastTrigger = hydLift;
      lowerTimer = 0;
      raiseTimer = 0;

      //200 msec per frame so 5 per second
      switch (hydLift)
      {
        //lower
        case 1:
          lowerTimer = aogConfig.lowerTime * 5;
          break;

        //raise
        case 2:
          raiseTimer = aogConfig.raiseTime * 5;
          break;
      }
    }

    //countdown if not zero, make sure up only
    if (raiseTimer)
    {
      raiseTimer--;
      lowerTimer = 0;
    }
    if (lowerTimer) lowerTimer--;

    //if anything wrong, shut off hydraulics, reset last
    if ((hydLift != 1 && hydLift != 2) || watchdogTimer > 10) //|| gpsSpeed < 2)
    {
      lowerTimer = 0;
      raiseTimer = 0;
      lastTrigger = 0;
    }

    if (aogConfig.isRelayActiveHigh)
    {
      isLower = isRaise = false;
      if (lowerTimer) isLower = true;
      if (raiseTimer) isRaise = true;
    }
    else
    {
      isLower = isRaise = true;
      if (lowerTimer) isLower = false;
      if (raiseTimer) isRaise = false;
    }

    //section relays
    SetRelays();


    //off to AOG
    send_Eth();





  } //end of timed loop

  delay(1);

  if (ReplyBufferSC[0] == 0x80 && ReplyBufferSC[1] == 0x81 && ReplyBufferSC[2] == 0x7F) //Data
  {

    if (ReplyBufferSC[3] == 239)  //machine data
    {
      uTurn = ReplyBufferSC[5];
      gpsSpeed = (float)ReplyBufferSC[6];//actual speed times 10, single uint8_t
    
      hydLift = ReplyBufferSC[7];
      tramline = ReplyBufferSC[8];  //bit 0 is right bit 1 is left

      relayLo = ReplyBufferSC[11];          // read relay control from AgOpenGPS
      relayHi = ReplyBufferSC[12];

      if (aogConfig.isRelayActiveHigh)
      {
        tramline = 255 - tramline;
        relayLo = 255 - relayLo;
        relayHi = 255 - relayHi;
      }

//      Serial.print("RelayLo_UDP   ;");
//      Serial.print(relayLo);
//      Serial.println(" , ");
      //Bit 13 CRC

      //reset watchdog
      watchdogTimer = 0;
    }

    else if (ReplyBufferSC[3] == 200) // Hello from AgIO
    {
      //Serial.println("238 //ED Machine Settings  ");
      if (ReplyBufferSC[7] == 1)
      {
        relayLo -= 255;
        relayHi -= 255;
        watchdogTimer = 0;
      }

      helloFromMachine[5] = relayLo;
      helloFromMachine[6] = relayHi;

      EthUDPToAOG.beginPacket(Eth_ipDestination, portDestination);
      EthUDPToAOG.write(helloFromMachine, sizeof(helloFromMachine));
      EthUDPToAOG.endPacket();
    }


    else if (ReplyBufferSC[3] == 238) //EE Machine Settings
    {
      //Serial.println("238 //ED Machine Settings  ");
      aogConfig.raiseTime = ReplyBufferSC[5];
      aogConfig.lowerTime = ReplyBufferSC[6];
      aogConfig.enableToolLift = ReplyBufferSC[7];

      //set1
      uint8_t sett = ReplyBufferSC[8];  //setting0
      if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;

      aogConfig.user1 = ReplyBufferSC[9];
      aogConfig.user2 = ReplyBufferSC[10];
      aogConfig.user3 = ReplyBufferSC[11];
      aogConfig.user4 = ReplyBufferSC[12];

      //crc

      //save in EEPROM and restart
      EEPROM.put(6, aogConfig);
      //resetFunc();
    }

    else if (ReplyBufferSC[3] == 201)
    {
      //make really sure this is the subnet pgn
      if (ReplyBufferSC[4] == 5 && ReplyBufferSC[5] == 201 && ReplyBufferSC[6] == 201)
      {
        networkAddress.ipOne = ReplyBufferSC[7];
        networkAddress.ipTwo = ReplyBufferSC[8];
        networkAddress.ipThree = ReplyBufferSC[9];

        //save in EEPROM and restart
        //EEPROM.put(50, networkAddress);
        resetFunc();
      }
    }

    //Scan Reply
    else if (ReplyBufferSC[3] == 202)
    {
      //make really sure this is the subnet pgn
      if (ReplyBufferSC[4] == 3 && ReplyBufferSC[5] == 202 && ReplyBufferSC[6] == 202)
      {
        uint8_t src_ip[] = { 192, 168, 1};
        //hello to AgIO
        uint8_t scanReply[] = { 128, 129, 123, 203, 7,
                                networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 123, 23
                              };
        //src_ip[0], src_ip[1], src_ip[2], 23};


        //checksum
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
        {
          CK_A = (CK_A + scanReply[i]);
        }
        scanReply[sizeof(scanReply)] = CK_A;

        static uint8_t ipDest[] = { 255, 255, 255, 255 };
        uint16_t portDest = 9999; //AOG port that listens

        //off to AOG
        EthUDPToAOG.beginPacket(ipDest, portDest);
        EthUDPToAOG.write(scanReply, sizeof(scanReply));
        EthUDPToAOG.endPacket();
      }
    }

    else if (ReplyBufferSC[3] == 236) //EC Relay Pin Settings
    {
      for (uint8_t i = 0; i < 24; i++)
      {
        pin[i] = ReplyBufferSC[i + 5];
      }

      //save in EEPROM and restart
      EEPROM.put(20, pin);
    }
  }


}

void SetRelays(void)
{
  //pin, rate, duration  130 pp meter, 3.6 kmh = 1 m/sec or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
  //gpsSpeed is 10x actual speed so 3.61111
  gpsSpeed *= 3.61111;
  //tone(13, gpsSpeed);


  //Load the current pgn relay state - Sections
  for (uint8_t i = 0; i < 8; i++)
  {
    relayState[i] = bitRead(relayLo, i);
  }
//  Serial.print("relayState   ;");
//  Serial.print(relayState[0]);
//  Serial.print(relayState[1]);
//  Serial.print(relayState[2]);
//  Serial.print(relayState[3]);
//  Serial.print(relayState[4]);
//  Serial.print(relayState[5]);
//  Serial.print(relayState[6]);
//  Serial.println("");

  for (uint8_t i = 0; i < 8; i++)
  {
    relayState[i + 8] = bitRead(relayHi, i);
  }

  // Hydraulics
  relayState[16] = isLower;
  relayState[17] = isRaise;

  //Tram
  relayState[18] = bitRead(tramline, 0); //right
  relayState[19] = bitRead(tramline, 1); //left

  //GeoStop
  relayState[20] = (geoStop == 0) ? 0 : 1;

  //pin[] = { 2, 25, 13, 17, 16, 27, 12,  }

  //if (pin[0]) digitalWrite( 2, relayState[pin[0] - 1]);
  //if (pin[1]) digitalWrite(35, relayState[pin[1] - 1]);
  //if (pin[2]) digitalWrite(13, relayState[pin[2] - 1]);
  //if (pin[3]) digitalWrite(17, relayState[pin[3] - 1]);
  //if (pin[4]) digitalWrite(16, relayState[pin[4] - 1]);
  //if (pin[5]) digitalWrite(27, relayState[pin[5] - 1]);
  //if (pin[6]) digitalWrite(12, relayState[pin[6] - 1]);

  //if (pin[7]) digitalWrite(11, relayState[pin[7] - 1]);

  //if (pin[8]) digitalWrite(12, relayState[pin[8]-1]);
  //if (pin[9]) digitalWrite(4, relayState[pin[9]-1]);

  //if (pin[10]) digitalWrite(IO#Here, relayState[pin[10]-1]);
  //if (pin[11]) digitalWrite(IO#Here, relayState[pin[11]-1]);
  //if (pin[12]) digitalWrite(IO#Here, relayState[pin[12]-1]);
  //if (pin[13]) digitalWrite(IO#Here, relayState[pin[13]-1]);
  //if (pin[14]) digitalWrite(IO#Here, relayState[pin[14]-1]);
  //if (pin[15]) digitalWrite(IO#Here, relayState[pin[15]-1]);
  //if (pin[16]) digitalWrite(IO#Here, relayState[pin[16]-1]);
  //if (pin[17]) digitalWrite(IO#Here, relayState[pin[17]-1]);
  //if (pin[18]) digitalWrite(IO#Here, relayState[pin[18]-1]);
  //if (pin[19]) digitalWrite(IO#Here, relayState[pin[19]-1]);
}
