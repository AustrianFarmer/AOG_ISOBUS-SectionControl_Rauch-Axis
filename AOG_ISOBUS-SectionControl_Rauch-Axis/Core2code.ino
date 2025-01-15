
//----Writing CAN Messages -------------------------------------------------------------------

void sendSectionFrameSwitch(uint8_t State, uint8_t SectionSwitch, uint8_t Variable1, uint8_t Variable2) {
  
  CanFrame SectionFrameSwitch = { 0 };
  SectionFrameSwitch.identifier = CAN_ID_TX; 
  SectionFrameSwitch.extd = 1;
  SectionFrameSwitch.data_length_code = 8;
  SectionFrameSwitch.data[0] = 0x00;
  SectionFrameSwitch.data[1] = State;
  SectionFrameSwitch.data[2] = SectionSwitch;
  SectionFrameSwitch.data[3] = 0x13;    
  SectionFrameSwitch.data[4] = Variable1;   
  SectionFrameSwitch.data[5] = 0x03;    
  SectionFrameSwitch.data[6] = Variable2;    
  SectionFrameSwitch.data[7] = 0xFF;
 
    
ESP32Can.writeFrame(SectionFrameSwitch);  


//  Serial.print("SectionFrame   ;");
//  Serial.print(SectionFrameSwitch.identifier,HEX);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.extd);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data_length_code);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data[0],HEX);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data[1],HEX);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data[2],HEX);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data[3],HEX);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data[4],HEX);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data[5],HEX);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data[6],HEX);
//  Serial.print(" , ");
//  Serial.print(SectionFrameSwitch.data[7],HEX);
//  Serial.println("");

}

 
//--- Position of rightmost and leftmost Bit with 1 ---------------------------------------

void OuterSection(uint8_t Byte_Section) {

  if(Byte_Section > 0)
  {
    for (uint8_t count_r = 0; count_r < 8; count_r++)
    {
      if(bitRead(Byte_Section, count_r)) PositionRight = count_r;
    }
      PositionRight = 7 - PositionRight;
 

    for (uint8_t count_l = 8; count_l > 0; count_l--)
    {
      if(bitRead(Byte_Section, count_l - 1)) PositionLeft = count_l - 1;
    }  
      
    
  }
  else
  {
     PositionLeft = SectionBorderLeft;
     PositionRight = SectionBorderRight;   
  }
}

//-----------------------------------------------------------------------------------------------------------



void Core2code( void * pvParameters ){

for (;;){

delay(10);

//--- Acknowledge AGP Message ----------------------------------------------

    if(ConfirmAGPmessage == true)
    {      
          sendSectionFrameSwitch(0x01, 0x094, 0xED, 0x0A);
          delay(5);
          sendSectionFrameSwitch(0x00, 0x094, 0xED, 0x0A);
          ConfirmAGPmessage = false;
    }

//---------------------------------------------------------------------------

    sectionOldAOG = sectionAOG;
    sectionAOG = relayLo;
    sectionOldISOBUS = sectionISOBUS;
    sectionISOBUS = buffSectOn;

delay(10);

//--- Section combination the Spreader can not handle ------------------------

//The following Section combinations can not be handled by the Spreader
//which leads to an automatic Section shutoff on the ISOBUS Terminal.

       //Section 2
       //Section 3
       //Section 2+3
       //Section 6
       //Section 7
       //Section 6+7

//Therfore one solution is to not allow those section combinations in AOG by automatically turning on a neighbouring section. 
//  In case of Section 2+3 or Section 6+7: The neigbouring section which was switched off last is turned on again.
//  In case of Section 2 or Section 7: The outer section is turned on again (Section 1 or Section 8).
//  In case of Section 3 or Section 6: The neighbouring inner section is turned on again (Section 4 or Section 5).

    if(sectionAOG == 0x02 || sectionAOG == 0x04 || sectionAOG == 0x06 || sectionAOG == 0x20 || sectionAOG == 0x40 || sectionAOG == 0x60)
    {
      
      switch(sectionAOG)
      {
          
        case 0x02:
        
//          if(sectionOldAOG == 0x03) sectionAOG = 0x03;
//          else sectionAOG = 0x0E;   
          sectionAOG = 0x03;
          
          break;

        case 0x04:
        
//          if(sectionOldAOG == 0x07) sectionAOG = 0x07;
//          else sectionAOG = 0x0C;  
          sectionAOG = 0x0C;

          break;

        case 0x06:

          if(sectionOldAOG == 0x07) sectionAOG = 0x07;
          else sectionAOG = 0x0E;  

          break;

        case 0x20:

//          if(sectionOldAOG == 0xE0) sectionAOG = 0xE0;
//          else sectionAOG = 0x30;  
          sectionAOG = 0x30;

          break;

        case 0x40:

//          if(sectionOldAOG == 0xC0) sectionAOG = 0xC0;
//          else sectionAOG = 0x70;               
          sectionAOG = 0xC0;

          break;

        case 0x60:

          if(sectionOldAOG == 0xE0) sectionAOG = 0xE0;
          else sectionAOG = 0x70;  

          break;
      }
    }

//Another solution is to turn off Section 2, 3, 2+3, 6, 7, 6+7 instead of turning on a neighbouring section (see below).

delay(10);

//--- Reading ISOBUS Sections and AOG Sections ------------------------------

  OuterSection(sectionISOBUS);

    SectionLeftISOBUS = PositionLeft;
    SectionRightISOBUS = PositionRight;


  OuterSection(sectionAOG);

    SectionLeftAOG = PositionLeft;
    SectionRightAOG = PositionRight;

delay(10);

//--- Filling Bits between rightmost and leftmost Bit with 1 -----------------

      for (uint8_t i = (SectionLeftAOG); i < (8 - SectionRightAOG); i++)
        {        
          bitWrite(sectionAOG, i, 1);
          delay(10);
        }

//      Serial.print("RelayLo_Bit_filled   ;");
//      Serial.print(sectionAOG);
//      Serial.println(" , ");
           
delay(10);

//--- Sending Sections from ISOBUS Terminal to AOG ------------------------------------------------

if(sectionISOBUS != sectionOldISOBUS && sectionAOG != sectionISOBUS && sectionAOG == sectionOldAOG)
{
  PGN_234[5] = 0x02;                       // Turns off Main Auto Section Switch in AOG when pushing Section buttons on ISOBUS Terminal.
  send_Eth234();                            
                   
  PGN_234[5] = 0x00;
  PGN_234[9] = buffSectOn,HEX;
  PGN_234[10] = ~buffSectOn,HEX;
  delay(10);
  send_Eth234();

  delay(100);                                // delay to make sure the sections sent back from AOG (relayLo) are the same as from ISOBUS Terminal (buffSectOn) 
}

    
//--- Sending Sections from AOG to ISOBUS Terminal ------------------------------------------------
 
if(MainSwitchCAN == true && sectionAOG != sectionOldAOG && sectionAOG != sectionISOBUS && sectionISOBUS == sectionOldISOBUS)
{

//      Serial.print("MainSwitchCAN   ;");
//      Serial.println(MainSwitchCAN);
//      Serial.print("sectionAOG   ;");
//      Serial.println(sectionAOG);
//      Serial.print("sectionOldAOG   ;");
//      Serial.println(sectionOldAOG);
//      Serial.print("sectionISOBUS   ;");
//      Serial.println(sectionISOBUS);
//      Serial.print("sectionOldISOBUS   ;");
//      Serial.println(sectionOldISOBUS);

                              
                              
  if(sectionISOBUS == 0 && (SectionLeftAOG < SectionRightAOG) && (sectionAOG > 0))
  {
    SectionLeftISOBUS = SectionBorderRight;
    SectionRightISOBUS = SectionBorderLeft;
  }


    SectionLeft = SectionLeftISOBUS - SectionLeftAOG;
    SectionRight = SectionRightISOBUS - SectionRightAOG;

delay(50);

  if((sectionAOG == 0) && (sectionISOBUS > 0))
  {
      SectionA1 = SectionBorderLeft - SectionLeftISOBUS - SectionRightISOBUS;
//      Serial.print("ButtonA1   ;");
//      Serial.print(SectionA1,DEC);
//      Serial.println(" , ");

      for (uint8_t klick = 0; klick < SectionA1; klick++)
      {
        sendSectionFrameSwitch(0x01, 0x0A1, 0xF2, 0x01);
        delay(10);
        sendSectionFrameSwitch(0x00, 0x0A1, 0xF2, 0x01);
        delay(10);
      }
      delay(switchFactor);      
  }

  else
  {
  
    if(SectionLeft > 0) 
    {
      Section98 = SectionLeft;
//      Serial.print("Button98   ;");
//      Serial.print(Section98,DEC);
//      Serial.println(" , ");

      for (uint8_t klick = 0; klick < Section98; klick++)
      {
        sendSectionFrameSwitch(0x01, 0x098, 0xF2, 0x01);
        delay(10);
        sendSectionFrameSwitch(0x00, 0x098, 0xF2, 0x01);
        delay(10);
      }
      delay(switchFactor);       
    }

    if(SectionRight > 0) 
    {
      SectionA4 = SectionRight;
//      Serial.print("ButtonA4   ;");
//      Serial.print(SectionA4,DEC);
//      Serial.println(" , ");

      for (uint8_t klick = 0; klick < SectionA4; klick++)
      {
        sendSectionFrameSwitch(0x01, 0x0A4, 0xF2, 0x01);
        delay(10);
        sendSectionFrameSwitch(0x00, 0x0A4, 0xF2, 0x01);
        delay(10);
      }
      delay(switchFactor);       
    }
    
    if(SectionLeft < 0)
    {
      SectionA1 = -1 * SectionLeft;
//      Serial.print("ButtonA1   ;");
//      Serial.print(SectionA1,DEC);
//      Serial.println(" , ");
      
      for (uint8_t klick = 0; klick < SectionA1; klick++)
      {
        sendSectionFrameSwitch(0x01, 0x0A1, 0xF2, 0x01);
        delay(10);
        sendSectionFrameSwitch(0x00, 0x0A1, 0xF2, 0x01);
        delay(10);
      }
      delay(switchFactor);
    }
    
    if(SectionRight < 0)
    {
      SectionA6 = -1 * SectionRight;
//      Serial.print("ButtonA6   ;");
//      Serial.print(SectionA6,DEC);
//      Serial.println(" , ");

      for (uint8_t klick = 0; klick < SectionA6; klick++)
      {
        sendSectionFrameSwitch(0x01, 0x0A6, 0xF2, 0x01);
        delay(10);
        sendSectionFrameSwitch(0x00, 0x0A6, 0xF2, 0x01);
        delay(10);
      }
      delay(switchFactor);
    }
  }

buffSectOn = sectionAOG;
sectionISOBUS = buffSectOn;

//--- Turn off Section 2, 3, 2+3, 6, 7, 6+7 instead of turning on a neighbouring section (see above) -------

//    if(sectionAOG == 0x02 || sectionAOG == 0x04 || sectionAOG == 0x06 || sectionAOG == 0x20 || sectionAOG == 0x40 || sectionAOG == 0x60)
//    {
//      sectionAOG = 0x00;
//      buffSectOn = 0x00;
//      
//      PGN_234[9] = 0x00;
//      PGN_234[10] = 0xFF;
//      send_Eth234();
//    }
//-----------------------------------------------------------------------------------------------------------

delay(speedFactor);

}
}
}
