
void Core1code( void * pvParameters ){

for (;;){

//---- Reading CAN Messages from ISOBUS Terminal -------------------------------------------

  
    CanFrame rxFrame;
    // You can set custom timeout, default is 1000
    ESP32Can.readFrame(rxFrame, 1000);


//--- AGP Message --------------------------------------------------------------------------
      
      if(rxFrame.identifier == CAN_ID_RX && rxFrame.data[0] == 0xAD && rxFrame.data[1] == 0x00 && rxFrame.data[2] == 0x00 && rxFrame.data[3] == 0xED && rxFrame.data[4] == 0x03 && rxFrame.data[5] == 0xFF && rxFrame.data[6] == 0xFF && rxFrame.data[7] == 0xFF)
        {
            ConfirmAGPmessage = true;
        }


//--- ISOBUS Terminal Main Switch On/Off ----------------------------------------------------

      if(rxFrame.identifier == CAN_ID_RX && rxFrame.data[0] == 0xA8 && rxFrame.data[1] == 0x18)
        {  

//            Serial.printf("Section Status: %03X  \r\n", rxFrame.data[4]); 
         
          switch(rxFrame.data[4])
          {

            case 0x60:
            
             MainSwitchCAN = true;

              break;

            case 0x40:

             MainSwitchCAN = false;
      
             PGN_234[5] = 0x02;           //Section Switch in AOG (0 = Manual, 1 = Auto, 2 = Off)
             PGN_234[9] = 0x00;           //Sections 1-8 On in AOG (0x00 = all Sections off)
             PGN_234[10] = 0xFF;          //Sections 1-8 Off in AOG (0xFF = all Sections off)
             send_Eth234();               //Send to AOG

              break;
           }
         }

//--- Section Messages from ISOBUS Terminal -------------------------------------------------
        
      if(rxFrame.identifier == CAN_ID_RX2 && rxFrame.data[2] == 0xA1)
        {  

//            Serial.printf("Section Status_1-4: %03X  \r\n", rxFrame.data[4]); 
//            Serial.printf("Section Status_5-8: %03X  \r\n", rxFrame.data[5]);

          
          switch(rxFrame.data[4])
          {
          
            case 0x00:

              bitWrite(buffSectOn, 0, 0);
              bitWrite(buffSectOn, 1, 0);
              bitWrite(buffSectOn, 2, 0);
              bitWrite(buffSectOn, 3, 0);

              break;

            case 0x55:

              bitWrite(buffSectOn, 0, 1);
              bitWrite(buffSectOn, 1, 1);
              bitWrite(buffSectOn, 2, 1);
              bitWrite(buffSectOn, 3, 1);

              break;  
               
            case 0x54:

              bitWrite(buffSectOn, 0, 0);
              bitWrite(buffSectOn, 1, 1);
              bitWrite(buffSectOn, 2, 1);
              bitWrite(buffSectOn, 3, 1);

              break;

            case 0x50:

              bitWrite(buffSectOn, 0, 0);
              bitWrite(buffSectOn, 1, 0);
              bitWrite(buffSectOn, 2, 1);
              bitWrite(buffSectOn, 3, 1);

              break;

            case 0x40:

              bitWrite(buffSectOn, 0, 0);
              bitWrite(buffSectOn, 1, 0);
              bitWrite(buffSectOn, 2, 0);
              bitWrite(buffSectOn, 3, 1);

              break;

            case 0x15:

              bitWrite(buffSectOn, 0, 1);
              bitWrite(buffSectOn, 1, 1);
              bitWrite(buffSectOn, 2, 1);
              bitWrite(buffSectOn, 3, 0);

              break;

            case 0x05:

              bitWrite(buffSectOn, 0, 1);
              bitWrite(buffSectOn, 1, 1);
              bitWrite(buffSectOn, 2, 0);
              bitWrite(buffSectOn, 3, 0);

              break;

            case 0x01:

              bitWrite(buffSectOn, 0, 1);
              bitWrite(buffSectOn, 1, 0);
              bitWrite(buffSectOn, 2, 0);
              bitWrite(buffSectOn, 3, 0);

              break;
                                                                                                                     
         }

          switch(rxFrame.data[5])
          {
          
            case 0x00:

              bitWrite(buffSectOn, 4, 0);
              bitWrite(buffSectOn, 5, 0);
              bitWrite(buffSectOn, 6, 0);
              bitWrite(buffSectOn, 7, 0);

              break;

            case 0x55:

              bitWrite(buffSectOn, 4, 1);
              bitWrite(buffSectOn, 5, 1);
              bitWrite(buffSectOn, 6, 1);
              bitWrite(buffSectOn, 7, 1);

              break;  
               
            case 0x54:

              bitWrite(buffSectOn, 4, 0);
              bitWrite(buffSectOn, 5, 1);
              bitWrite(buffSectOn, 6, 1);
              bitWrite(buffSectOn, 7, 1);

              break;

            case 0x50:

              bitWrite(buffSectOn, 4, 0);
              bitWrite(buffSectOn, 5, 0);
              bitWrite(buffSectOn, 6, 1);
              bitWrite(buffSectOn, 7, 1);

              break;

            case 0x40:

              bitWrite(buffSectOn, 4, 0);
              bitWrite(buffSectOn, 5, 0);
              bitWrite(buffSectOn, 6, 0);
              bitWrite(buffSectOn, 7, 1);

              break;

            case 0x15:

              bitWrite(buffSectOn, 4, 1);
              bitWrite(buffSectOn, 5, 1);
              bitWrite(buffSectOn, 6, 1);
              bitWrite(buffSectOn, 7, 0);

              break;

            case 0x05:

              bitWrite(buffSectOn, 4, 1);
              bitWrite(buffSectOn, 5, 1);
              bitWrite(buffSectOn, 6, 0);
              bitWrite(buffSectOn, 7, 0);

              break;

            case 0x01:

              bitWrite(buffSectOn, 4, 1);
              bitWrite(buffSectOn, 5, 0);
              bitWrite(buffSectOn, 6, 0);
              bitWrite(buffSectOn, 7, 0);

              break;
                                                                                                                     
         }     
       }


//      Serial.print("buffSect   ;");
//      Serial.print(buffSectOn);
//      Serial.println(" , ");

  }
}
