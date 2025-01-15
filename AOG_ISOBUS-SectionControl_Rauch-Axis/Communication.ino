//-------------------------------------------------------------------------------------------------

void read_Eth() {
  unsigned int packetLength;
  //read Data
  packetLength = EthUDPFromAOG.parsePacket();
  //Serial.println(" Hallo 1 ");
  if (packetLength > 0) {
    //Serial.println(packetLength);
    EthUDPFromAOG.read(ReplyBufferSC, packetLength);
  }
}

//-------------------------------------------------------------------------------------------------

void send_Eth() {

  int16_t CK_A = 0;
  for (uint8_t i = 2; i < PGN_237_Size; i++)
  {
    CK_A = (CK_A + PGN_237[i]);
  }
  PGN_237[PGN_237_Size] = CK_A;

  EthUDPToAOG.beginPacket(Eth_ipDestination, portDestination);
  EthUDPToAOG.write(PGN_237, sizeof(PGN_237));
  EthUDPToAOG.endPacket();
  
} //Hallo 1

//-------------------------------------------------------------------------------------------------


void send_Eth234() {

  int16_t CK_A = 0;
  for (uint8_t i = 2; i < PGN_234_Size; i++)
  {
    CK_A = (CK_A + PGN_234[i]);
  }
  PGN_234[PGN_234_Size] = CK_A;

  EthUDPToAOG.beginPacket(Eth_ipDestination, portDestination);
  EthUDPToAOG.write(PGN_234, sizeof(PGN_234));
  EthUDPToAOG.endPacket();
  
} //Hallo 1

//--------------------------------------------------------------------------------------------------
