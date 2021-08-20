#include "heltec.h"

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6

  
byte server_Address = 0xFF;     // address of this device
byte client_Address = 0xBB;      // destination to send to

byte localAddress = server_Address;
byte destination = client_Address;

String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

void setup()
{
   //WIFI Kit series V1 not support Vext control
  Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("Heltec.LoRa init succeeded.");
}

void loop()
{
  /*if (millis() - lastSendTime > interval)
  {
    String message = "Servidor: " + (String)random(0, 100);   // send a message
    sendMessage(message);
    Serial.println("[SENDING] " + message);
    Serial.println("");
    lastSendTime = millis();            // timestamp the message
    interval = random(2000) + 1000;     // 2-3 seconds
    LoRa.receive();                     // go back into receive mode
  }*/
}

void sendMessage(String outgoing)
{
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize)
{
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";                 // payload of packet

  while (LoRa.available())             // can't use readString() in callback
  {
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLength != incoming.length())   // check length for error
  {
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF)
  {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.print("[RECEIVED] From 0x" + String(sender, HEX));
  Serial.println(" to 0x" + String(recipient, HEX));
  Serial.println(" - ID: " + String(incomingMsgId));
  Serial.println(" - Length " + String(incomingLength));
  Serial.println(" : " + incoming);
  Serial.println(" - RSSI: " + String(LoRa.packetRssi()));
  //Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}
