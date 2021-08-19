#include "heltec.h"

#define BAND        915E6    //you can set band here directly,e.g. 868E6,915E6

#define NUMERO_TICKS_ESPERA 3 //qtde de ticks que a tarefa vai esperar para conseguir o semaforo

byte server_Address = 0xFF;         // address of this device
byte client_Address = 0xBB;            // destination to send to

byte localAddress = client_Address;
byte destination = server_Address;

SemaphoreHandle_t Mutex_teste;

String outgoing;                            // outgoing message
byte msgCount = 0;                        // count of outgoing messages
long lastSendTime = 0;                // last send time
int interval = 2000;                    // interval between sends

int varTest = 0;

void sendMessage(String outgoing)
{
    LoRa.beginPacket();                                     // start packet
    LoRa.write(destination);                            // add destination address
    LoRa.write(localAddress);                         // add sender address
    LoRa.write(msgCount);                                 // add message ID
    LoRa.write(outgoing.length());                // add payload length
    LoRa.print(outgoing);                                 // add payload
    LoRa.endPacket();                                         // finish packet and send it
    msgCount++;                                                     // increment message ID
}

void onReceive(int packetSize)
{
    if (packetSize == 0) return;                    // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();                    // recipient address
    byte sender = LoRa.read();                        // sender address
    byte incomingMsgId = LoRa.read();         // incoming msg ID
    byte incomingLength = LoRa.read();        // incoming msg length

    String incoming = "";                                 // payload of packet

    while (LoRa.available())                         // can't use readString() in callback
    {
        incoming += (char)LoRa.read();            // add bytes one by one
    }

    if (incomingLength != incoming.length())     // check length for error
    {
        Serial.println("error: message length does not match length");
        return;                                                         // skip rest of function
    }

    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xFF)
    {
        Serial.println("This message is not for me.");
        return;                                                         // skip rest of function
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

void task1(void * parameters){
    for(;;){
        if( xSemaphoreTake( Mutex_teste, ( TickType_t ) NUMERO_TICKS_ESPERA ) == pdTRUE){
            /*se nao estourar NUMERO_TICKS_ESPERA, conseguiu o semaforo*/

            varTest = varTest + 10;

            String message = "Cliente 1: " + (String)varTest;     // send a message
            sendMessage(message);
            Serial.println("[SENDING 1] " + message);
            LoRa.receive();                                         // go back into receive mode

            /*ja utilizei a variavel compartilhada, então libero o semaforo*/
            xSemaphoreGive( Mutex_teste );
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS);
    }
}

void task2(void * parameters){
    for(;;){
        if( xSemaphoreTake( Mutex_teste, ( TickType_t ) NUMERO_TICKS_ESPERA ) == pdTRUE){
            /*se nao estourar NUMERO_TICKS_ESPERA, conseguiu o semaforo*/

            varTest = varTest - 1;

            /*ja utilizei a variavel compartilhada, então libero o semaforo*/
            xSemaphoreGive( Mutex_teste );
        }
        vTaskDelay(100/ portTICK_PERIOD_MS);
    }
}

void setup()
{
    Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

    LoRa.onReceive(onReceive);
    LoRa.receive();
    Serial.println("Heltec.LoRa init succeeded.");

    Mutex_teste = xSemaphoreCreateMutex();  // Criacao do semaforo de leitura

    xTaskCreate(task1,"Teste 1", 5000, NULL, 1, NULL);
    xTaskCreate(task2,"Teste 2", 1000, NULL, 2, NULL);
}

void loop()
{
    /**/
}
