/**
* @file lora_server.ino
* @brief Código do Servidor
* @author by Cléber Werlang, Cristian Wülfing
* @link https://github.com/CristianAugusto/ELC1048_FinalProject
* @date 08-2021
*/
#include "heltec.h"

/// MACRO de tempo de espera para tentar novamente o semáforo
#define WAIT_TICKS 3

byte localAddress = 0xFF;
byte clientAddress = 0xBB;

int aux = 0;

/// telemetryService
typedef struct
{
    SemaphoreHandle_t mutex;
    String outBuffer;
    String inBuffer;
    byte msgCount;
} telemetryServiceStruct;

/// dataStorage
typedef struct
{
    SemaphoreHandle_t mutex;
    double leitura;
    double samples[100];
    int index;
} dataStorageStruct;

telemetryServiceStruct telemetryService;
dataStorageStruct dataStorage;

void telemetryInfoReceiver(int packetSize)
{
    if (packetSize == 0)
        return;

    if(xSemaphoreTake(telemetryService.mutex, portMAX_DELAY) == pdTRUE)
    {
        telemetryService.inBuffer = "";
        int recipient = LoRa.read();
        //byte sender = LoRa.read();

        while (LoRa.available())
            telemetryService.inBuffer += (char)LoRa.read();

        if (LoRa.read() != telemetryService.inBuffer.length())
        {
            Serial.println("[telemetryReceiver] ERROR: message lengths doesn't match.");
            return;
        }

        // if the recipient isn't this device or broadcast,
        if (recipient != localAddress)
        {
            Serial.println("[telemetryReceiver] ERROR: this message isn't for me.");
            return;
        }

        Serial.println("[telemetryReceiver] Received: " + telemetryService.inBuffer);

        /*
        Serial.print("[RECEIVED] From 0x" + String(sender, HEX));
        Serial.println(" to 0x" + String(recipient, HEX));
        Serial.println(" - ID: " + String(sender));
        Serial.println(" : " + telemetryService.inBuffer);
         */

        // enviar comando como retorno

        // aumentar velocidade  -> status = 1, speed = x
        // diminuir velocidade  -> status = 1, speed = x
        // desligar             -> status = 0, speed = 0

        /*if(xSemaphoreTake(currentSensor.mutex, portMAX_DELAY) == pdTRUE)
        {
            telemetryService.outBuffer = (String)currentSensor.leitura;

            xSemaphoreGive(currentSensor.mutex);
        }*/

        aux++;
        if(aux == 10)
        {
            telemetryService.outBuffer = "1";
            printf("[actionSender] Sent: wait.\n");
        }
        else if(aux == 20)
        {
            telemetryService.outBuffer = "2";
            printf("[actionSender] Sent: turn on.\n");
        }
        else if(aux == 30)
        {
            telemetryService.outBuffer = "3";
            printf("[actionSender] Sent: keep on %dA.\n", 5);
        }
        else if(aux == 40)
        {
            telemetryService.outBuffer = "0";
            printf("[actionSender] Sent: turn off.\n");
            aux=0;
        }

        LoRa.beginPacket();                                 // start packet
        LoRa.write(clientAddress);                          // add clientAddress address
        LoRa.write(localAddress);                           // add sender address
        LoRa.write(telemetryService.msgCount);              // add message ID
        LoRa.write(telemetryService.outBuffer.length());    // add payload length
        LoRa.print(telemetryService.outBuffer);             // add payload
        LoRa.endPacket();                                   // finish packet and send it
        telemetryService.msgCount++;                        // increment message ID

        LoRa.receive(); // coloca o LoRa em modo listening novamente

        Serial.println("[actionSender] Sent: " + telemetryService.outBuffer);

        xSemaphoreGive(telemetryService.mutex);
    }
}

/// Função que executa apenas uma vez e sempre que o microcontrolador é ligado.
void setup()
{
    Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, 915E6 /*long BAND*/);

    Serial.println("[MAIN] Launching Heltec.LoRa service.");
    LoRa.onReceive(telemetryInfoReceiver);
    LoRa.receive();
    Serial.println("[MAIN] Heltec.LoRa init succeeded.");

    //
    telemetryService.mutex = xSemaphoreCreateMutex();
    telemetryService.outBuffer = "";
    telemetryService.inBuffer = "";
    telemetryService.msgCount = 0;
    //

    Serial.println("[MAIN] Launching dataStorager thread.");
    xTaskCreate(currentSensorReader,"dataStorager", 2000, NULL, 1, NULL); // prioridade 2

    Serial.println("[MAIN] Waiting for client data.");
}
/// Função utilizada normalmente quando o propósito do código não é RTOS.
void loop()
{
    /* */
}
