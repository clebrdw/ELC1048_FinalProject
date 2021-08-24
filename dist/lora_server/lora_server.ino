/**
* @file lora_server.ino
* @brief Código do Servidor
* @author by Cléber Werlang, Cristian Wülfing
* @link https://github.com/CristianAugusto/ELC1048_FinalProject
* @date 08-2021
*/

#include "heltec.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

/// MACRO de tempo de espera para tentar novamente o semáforo
#define WAIT_TICKS 3

byte localAddress = 0xFF;
byte clientAddress = 0xBB;

/// telemetryService
typedef struct
{
    SemaphoreHandle_t mutex;
    String outBuffer;
    String inBuffer;
} telemetryServiceStruct;

/// serialService
typedef struct
{
    SemaphoreHandle_t mutex;
    String inBuffer;
} serialServiceStruct;

/// dataStorage
typedef struct
{
    SemaphoreHandle_t mutex;
    double leitura;
    double samples[100];
    int index;
} dataStorageStruct;

telemetryServiceStruct telemetryService;
serialServiceStruct serialService;
dataStorageStruct dataStorage;

void telemetryInfoReceiver(int packetSize)
{
    if (packetSize == 0)
        return;

    if(xSemaphoreTake(telemetryService.mutex, portMAX_DELAY) == pdTRUE)
    {
        
        // read packet header bytes:
        int destination = LoRa.read();          // destination address
        byte incomingLength = LoRa.read();    // incoming msg length

        telemetryService.inBuffer = "";
        while (LoRa.available())
            telemetryService.inBuffer += (char)LoRa.read();
        
        if (incomingLength != telemetryService.inBuffer.length())
        {
            Serial.println("[telemetryReceiver] ERROR: message lengths doesn't match.");
            goto libera;
        }

        // if the destination isn't this device or broadcast,
        if (destination != localAddress)
        {
            Serial.println("[telemetryReceiver] ERROR: this message isn't for me.");
            goto libera;
        }

        Serial.println("[telemetryReceiver] Received: " + telemetryService.inBuffer + "A");

        libera:
        xSemaphoreGive(telemetryService.mutex);
        
        LoRa.receive();
    }
}

void telemetryActionSender(void * parameters)
{
    for(;;)
    {
        if( xSemaphoreTake(telemetryService.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            if(telemetryService.inBuffer != "-1")
            {
                // enviar comando como retorno
        
                // aumentar velocidade  -> status = 1, speed = x
                // diminuir velocidade  -> status = 1, speed = x
                // desligar             -> status = 0, speed = 0
        
                /*if(xSemaphoreTake(currentSensor.mutex, portMAX_DELAY) == pdTRUE)
                {
                    telemetryService.outBuffer = (String)currentSensor.leitura;
        
                    xSemaphoreGive(currentSensor.mutex);
                }*/

                //telemetryService.inBuffer
                if(serialService.inBuffer.equals(""))
                {
                    telemetryService.outBuffer = "0;0";
                    Serial.println("[actionSender] Sending: OK");
                }
                else
                {
                    char buf[sizeof(telemetryService.inBuffer)];
                    String tmp[2];
                    int i = 0;
                    serialService.inBuffer.toCharArray(buf, sizeof(buf));
                    char *p = buf;
                    char *str;
                    while ((str = strtok_r(p, ";", &p)) != NULL)
                    {
                        tmp[i++] = str;
                    }

                    if(tmp[0].equals("1"))
                    {
                        telemetryService.outBuffer = serialService.inBuffer;
                        Serial.println("[actionSender] Sending: wait");
                    }
                    else if(tmp[0].equals("2"))
                    {
                        telemetryService.outBuffer = serialService.inBuffer;
                        Serial.println("[actionSender] Sending: turn on");
                    }
                    else if(tmp[0].equals("3"))
                    {
                        telemetryService.outBuffer = serialService.inBuffer;
                        Serial.println("[actionSender] Sending: keep on " + tmp[1] + "A");
                    }
                    else if(tmp[0].equals("4"))
                    {
                        telemetryService.outBuffer = serialService.inBuffer;
                        Serial.println("[actionSender] Sending: turn off");
                    }
                    else
                    {
                        Serial.println("[SERIAL] Command not found");
                    }
                    serialService.inBuffer = "";
                }
        
                LoRa.beginPacket();                                 // start packet
                LoRa.write(clientAddress);                          // add clientAddress address
                LoRa.write(telemetryService.outBuffer.length());     // add payload length
                LoRa.print(telemetryService.outBuffer);              // add payload
                LoRa.endPacket();                                   // finish packet and send it

                telemetryService.inBuffer = "-1";

                Serial.println("[actionSender] Command sent successfully.");

                Serial.println("");
                 
                LoRa.receive(); // coloca o LoRa em modo listening novamente
            }

            xSemaphoreGive(telemetryService.mutex);
        }
        vTaskDelay(100/ portTICK_PERIOD_MS); // 100ms
    }
}

void serialReader(void * parameters)
{
    for(;;)
    {
        if( xSemaphoreTake(serialService.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            if(Serial.available() > 0)
            {
                serialService.inBuffer = Serial.readString();
                Serial.println("[SERIAL] Comando recebido: " + serialService.inBuffer);
            }
            xSemaphoreGive(serialService.mutex);
        }
        vTaskDelay(10/ portTICK_PERIOD_MS); // 10ms
    }
}
/// Função que armazena os dados recebidos no Cartão de Memória
void salvaDados(void * parameters)
{
    for(;;)
    {
        if( xSemaphoreTake(telemetryService.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            
            // Declara variavel do tipo File
            File myFile;
            // Abre o arquivo para adicionar algo
            myFile = SD.open("/data_sensor.txt", FILE_APPEND);
            // Adiciona algo
            myFile.print((String)telemetryService.inBuffer);
            // Fecha o arquivo
            myFile.close();

            xSemaphoreGive(telemetryService.mutex);
        }
        vTaskDelay(10/ portTICK_PERIOD_MS); // 10ms
    }
}

/// Função que executa apenas uma vez e sempre que o microcontrolador é ligado.
void setup()
{
    Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, false /*PABOOST Enable*/, 915E6 /*long BAND*/);

    Serial.println("[MAIN] Launching Heltec.LoRa service.");
    LoRa.onReceive(telemetryInfoReceiver);
    LoRa.receive();
    Serial.println("[MAIN] Heltec.LoRa init succeeded.");


    /*if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    // Declara variavel do tipo File
    File myFile;
    // Cria o arquivo data_sensor.txt
    myFile = SD.open("/data_sensor.txt", FILE_WRITE);
    // Fecha o arquivo
    myFile.close();*/


    //
    telemetryService.mutex = xSemaphoreCreateMutex();
    telemetryService.outBuffer = "1"; // iniciar com wait
    telemetryService.inBuffer = "";
    //
    Serial.println("[MAIN] Launching telemetryActionSender thread.");
    xTaskCreate(telemetryActionSender,"telemetryActionSender", 2000, NULL, 1, NULL); // prioridade 2
    //

    //
    serialService.mutex = xSemaphoreCreateMutex();
    serialService.inBuffer = "";
    //
    Serial.println("[MAIN] Launching serialReader thread.");
    xTaskCreate(serialReader,"serialReader", 2000, NULL, 1, NULL); // prioridade 2
    //

    /*// Cria a task para salvar os dados no Cartao de Memoria
    xTaskCreate(salvaDados,"salvaDados", 2000, NULL, 3, NULL); // prioridade 3*/

    /*Serial.println("[MAIN] Launching dataStorager thread.");
    xTaskCreate(currentSensorReader,"dataStorager", 2000, NULL, 1, NULL); // prioridade 2*/

    Serial.println("[MAIN] Waiting for client data.");
}
/// Função utilizada normalmente quando o propósito do código não é RTOS.
void loop()   
{  
  /* */
}  
