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

#define SD_CS 23
#define SD_SCK 17
#define SD_MOSI 12
#define SD_MISO 13

File MicroSDcard;

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
    String buffer;
    double samples[10];
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

        if(xSemaphoreTake(dataStorage.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            dataStorage.samples[++dataStorage.index] = telemetryService.inBuffer;

            xSemaphoreGive(dataStorage.mutex);
        }

        if (incomingLength != telemetryService.inBuffer.length())
        {
            Serial.println("[telemetryReceiver] ERROR: message lengths doesn't match.");
            goto libera;
        }

        if (destination != localAddress)
        {
            Serial.println("[telemetryReceiver] ERROR: this message isn't for me.");
            goto libera;
        }

        Serial.println("[telemetryReceiver] Received: " + telemetryService.inBuffer + "A");

        libera:
        xSemaphoreGive(telemetryService.mutex);
        
        LoRa.receive(); // coloca o LoRa em modo listening novamente
    }
}

void telemetryActionSender(void * parameters)
{
    for(;;)
    {
        if( xSemaphoreTake(telemetryService.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            if(telemetryService.inBuffer != "-1") // Verifica se existe leitura para processar
            {
                if(serialService.inBuffer.equals(""))
                {
                    telemetryService.outBuffer = "0;0";
                    Serial.println("[actionSender] Sending: OK");
                }
                else
                {
                    /// Dividindo as informações da string em variáveis
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

                    if(tmp[0].equals("1")) // aguardar
                    {
                        telemetryService.outBuffer = serialService.inBuffer;
                        Serial.println("[actionSender] Sending: wait");
                    }
                    else if(tmp[0].equals("2")) // ligar (sem limite definido)
                    {
                        telemetryService.outBuffer = serialService.inBuffer;
                        Serial.println("[actionSender] Sending: turn on");
                    }
                    else if(tmp[0].equals("3")) // manter em xA
                    {
                        telemetryService.outBuffer = serialService.inBuffer;
                        Serial.println("[actionSender] Sending: keep on " + tmp[1] + "A");
                    }
                    else if(tmp[0].equals("4")) // desligar
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
                LoRa.write(telemetryService.outBuffer.length());    // add payload length
                LoRa.print(telemetryService.outBuffer);             // add payload
                LoRa.endPacket();                                   // finish packet and send it

                telemetryService.inBuffer = "-1"; // sinaliza que a leitura foi processada

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
void dataLogging(void * parameters)
{
    for(;;)
    {
        if(xSemaphoreTake(dataStorage.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            while(dataStorage.index >= 0) // Armazena todas as leituras disponíveis em uma única string
            {
                dataStorage.buffer += dataStorage.samples[dataStorage.index--] + ", ";
            }
            if(!dataStorage.buffer.equals(""))
            {
                #if 0
                    // Abre o arquivo para adicionar algo
                    MicroSDcard = SD.open("/data_sensor.txt", FILE_APPEND);
                    // Adiciona algo
                    myFile.print(dataStorage.buffer);
                    // Fecha o arquivo
                    myFile.close();
                #endif

                #if 1
                    for(int i=0; i<1000; i++){ } // Simula escrita no cartão SD
                #endif

                dataStorage.buffer = ""; // reseta buffer
            }
            xSemaphoreGive(dataStorage.mutex);
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS); // 1000ms
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

    Serial.println("[MAIN] Mounting MicroSD card.");
    SPIClass sd_spi(HSPI);
    sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sd_spi))
        Serial.println("[MAIN] MicroSD card mounting failed.");
    else
        Serial.println("[MAIN] MicroSD card mounting succeeded.");

    telemetryService.mutex = xSemaphoreCreateMutex();
    telemetryService.outBuffer = "1"; // iniciar com wait
    telemetryService.inBuffer = "";

    serialService.mutex = xSemaphoreCreateMutex();
    serialService.inBuffer = "";

    dataStorage.mutex = xSemaphoreCreateMutex();
    dataStorage.buffer = "";
    dataStorage.index = 0;

    Serial.println("[MAIN] Launching telemetryActionSender thread.");
    xTaskCreate(telemetryActionSender,"telemetryActionSender", 2000, NULL, 3, NULL); // prioridade 3

    Serial.println("[MAIN] Launching serialReader thread.");
    xTaskCreate(serialReader,"serialReader", 2000, NULL, 1, NULL); // prioridade 1

    /// Cria a task para salvar os dados no Cartao de Memoria
    Serial.println("[MAIN] Launching dataLogging thread.");
    xTaskCreate(dataLogging,"dataLogging", 2000, NULL, 2, NULL); // prioridade 2

    Serial.println("[MAIN] Waiting for client data.");
}
/// Função utilizada normalmente quando o propósito do código não é RTOS.
void loop()   
{  
  /* */
}  
