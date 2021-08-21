#include "heltec.h"

#define NUMERO_TICKS_ESPERA 3 //qtde de ticks que a tarefa vai esperar para conseguir o semaforo

byte localAddress = 0xBB;
byte serverAddress = 0xFF;

typedef struct // telemetryService
{
    SemaphoreHandle_t mutex;
    String outBuffer;
    String inBuffer;
    byte msgCount;
} telemetryServiceStruct;

typedef struct // currentSensor
{
    SemaphoreHandle_t mutex;
    double leitura;
    double samples[100];
    int index;
} currentSensorStruct;

typedef struct // motorConfigStruct
{
    bool status;
    long current;

} motorConfigStruct;

typedef struct // electricalRelay
{
    SemaphoreHandle_t mutex;
    bool status;
} electricalRelayStruct;

typedef struct // electricalMotorStruct
{
    motorConfigStruct config;
} electricalMotorStruct;

currentSensorStruct currentSensor;
electricalRelayStruct electricalRelay;
telemetryServiceStruct telemetryService;
electricalMotorStruct electricalMotor;

/*
long lastSendTime = 0;  // last send time
int interval = 2000;    // interval between sends
*/

void currentSensorReader(void * parameters)
{
    for(;;)
    {
        if(xSemaphoreTake(currentSensor.mutex, portMAX_DELAY) == pdTRUE)
        {
            if(currentSensor.index == 100)
            {
                currentSensor.index = 0;

                double maior = currentSensor.samples[0], menor = currentSensor.samples[0];
                for(int i=1; i < 100; i++)
                {
                    if(currentSensor.samples[i] > maior)
                        maior = currentSensor.samples[i];

                    if(currentSensor.samples[i] < menor)
                        menor = currentSensor.samples[i];
                }

                if(electricalRelay.status)
                {
                    currentSensor.leitura += sqrt((maior - menor)/2);
                }
                else
                {
                    if(currentSensor.leitura > 0)
                    {
                        currentSensor.leitura -= sqrt((maior - menor)/2);

                        if(currentSensor.leitura < 0)
                            currentSensor.leitura = 0;
                    }
                }

                // teste //
                //Serial.println("[currentSensorReader] Read: " + (String)currentSensor.leitura);
            }
            else
            {
                currentSensor.samples[currentSensor.index] = ( random(1, 60) / (double)100 ) * sin(((currentSensor.index)/(double)1000)*(2*3.1415));
                currentSensor.index++;
            }
            xSemaphoreGive(currentSensor.mutex);
        }
        vTaskDelay(1/ portTICK_PERIOD_MS); // 1ms
    }
}

void telemetryInfoSender(void * parameters)
{
    for(;;)
    {
        if(xSemaphoreTake(currentSensor.mutex, portMAX_DELAY) == pdTRUE)
        {
            telemetryService.outBuffer = (String)currentSensor.leitura;

            xSemaphoreGive(currentSensor.mutex);
        }

        if(xSemaphoreTake(telemetryService.mutex, portMAX_DELAY) == pdTRUE)
        {
            LoRa.beginPacket();                                 // start packet
            LoRa.write(serverAddress);                          // add serverAddress address
            LoRa.write(localAddress);                           // add sender address
            LoRa.write(telemetryService.msgCount);              // add message ID
            LoRa.write(telemetryService.outBuffer.length());    // add payload length
            LoRa.print(telemetryService.outBuffer);             // add payload
            LoRa.endPacket();                                   // finish packet and send it
            telemetryService.msgCount++;                        // increment message ID

            LoRa.receive(); // coloca o LoRa em modo listening novamente

            Serial.println("[telemetrySender] Sent: " + telemetryService.outBuffer);

            xSemaphoreGive(telemetryService.mutex);
        }
        vTaskDelay(1000/ portTICK_PERIOD_MS); // ~1500ms // 1.5s
    }
}

void telemetryActionReceiver(int packetSize)
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
            Serial.println("[actionReceiver] ERROR: message length does not match length.");
            return;
        }

        // if the recipient isn't this device or broadcast,
        if (recipient != localAddress)
        {
            Serial.println("[actionReceiver] ERROR: this message is not for me.");
            return;
        }

        /*
        Serial.print("[RECEIVED] From 0x" + String(sender, HEX));
        Serial.println(" to 0x" + String(recipient, HEX));
        Serial.println(" - ID: " + String(sender));
        Serial.println(" : " + telemetryService.inBuffer);
         */

        xSemaphoreGive(telemetryService.mutex);
    }

    if(xSemaphoreTake(electricalRelay.mutex, portMAX_DELAY) == pdTRUE)
    {
        motorConfigStruct newConfig;

        if(telemetryService.inBuffer.equals("1")) // aguardar
        {
            printf("[actionReceiver] Received: wait.\n");
        }
        else if(telemetryService.inBuffer.equals("2")) // ligar (sem limite)
        {
            newConfig.status = true;
            newConfig.current = -1;
            electricalMotor.config = newConfig;
            printf("[actionReceiver] Received: turn on.\n");
        }
        else if(telemetryService.inBuffer.equals("3")) // manter
        {
            newConfig.status = true;
            newConfig.current = 5; // receber valor por socket
            electricalMotor.config = newConfig;
            printf("[actionReceiver] Received: keep on %dA.\n", 5);
        }
        else if(telemetryService.inBuffer.equals("0")) // desligar
        {
            newConfig.status = false;
            newConfig.current = 0;
            electricalMotor.config = newConfig;
            printf("[actionReceiver] Received: turn off.\n");
        }
        else
            printf("[actionReceiver] Received: unknown.\n");

        xSemaphoreGive(electricalRelay.mutex);
    }
}

void electricalRelayControl(void * parameters) // criar thread que verifica se o valor atual é diferente etc
{
    for(;;)
    {
        if(xSemaphoreTake(electricalRelay.mutex, portMAX_DELAY) == pdTRUE)
        {

            if(electricalMotor.config.status && (electricalMotor.config.current > currentSensor.leitura || electricalMotor.config.current == -1))
                electricalRelay.status = true;
            else
                electricalRelay.status = false;

            xSemaphoreGive(electricalRelay.mutex);
        }
        vTaskDelay(100/ portTICK_PERIOD_MS); // 100ms
    }
}
/*void task2(void * parameters){
    for(;;){
        if( xSemaphoreTake( Mutex_teste, ( TickType_t ) NUMERO_TICKS_ESPERA ) == pdTRUE){
            //se nao estourar NUMERO_TICKS_ESPERA, conseguiu o semaforo

            varTest = varTest - 1;

            //ja utilizei a variavel compartilhada, então libero o semaforo
            xSemaphoreGive( Mutex_teste );
        }
        vTaskDelay(150/ portTICK_PERIOD_MS);
    }
}
*/

void setup()
{
    Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, 915E6 /*long BAND*/);

    Serial.println("[MAIN] Launching Heltec.LoRa service.");
    LoRa.onReceive(telemetryActionReceiver);
    LoRa.receive();
    Serial.println("[MAIN] Heltec.LoRa init succeeded.");

    //
    telemetryService.mutex = xSemaphoreCreateMutex();
    telemetryService.outBuffer = "";
    telemetryService.inBuffer = "";
    telemetryService.msgCount = 0;
    //
    currentSensor.mutex = xSemaphoreCreateMutex();
    currentSensor.index    = 0;
    currentSensor.leitura    = 0;
    //
    electricalMotor.config.status = false;
    electricalMotor.config.current = 0;
    //
    electricalRelay.mutex = xSemaphoreCreateMutex();
    electricalRelay.status = false;
    //

    // exemplo //
    motorConfigStruct newConfig;
    newConfig.status = true;
    newConfig.current = 5;
    electricalMotor.config = newConfig;
    //

    Serial.println("[MAIN] Launching currentSensorReader thread.");
    xTaskCreate(currentSensorReader,"currentSensorReader", 2000, NULL, 2, NULL); // prioridade 2

    Serial.println("[MAIN] Launching telemetryInfoSender thread.");
    xTaskCreate(telemetryInfoSender,"telemetryInfoSender", 2000, NULL, 1, NULL); // prioridade 1

    Serial.println("[MAIN] Launching electricalRelayControl thread.");
    xTaskCreate(electricalRelayControl,"electricalRelayControl", 2000, NULL, 3, NULL); // prioridade 3

    Serial.println("[MAIN] Waiting for server command.");

    /*xTaskCreate(task1,"Teste 1", 2000, NULL, 2, NULL);
    xTaskCreate(task2,"Teste 2", 1000, NULL, 1, NULL);*/
}

void loop()
{
  /*if (millis() - lastSendTime > interval)
  {
    varTest = varTest + 10;

    sendMessage((String)varTest);
    Serial.println("Sending ");
    lastSendTime = millis();            // timestamp the message
    interval = 2000;     // 2-3 seconds
    LoRa.receive();                     // go back into receive mode
  }*/
}
