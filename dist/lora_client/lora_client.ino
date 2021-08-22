/**
* @file lora_client.ino
* @brief Código do Cliente
* @author by Cléber Werlang, Cristian Wülfing
* @link https://github.com/CristianAugusto/ELC1048_FinalProject
* @date 08-2021
*/
#include "heltec.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 2,1,0,4,5,6,7,3, POSITIVE); 

/// MACRO de tempo de espera para tentar novamente o semáforo
#define WAIT_TICKS 3

#define LED_PIN 32

byte localAddress = 0xBB;
byte serverAddress = 0xFF;

/// telemetryService
typedef struct 
{
    SemaphoreHandle_t mutex;
    String outBuffer;
    String inBuffer;
    byte msgCount;
} telemetryServiceStruct;

/// currentSensor
typedef struct 
{
    SemaphoreHandle_t mutex;
    double leitura;
    double samples[10];
    int index;
} currentSensorStruct;

/// motorConfigStruct
typedef struct 
{
    bool status;
    long current;

} motorConfigStruct;

/// electricalRelay
typedef struct 
{
    SemaphoreHandle_t mutex;
    bool status;
} electricalRelayStruct;

/// electricalMotorStruct
typedef struct 
{
    motorConfigStruct config;
} electricalMotorStruct;

currentSensorStruct currentSensor;
electricalRelayStruct electricalRelay;
telemetryServiceStruct telemetryService;
electricalMotorStruct electricalMotor;

/// Função que simula a leitura de corrente
void currentSensorReader(void * parameters)
{
    for(;;)
    {
        /*if( xSemaphoreTake( Mutex_teste, ( TickType_t ) WAIT_TICKS ) == pdTRUE)*/
        if(xSemaphoreTake(currentSensor.mutex, portMAX_DELAY) == pdTRUE)
        {
            if(currentSensor.index == 10)
            {
                currentSensor.index = 0;

                double maior = currentSensor.samples[0], menor = currentSensor.samples[0];
                for(int i=1; i < 10; i++)
                {
                    if(currentSensor.samples[i] > maior)
                        maior = currentSensor.samples[i];

                    if(currentSensor.samples[i] < menor)
                        menor = currentSensor.samples[i];
                }

                double sinal = sqrt((maior - menor)/2);

                if(electricalRelay.status)
                {
                    if(currentSensor.leitura > 15) // corrente máxima
                        currentSensor.leitura -= sinal;
                    else
                        currentSensor.leitura += sinal;
                }
                else
                {
                    if(currentSensor.leitura > 0)
                    {
                        if(currentSensor.leitura < 0) // corrente mínima
                            currentSensor.leitura = 0;
                        else
                            currentSensor.leitura -= sinal;
                    }
                }

                //Serial.println("[currentSensorReader] Read: " + (String)currentSensor.leitura);
            }
            else
            {
                currentSensor.samples[currentSensor.index] = ( random(1, 60) / (double)100 ) * sin(((currentSensor.index)/(double)1000)*(2*3.1415));
                currentSensor.index++;
            }
            xSemaphoreGive(currentSensor.mutex);
        }
        vTaskDelay(10/ portTICK_PERIOD_MS); // 100ms
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

            Serial.println("[telemetrySender] Sent: " + telemetryService.outBuffer + "A");

            xSemaphoreGive(telemetryService.mutex);
        }
        vTaskDelay(1500/ portTICK_PERIOD_MS); // ~2000ms // 2s ->(1.5seg + 0.5seg de atraso LoRa) 
    }
}

void telemetryActionReceiver(int packetSize)
{
    if (packetSize == 0)
        return;

    if(xSemaphoreTake(telemetryService.mutex, portMAX_DELAY) == pdTRUE)
    {

        // read packet header bytes:
        int recipient = LoRa.read();          // recipient address
        byte sender = LoRa.read();            // sender address
        byte incomingMsgId = LoRa.read();     // incoming msg ID
        byte incomingLength = LoRa.read();    // incoming msg length

        telemetryService.inBuffer = "";
        while (LoRa.available())
            telemetryService.inBuffer += (char)LoRa.read();
        
        if (incomingLength != telemetryService.inBuffer.length())
        {
            Serial.println("[telemetryReceiver] ERROR: message lengths doesn't match.");
            goto libera;
        }

        // if the recipient isn't this device or broadcast,
        if (recipient != localAddress)
        {
            Serial.println("[telemetryReceiver] ERROR: this message isn't for me.");
            goto libera;
        }

        /*
        Serial.print("[RECEIVED] From 0x" + String(sender, HEX));
        Serial.println(" to 0x" + String(recipient, HEX));
        Serial.println(" - ID: " + String(incomingMsgId));
        Serial.println(" : " + telemetryService.inBuffer);
         */

        libera: // verificar outros possíveis casos
        xSemaphoreGive(telemetryService.mutex);
    }

    char buf[sizeof(telemetryService.inBuffer)];
    
    String tmp[2];
    int i = 0;
    
    telemetryService.inBuffer.toCharArray(buf, sizeof(buf));
    char *p = buf;
    char *str;
    while ((str = strtok_r(p, ";", &p)) != NULL)
    {
        tmp[i++] = str;
    }

    //Serial.println("[actionReceiver] Received: '" + telemetryService.inBuffer);

    if(xSemaphoreTake(electricalRelay.mutex, portMAX_DELAY) == pdTRUE)
    {
        motorConfigStruct newConfig;

        if(tmp[0].equals("1")) // aguardar
        {
            Serial.println("[actionReceiver] Received: wait");
        }
        else if(tmp[0].equals("2")) // ligar (sem limite)
        {
            newConfig.status = true;
            newConfig.current = -1;
            electricalMotor.config = newConfig;
            Serial.println("[actionReceiver] Received: turn on");
        }
        else if(tmp[0].equals("3")) // manter
        {
            newConfig.status = true;
            newConfig.current = tmp[1].toInt(); // receber valor por socket
            electricalMotor.config = newConfig;
            Serial.println("[actionReceiver] Received: keep on " + tmp[1] + "A");
        }
        else if(tmp[0].equals("4")) // desligar
        {
            newConfig.status = false;
            newConfig.current = 0;
            electricalMotor.config = newConfig;
            Serial.println("[actionReceiver] Received: turn off");
        }
        else if(tmp[0].equals("0")) // OK
        {
            Serial.println("[actionReceiver] Received: OK");
        }
        else
            Serial.println("[actionReceiver] Received: unknown");
            
        Serial.println("");

        xSemaphoreGive(electricalRelay.mutex);
    }
}
/// Função que cria uma thread que verifica se o valor atual é diferente etc
void electricalRelayControl(void * parameters) 
{
    for(;;)
    {
        if(xSemaphoreTake(electricalRelay.mutex, portMAX_DELAY) == pdTRUE)
        {

            if(electricalMotor.config.status && (electricalMotor.config.current > currentSensor.leitura || electricalMotor.config.current == -1))
            {
                electricalRelay.status = true;
                digitalWrite(LED_PIN, HIGH);
            }
            else
            {
                electricalRelay.status = false;
                digitalWrite(LED_PIN, LOW);
            }

            xSemaphoreGive(electricalRelay.mutex);
        }
        vTaskDelay(100/ portTICK_PERIOD_MS); // 100ms
    }
}

/// Função que executa apenas uma vez e sempre que o microcontrolador é ligado.
void setup()
{
    Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, 915E6 /*long BAND*/);

    lcd.begin(16,2);

    pinMode(LED_PIN, OUTPUT); // led

    Serial.println("[MAIN] Launching Heltec.LoRa service.");
    LoRa.onReceive(telemetryActionReceiver);
    LoRa.receive();
    Serial.println("[MAIN] Heltec.LoRa init succeeded.");

    //
    telemetryService.mutex      = xSemaphoreCreateMutex();
    telemetryService.outBuffer  = "";
    telemetryService.inBuffer   = "";
    telemetryService.msgCount   = 0;
    //
    currentSensor.mutex     = xSemaphoreCreateMutex();
    currentSensor.index     = 0;
    currentSensor.leitura   = 0;
    //
    electricalMotor.config.status = false;
    electricalMotor.config.current = 0;
    //
    electricalRelay.mutex = xSemaphoreCreateMutex();
    electricalRelay.status = false;
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
/// Função utilizada normalmente quando o propósito do código não é RTOS.
void loop()
{
    /* */
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print((String)currentSensor.leitura);
    lcd.setCursor(0,1);
    lcd.print("Modulo I2C");
    delay(1000);
}
