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

/// Configuração do canal I2C para o Display LCD 16x2
LiquidCrystal_I2C displayLCD(0x3F, 2,1,0,4,5,6,7,3, POSITIVE); 

/// Enumeração criada para identificar os possíveis estados da carga
enum State { WAIT, TURN_ON, KEEP_ON, TURN_OFF };

/// MACRO de tempo de espera para tentar novamente o semáforo
#define WAIT_TICKS 3
/// MACRO da porta utilizada pelo relé
#define LED_PIN 32

byte localAddress = 0xBB;
byte serverAddress = 0xFF;

/// telemetryService
typedef struct 
{
    SemaphoreHandle_t mutex;
    String outBuffer;
    String inBuffer;
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
    State state;
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

/// displayLCDStruct
typedef struct 
{
    String currentBuffer;
    String targetBuffer;
    String leituraBuffer;
    String stateBuffer;
    String finalBuffer;
    SemaphoreHandle_t mutex;
} displayLCDStruct;

currentSensorStruct currentSensor;
electricalRelayStruct electricalRelay;
telemetryServiceStruct telemetryService;
electricalMotorStruct electricalMotor;
displayLCDStruct displayStruct;

void currentSensorReader(void * parameters);
void telemetryInfoSender(void * parameters);
void telemetryActionReceiver(int packetSize);
void electricalRelayControl(void * parameters);
void displayControl(void * parameters);

/// Função que simula a leitura de corrente e em seguida armazena o valor em uma variavel temporária
void currentSensorReader(void * parameters)
{
    for(;;)
    {
        bool tmpStatus = false;
        if(xSemaphoreTake(electricalRelay.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            tmpStatus = electricalRelay.status;

            xSemaphoreGive(electricalRelay.mutex);
        }

        if(xSemaphoreTake(currentSensor.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
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

                if(tmpStatus)
                {
                    if(currentSensor.leitura > 15) // corrente máxima
                        currentSensor.leitura -= sinal;
                    else
                        currentSensor.leitura += sinal;
                }
                else
                {
                    if((currentSensor.leitura -= sinal) < 0) // corrente mínima
                        currentSensor.leitura = 0;
                }

                #if 0
                    Serial.println("[currentSensorReader] Read: " + (String)currentSensor.leitura);
                #endif
            }
            else
            {
                currentSensor.samples[currentSensor.index] = ( random(1, 60) / (double)100 ) * sin(((currentSensor.index)/(double)1000)*(2*3.1415));
                currentSensor.index++;
            }
            xSemaphoreGive(currentSensor.mutex);
        }

        vTaskDelay(10/ portTICK_PERIOD_MS); // 10ms
    }
}
/// Tarefa responsável pelo envio periódico da telemetria para o servidor
void telemetryInfoSender(void * parameters)
{
    for(;;)
    {
        if(xSemaphoreTake(currentSensor.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            telemetryService.outBuffer = (String)currentSensor.leitura;

            xSemaphoreGive(currentSensor.mutex);
        }

        if(xSemaphoreTake(telemetryService.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            LoRa.beginPacket();                               // start packet
            LoRa.write(serverAddress);                        // add serverAddress address
            LoRa.write(telemetryService.outBuffer.length());  // add payload length
            LoRa.print(telemetryService.outBuffer);           // add payload
            LoRa.endPacket();                                 // finish packet and send it

            Serial.println("[telemetrySender] Sent: " + telemetryService.outBuffer + "A");

            LoRa.receive(); // coloca o LoRa em modo listening novamente

            xSemaphoreGive(telemetryService.mutex);
        }

        vTaskDelay(1500/ portTICK_PERIOD_MS); // ~2000ms // 2s ->(1.5seg + 0.5seg de consumo do LoRa) 
    }
}
/// Tarefa responsável pelo recebimento e tratamento dos comandos vindos do Servidor.
void telemetryActionReceiver(int packetSize)
{
    if (packetSize == 0)
        return;

    String splitter[2];
    char buf[sizeof(telemetryService.inBuffer)];
    int i = 0;
    char *p = buf;
    char *str;
    
    if(xSemaphoreTake(telemetryService.mutex, portMAX_DELAY) == pdTRUE)
    {
        // read packet header bytes
        int destination = LoRa.read();      // destination address
        byte incomingLength = LoRa.read();  // incoming msg length

        telemetryService.inBuffer = "";
        while (LoRa.available())
            telemetryService.inBuffer += (char)LoRa.read();
        
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

        // Dividindo as informações da string em variáveis
        telemetryService.inBuffer.toCharArray(buf, sizeof(buf));
        while ((str = strtok_r(p, ";", &p)) != NULL)
        {
            splitter[i++] = str;
        }

        #if 0
            Serial.println("[actionReceiver] Received: " + telemetryService.inBuffer);
        #endif

        libera:
        xSemaphoreGive(telemetryService.mutex);
    }

    if(xSemaphoreTake(electricalRelay.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
    {
        motorConfigStruct newConfig;
        /** ' 1 ' - Aguardar: mantem estado atual*/
        if(splitter[0].equals("1")) // aguardar
        {
            newConfig.status = electricalMotor.config.status;
            newConfig.current = electricalMotor.config.current;
            newConfig.state = WAIT;
            electricalMotor.config = newConfig;
            Serial.println("[actionReceiver] Received: wait");
        }
        /** ' 2 ' - Ligar: Liga a carga independentemente do valor de corrente*/
        else if(splitter[0].equals("2")) // ligar (sem limite definido)
        {
            newConfig.status = true;
            newConfig.current = -1;
            newConfig.state = TURN_ON;
            electricalMotor.config = newConfig;
            Serial.println("[actionReceiver] Received: turn on");
        }
        /** ' 3;valor ' - Manter em 'valor' Ampere: Configura o valor de corrente para a carga */
        else if(splitter[0].equals("3")) // manter em xA
        {
            newConfig.status = true;
            newConfig.current = splitter[1].toInt();
            newConfig.state = KEEP_ON;
            electricalMotor.config = newConfig;
            Serial.println("[actionReceiver] Received: keep on " + splitter[1] + "A");
        }
        /** ' 4 ' - Desligar: Desliga a carga */
        else if(splitter[0].equals("4")) // desligar
        {
            newConfig.status = false;
            newConfig.current = 0;
            newConfig.state = TURN_OFF;
            electricalMotor.config = newConfig;
            Serial.println("[actionReceiver] Received: turn off");
        }
        /** ' 0 ' - Teste: Utilizado apenas para testar a comunicação, não produz qualquer efeito sobre a carga.*/
        else if(splitter[0].equals("0")) // OK
        {
            Serial.println("[actionReceiver] Received: OK");
        }
        else
            Serial.println("[actionReceiver] Received: unknown");
            
        Serial.println("");

        xSemaphoreGive(electricalRelay.mutex);
    }
}
/// Função que faz a ativação do relé conforme os dados atualizados pelas outras tarefas
void electricalRelayControl(void * parameters) 
{
    for(;;)
    {
        double leituraAtual = 0;
        if(xSemaphoreTake(currentSensor.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            leituraAtual = currentSensor.leitura;

            xSemaphoreGive(currentSensor.mutex);
        }

        if(xSemaphoreTake(electricalRelay.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            if(electricalMotor.config.status && (electricalMotor.config.current > leituraAtual || electricalMotor.config.current == -1))
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
        vTaskDelay(400/ portTICK_PERIOD_MS); // 400ms
    }
}
/// Função que demostra constantemente no display o status atual da carga.
void displayControl(void * parameters) 
{
    for(;;)
    {
        if(xSemaphoreTake(currentSensor.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            displayStruct.leituraBuffer = (String)currentSensor.leitura;
            
            xSemaphoreGive(currentSensor.mutex);
        }

        if(xSemaphoreTake(electricalRelay.mutex, ( TickType_t ) WAIT_TICKS ) == pdTRUE)
        {
            displayStruct.targetBuffer = (String)electricalMotor.config.current + "A";
            
            xSemaphoreGive(electricalRelay.mutex);
        }

        if(!displayStruct.leituraBuffer.equals(""))
        {
            switch (electricalMotor.config.state)
            {
                case WAIT:
                    displayStruct.stateBuffer = "WAITING";
                    break;
                case TURN_ON:
                    displayStruct.stateBuffer = "TURNING ON";
                    break;
                case KEEP_ON:
                    displayStruct.stateBuffer = "KEEPING ON " + displayStruct.targetBuffer;
                    break;
                case TURN_OFF:
                    displayStruct.stateBuffer = "TURNING OFF";
                    break;
                default:
                    displayStruct.stateBuffer = "UNKNOWN";
                    break;
            }

            displayStruct.finalBuffer = displayStruct.leituraBuffer + displayStruct.stateBuffer;

            if(!displayStruct.currentBuffer.equals(displayStruct.finalBuffer))
            {
                if(xSemaphoreTake(telemetryService.mutex, portMAX_DELAY) == pdTRUE) // evita que o LoRa trabalhe ao mesmo tempo, que causa problemas no I2C devido à queda de tensão
                {
                    Wire.setClock(1000); // Forçar clock I2C para manter seu funcionamento correto
                  
                    displayLCD.clear();
                    displayLCD.setCursor(0,0);
                    displayLCD.print(displayStruct.leituraBuffer + "A");
                    displayLCD.setCursor(0,1);
                    displayLCD.print(displayStruct.stateBuffer);
    
                    displayStruct.currentBuffer = displayStruct.finalBuffer;
                   
                    xSemaphoreGive(telemetryService.mutex);
                }
            }
        }
        
        vTaskDelay(1500/ portTICK_PERIOD_MS); // 1500ms / 1.5s
    }
}

/// Função que executa apenas uma vez e sempre que o microcontrolador é ligado.
void setup()
{
    Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, false /*PABOOST Enable*/, 915E6 /*long BAND*/); // desativar PABOOST para reduzir consumo

    displayLCD.begin(16, 2); // (comprimento, altura) do display LCD

    pinMode(LED_PIN, OUTPUT); // Electrical Relay simulator's LED

    Serial.println("[MAIN] Launching Heltec.LoRa service.");
    LoRa.onReceive(telemetryActionReceiver);
    LoRa.receive();
    Serial.println("[MAIN] Heltec.LoRa init succeeded.");

    //
    displayStruct.currentBuffer = "";
    displayStruct.targetBuffer  = "";
    displayStruct.stateBuffer   = "";
    displayStruct.leituraBuffer = "";
    displayStruct.finalBuffer   = "";
    //
    telemetryService.mutex      = xSemaphoreCreateMutex();
    telemetryService.outBuffer  = "";
    telemetryService.inBuffer   = "";
    //
    currentSensor.mutex     = xSemaphoreCreateMutex();
    currentSensor.index     = 0;
    currentSensor.leitura   = 0;
    //
    electricalMotor.config.status   = false;
    electricalMotor.config.current  = 0;
    electricalMotor.config.state    = WAIT;
    //
    electricalRelay.mutex   = xSemaphoreCreateMutex();
    electricalRelay.status  = false;
    //

    Serial.println("[MAIN] Launching currentSensorReader thread.");
    xTaskCreate(currentSensorReader,"currentSensorReader", 2000, NULL, 2, NULL); // prioridade 2

    Serial.println("[MAIN] Launching telemetryInfoSender thread.");
    xTaskCreate(telemetryInfoSender,"telemetryInfoSender", 2000, NULL, 5, NULL); // prioridade 5

    Serial.println("[MAIN] Launching electricalRelayControl thread.");
    xTaskCreate(electricalRelayControl,"electricalRelayControl", 2000, NULL, 4, NULL); // prioridade 4

    Serial.println("[MAIN] Launching displayControl thread.");
    xTaskCreate(displayControl,"displayControl", 2000, NULL, 3, NULL); // prioridade 3

    Serial.println("[MAIN] Waiting for server command.\n");
}
/// Função utilizada normalmente quando o propósito do código não é RTOS.
void loop()
{
    /* */
}
