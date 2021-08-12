#define NUMERO_TICKS_ESPERA 3 //qtde de ticks que a tarefa vai esperar para conseguir o semaforo

SemaphoreHandle_t Semaforo_leitura;   // Declaracao do Semaforo (leitura)

float leitura = 0;

void task_sensor(void * parameters){
  /* Inicializa o pino 36 como entrada*/
  pinMode(36, INPUT);
  for(;;){
    if( xSemaphoreTake( Semaforo_leitura, ( TickType_t ) NUMERO_TICKS_ESPERA ) == pdTRUE){
      /*se nao estourar NUMERO_TICKS_ESPERA, conseguiu o semaforo*/
      
      /* leitura do sensor, porta 36 */
      leitura = analogRead(36);
      Serial.print(" - Leitura:");
      Serial.print(leitura);
      
      /*ja utilizei a variavel compartilhada, então libero o semaforo*/
      xSemaphoreGive( Semaforo_leitura );
    }
    vTaskDelay(100/ portTICK_PERIOD_MS);
  } 
}

void task_motor(void * parameters){
  /* Inicializa o pino 25 como saida*/
  pinMode(25, OUTPUT);
  for(;;){
    if( xSemaphoreTake( Semaforo_leitura, ( TickType_t ) NUMERO_TICKS_ESPERA ) == pdTRUE){
      /*se nao estourar NUMERO_TICKS_ESPERA, conseguiu o semaforo*/
      
      if(leitura > 2900){
        digitalWrite(25, HIGH);
        Serial.print("\n Motor Ligado");
      }else{
        digitalWrite(25, LOW);
        Serial.print("\n Motor Desligado");
      }

      /*ja utilizei a variavel compartilhada, então libero o semaforo*/
      xSemaphoreGive( Semaforo_leitura );
    }
    vTaskDelay(100/ portTICK_PERIOD_MS);
  }
}


void setup() {

  //Inicializa Serial (velocidade em bps)
  Serial.begin(9600); 
  //Espera a Serial ficar pronta e segue em frente
  while(!Serial){}
  // Cria o semaforo para a variavel contador  

  Semaforo_leitura = xSemaphoreCreateMutex();  // Criacao do semaforo de leitura

  xTaskCreate(task_sensor,"Sensor", 1000, NULL, 1, NULL);
  xTaskCreate(task_motor,"Motor", 1000, NULL, 2, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}
