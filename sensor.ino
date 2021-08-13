 
// Carrega as bibliotecas
#include "Arduino.h"
#include <heltec.h>
#include "EmonLib.h" 

long lastMillis = 0;
long loops = 0;

// Criamos uma instancia (chamada)
EnergyMonitor emon1;

// Tensao da rede eletrica (127V OU 220V)
  int rede = 220;

// Pino do sensor SCT 013 20A/1V
  int pino_sct = 37;

// *Fundo de Escala*
// --------------------------------------------------------------------------------------------- //
// Limitamos valor MIN de medicao para que seja igual a 20mA, o que resulta
// em aproximadamente 2.5 Watts (APENAS) considerando nossos 127VAC.
// --------------------------------------------------------------------------------------------- //
// OBS: Este valor de fundo de escala serve como verdadeiro "supressor de ruidos",
//      uma vez que o sensor analógico dificilmente medirá um ZERO absoluto via HARDWARE apenas.
//      Por esta razao configuramos via software o conjunto/circuito de hardware utilizado. 

float fundoEscala = 0.070;

int counter = 0;
float correntes[1000];
int aux = 0;

void setup() 
{

  Heltec.begin(false /*Display Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, 915E6 /**/);
  Serial.begin(115200);
   
  // Pino, calibracao - Const. da Corrente = Ratio/BurdenResistor
  emon1.current(pino_sct, 4.5);
  //emon1.current(pino_sct, 20);
} 
  
void loop() 
{ 
    // Calculamos a Corrente RMS (Amperes)
    correntes[aux++] = emon1.calcIrms(1480);     // Calcula a Corrente RMS

   
    //corrente =  ((Irms > fundoEscala) ? Irms : 0.0);
    //corrente = (corrente + Irms) / 2;
    //corrente = Irms;
    /*if(counter == 50) //500Hz
    {
        counter = 0;*/
        
        /*Heltec.display -> drawString(50, 0, (String)corrente + "A");
        Heltec.display -> drawString(50, 10, (String)(rede * corrente) + "W");
        
        Heltec.display -> display();*/
    
        // Pequena secao onde "zeramos" o display caso a medição 
        // alcance nosso limite MIN (fundo da escala - supressor de ruidos)
        // ou Informamos que o "Consumo está sendo inferior a 20mA".
      
        
        
        /*Heltec.display -> clear();*/
    /*}
    else
        counter++;*/

  /*long currentMillis = millis();
  loops++;*/
  
  /* By doing complex math, reading sensors, using the "delay" function,
  *  etc you will increase the time required to finish the loop,
  *  which will decrease the number of loops per second.
  */

    if(aux >= 1000)
    {
        Serial.print("y2 = [");
        for(int i = 0; i < 1000; i++)
        {
              Serial.print((String)correntes[i] + ",");
        }
        Serial.println("];");
        Serial.println((String)aux);
    
        aux = 0;
        
    }
    
    //Serial.println((String)aux);

  /*if(currentMillis - lastMillis > 1000){

    Serial.print("y2 = [");
    for(int i = 0; i < 2000; i++)
    {
          Serial.print((String)correntes[i] + ",");
    }
    Serial.println("];");
    Serial.println((String)aux);
    
    //Serial.println("Leitura: " + (String)corrente + "A - Potência: " + (String)(rede * corrente) + "W");
    
    lastMillis = currentMillis;
    loops = 0;
  }*/

  delay(5);
}
