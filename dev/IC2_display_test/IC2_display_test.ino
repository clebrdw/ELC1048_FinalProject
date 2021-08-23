/**
 * Firmware para controlar um display LCD 16x2 com I2C
 * 
 * Autora: Ana Paula Messina - tecdicas
 * 
 * 16/10/2019
 * 
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Alterar o endereço conforme modulo I2C
LiquidCrystal_I2C lcd(0x3F, 2,1,0,4,5,6,7,3, POSITIVE); 

void setup()
{
  lcd.begin (16,2);
}
 
void loop()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("tecdicas.com");
  lcd.setCursor(0,1);
  lcd.print("Modulo I2C");
  delay(2000);
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Ola mundo!");
  delay(1000);
}
