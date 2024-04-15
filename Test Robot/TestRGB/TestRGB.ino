#include <Wire.h> 
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X); 

#define TCAADDR 0x70
void tcaselect(uint8_t i) { 
    if (i > 7) return; 
    Wire.beginTransmission(TCAADDR); 
    Wire.write(1 << i); 
    Wire.endTransmission(); 
}

void setup() {
    Serial.begin(9600); 
    Wire.begin();
    
    tcaselect(7); 
    if (tcs.begin()) { 
        Serial.println("Inicializando"); 
        delay(2000); 
    } else {
        Serial.println("Error"); 
        Serial.println("Debe de revisar las conexiones e iniciar nuevamente");
        while (1) delay(500); 
    }
}

void loop() {
  
    tcaselect(7); 
    uint16_t R, G, B, C; 

    tcs.getRawData(&R, &G, &B, &C);

    analogWrite(11, R);
    analogWrite(10, G);
    analogWrite(9, B);

    Serial.println(" R = " + String(R) + " G = " + String(G) + " B = " + String(B));
    delay(300); 
}
