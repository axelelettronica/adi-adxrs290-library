/*
    ADI ADXRS290 Library - get gyroscope X-Y Axises and temperature values

    This example shows how to retrieve the x,y,z values 
    from the ADXRS290 gyroscope information

    Created 2 November 2016 by Seve (seve@ioteam.it)

    This example is in the public domain
    https://github.com/axelelettronica/adi-adxrs290-library

    more information available here:
    http://www.analog.com/en/products/mems/mems-gyroscopes/adxrs290.html
 */


#include <Arduino.h>
#include <SPI.h>
#include <ADXRS290.h>

volatile uint32_t i = 0;
float x, y, z;
float temp;

bool led = false;
#define TRIGGER_LED  digitalWrite(PIN_LED, led ? 225 : 0); \
                     led =!led

void setup(void)
{
    Serial.begin(115200);
    
    SPI1.begin();
    adiGyroscope.begin(ADXRS290_CS, &SPI1, ADXRS290_EINT); 

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    // Waiting for the console to continue
    while (!Serial) {
   	;
    }

    adiGyroscope.tempSensorEnable(true);
}



void loop(void)
{    
    if (!(i%5)) {
        TRIGGER_LED;
        
        adiGyroscope.setMeasurementMode();
        
        adiGyroscope.readXY(&x, &y);
        Serial.print("\tX / Y =  ");
        Serial.print(x);
        Serial.print(" / ");
        Serial.print(y); 
        Serial.print("  deg/s");
        
        if (!(i%100)) {
            temp = adiGyroscope.readTemperature();       
            Serial.print("\tTemperature = ");
            Serial.println(temp);
        } else {
            Serial.print("\n");            
        }
        adiGyroscope.setStandbyMode();
    }   
    ++i;
    delay(10);
}

