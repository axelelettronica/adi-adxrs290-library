/*
    ADI ADXRS290 Library - get gyroscope information

    This example shows how to retrieve the x,y,z values 
    from the ADXRS290 gyroscope information

    Created 2 November 2016 by Seve (seve@ioteam.it)

    This example is in the public domain
    https://github.com/axelelettronica

    more information available here:
    http://www.analog.com/en/products/mems/mems-gyroscopes/adxrs290.html
 */


#include <Arduino.h>
#include <SPI.h>
#include <ADXRS290.h>

float x, y;
float temp = 0;


void setup(void)
{
    SerialUSB.begin(115200);

    SPI1.begin();
    delay(1000);

    // Stop the execution till a Serial console is connected
    while (!SerialUSB) {
        ;
    }

    adiGyroscope.begin(ADXRS290_CS, &SPI1, ADXRS290_EINT);
    adiGyroscope.standbyModeEnable(false);
    adiGyroscope.tempSensorEnable(true);
}




void loop(void)
{    
    temp = adiGyroscope.readTemperature();
    SerialUSB.print("Temperature = ");
        
    SerialUSB.print(temp); 
    x = adiGyroscope.readX();
    SerialUSB.print("\tX axis = ");
    SerialUSB.print(x);
    y = adiGyroscope.readY();
    SerialUSB.print("\tY axis = ");
    SerialUSB.println(y);            

    delay(1000);
}

