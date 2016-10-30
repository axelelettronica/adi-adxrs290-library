/*
 * ADXRS290.h
 *
 * Created: 05/06/2016 19:49:15
 *  Author: searobin
 */

#ifndef __ADXRS290_H__
#define __ADXRS290_H__

#include <Arduino.h>
//#include "ADXRS290Reg.h"
#include <SPI.h>

class ADXRS290
{
private:
    uint8_t  _address;
    uint8_t  _ss;
    uint8_t  _irq;
    SPIClass *_spi;
    
public:
    ADXRS290() {}
    char begin(uint8_t ss, SPIClass *spi=&SPI1,  uint8_t irq = 0);       
    ~ADXRS290() {}


protected:
    uint8_t readByteInternal(uint8_t address);
    void writeByteInternal(uint8_t address, uint8_t data);
    unsigned int readRegister(byte thisRegister, int bytesToRead);
    void writeRegister(byte thisRegister, byte thisValue);

public:
    void standbyModeEnable(bool standbyMode);

    void setLowPassFilter(int lowFreqPole);
    void setHighPassFilter(int highFreqPole);
    void interruptModeEnable(bool activate);
    int readX();
    int readY();

    void tempSensorEnable(bool enable);
    float readTemperature();

    //int readRevNumber();
    //int readSerialNumber();
};

extern  ADXRS290 adiGyroscope;

#endif /* ADXRS290_H_ */
