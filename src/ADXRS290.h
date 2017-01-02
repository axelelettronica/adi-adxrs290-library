/*
 * ADXRS290.h
 *
 * Created: 05/06/2016 19:49:15
 *  Author: searobin
 */

#ifndef __ADXRS290_H__
#define __ADXRS290_H__

#include <Arduino.h>
#include "gyroscope/ADXRS290Reg.h"
#include <SPI.h>

class ADXRS290
{
private:
    uint8_t  _address;
    uint8_t  _ss;
    uint8_t  _irq;
    SPIClass *_spi;
    bool     _isStandby;
    // Attributes
    uint8_t  _rev;
    uint32_t _sn;

public:
    ADXRS290();
    char begin(uint8_t ss, SPIClass *spi=&SPI,  uint8_t irq = 0);       
    ~ADXRS290() {}
        
private:

    uint8_t readNRegisters(byte address, byte buffer[], int bytesToRead);
    unsigned int readByteInternal(byte thisRegister, int bytesToRead); // up to 2
    void writeByteInternal(byte thisRegister, byte thisValue);
    unsigned int readNBytesInternal(byte startRegister, byte buffer[], int bytesToRead);
    
protected:

    void standbyReadXY(float *x, float *y);
    float standbyReadTemperature();
    int readX();
    int readY();
    void setStandby(bool standbyMode);
public:

    void setStandbyMode() {setStandby(true);}
    void setMeasurementMode()  {setStandby(false);}       
    bool isStandbyMode() {return (_isStandby ? true : false);}
        
    void setLowPassFilter(int lowFreqPole);
    void setHighPassFilter(int highFreqPole);
    int getLowPassFilter(void);
    int getHighPassFilter(void);
    void interruptModeEnable(bool activate);
    void tempSensorEnable(bool enable);
    
    void readXY(float *x, float *y);

    float readTemperature();
    bool check();

    // Debug purposes
    uint8_t readRegister(uint8_t address);
    void    writeRegister (uint8_t address, uint8_t data);
};

extern  ADXRS290 adiGyroscope;

#endif /* ADXRS290_H_ */
