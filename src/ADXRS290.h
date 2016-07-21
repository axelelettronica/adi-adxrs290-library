/*
 * ADXRS290.h
 *
 * Created: 05/06/2016 19:49:15
 *  Author: searobin
 */

#ifndef ADXRS290_SENSOR_H_
#define ADXRS290_SENSOR_H_

#include <Arduino.h>


class ADXRS290
{
private:
    uint8_t _address;
    uint8_t  _clk, _miso, _mosi, _ss, _irq;

public:
   // ADXRS290() {}
    ADXRS290(uint8_t _sck, uint8_t _miso,uint8_t _mosi, uint8_t _ss, uint8_t _irq = 0);
    ~ADXRS290() {}

protected:
    uint8_t readByteInternal(uint8_t address);
    void writeByteInternal(uint8_t address, uint8_t data);
    unsigned int readRegister(byte thisRegister, int bytesToRead);
    void writeRegister(byte thisRegister, byte thisValue);

public:
    bool begin(void);
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

//extern  ADXRS290 adiGyroscope;

#endif /* ADXRS290_H_ */
