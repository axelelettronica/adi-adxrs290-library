/*
 * ADXRS290_Sensor.c
 *
 * Created: 05/06/2016 19:49:04
 *  Author: searobin
 */

#include <Wire.h>
#include <SPI.h>
#include "ADXRS290Reg.h"
#include "ADXRS290.h"

const byte READ  = 0b10000000;     // read command
const byte WRITE = 0b01111111;     // write command


//Read from or write to register from the SCP1000:
unsigned int 
ADXRS290::readRegister(byte thisRegister, int bytesToRead) {
    byte inByte = 0;           // incoming byte from the SPI
    unsigned int result = 0;   // result to return

    // now combine the address and the command into one byte
    volatile byte dataToSend = thisRegister | READ;
    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    //delay(2);
    
    // take the chip select low to select the device:
    digitalWrite(_ss, LOW);
    // send the device the register you want to read:
    SPI1.transfer(dataToSend);
    // send a value of 0 to read the first byte returned:
    result = SPI1.transfer(0x00);
    // decrement the number of bytes left to read:
    bytesToRead--;
    // if you still have another byte to read:
    if (bytesToRead > 0) {
        // shift the first byte left, then get the second byte:
        result = result << 8;
        inByte = SPI1.transfer(0x00);
        // combine the byte you just got with the previous one:
        result = result | inByte;
        // decrement the number of bytes left to read:
        bytesToRead--;
    }

    // take the chip select high to de-select:
    digitalWrite(_ss, HIGH);

    SPI1.endTransaction();
    return (result);
}

void 
ADXRS290::writeRegister(byte thisRegister, byte thisValue) {

  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister & WRITE;
  
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  
  // take the chip select low to select the device:
  digitalWrite(_ss, LOW);
  //delay(2);
  SPI1.transfer(dataToSend); //Send register location
  SPI1.transfer(thisValue);  //Send value to record into register
  // take the chip select high to de-select:
  digitalWrite(_ss, HIGH);
  
  SPI1.endTransaction();
}


uint8_t
ADXRS290::readByteInternal(uint8_t address)
{
    return (uint8_t) readRegister(address, 1);
}

void
ADXRS290::writeByteInternal(uint8_t address, uint8_t data)
{
    readRegister(address, data);
}


/**** *************************************************************************/
/*                                 Gyroscope                                 */
/*****************************************************************************/

ADXRS290::ADXRS290(uint8_t clk, uint8_t miso, uint8_t mosi, uint8_t ss, uint8_t irq):
_clk(clk),
_miso(miso),
_mosi(mosi),
_ss(ss),
_irq(irq)
//_reset(0),
//_//usingSPI(true),
//_//hardwareSPI(false)
{
    pinMode(_ss, OUTPUT);
    //pinMode(_irq, INPUT);


}
bool ADXRS290::begin(void)
{
    volatile uint8_t data;

    data = readByteInternal(ADXRS290_ANALOG_ID);  
    if (data != ADXRS290_ANALOG_ID_RETURN){
        // Wrong Device ID
        return false;
    }

    data = readByteInternal(ADXRS290_MEMS_ID);  
    if (data != ADXRS290_MEMS_ID_RETURN){
        // Wrong MEMS ID
        return false;
    }

    return true;
}

void 
ADXRS290::standbyModeEnable(bool standByMode)
{
    uint8_t data;

    data = readByteInternal(ADXRS290_POW_CTRL_REG);  
    
    if(standByMode) {
        data |= ADXRS290_POW_CTRL_STDBY_MASK;
    } else {
        data &= ~ADXRS290_POW_CTRL_STDBY_MASK;
    }
    writeByteInternal(ADXRS290_POW_CTRL_REG, data);
}

void 
ADXRS290::interruptModeEnable(bool activate)
{
    uint8_t data;

    data = readByteInternal(ADXRS290_DATA_READY_REG);  
    
    data &= ~ADXRS290_DATA_READY_INT_MASK;
    if(activate) {
        data |=  0x01;
    }
    writeByteInternal(ADXRS290_DATA_READY_REG, data);
}


void 
ADXRS290::setLowPassFilter(int lowFreqPole)
{
    uint8_t data;

    data = readByteInternal(ADXRS290_BANDPASS_FILTER);  
    data &= ~ADXRS290_BPF_LPF_MASK;
    data |= (lowFreqPole & ADXRS290_BPF_LPF_MASK); 
    writeByteInternal(ADXRS290_BANDPASS_FILTER, data);
}

void 
ADXRS290::setHighPassFilter(int highFreqPole)
{
    uint8_t data;

    data = readByteInternal(ADXRS290_BANDPASS_FILTER);  
    data &= ~ADXRS290_BPF_HPF_MASK;
    data |= (highFreqPole & ADXRS290_BPF_HPF_MASK); 
    writeByteInternal(ADXRS290_BANDPASS_FILTER, data);
}


void 
ADXRS290::tempSensorEnable(bool enable)
{
    uint8_t data;

    data = readByteInternal(ADXRS290_POW_CTRL_REG);  
    
    data &= ~ADXRS290_POW_CTRL_TEMP_EN_MASK;
    if(enable) {
        data |=  ADXRS290_POW_CTRL_TEMP_EN_MASK;
    }
    writeByteInternal(ADXRS290_POW_CTRL_REG, data);
}



int
ADXRS290::readX()
{
    uint16_t data = 0;
    uint8_t lsb = 0;
    uint8_t msb = 0;
    int16_t signed_data = 0;

    // send the device the register you want to read:  
    SPI1.transfer(ADXRS290_GYR_X_L);  
    // send a value of 0 to read the first byte returned:  
    lsb = SPI1.transfer(0xFF); 
    msb = SPI1.transfer(0xFF); 

    data = lsb;
    data |= msb << 8; // MSB

    signed_data = (int16_t)data;

    return signed_data;
}


int
ADXRS290::readY()
{
    uint16_t data = 0;
    uint8_t lsb = 0;
    uint8_t msb = 0;
    int16_t signed_data = 0;

    // send the device the register you want to read:  
    SPI1.transfer(ADXRS290_GYR_Y_L);  
    // send a value of 0 to read the first byte returned:  
    lsb = SPI1.transfer(0xFF); 
    msb = SPI1.transfer(0xFF); 

    data = lsb;
    data |= msb << 8; // MSB

    signed_data = (int16_t)data;

    return signed_data;
}


float
ADXRS290::readTemperature()
{
    uint16_t data = 0;
    uint8_t lsb = 0;
    uint8_t msb = 0;
    int16_t signed_data = 0;

    // send the device the register you want to read:  
    SPI1.transfer(ADXRS290_TEMP_L);  
    // send a value of 0 to read the first byte returned:  
    lsb = SPI1.transfer(0xFF); 
    msb = SPI1.transfer(0xFF); 

    data = lsb;
    data |= (0xF & msb) << 8; // MSB

    // HOW CONVERT 12 bit complemetn 2 ???
    signed_data = (int16_t)data;


    // Decode Temp measurement 
    return (((float)signed_data)/10.0);
}



//ADXRS290 adiGyroscope;
