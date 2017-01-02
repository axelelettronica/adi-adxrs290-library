/*
 * ADXRS290_Sensor.c
 *
 * Created: 05/06/2016 19:49:04
 *  Author: searobin
 */


#include "ADXRS290.h"
#include <SPI.h>

const byte READ  = 0b10000000;     // read command
const byte WRITE = 0b01111111;     // write command

#define READBUF_SIZE  8
static byte readBuf[READBUF_SIZE] = {};

static const SPISettings adxrs290_SPISettings(1000000L, MSBFIRST, SPI_MODE3);

//Read from or write to register from the SCP1000:
unsigned int 
ADXRS290::readByteInternal(byte thisRegister, int bytesToRead) {
    byte inByte = 0;           // incoming byte from the SPI
    unsigned int result = 0;   // result to return

    // now combine the address and the command into one byte
    volatile byte dataToSend = thisRegister | READ;
   _spi->beginTransaction(adxrs290_SPISettings);
   
    // take the chip select low to select the device:
    digitalWrite(_ss, LOW);
    // send the device the register you want to read:
    _spi->transfer(dataToSend);
    // send a value of 0 to read the first byte returned:
    result = _spi->transfer(0x00);
    // decrement the number of bytes left to read:
    bytesToRead--;
    // if you still have another byte to read:
    if (bytesToRead > 0) {
        // shift the first byte left, then get the second byte:
        result = result << 8;
        inByte = _spi->transfer(0x00);
        // combine the byte you just got with the previous one:
        result = result | inByte;
        // decrement the number of bytes left to read:
        bytesToRead--;
    }

    // take the chip select high to de-select:
    digitalWrite(_ss, HIGH);
    delayMicroseconds(1);
    _spi->endTransaction();
    return (result);
} 

unsigned int
ADXRS290::readNBytesInternal(byte startRegister, byte buffer[], int bytesToRead) {
    
    // now combine the address and the command into one byte
    volatile byte dataToSend = startRegister | READ;
    _spi->beginTransaction(adxrs290_SPISettings);
    
    // take the chip select low to select the device:
    digitalWrite(_ss, LOW);
    // send the device the register you want to read:
    _spi->transfer(dataToSend);
    // send a value of 0 to read the first byte returned:
    buffer[bytesToRead-1] = _spi->transfer(0x00);
    // decrement the number of bytes left to read:
    bytesToRead--;
    // if you still have another byte to read:
    while (bytesToRead > 0) {
        buffer[bytesToRead-1] = _spi->transfer(0x00);
        // decrement the number of bytes left to read:
        bytesToRead--;
    }

    // take the chip select high to de-select:
    digitalWrite(_ss, HIGH);
    delayMicroseconds(1);
    _spi->endTransaction();
    return (0);
}


void 
ADXRS290::writeByteInternal(byte thisRegister, byte thisValue) {

  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister;// & WRITE;
  byte data[2];
  data[0] = dataToSend;
  data[1] = thisValue;
  _spi->beginTransaction(adxrs290_SPISettings);
  
  // take the chip select low to select the device:
  digitalWrite(_ss, LOW);
  _spi->transfer(data, 2);
  digitalWrite(_ss, HIGH);
  
  _spi->endTransaction();

}



uint8_t
ADXRS290::readNRegisters(byte address, byte buffer[], int bytesToRead)
{
    return (uint8_t) readNBytesInternal(address, buffer, bytesToRead);
}


uint8_t
ADXRS290::readRegister(uint8_t address)
{
    return (uint8_t) readByteInternal(address, 1);
}

void
ADXRS290::writeRegister (uint8_t address, uint8_t data)
{
   writeByteInternal(address, data);
}

/*****************************************************************************/
/*                                 Gyroscope                                 */
/*****************************************************************************/
ADXRS290::ADXRS290()
{

    _address = 0;
    _ss = 0;
    _irq = 0;
    _spi = NULL;
    _sn = 0;
    _rev = 0;
}    

bool ADXRS290::check(void)
{
    volatile uint8_t data;
    volatile int i = 0;
        
    data = readRegister(ADXRS290_ANALOG_ID);
    if (data != ADXRS290_ANALOG_ID_RETURN){
        // Wrong Device ID
        return false;
    }

    data = readRegister(ADXRS290_MEMS_ID);
    if (data != ADXRS290_MEMS_ID_RETURN){
        // Wrong MEMS ID
        return false;
    }

    data = readRegister(ADXRS290_DEV_ID);
    if (data != ADXRS290_DEV_ID_RETURN){
        // Wrong MEMS ID
        return false;
    }
    _rev = readRegister(ADXRS290_REV_NUM);

    if (_rev == 0) {
        return false;
    }
    
    // reading Serial NUmber
    for (i = ADXRS290_SERIALNUM_START; i <= ADXRS290_SERIALNUM_END; ++i) {
        data = readRegister(i);
        _sn |= data << (8*(i-ADXRS290_SERIALNUM_START));
    }
    
    if (_sn == 0) {
        return false;
    }
    
    return true;
}
char ADXRS290::begin(uint8_t ss, SPIClass *spi,  uint8_t irq)
{    
    this->_irq = irq;   
    this->_spi = spi;
    this->_ss = ss;
    this->_isStandby = false;

    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
   // pinMode(_irq, OUTPUT);
   // digitalWrite(_irq, LOW);
    pinMode(_irq, INPUT);

    if (check() == false) {
        return -1;
    }

    tempSensorEnable(false);
    setStandby(true); 
    return 0;
}

void 
ADXRS290::setStandby(bool standByMode)
{
    volatile uint8_t data;
    bool change = false;
    
    _isStandby = standByMode;
    data = readRegister(ADXRS290_POW_CTRL_REG);  
    
    if (data & ADXRS290_POW_CTRL_STDBY_MASK) {
        // current status on -> setting standby
        if (standByMode) {
             data &= ~ADXRS290_POW_CTRL_STDBY_MASK;

             change = true;
        }
    } else {
        // current status off -> setting measurement mode
        if (!standByMode) {
             data |= ADXRS290_POW_CTRL_STDBY_MASK; 
             change = true;
        }
    }
    if(change) {
        writeRegister(ADXRS290_POW_CTRL_REG, data);
        delay(100);
    }
}


void 
ADXRS290::interruptModeEnable(bool activate)
{
    volatile uint8_t data;

    data = readRegister(ADXRS290_DATA_READY_REG);  
    
    data &= ~ADXRS290_DATA_READY_INT_MASK;
    if(activate) {
        data |=  0x01;
    }
    writeRegister(ADXRS290_DATA_READY_REG, data);
}

int 
ADXRS290::getLowPassFilter(void)
{
    volatile uint8_t data;

    data = readRegister(ADXRS290_BANDPASS_FILTER);  
    return (data & ADXRS290_BPF_LPF_MASK);
}

void 
ADXRS290::setLowPassFilter(int lowFreqPole)
{
    volatile uint8_t data;

    data = readRegister(ADXRS290_BANDPASS_FILTER);  
    data &= ~ADXRS290_BPF_LPF_MASK;
    data |= (lowFreqPole & ADXRS290_BPF_LPF_MASK); 
    writeRegister(ADXRS290_BANDPASS_FILTER, data);
}

int 
ADXRS290::getHighPassFilter(void)
{
    volatile uint8_t data;

    data = readRegister(ADXRS290_BANDPASS_FILTER);  
    return ((data & ADXRS290_BPF_HPF_MASK) >> ADXRS290_BPF_HPF_OFFSET);
}

void 
ADXRS290::setHighPassFilter(int highFreqPole)
{
    volatile uint8_t data;

    data = readRegister(ADXRS290_BANDPASS_FILTER);  
    data &= ~ADXRS290_BPF_HPF_MASK;
    data |= ((highFreqPole << ADXRS290_BPF_HPF_OFFSET) & 
              ADXRS290_BPF_HPF_MASK); 
    writeRegister(ADXRS290_BANDPASS_FILTER, data);
}


void 
ADXRS290::tempSensorEnable(bool enable)
{
    volatile uint8_t data;

    data = readRegister(ADXRS290_POW_CTRL_REG);  
    
    data &= ~ADXRS290_POW_CTRL_TEMP_EN_MASK;
    if(enable) {
        data |=  ADXRS290_POW_CTRL_TEMP_EN_MASK;
    }
    writeRegister(ADXRS290_POW_CTRL_REG, data);   
}



void
ADXRS290::readXY(float *x, float *y)
{
    volatile int16_t i16x, i16y;
    volatile uint16_t data = 0;
    
    if (isStandbyMode()) {
        return standbyReadXY(x, y);
    }
    
    readNRegisters(ADXRS290_GYR_X_L, readBuf, 4);
    data = readBuf[3];
    data |= (0xFF & readBuf[2]) << 8; // MSB
    // CONVERT 12 bit, into 2 complement
    i16x = (int16_t)data;
    
    data = readBuf[1];
    data |= (0xFF & readBuf[0]) << 8; // MSB
    // CONVERT 12 bit, into 2 complement
    i16y = (int16_t)data;
    
    *x  = (float) i16x / 200.0; // sensitivity 1/200 per LSB
    *y  = (float) i16y / 200.0;
} 

   
float
ADXRS290::readTemperature()
{    
    int16_t signed_data = 0;
    volatile uint16_t data = 0;

    if (isStandbyMode()) {
        return standbyReadTemperature();
    }
    
    readNRegisters(ADXRS290_TEMP_L, readBuf, 2);
    data = readBuf[1];
    data |= (0x0F & readBuf[0]) << 8; // MSB

    // CONVERT 12 bit, into 2 complement
    signed_data = (int16_t)data;
        
    // Decode Temp measurement 
    return (((float)signed_data)/10.0);
}

/*****************************************************************************/
/*                Methods that can be used in standby Mode                   */
/*****************************************************************************/


int
ADXRS290::readX()
{
    volatile uint16_t data = 0;
    volatile uint8_t lsb = 0;
    volatile uint8_t msb = 0;
    int16_t signed_data = 0;

    lsb = readRegister(ADXRS290_GYR_X_L);
    msb = readRegister(ADXRS290_GYR_X_H);

    data = lsb;
    data |= msb << 8; // MSB

    signed_data = (int16_t)data;

    return signed_data;
}


int
ADXRS290::readY()
{
    volatile uint16_t data = 0;
    volatile uint8_t lsb = 0;
    volatile uint8_t msb = 0;
    int16_t signed_data = 0;

    lsb = readRegister(ADXRS290_GYR_Y_L);
    msb = readRegister(ADXRS290_GYR_Y_H);

    data = lsb;
    data |= msb << 8; // MSB

    signed_data = (int16_t)data;

    return signed_data;
}

float
ADXRS290::standbyReadTemperature()
{
    int16_t signed_data = 0;
    volatile uint16_t data = 0;    
    volatile uint8_t lsb = 0;
    volatile uint8_t msb = 0;

    // In standby we can read one byte at the time
    lsb = readRegister(ADXRS290_TEMP_L);
    msb = readRegister(ADXRS290_TEMP_H);


    data = lsb;
    data |= (0xFF & msb) << 8; // MSB

    // CONVERT 12 bit, into 2 complement
    signed_data = (int16_t)data;
    
    // Decode Temp measurement
    return (((float)signed_data)/10.0);
}

void
ADXRS290::standbyReadXY(float *x, float *y)
{
    volatile int16_t i16x, i16y;
   
    // In standby we can read one byte at the time
    i16x = readX();
    i16y = readY();
    
    *x  = (float) i16x / 200.0; // sensitivity 1/200 per LSB
    *y  = (float) i16y / 200.0;
}

ADXRS290 adiGyroscope;
