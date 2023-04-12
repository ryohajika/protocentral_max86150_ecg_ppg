/***************************************************
  Arduino library written for the Maxim MAX86150 ECG and PPG integrated sensor

	Written by Ashwin Whitchurch, ProtoCentral Electronics (www.protocentral.com)

	https://github.com/protocentral/protocentral_max86150

  Based on code written by Peter Jansen and Nathan Seidle (SparkFun) for the MAX30105 sensor
  BSD license, all text above must be included in any redistribution.
 *****************************************************/

#pragma once

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define MAX86150_ADDRESS          0x5E //7-bit I2C Address
//Note that MAX30102 has the same I2C address and Part ID

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

//Define the size of the I2C buffer based on the platform the user has
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

  //I2C_BUFFER_LENGTH is defined in Wire.H
  #define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

  //SAMD21 uses RingBuffer.h
  #define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#else

  //The catch-all default is 32
  #define I2C_BUFFER_LENGTH 32

#endif

static const uint8_t MAX86150_INTSTAT1 =        0x00;
static const uint8_t MAX86150_INTSTAT2 =        0x01;
static const uint8_t MAX86150_INTENABLE1 =        0x02;
static const uint8_t MAX86150_INTENABLE2 =        0x03;

static const uint8_t MAX86150_FIFOWRITEPTR =     0x04;
static const uint8_t MAX86150_FIFOOVERFLOW =     0x05;
static const uint8_t MAX86150_FIFOREADPTR =     0x06;
static const uint8_t MAX86150_FIFODATA =        0x07;

static const uint8_t MAX86150_FIFOCONFIG =         0x08;
static const uint8_t MAX86150_FIFOCONTROL1 =    0x09;
static const uint8_t MAX86150_FIFOCONTROL2 =     0x0A;

static const uint8_t MAX86150_SYSCONTROL =      0x0D;
static const uint8_t MAX86150_PPGCONFIG1 =         0x0E;
static const uint8_t MAX86150_PPGCONFIG2 =         0x0F;
static const uint8_t MAX86150_PROXINTTHRESH =   0x10;

static const uint8_t MAX86150_LED1_PULSEAMP =     0x11;   // IR
static const uint8_t MAX86150_LED2_PULSEAMP =     0x12;   // Red
static const uint8_t MAX86150_LED_RANGE =       0x14;
static const uint8_t MAX86150_LED_PILOT_PA     =     0x15;

static const uint8_t MAX86150_ECG_CONFIG1     =     0x3C;
static const uint8_t MAX86150_ECG_CONFIG3     =     0x3E;

static const uint8_t MAX86150_PARTID =             0xFF;

// MAX86150 Commands
// FIFO Almost Full Flag (INT1)
static const uint8_t MAX86150_INT_A_FULL_MASK =         (byte)~0b10000000;
static const uint8_t MAX86150_INT_A_FULL_ENABLE =         0x80;
static const uint8_t MAX86150_INT_A_FULL_DISABLE =         0x00;

// new PPG FIFO data ready (INT1)
static const uint8_t MAX86150_INT_DATA_RDY_MASK =       (byte)~0b01000000;
static const uint8_t MAX86150_INT_DATA_RDY_ENABLE =     0x40;
static const uint8_t MAX86150_INT_DATA_RDY_DISABLE =    0x00;

// Ambient Light Cancellation Overflow (INT1)
static const uint8_t MAX86150_INT_ALC_OVF_MASK =        (byte)~0b00100000;
static const uint8_t MAX86150_INT_ALC_OVF_ENABLE =         0x20;
static const uint8_t MAX86150_INT_ALC_OVF_DISABLE =     0x00;

// Proximity Interrupt (INT1)
static const uint8_t MAX86150_INT_PROX_INT_MASK =       (byte)~0b00010000;
static const uint8_t MAX86150_INT_PROX_INT_ENABLE =     0x10;
static const uint8_t MAX86150_INT_PROX_INT_DISABLE =    0x00;

// Power Ready Flag (INT1)
static const uint8_t MAX86150_INT_PWR_RDY_MASK =        (byte)~0b00000001;
static const uint8_t MAX86150_INT_PWR_RDY_ENABLE =      0x01;
static const uint8_t MAX86150_INT_PWR_RDY_DISABLE =     0x00;

// New ECG FIFO data ready (INT2)
static const uint8_t MAX86150_INT_ECG_RDY_MASK =        (byte)~0b00000100;
static const uint8_t MAX86150_INT_ECG_RDY_ENABLE =      0x04;
static const uint8_t MAX86150_INT_ECG_RDY_DISABLE =     0x00;

// INT EN 1 0x02
// [7: A_FULL_EN][6: PPG_RDY_EN][5: ALC_OVF_ON][4: PROX_INT_EN]...

// INT EN 2 0x03
// [7: VDD_OOR_EN]...[2: ECG_RDY_EN]...

static const uint8_t MAX86150_SAMPLEAVG_MASK =  (byte)~0b00000111;
static const uint8_t MAX86150_SAMPLEAVG_1 =     0x00;
static const uint8_t MAX86150_SAMPLEAVG_2 =     0x01;
static const uint8_t MAX86150_SAMPLEAVG_4 =     0x02;
static const uint8_t MAX86150_SAMPLEAVG_8 =     0x03;
static const uint8_t MAX86150_SAMPLEAVG_16 =     0x04;
static const uint8_t MAX86150_SAMPLEAVG_32 =     0x05;

static const uint8_t MAX86150_ROLLOVER_MASK =     0xEF;
static const uint8_t MAX86150_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX86150_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX86150_A_FULL_MASK =     0xF0;

static const uint8_t MAX86150_SHUTDOWN_MASK =     (byte)~0b00000010;
static const uint8_t MAX86150_SHUTDOWN =         0x02;
static const uint8_t MAX86150_WAKEUP =             0x00;

static const uint8_t MAX86150_RESET_MASK =         (byte)~0b00000001;
static const uint8_t MAX86150_RESET =             0x01;

static const uint8_t MAX86150_MODE_MASK =         0xF8;
static const uint8_t MAX86150_MODE_REDONLY =     0x02;
static const uint8_t MAX86150_MODE_REDIRONLY =     0x03;
static const uint8_t MAX86150_MODE_MULTILED =     0x07;

static const uint8_t MAX86150_ADCRANGE_MASK =     (byte)~0b11000000;
static const uint8_t MAX86150_ADCRANGE_4096 =     0x00;
static const uint8_t MAX86150_ADCRANGE_8192 =     0x40;
static const uint8_t MAX86150_ADCRANGE_16384 =     0x80;
static const uint8_t MAX86150_ADCRANGE_32768 =     0xC0;

static const uint8_t MAX86150_SAMPLERATE_MASK = (byte)~0b00111100;
static const uint8_t MAX86150_SAMPLERATE_50 =     0x04;
static const uint8_t MAX86150_SAMPLERATE_100 =     0x10;
static const uint8_t MAX86150_SAMPLERATE_200 =     0x14;
static const uint8_t MAX86150_SAMPLERATE_400 =     0x18;
static const uint8_t MAX86150_SAMPLERATE_800 =     0x1C;
static const uint8_t MAX86150_SAMPLERATE_1000 = 0x2C;
static const uint8_t MAX86150_SAMPLERATE_1600 = 0x24;
static const uint8_t MAX86150_SAMPLERATE_3200 = 0x28;

static const uint8_t MAX86150_PULSEWIDTH_MASK = (byte)~0b00000011;
static const uint8_t MAX86150_PULSEWIDTH_50 =     0x00;
static const uint8_t MAX86150_PULSEWIDTH_100 =     0x01;
static const uint8_t MAX86150_PULSEWIDTH_200 =     0x02;
static const uint8_t MAX86150_PULSEWIDTH_400 =     0x03;

static const uint8_t MAX86150_LED1_RGE_MASK =   (byte)~0b00000011;
static const uint8_t MAX86150_LED1_RGE_50 =     0x00;
static const uint8_t MAX86150_LED1_RGE_100 =    0x01;
static const uint8_t MAX86150_LED2_RGE_MASK =   (byte)~0b00001100;
static const uint8_t MAX86150_LED2_RGE_50 =     0x00;
static const uint8_t MAX86150_LED2_RGE_100 =    0x04;

static const uint8_t MAX86150_SLOT1_MASK =         0xF0;
static const uint8_t MAX86150_SLOT2_MASK =         0x0F;
static const uint8_t MAX86150_SLOT3_MASK =         0xF0;
static const uint8_t MAX86150_SLOT4_MASK =         0x0F;

static const uint8_t SLOT_NONE =                0x00;
static const uint8_t SLOT_RED_LED =             0x01;
static const uint8_t SLOT_IR_LED =              0x02;
static const uint8_t SLOT_RED_PILOT =           0x09;
static const uint8_t SLOT_IR_PILOT =            0x0A;
static const uint8_t SLOT_ECG =                 0x0D;

static const uint8_t MAX_30105_EXPECTEDPARTID = 0x1E;


class MAX86150 {
 public:
  MAX86150(void);

  boolean begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX86150_ADDRESS);

  uint32_t getRed(void); //Returns immediate red value
  uint32_t getIR(void); //Returns immediate IR value
  int32_t getECG(void); //Returns immediate ECG value
  bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

  // Configuration
  void softReset();
  void shutDown();
  void wakeUp();

  void setLEDMode(uint8_t mode);

  void setADCRange(uint8_t adcRange);
  void setSampleRate(uint8_t sampleRate);
  void setPulseWidth(uint8_t pulseWidth);

  void setPulseAmplitudeRed(uint8_t value);
  void setPulseAmplitudeIR(uint8_t value);
  void setPulseAmplitudeProximity(uint8_t value);

  void setProximityThreshold(uint8_t threshMSB);

  //Multi-led configuration mode (page 22)
  void enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
  void disableSlots(void);

  // Data Collection

  //Interrupts (page 13, 14)
  uint8_t getINT1(void); //Returns the main interrupt group
  uint8_t getINT2(void); //Returns the temp ready interrupt
  void enableAFULL(void); //Enable/disable individual interrupts
  void disableAFULL(void);
  void enableDATARDY(void);
  void disableDATARDY(void);
  void enableALCOVF(void);
  void disableALCOVF(void);
  void enablePROXINT(void);
  void disablePROXINT(void);
  void enableDIETEMPRDY(void);
  void disableDIETEMPRDY(void);

  //FIFO Configuration (page 18)
  void setFIFOAverage(uint8_t samples);
  void enableFIFORollover();
  void disableFIFORollover();
  void setFIFOAlmostFull(uint8_t samples);

  //FIFO Reading
  uint16_t check(void); //Checks for new data and fills FIFO
  uint8_t available(void); //Tells caller how many new samples are available (head - tail)
  void nextSample(void); //Advances the tail of the sense array
  uint32_t getFIFORed(void); //Returns the FIFO sample pointed to by tail
  uint32_t getFIFOIR(void); //Returns the FIFO sample pointed to by tail
  int32_t getFIFOECG(void); //Returns the FIFO sample pointed to by tail

  uint8_t getWritePointer(void);
  uint8_t getReadPointer(void);
  void clearFIFO(void); //Sets the read/write pointers to zero

  //Proximity Mode Interrupt Threshold
  void setPROXINTTHRESH(uint8_t val);

  // Die Temperature
  float readTemperature();
  float readTemperatureF();

  // Detecting ID/Revision
  uint8_t getRevisionID();
  uint8_t readPartID();
	uint8_t readRegLED();

  // Setup the IC with user selectable settings
  void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);
  void ppgSetup(bool enRed, bool enIR,
                uint8_t redCurrentRange, uint8_t irCurrentRange,
                uint8_t redPower, uint8_t irPower, uint8_t sampleAverage,
                uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange);

  // Low-level I2C communication
  uint8_t readRegister8(uint8_t address, uint8_t reg);
  void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

 private:
  TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
  int _i2caddr;

  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  byte activeDevices; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

  uint8_t revisionID;

  void readRevisionID();

  void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

   #define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
  typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    int32_t ecg[STORAGE_SIZE];
    byte head;
    byte tail;
  } sense_struct; //This is our circular buffer of readings from the sensor

  sense_struct sense;

};
