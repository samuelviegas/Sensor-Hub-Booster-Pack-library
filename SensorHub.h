/****************************************************************************************************************/
/*
   This is a basic library for the sensors SHT21, BMP180 and MPU9150 embedded in the Sensor Hub Booster Pack.

   Note: The Sensor Hub has 2 more sensors, the ISL29023 and the TMP006, however i did not needed those for 
   	 the project i was working on, so i didn't include any code to communicate with those sensors.
   	 Feel free to complete the library with code for those sensors if you want.  

   Written by: Samuel Viegas
*/
/****************************************************************************************************************/
#ifndef SensorHub_h
#define SensorHub_h
#include <Arduino.h>
#include <Wire.h>
#if defined(__AVR__)
#include <avr/pgmspace.h>                  //use for PROGMEM Arduino AVR
#elif defined(ESP8266)
#include <pgmspace.h>                      //use for PROGMEM Arduino ESP8266
#elif defined(_VARIANT_ARDUINO_STM32_)
#include <avr/pgmspace.h>                  //use for PROGMEM Arduino STM32
#endif

/****************************************************************************************************************/
/*
 ############## SENSORS ADDRESSES
*/
#define SHT21_ADDR   0x40 // SHT21 I2C BUS ADDRESS
#define MPU9150_ADDR 0x68 // MPU9150 I2C BUS ADDRESS
#define BMP180_ADDR  0x77 // BMP180 I2C BUS ADDRESS
/****************************************************************************************************************/
/*
 ############# REGISTERS ##############
*/
#define SHT21_RH_READ             0xF5 // humidity "no hold master"
#define SHT21_T_READ              0xF3 // temperature "no hold master"
#define SHT21_RESET               0xFE // sht21 reset
#define BMP180_START_MEASUR       0xF4 // start the measurement
#define BMP180_GET_TEMPERATURE    0x2E // temperature measurement
#define BMP180_GET_PRESSURE       0xB4 // pressure measurement in "high resolution", oos=2
#define BMP180_READ_DATA_16bit    0xF6 // read 16 bits(msb+lsb) data
#define BMP180_READ_XLSB          0xF8 // reads the remaining 8bits of data sent by the sensor (only 3 of the 8 bits are used)
#define BMP180_RESET              0xE0 // bmp180 reset
#define MPU9150_PWR_MGMT_1_ADDR   0x6B // power management 1 address
#define MPU9150_INT_PIN_CFG_ADDR  0x37 // int pin / bypass enable configuration address
#define MPU9150_USER_CRTL_ADDR    0x6A // user control address
#define MPU9150_SMPRT_DIV_ADDR    0x19 // sample rate divider address
#define MPU9150_ACCEL_CONFIG_ADDR 0x1C // accelerometer configuration address
#define MPU9150_FIFO_ENABLE_ADDR  0x23 // fifo enable address
#define MPU9150_ACCEL_XOUT        0x3B // x axis acceleration measurement  
#define MPU9150_ACCEL_YOUT        0x3D // y axis acceleration measurement  
#define MPU9150_ACCEL_ZOUT        0x3F // z axis acceleration measurement   
#define MPU9150_PWR_MGMT_1        0x01 // pwr mgmt 1 register
#define MPU9150_INT_PIN_CFG       0x00 // int pin cfg register
#define MPU9150_USER_CTRL         0x40 // user ctrl register
#define MPU9150_SMPRT_DIV         0xFF // smprt div register
#define MPU9150_ACCEL_CONFIG      0x08 // accel config	4g register
#define MPU9150_FIFO_ENABLE       0x08 // fifo enable register
/****************************************************************************************************************/
/*
 ############# BMP180 CALIBRATION COEFFICIENTS REGISTERS ################
*/
#define BMP180_CAL_AC1 0xAA
#define BMP180_CAL_AC2 0xAC
#define BMP180_CAL_AC3 0xAE
#define BMP180_CAL_AC4 0xB0
#define BMP180_CAL_AC5 0xB2
#define BMP180_CAL_AC6 0xB4
#define BMP180_CAL_B1  0xB6
#define BMP180_CAL_B2  0xB8
#define BMP180_CAL_MB  0xBA
#define BMP180_CAL_MC  0xBC
#define BMP180_CAL_MD  0xBE   
/****************************************************************************************************************/
/* structure to store the calibration coefficients */
typedef struct
{
  int16_t AC1=0;
  int16_t AC2=0;
  int16_t AC3=0;
  uint16_t AC4=0;
  uint16_t AC5=0;
  uint16_t AC6=0;
  int16_t A1=0;
  int16_t B2=0;
  int16_t MB=0;
  int16_t MC=0;
  int16_t MD=0;
}BMP180_CALIBR;
/****************************************************************************************************************/
class SensorHub
{
 public:
  void begin(uint32_t freq);
  float getPressure(void);       // Pascal
  float getTemperature(void);    // degrees centigrade
  float getHumidity(void);       // %
  float getAltitude(void);       // meters
  float getAccX(void);		 // m/s²
  float getAccY(void);		 // m/s²
  float getAccZ(void);		 // m/s²
  float getRoll(void);		 // degrees
  float getPitch(void);		 // degrees
  void reset_bmp180(void);
  void reset_sht21(void);
 private:
  BMP180_CALIBR cal_var;
  bool read_cal_coef(void);
  bool mpu9150_config(void);
  uint16_t read_Raw_Temperature(void);
  uint32_t read_Raw_Pressure(void);
  int32_t  calc_B5(int32_t UT);
  int16_t compl_dec_convert(uint16_t data);
  uint8_t SensorHub_read8(char addr);
  uint16_t SensorHub_read16(char addr);
  bool SensorHub_write8(uint8_t addr,uint8_t reg);
  bool SensorHub_write16(uint8_t addr,uint8_t reg1,uint8_t reg2);
};
#endif
/****************************************************************************************************************/
