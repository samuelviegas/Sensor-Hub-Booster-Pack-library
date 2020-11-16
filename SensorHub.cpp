/****************************************************************************************************************/
/*
   This is a basic library for the sensors SHT21, BMP180 and MPU9150 embedded in the Sensor Hub Booster Pack.

   Note: The Sensor Hub has 2 more sensors, the ISL29023 and the TMP006, however i did not needed those for 
   	 the project were i used the Sensor Hub, so i didn't include any code to communicate with those sensors.
   	 Feel free to complete the library with code for those sensors if you want.  

   Written by: Samuel Viegas
*/
/****************************************************************************************************************/
#include "Arduino.h"
#include "SensorHub.h"
#include <Wire.h>
/****************************************************************************************************************/
/*
    begin()

    Initializes the I2C bus communication with the frequency freq
*/
void SensorHub::begin(uint32_t freq) 
{
  Wire.begin();
  // initializes with the default frequency of 100kHz if its initialize as 0
  if(freq<=0)
	  freq=100000;
  Wire.setClock(freq);
  if(read_cal_coef())
	  Serial.println("Calibration coefficients successfully read from the BMP180 sensor!");
  else
	Serial.println("An error occur while trying to get the calibration coefficients from the BMP180 sensor!");
  if(mpu9150_config())
	  Serial.println("MPU9150 sensor successfully initialized!");
  else
	Serial.println("An error occur while initializing MPU9150 sensor!");
  delay(250);
}
/****************************************************************************************************************/
/*
    SENSOR MPU9150

    mpu9150_config()

    Initializes the MPU9150 sensor
*/
bool SensorHub::mpu9150_config()
{
  char cont=0;
  char error[42];
  if(SensorHub_write16(MPU9150_ADDR,MPU9150_PWR_MGMT_1_ADDR,MPU9150_PWR_MGMT_1))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  if(SensorHub_write16(MPU9150_ADDR,MPU9150_INT_PIN_CFG_ADDR,MPU9150_INT_PIN_CFG))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  if(SensorHub_write16(MPU9150_ADDR,MPU9150_USER_CRTL_ADDR,MPU9150_USER_CTRL))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  if(SensorHub_write16(MPU9150_ADDR,MPU9150_SMPRT_DIV_ADDR,MPU9150_SMPRT_DIV))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  if(SensorHub_write16(MPU9150_ADDR,MPU9150_ACCEL_CONFIG_ADDR,MPU9150_ACCEL_CONFIG))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  if(SensorHub_write16(MPU9150_ADDR,MPU9150_FIFO_ENABLE_ADDR,MPU9150_FIFO_ENABLE))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  if(cont==0)
  	return true;  
  else
  {
	snprintf(erro,sizeof(error),"%d collisions occurred on the I2C bus",cont);
	Serial.println(error);
	return false;
  }   
}
/****************************************************************************************************************/
/*
    SENSOR BMP180

    read_cal_coef()

    Reads the calibration coefficients stored in the BMP180 sensor's EEPROM 
*/
bool SensorHub::read_cal_coef()
{
  char cont=0;
  char error[42];
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_AC1))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.AC1=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_AC2))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.AC2=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_AC3))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.AC3=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_AC4))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.AC4=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_AC5))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.AC5=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_AC6))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.AC6=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_B1))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.A1=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_B2))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.B2=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_MB))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.MB=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_MC))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.MC=SensorHub_read16(BMP180_ADDR);
  if(SensorHub_write8(BMP180_ADDR,BMP180_CAL_MD))
  {
	Serial.println("An error occurred! Collision on I2C bus!");
	cont++;
  }
  cal_var.MD=SensorHub_read16(BMP180_ADDR);
  if(cont==0)
  	return true;  
  else
  {
	snprintf(erro,sizeof(error),"%d collisions occurred on the I2C bus",cont);
	Serial.println(error);
	return false;
  }
}
/****************************************************************************************************************/
/*
    SENSOR BMP180

    getPressure()

    Calculates the pressure in Pascal

    Mathematical formulas provided in BMP180's datasheet
*/
float SensorHub::getPressure(void)
{
  int32_t  UT,UP,B3,B5,B6,X1,X2,X3,pressure;
  uint32_t B4,B7;
  UT=read_Raw_Temperature();
  UP=read_Raw_Pressure();
  B5=calc_B5(UT);
  B6=B5-4000;
  X1=((int32_t)cal_var.B2*((B6*B6)>>12))>>11;
  X2=((int32_t)cal_var.AC2*B6)>>11;
  X3=X1+X2;
  B3=((((int32_t)cal_var.AC1*4+X3)<<2)+2)/4;
  X1=((int32_t)cal_var.AC3*B6)>>13;
  X2=((int32_t)cal_var.A1*((B6*B6)>>12))>>16;
  X3=((X1+X2)+2)>>2;
  B4=((uint32_t)cal_var.AC4*(X3+32768L))>>15;
  B7=(UP-B3)*(50000UL>>2);
  if(B7<0x80000000)
	  pressure=(B7*2)/B4;
  else
	  pressure=(B7/B4)*2;
  X1=pow((pressure>>8),2);
  X1=(X1*3038L)>>16;
  X2=(-7357L*pressure)>>16;
  return pressure=pressure+((X1+X2+3791L)>>4);
}
/****************************************************************************************************************/
/*
    SENSOR BMP180

    getAltitude()

    Calculates the altitude in meters

*/
float SensorHub::getAltitude(void)
{
  uint32_t atm=101325;	// 101325 Pascal -> Mean sea-level atmospheric pressure on Earth
  float pres=getPressure();
  return (44330*(1-(pow((pres/atm),0.1903))));
}
/****************************************************************************************************************/
/*
    SENSOR SHT21

    getTemperature()

    Calculates temperature in centigrade degrees (°C)
*/
float SensorHub::getTemperature(void)
{
  uint16_t temp;
  if(SensorHub_write8(SHT21_ADDR,SHT21_T_READ))
	Serial.println("An error occurred! Collision on I2C bus! Sensor: SHT21");
  delay(75); // sensor's conversion time
  temp=SensorHub_read16(SHT21_ADDR);
  temp&=~0x0003; // it turns the 2 least significant bits to '0', accordingly to the datasheet information
  return (-46.85+((175.72/65536)*((float)temp)));
}
/****************************************************************************************************************/
/*
    SENSOR SHT21

    getHumidity();

    Calculates humidity in % (ratio of the partial pressure of water vapor to the equilibrium vapor pressure of water at a given temperature)
*/
float SensorHub::getHumidity(void)
{
  uint16_t hum;
  if(SensorHub_write8(SHT21_ADDR,SHT21_RH_READ))
	Serial.println("An error occurred! Collision on I2C bus! Sensor: SHT21");
  delay(50); // sensor's conversion time
  hum=SensorHub_read16(SHT21_ADDR);
  hum&=~0x0003; // it turns the 2 least significant bits to '0', accordingly to the datasheet information
  return (-6.0+((125.0/65536)*((float)hum)));
}
/****************************************************************************************************************/
/*
    SENSOR MPU9150

    getAccX();

    Calculates the acceleration in the X axis in m/s²

*/
float SensorHub::getAccX(void)
{
  int16_t accx;
  if(SensorHub_write8(MPU9150_ADDR,MPU9150_ACCEL_XOUT))
  	Serial.println("An error occurred! Collision on I2C bus! Sensor: MPU9150");
  accx=compl_dec_convert(SensorHub_read16(MPU9150_ADDR));
  return (((float)accx*4)/32768)*(-9.81);
}
/****************************************************************************************************************/
/*
    SENSOR MPU9150

    getAccY();

    Calculates the acceleration in the Y axis in m/s²

*/
float SensorHub::getAccY(void)
{
  int16_t accy;
  if(SensorHub_write8(MPU9150_ADDR,MPU9150_ACCEL_YOUT))
  	Serial.println("An error occurred! Collision on I2C bus! Sensor: MPU9150"); 
  accy=compl_dec_convert(SensorHub_read16(MPU9150_ADDR));
  return (((float)accy*4)/32768)*(-9.81);
}
/****************************************************************************************************************/
/*
    SENSOR MPU9150

    getAccZ();

    Calculates the acceleration in the Z axis in m/s²

*/
float SensorHub::getAccZ(void)
{
  int16_t accz;
  if(SensorHub_write8(MPU9150_ADDR,MPU9150_ACCEL_ZOUT))
  	Serial.println("An error occurred! Collision on I2C bus! Sensor: MPU9150");
  accz=compl_dec_convert(SensorHub_read16(MPU9150_ADDR));
  return (((float)accz*4)/32768)*9.81;
}
/****************************************************************************************************************/
/*
    SENSOR MPU9150

    getRoll();

    Calculates the longitudinal axis (roll) in degrees

*/
float SensorHub::getRoll(void)
{
  float acc_z=getAccZ();
  float acc_x=getAccX();
  return round(atan2(acc_x,acc_z)*(float)(180/3.14159265));
}
/****************************************************************************************************************/
/*
    SENSOR MPU9150

    getPitch();

    Calculates the transverse axis (pitch) in degrees

*/
float SensorHub::getPitch(void)
{
  float acc_z=getAccZ();
  float acc_y=getAccY();
  return round(atan2(acc_y,acc_z)*(float)(180/3.14159265));
}
/****************************************************************************************************************/
/*
    reset_bmp180();

    Resets BMP180 sensor
*/
void SensorHub::reset_bmp180(void)
{
  if(SensorHub_write8(BMP180_ADDR,BMP180_RESET))
	  Serial.println("An error occurred! Collision on I2C bus! (reset)");
  else
	  Serial.println("Reset concluded (BMP180)!");
}
/****************************************************************************************************************/
/*
    reset_sht21();

    Resets SHT21 sensor
*/
void SensorHub::reset_sht21(void)
{
  if(SensorHub_write8(SHT21_ADDR,SHT21_RESET))
	  Serial.println("An error occurred! Collision on I2C bus! (reset)");
  else
	  Serial.println("Reset concluded (SHT21)!");
}
/****************************************************************************************************************/
/*
    SENSOR BMP180

    read_Raw_Temperature();

    Reads raw temperature data in 16 bits
*/
uint16_t SensorHub::read_Raw_Temperature(void)
{
  if(SensorHub_write16(BMP180_ADDR,BMP180_START_MEASUR,BMP180_GET_TEMPERATURE)) // writes 16 bits,reg_addr + reg
	  Serial.println("An error occurred! Collision on I2C bus! Sensor: BMP180");
  delay(15); // sensor's conversion time
  if(SensorHub_write8(BMP180_ADDR,BMP180_READ_DATA_16bit))
	  Serial.println("An error occurred! Collision on I2C bus! Sensor: BMP180");
  return SensorHub_read16(BMP180_ADDR); // outputs msb + lsb (16bits)
}
/****************************************************************************************************************/
/*
    SENSOR BMP180

    read_Raw_Pressure()

    Reads the raw pressure value in 18 bits
    
*/
uint32_t SensorHub::read_Raw_Pressure(void)
{
  uint32_t rawPressure;
  if(SensorHub_write16(BMP180_ADDR,BMP180_START_MEASUR,BMP180_GET_PRESSURE)) // writes 16 bits,reg_addr + reg
	  Serial.println("An error occurred! Collision on I2C bus! Sensor: BMP180");
  delay(15); // sensor's conversion time
  if(SensorHub_write8(BMP180_ADDR,BMP180_READ_DATA_16bit))
 	  Serial.println("An error occurred! Collision on I2C bus! Sensor: BMP180");
  rawPressure=SensorHub_read16(BMP180_ADDR); // gets msb + lsb (16bits)
  rawPressure<<=8; // shift's 8 bits to the left
  if(SensorHub_write8(BMP180_ADDR,BMP180_READ_XLSB))
 	  Serial.println("An error occurred! Collision on I2C bus! Sensor: BMP180");
  rawPressure|=SensorHub_read8(BMP180_ADDR); // gets xlsb (8bits) and bitwise's OR with rawPressure e xlsb
  return rawPressure>>=6; // outputs the raw pressure in 18 bits, with 2 of them being resolution
}
/****************************************************************************************************************/
/*
    SENSOR BMP180

    calc_B5()

    Calculates B5 value
*/
int32_t SensorHub::calc_B5(int32_t UT)
{
  int32_t X1=((UT-(int32_t)cal_var.AC6)*(int32_t)cal_var.AC5)>>15;
  int32_t X2=((int32_t)cal_var.MC<<11)/(X1+(int32_t)cal_var.MD);
  return X1+X2;
}
/****************************************************************************************************************/
/*
    2compl_dec_convert();

    Converts two's complement to decimal
    	
*/
int16_t SensorHub::compl_dec_convert(uint16_t data)
{
  int16_t dec;
  if(data>32768)
	return dec=(~data+1)*(-1);
  else	
  	return data;
}
/****************************************************************************************************************/
/*
    SensorHub_read8()

    Reads an 8 bit value from a certain sensor in the Sensor Hub
    	
*/
uint8_t SensorHub::SensorHub_read8(char addr)
{
  uint8_t data;
  Wire.requestFrom(addr,1); // Requests 1 byte
  delay(50);
  data=Wire.read();
  return data;
}
/****************************************************************************************************************/
/*
    SensorHub_read16()

    Reads a 16 bit value from a certain sensor in the Sensor Hub
    	
*/
uint16_t SensorHub::SensorHub_read16(char addr)
{
  uint8_t data[2];
  Wire.requestFrom(addr,2); // Requests 2 bytes
  delay(50);
  data[0]=Wire.read(); // MSB
  data[1]=Wire.read(); // LSB
  return (data[0]<<8)+data[1]; // concatenates MSB and LSB in a 16-bit variable
}
/****************************************************************************************************************/
/*
    SensorHub_write8()

    Writes an 8 bit value to a certain sensor in the Sensor Hub
    	
*/
bool SensorHub::SensorHub_write8(uint8_t addr,uint8_t reg)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission())
      return 1;
  else
      return 0;
}
/****************************************************************************************************************/
/*
    SensorHub_write16()

    Writes a 16 bit value to a certain sensor in the Sensor Hub
    	
*/
bool SensorHub::SensorHub_write16(uint8_t addr,uint8_t reg1,uint8_t reg2)
{
  Wire.beginTransmission(addr);
  Wire.write(reg1);
  Wire.write(reg2);
  if(Wire.endTransmission())
      return 1;
  else
      return 0;
}
/****************************************************************************************************************/
