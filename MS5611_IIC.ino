/*
 * The aim is to write IIC to read from MS5611 using standard arduino library
 * <Wire.h>
 * 
 * Training to write IIC according to its datasheet
 * Referring to MS5611-01BA03 datasheet

  The MS5611-01BA has only five basic commands:
  1. Reset
  2. Read PROM (128 bit of calibration words)
  3. D1 conversion
  4. D2 conversion
  5. Calculate temperature 
  6. Calculate temperature compensated pressure
 * 
 * references: Rob Tillaart. VERSION: 0.2.1
 * 1. https://github.com/RobTillaart/MS5611/blob/master/MS5611.cpp
 * 2. https://github.com/RobTillaart/MS5611/blob/master/MS5611.h
 *
 */


#include<Wire.h>

// datasheet page 10
#define MS5611_CMD_READ_ADC       0x00
#define MS5611_CMD_READ_PROM      0xA0
#define MS5611_CMD_RESET          0x1E
#define MS5611_CMD_CONVERT_D1     0x40
#define MS5611_CMD_CONVERT_D2     0x50
#define MS5611_address            0x77

// public variables
int _result = -999;

//byte prom[16];    // merge prom reading and C in a function. 
uint16_t  C[8]; // C[0] and C[7] not used
byte ADC_D1[3];
byte ADC_D2[3];
char output[512];
char output2[512];

// public variables
// calibration data
uint32_t C1;    // must be unsigned
uint32_t C2;    // must be unsigned
float C3;
float C4;
uint32_t C5;    // must be unsigned
float C6;
uint32_t dT;    // difference between actual and reference temperature

// 1. Reset
void ms5611_reset()
{
  Wire.beginTransmission(MS5611_address);
  Wire.write(MS5611_CMD_RESET);
 
  _result = Wire.endTransmission();

}


// 2. Read PROM (128 bit of calibration words)
// read from 8 register (16 bits) - two bytes per read. then concatenate.
// Ex: 0xA1 = upper byte, 0xA0 = lower byte
// address 0 - 7
// 0xA0,0xA2,0xA4,0xA6,0xA8,0xAA,0xA8,0xAC
// then merge 2 bytes into C[i] of 16-bit length
// example: C[1] = uppper_byte + lower_byte

void ms5611_readProm()
{

for (uint8_t reg = 0; reg < 8; reg++)
  {
    uint8_t offset = reg * 2;
    Wire.beginTransmission(MS5611_address);
    Wire.write(MS5611_CMD_READ_PROM + offset); // starting from 0xA0, then offset of 2 ==> 0xA2...
    Wire.endTransmission();
  
    Wire.beginTransmission(MS5611_address);
    Wire.requestFrom(MS5611_address, 2);      // read 2 bytes from PROM. First read = upper_byte, second read = lower_byte

    int i = 0;
    while(Wire.available())
    {
     
      if (i == 0){  // first read  // by default 0x100  ==> 0x0100 == 16-bits
          C[reg] = Wire.read()*0x0100; // shift to left 1 byte // upper byte
          }
      if (i == 1){  // second read
          C[reg] += Wire.read(); // shift to left 1 byte // upper byte
          }
      i++;
     }
     Serial.println(C[reg], HEX); // check the data
  }
  Wire.endTransmission();

// once calibration data C1-C6 are read. Multiply with the magic number (refer to datasheet).
// public variables
// Prepare calibration data for later calculation of temperature and pressure.
// arrange C1 - C6  from prom[16]
C1  = C[1]*32768L;         //L = 32-bits.  2^15 = 0x00008000
C2  = C[2]*65536L;         //L = 32-bits.  2^16 = 0x00010000
C3  = C[3]*3.90625E-3;     // 1/2^8
C4  = C[4]*7.8125E-3;      // 1/2^7

// C5 = C[5] * 2^8 
// 32-bits. 256L = 0x0000 0100
// if only 256 = 0x0100 by default. which result C[5] upper byte being trimmed after multiplied.
C5  = C[5]*256L;
C6  = C[6]*1.1920928955E-7;  // C6 = C[6] / 2^23

 }

// 3. D1 conversion
// Digital Pressure value
void ms5611_D1()
{
  Wire.beginTransmission(MS5611_address);
  Wire.write(MS5611_CMD_CONVERT_D1); // starting from 0x40
  Wire.endTransmission();

  delay(1);

  // read ADC
  Wire.beginTransmission(MS5611_address);
  Wire.write(MS5611_CMD_READ_ADC); // starting from 0x00
  Wire.endTransmission();
  
  Wire.beginTransmission(MS5611_address);
  Wire.requestFrom(MS5611_address, 3);

  int i = 0;
  while(Wire.available())
  {
    ADC_D1[i] = Wire.read();
    /*
    Serial.print("ADC_D1 ");
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print(" = ");
    Serial.println(ADC_D1[i], HEX);
    */
    i++;
    }
}

// 4. D2 conversion
// Digital temperature value
void ms5611_D2()
{
  Wire.beginTransmission(MS5611_address);
  Wire.write(MS5611_CMD_CONVERT_D2); // starting from 0x50
  Wire.endTransmission();

  delay(1);

  // read ADC
  Wire.beginTransmission(MS5611_address);
  Wire.write(MS5611_CMD_READ_ADC); // starting from 0x00
  Wire.endTransmission();
  
  Wire.beginTransmission(MS5611_address);
  Wire.requestFrom(MS5611_address, 3);

  int i = 0;
  while(Wire.available())
  {
    ADC_D2[i] = Wire.read();
    /*
    Serial.print("ADC_D2 ");
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print(" = ");
    Serial.println(ADC_D2[i], HEX);
    */
    i++;
    }
}

// 5. Calculate temperature
// refer to the datasheet. The formula

uint32_t calTemperature()
{
  // signed int 32
  // formula
  // dT = D2 - C5*2^8
  // Temp = 2000 + dT * C6/2^23

  // merge 3 bytes of D2 ADC_read
  // this method 1: using multiplier
  //uint32_t D2_val = ADC_D2[0] * 65536UL; // upper byte // 65536UL; // shift left 3 bytes
  //D2_val += ADC_D2[1] * 256UL; //256UL; //second byte // shift left 2 bytes
  //D2_val += ADC_D2[2];

  // or using this method: using bit left-shift and merge
  uint32_t D2_val = (((uint32_t)ADC_D2[0]) << 16) | (((uint32_t)ADC_D2[1]) << 8) | ADC_D2[2];
 
 /*
  Serial.print("D2 value = ");
  Serial.print(D2_val);
  Serial.print("    ");
  Serial.println(D2_val, HEX);

  Serial.print("C1 =  ");
  Serial.print(C1, HEX);
  Serial.print("    ");
  Serial.println(C1);

  Serial.print("C2 =  ");
  Serial.print(C2, HEX);
  Serial.print("    ");
  Serial.println(C2);

  Serial.print("C3 =  ");
  Serial.print(C3, HEX);
  Serial.print("    ");
  Serial.println(C3, 10);
  
  Serial.print("C4 =  ");
  Serial.print(C4, HEX);
  Serial.print("    ");
  Serial.println(C4, 10);

  Serial.print("C5 =  ");
  Serial.print(C5, HEX);
  Serial.print("    ");
  Serial.println(C5);

  Serial.print("C6 =  ");
  Serial.print(C6, HEX);
  Serial.print("    ");
  Serial.println(C6, 10);
*/

 // applying the formula
  dT = D2_val - C5;       //dT = D2 - C5*2^8
  uint32_t Temp = 2000 + dT*C6;    //Temp = 2000 + dT * C6/2^23
  //Serial.print("dT=  ");
  //Serial.println(dT);
  
  //Serial.print("dT =  ");
  //Serial.println(dT);
  //Serial.print("Temp=  ");
  //Serial.println(Temp);

  return Temp;
}


// 6. Calculate temperature compensated pressure
// signed int 64
// use calibration data C1-C5
// offset at actual temperature, OFF = C2*2^16 + (C4*dT)/2^7
// Sensitivity at actual temperature. SENS = C1*2^15 + (C3*dT)/2^8
// Temperature compensated pressure, P = (D1*SENS/2^21-OFF)/2^15

float calPressure()
{
  // applying public variables, C1-> C6
  // offset at actual temperature, OFF = C2+ C4*dT
  float OFF = C2 + C4*dT;
  //Serial.print("offset = ");
  //Serial.println(OFF);
  // Sensitivity at actual temperature. SENS = C1*2^15 + (C3*dT)/2^8
  float SENS = C1 + C3*dT;
  //Serial.print("SENS = ");
  //Serial.println(SENS);
  
  // Temperature compensated pressure, P = (D1*SENS*k1-OFF)*k2
  float k1  = 4.76837158E-7;  // 1/2^21
  float k2  = 3.051757813E-5;  // 1/2^15
  // or using this method: using bit left-shift and merge
  uint32_t D1_val = (((uint32_t)ADC_D1[0]) << 16) | (((uint32_t)ADC_D1[1]) << 8) | ADC_D1[2];
  //Serial.print("D1 value = ");
  //Serial.print(D1_val);
  //Serial.print("    ");
  //Serial.println(D1_val, HEX);
  // P = (D1_val*SENS*k1-OFF)k2
  float P = D1_val*SENS*k1-OFF;
  P *=k2;
  
  return P;
  
  //Serial.print("pressure = ");
  //Serial.println(P);
}


void setup()
{
  Wire.begin();
  Serial.begin(9600);
  //Serial.print(" before reset, result = ");
  //Serial.println(_result);

    // Step 1: reset
  ms5611_reset(); // ** address 0x77
  //Serial.print(" after reset, result = ");
  //Serial.println(_result);
  delay(5);

  ms5611_readProm();

}

void loop()
{
  ms5611_D1();
  ms5611_D2();
  Serial.print("Temperature = ");
  Serial.print(calTemperature()*0.01, 2);
  Serial.print("°");
  Serial.print("C");
  Serial.print("\t\t");
  Serial.print("Pressure = ");
  Serial.print(calPressure()*0.01,2);
  Serial.println(" mBar");
  delay(500);
}
