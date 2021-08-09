// Teensy 4.0 with MPU6050 accelerator and gyroscope
// Copyright (C) 2021 https://www.roboticboat.uk
// 36f0643a-6ac6-4842-b820-ae427d55a1f3
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


// NOTE There is currently is NO CALIBRATION of the MPU6050

#include <Wire.h>

#define GYRO_CONFIG_Register     0x1B
#define ACCEL_CONFIG_Register    0x1C
#define ACCEL_XOUT_H_Register    0x3B
#define PWR_MGMT_1_Register      0x6B


uint8_t _i2cAddress = 0x68;
uint8_t receiveBuffer[14];
int nReceived;
        
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t temp;

void setup() 
{
  // Keep User updated
  Serial.begin(38400);

  // Start the i2c bus
  Wire.begin();

  // Setup device
  SetupMPU6050(); 
}

void loop() 
{
  // Get latest readings
  readMPU6050(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ, &temp);

  // Update the User
  Serial.print("$ACC,\t");
  Serial.print(accelX*2.0f/32768,3); 
  Serial.print("\t");
  Serial.print(accelY*2.0f/32768,3);
  Serial.print("\t");
  Serial.print(accelZ*2.0f/32768,3);
  Serial.print(" g\t\t");
    
  Serial.print("$GYR,\t");
  Serial.print(gyroX*250.0f/32768,3); 
  Serial.print("\t");
  Serial.print(gyroY*250.0f/32768,3);  
  Serial.print("\t");
  Serial.print(gyroZ*250.0f/32768,3); 
  Serial.print(" dps\t");

  // Page 31 of MPU6050 manual
  Serial.print("$TMP,\t");
  Serial.print(temp/340.0f+36.53,1);
  Serial.println(" degrees C");
    
  delay(100);
}

void readMPU6050(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* temp) 
{
    readRegisters(ACCEL_XOUT_H_Register, 14, receiveBuffer);

    *ax = (( (int16_t) receiveBuffer[0]) << 8) | receiveBuffer[1];
    *ay = (( (int16_t) receiveBuffer[2]) << 8) | receiveBuffer[3];
    *az = (( (int16_t) receiveBuffer[4]) << 8) | receiveBuffer[5];
    *temp = (( (int16_t) receiveBuffer[6]) << 8) | receiveBuffer[7];
    *gx = (( (int16_t) receiveBuffer[8]) << 8) | receiveBuffer[9];
    *gy = (( (int16_t) receiveBuffer[10]) << 8) | receiveBuffer[11];
    *gz = (( (int16_t) receiveBuffer[12]) << 8) | receiveBuffer[13];
}


void SetupMPU6050()
{
  // Allocate memory.
  uint8_t currentValue;
  uint8_t newValue;
  uint8_t updatedValue;

  // PWR_MGMT_1_Register allows the user to configure the power mode and clock source
  // Upon power up, the clock source defaults to the internal oscillator. However, it is
  // recommended [in the manual] that the device be configured to use one of the gyroscopes
  // Set the clock select
  // Read the register
  readRegister(PWR_MGMT_1_Register, &currentValue);

  // Update new value
  // theseBytes                 0b00000xxx
  // Internal 8Mhz oscillator   0b00000000
  // PLL with X-axis gyroscope  0b00000001
  // PLL with Y-axis gyroscope  0b00000010
  // PLL with Z-axis gyroscope  0b00000011
  newValue = bitWriteByte(currentValue, 0b00000111, 0b00000001);

  // Also by setting Sleep bit to 1, the MPU6050 can be put into low power sleep mode
  // We don't want that so ensure Sleep bit is set to 0
  // Update value for correct instruction. Clear bit 6
  updatedValue = bitWriteByte(newValue, 0b01000000, 0b00000000);

  // Write to the register
  writeRegister(PWR_MGMT_1_Register, updatedValue);
  
  
  // Set the Accelerator sensitivity (page 15 of RM-MPU-6000A-00)
  // Read the register
  readRegister(ACCEL_CONFIG_Register, &currentValue);

  // Update new value
  // theseBytes     0b000xx000
  // ACCEL_FS_2     0b00000000 highest sensitivity +/- 2g
  // ACCEL_FS_4     0b00001000
  // ACCEL_FS_8     0b00010000
  // ACCEL_FS_16    0b00011000
  newValue = bitWriteByte(currentValue, 0b00011000, 0b00000000);

  // Write to the register
  writeRegister(ACCEL_CONFIG_Register, newValue);


  // Set the Gyroscope sensitivity
  // Read the register
  readRegister(GYRO_CONFIG_Register, &currentValue);

  // Update new value
  // theseBytes     0b000xx000
  // GYRO_FS_250    0b00000000 highest sensitivity
  // GYRO_FS_500    0b00001000
  // GYRO_FS_1000   0b00010000
  // GYRO_FS_2000   0b00011000
  newValue = bitWriteByte(currentValue, 0b00011000, 0b00000000);

  // Write to the register
  writeRegister(GYRO_CONFIG_Register, newValue);

}

//======== Bit operations =============

uint8_t bitWriteByte(uint8_t originalBits, uint8_t theseBits, uint8_t overrideBits)
{
  // originalBits is say 76543210  For example 01001110
  // theseBits    is say 000xxx00              00011100
  // overrideBits is say hgfedcba              00010100
  // Want the result     765edc10              01010110

  // Show the User the originalBits in binary
  //Serial.println(originalBits, BIN);

  // Set to zero the bits wanting to override in the originalBits
  originalBits = originalBits & ~(theseBits);
  //Serial.println(originalBits, BIN);

  // Set to zero the bits not interested in the overrideBits 
  overrideBits = overrideBits & theseBits;

  // Ok. Just need to bitwise combine
  uint8_t resultBits = originalBits | overrideBits;
  //Serial.println(resultBits, BIN);

  return resultBits;
}

//========== i2c commands ===============
//
// NACK errors are
// 0: OK - successful send
// 1: Send receiveBuffer too large for the twi receiveBuffer.
// 2: Register address was sent and a NACK received. Master should send a STOP condition.
// 3: Data was sent and a NACK received. Slave has sent all data. Master can send a STOP condition.
// 4: A different twi error took place
// 5: Time out on waiting for data to be received.
// 6: Checking data was correctly written failed

uint8_t readRegister(uint8_t registerAddress, uint8_t* dest)
{
  // Initialise the transmit message buffer
  Wire.beginTransmission(_i2cAddress); 
  
  // Add the i2c module Register address to transmit message buffer
  Wire.write(registerAddress); 

  // End transmission
  uint8_t nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if (nackCatcher != 0) return nackCatcher;
  
  // One byte to receive
  nReceived = Wire.requestFrom(_i2cAddress, (uint8_t)1); 

  // Something has gone wrong
  if (nReceived != 1) return (uint8_t)5;
  
  // Read the data
  *dest = Wire.read();

  // All ok
  return (uint8_t)0;
}

uint8_t readRegisters(uint8_t registerAddress, uint8_t numBytes, uint8_t* dest)
{
  // Initialise the transmit message buffer
  Wire.beginTransmission(_i2cAddress); 
  
  // Add the i2c module Register address to transmit message buffer
  Wire.write(registerAddress); 

  // End transmission
  uint8_t nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if (nackCatcher != 0) return nackCatcher;
  
  // Number of bytes to receive
  nReceived = Wire.requestFrom(_i2cAddress, numBytes); 

  if (nReceived != numBytes) return (uint8_t)5;
  
  uint8_t i = 0; 

  // read the data into the receiveBuffer
  while( Wire.available() )
  {
    dest[i++] = Wire.read();
  }

  // All ok
  return (uint8_t)0;
}

uint8_t writeRegister(uint8_t registerAddress, uint8_t dataByte){
  
  // Initialise the transmit message buffer
  Wire.beginTransmission(_i2cAddress); 

  // Add the i2c module Register address to transmit message buffer
  Wire.write(registerAddress); 

  // Add the command data to the transmit message buffer
  Wire.write(dataByte);

  // Transmit message and accept NACK response
  // The Wire.endTransmission() is used when writing data only
  uint8_t nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if (nackCatcher != 0) return nackCatcher;
  
  // Wait for 10ms
  delay(10);

  // Allocate memory.
  uint8_t checkByte;

  // We are now going to check the operation was successful
  // This is not really required, but most of the time we are debugging
  // Read the Register
  readRegister(registerAddress, &checkByte);

  // Check the data read back is the same
  if (checkByte == dataByte) return (uint8_t)0;

  // Update the User
  Serial.println("ERROR: i2c module register not accepting writes");

  // An error happened
  return (uint8_t)6;
}
