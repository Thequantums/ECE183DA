/*MPU9250 Sensor Test Code
 * By: stevenvo
 * Original Source: https://github.com/stevenvo/mpuarduino/blob/master/mpuarduino.ino
 * Edited for EE183DA by Nathan Pilbrough
 * 
 * Description: Basic code to test the functionality of the
 * MPU9250 sensor. Refer to the startup guide on CCLE for 
 * more information. NOTE: this is meant to help confirm 
 * communication with the sensor, calibration is still required
 */
#include <Wire.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define SDA_PORT 14
#define SCL_PORT 12

//calibrating variables
float magCalibration[3], mRes, mag_scale[3], mag_bias[3];


// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin(SDA_PORT,SCL_PORT);
  Serial.begin(115200);

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

    uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=(Mag[1]<<8 | Mag[0]);
  int16_t my=(Mag[3]<<8 | Mag[2]);
  int16_t mz=(Mag[5]<<8 | Mag[4]);

   magCalibration[0] =  (float)(mx - 128)/256.0f + 1.0f;
   magCalibration[1] =  (float)(my - 128)/256.0f + 1.0f;  
   magCalibration[2] =  (float)(mz - 128)/256.0f + 1.0f;
  
   // 16 bits registers - 10.0f*4912.0f/8190.0f
   mRes = 10.0f * 4912.0f/8190.0f;
  
  uint16_t ii = 0, sample_count = 0;
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};

  delay(4000);
  
// shoot for ~fifteen seconds of mag data
  /*
  if(_Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  if(_Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
  */ 
  //assume at 100 Hz ODR, new mag data is available every 10 ms
  sample_count = 1500;
  for(ii = 0; ii < sample_count; ii++) {
    // :::  Magnetometer ::: 

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag_temp[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag_temp); // Read the x-, y-, and z-axis calibration values
    
      // Magnetometer
   int16_t mag_temp[3];    
   mag_temp[0]=(Mag_temp[1]<<8 | Mag_temp[0]);
   mag_temp[1]=(Mag_temp[3]<<8 | Mag_temp[2]);
   mag_temp[2]=(Mag_temp[5]<<8 | Mag_temp[4]);
    
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(100);
 }

    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    mag_bias[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    mag_bias[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
    mag_bias[2] = (float) mag_bias[2]*mRes*magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0;

    mag_scale[0] = avg_rad/((float)mag_scale[0]);
    mag_scale[1] = avg_rad/((float)mag_scale[1]);
    mag_scale[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
  
  
}

long int cpt=0;
// Main loop, read and display data
void loop()
{
  
  // _______________
  // ::: Counter :::
  
  // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\t");
  
  // _____________________
  // :::  Magnetometer ::: 

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  int16_t mx=(Mag[1]<<8 | Mag[0]);
  int16_t my=(Mag[3]<<8 | Mag[2]);
  int16_t mz=(Mag[5]<<8 | Mag[4]);
  
  
  // 16 bits registers - 10.0f*4912.0f/8190.0f
  
      // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
      mx = (float)mx*mRes*magCalibration[0] - mag_bias[0];  // get actual magnetometer value, this depends on scale being set
      my = (float)my*mRes*magCalibration[1] - mag_bias[1];  
      mz = (float)mz*mRes*magCalibration[2] - mag_bias[2];  
      mx *= mag_scale[0];
      my *= mag_scale[1];
      mz *= mag_scale[2];
 
  
  // arc tangent of y/x 
  float heading = atan2(mx, my);

  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location. Mine is 0.0404 
  // Find yours here: http://www.magnetic-declination.com/
  
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0404;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PI; 

  Serial.print("\rHeading:\t");
  Serial.print(heading);
  Serial.print(" Radians   \t");
  Serial.print(headingDegrees);
  Serial.println(" Degrees   \t");

  Serial.print ("Magnetometer readings:"); 
  Serial.print ("\tMx:");
  Serial.print (mx); 
  Serial.print ("\tMy:");
  Serial.print (my);
  Serial.print ("\tMz:");
  Serial.print (mz);  
  Serial.println ("\t");
  
  
  // End of line
  delay(100); 
}







