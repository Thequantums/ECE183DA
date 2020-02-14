/*
  Wireless Servo Control, with ESP as Access Point

  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.

  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets
    
    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See 
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Ultra-thin power bank 
  * Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>

#include "debug.h"
#include "file.h"
#include "server.h"

// Include from mpu9250_sensor test
//*****************************************************
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
float mag_scale[3] = {0,0,0}; // magnetometer scale for mx,my,mz 
float mag_bias[3] = {0,0,0}; // magnetometer biase to substract from mx, my, mz
//*********************include from vl5310x_dual_sensor_test **********************************

#include <VL53L0X.h>
VL53L0X sensor;
VL53L0X sensor2;

//**************************************************************

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;


// WiFi AP parameters
char ap_ssid[13];  // name of access point of the ESP9250
char* ap_password = ""; // password for access point, "" means no password

// WiFi STA parameters
char* sta_ssid = 
  "..."; // station id to connect to wifi
char* sta_password = 
  "..."; // station password to connect to wifi 

char* mDNS_name = "paperbot"; // setup Domain name for ESP8266 as "paperbot" to access ESP8266 sever

String html; // output to html file
String css; //out

uint8_t client_id; // client id. Assume it is fixed for a connection

//****************** from mpu9250_sensor_test *****************************************

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address); // begin a transmission to the I2C slave device
  Wire.write(Register); // write register value to the address
  Wire.endTransmission(); // end transmission of the value
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); // request a slave device address: Address to get N bytes
  uint8_t index=0;
  // while there is still data to read from, read the data and store to Data
  while (Wire.available())
    Data[index++]=Wire.read(); // each read return 8 bits value 
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address); // begin a transmission to the I2C slave device
  Wire.write(Register); // write register value to the address
  Wire.write(Data); // Write Data to the slave device
  Wire.endTransmission(); // end tranmission of data
}

/*
 readGyro_data read data from Gyro in 8 bit per read() call and concantonate two 8 bits to form sign 16 bits
 which are stored in raw_data array. The function return raw_data back as a pointer
 */

void readGyro_data(int16_t* raw_data ) {
  
  uint8_t temp_data [7];
  Wire.beginTransmission(MPU9250_ADDRESS); // begin transmission to MPU9250 Address because Gyro has the same address as MPU9250
  Wire.write(0x43); //write 0x43 register value to the address, so specifing we're gonna read from Gyro
  Wire.endTransmission(false); // send a restart message after transmission. The bus will not be released
  uint8_t i = 0;
  Wire.requestFrom(MPU9250_ADDRESS,7); // request to read from Gyro in 7 bytes
    // while there is still data to read from, read the data and store to Data
  while (Wire.available()) {
      temp_data[i++] = Wire.read(); // each read return 8 bits value 
  }
  // concantonating the two 8 bits to form 16 sign value
   raw_data[0] = (temp_data[1]<<8 | temp_data[0]);
   raw_data[1] = (temp_data[2]<<8 | temp_data[3]);
   raw_data[2] = (temp_data[4]<<8 | temp_data[5]);
  
}

// This is a setup function for mpu9250 when arduino starts
void setup_mpu9250()
{
  // Arduino initializations
  Wire.begin(SDA_PORT,SCL_PORT); // initialize Serial Data line and Serial clock line on I2C bus for read and write
  Serial.begin(115200); // set serail frequency to 115200 bud

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

// ****** calibration code starts here **********
// Note: Commented the calibration code out because we alreay got fixed calibrated value that can be plugged in
/*
  uint16_t ii = 0, sample_count;
  int16_t mag_max[3] = {-32767, -32767, -32767}; // magentometer maximum value
  int16_t mag_min[3] = {32767, 32767, 32767}; // magentometr minimum value

  delay(1000);
  
  //assume at 100 Hz ODR, new mag data is available every 10 ms
  sample_count = 300; // samples number
  uint8_t ST2;
  uint8_t Mag_temp[7];
  
  int16_t mag_temp[3] = {0,0,0};
    
  // The loop reads raw data from magnetomer and concantonate the two 8 bits to form a value for mx,my,mz which are real
  // magnetic field data. It performs linear search to get the max and the min value of magnetic field in x,y,z. The biases
  // are calculated from the average of the max and min values. Because we wants to find the center point max and min magnetic field
  // which is influenced by earth magnetic field. then we want to substract that influence from raw data.
  //Scaling factor are calculated by taking average of the three differences of max and min from x,y,z then
  // use the average to divid the three differences of max and min. The scaler factor is to get rid of soft iron by metal around the car   
  for(ii = 0; ii < sample_count; ii++) {
      // :::  Magnetometer ::: 

      // Request first magnetometer single measurement
      I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
      // Read register Status 1 and wait for the DRDY: Data Ready
      do
      {
        I2Cread(MAG_ADDRESS,0x02,1,&ST2);
      }
      while (!(ST2&0x01));
      
      // Read magnetometer data  
      I2Cread(MAG_ADDRESS,0x03,7,Mag_temp); // Read the x-, y-, and z-axis values
    
      //Magnetometer 
      mag_temp[0]=(Mag_temp[1]<<8 | Mag_temp[0]);
      mag_temp[1]=(Mag_temp[3]<<8 | Mag_temp[2]);
      mag_temp[2]=(Mag_temp[5]<<8 | Mag_temp[4]);
    
      for (int jj = 0; jj < 3; jj++) {
        if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
        if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
      }
      Serial.println("Calibrating");
      Serial.println(ii);
      delay(135);
  }


    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2])/3.0;

    mag_scale[0] = avg_rad/((float)mag_scale[0]);
    mag_scale[1] = avg_rad/((float)mag_scale[1]);
    mag_scale[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
   */
}

//************************set up for vl530x*************
void setup_vl53l0 () {
  
  pinMode(D3, OUTPUT); // set pin output D3
  pinMode(D4, OUTPUT); // set pin output D4
  digitalWrite(D7, LOW); // set pin D7 0v
  digitalWrite(D8, LOW); //set pin D8 0v

  delay(500);
  digitalWrite(D3, HIGH); // set pin D3 high to use for sensor
  delay(150);
  Serial.println("00");
  
  sensor.init(true); // initalize sensor 1
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22); // set address sensor to unsigned bit valued at 22

  digitalWrite(D4, HIGH); //set pin D4 high to use for sensor
  delay(150);
  sensor2.init(true); // initialize sensor 2
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25); // set address sensor to unsigned bit valued at 25
  Serial.println("04");

  Serial.println("addresses set");

  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

 // The loop scan all address starting from 1 to 120 using I2C bus.
 // If endtransmission is success (0), then the address is found and print it out.
 // else keep looping remaining address.
 
  for (byte i = 1; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) // if success
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  delay(3000);  

}



void setup() {
  // setup Serial, LEDs and Motors( servo left and servo right)
    setupPins();
    
   //set access point ID of the arduino to "ESP_(chipid)" where chipID is the ID of machine
    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

   //Booting maching period = 4 seconds
    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush(); // flush out the print
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password); // set up ID and password for access point of the arduino from server.cpp
    LED_OFF;

    setupFile(); //initialize SPIFFS which is the flash memory on Arduino
    html = loadFile("/controls.html"); // The html code that provides buttons to send signals to the car (server)
    css = loadFile("/style.css"); // Style of the font and format of interface website
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP(); // intialize http server to create secure connection between server and client
    // initialize websocket for communication signal from website to the car and create an event driven response
    // which means whenever information passed from website, socket will channel the information and allow program to respone
    setupWS(webSocketEvent); 
    //setupMDNS(mDNS_name);
    
    //set up mpu 9250
    setup_mpu9250();

    //set up vl53l0 sensor for distance measurement
    setup_vl53l0();

    stop();
}

void loop() {

  // *************** web sever loop
    wsLoop(); // loop to keep socket alive and wait for event to response
    httpLoop(); // loop to create secure connection for serve and client
    
   // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  //continuously checking if magnetomter's data is ready 
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data after data is ready  
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
    //*mRes*magCalibration[0]
   //Note: the biase and Scale values are hard set to the calibration after sveral experiments 
   float f_mx = (float)mx - 64; // 64 is a bias
   float f_my = (float)my + 162.6;
   float f_mz = (float)mz + 1;
    f_mx *= 1.012; //1.012 is a scale factor
    f_my *= 1.01;
    f_mz *= 0.978;
    
  // arc tangent of y/x 
  float heading = atan2(f_mx, f_my);

    // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PI; 
  
  //float uc_headingDegrees = uc_heading * 180/PI;
  Serial.print("\rCalibrated Heading:\t");
  Serial.print(heading);
  Serial.print(" Radians   \t");
  Serial.print(headingDegrees);
  Serial.println(" Degrees   \t");
  
  Serial.print (" Calibrated Magnetometer readings:"); 
  Serial.print ("\tMx:");
  Serial.print (f_mx*0.6); 
  Serial.print ("\tMy:");
  Serial.print (f_my*0.6);
  Serial.print ("\tMz:");
  Serial.print (f_mz*0.6);  
  Serial.println ("\t");


//**************** Gyro Reading *****************
  // Set by pass mode for the Gyro
  int16_t gyro_data[3];
  readGyro_data(gyro_data);
  Serial.print("Gyro data:  ");
  Serial.print ((float)((gyro_data[0] * 250) / 32768));  
  Serial.print ("\t");
  Serial.print ((float)((gyro_data[1] * 250) / 32768));  
  Serial.print ("\t");
  Serial.print ((float)((gyro_data[2] * 250) / 32768));  
  Serial.print ("\t");
  Serial.println("");
  
    // End of line
    
//*********** loop for vl*********************************
  Serial.print("Lidar 1 range(mm): ");
  // print out distance output from sensor in millimeter if wait too long, time out.
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.print("  Lidar 2 range(mm): ");
  Serial.println(sensor2.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }

//*************Writting sensors' data to website************************************

  //Using websocket
   char sensors_output[20];  
   //sprintf(sensors_output, "Lidar 1 rang(mm): %.2f, Lidar 2: %.2f, Heading: %.2f, Gyro_Data: %.2f.\n", (1.0 * sensor.readRangeSingleMillimeters()), (1.0 * sensor2.readRangeSingleMillimeters()), headingDegrees,(float)((gyro_data[2] * 250) / 32768));
  //Serial.println(sensors_output);
  sprintf(sensors_output, "Range1: %.2f, Range2:  %.2f, Degree: %.2f, gyro(z): %.2f", (1.0 * sensor.readRangeSingleMillimeters()), (1.0 * sensor2.readRangeSingleMillimeters()), headingDegrees,(float)((gyro_data[2] * 250) / 32768) );
  //broadcast(sensors_output);
  wsSend(client_id, sensors_output);


    
}


//
// Movement Functions //
//

// drive the car with certain pmw left and pmw right
void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

// stop the car 
void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

// move the car forward
void forward() {
  DEBUG("forward");
  drive(0, 180);
}

//move the car backward 
void backward() {
  DEBUG("backward");
  drive(180, 0);
}
// move the car left
void left() {
  DEBUG("left");
  drive(180, 180);
}
//move the right
void right() {
  DEBUG("right");
  drive(0, 0);
}



//
// Setup //
//

void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    // set up motor
    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
}

/* Taking arguements as ID for websockets, type which is current state of the socket, 8 bits payload data and the total length of payload
 * The function is an event driven handler depended on states of the socket and response to payload data received from web by
 * performing control on the car
 */
void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {
    // switch statement to choose the current type of socket: disconnect, connect or binary
    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

             client_id = id; // assign the ID of the connection to write data to website in loop. 
            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid); // send IP of local machine 
            break;
        }
        case WStype_BIN:
            // if received binary data
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

           // drive the car by 180 - value for pwm left and pwm right for the data received from website
            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
             // IF received text message from
            DEBUG("On connection #", id)
            //print out payload on serial monitor
            DEBUG("  got text: ", (char *)payload);

            //these are cases for handling different payload data and control the motors
            if (payload[0] == '#') {
                if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Hello world!");
                }
                else if(payload[1] == 'F') 
                  forward();
                else if(payload[1] == 'B') 
                  backward();
                else if(payload[1] == 'L') 
                  left();
                else if(payload[1] == 'R') 
                  right();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else 
                  stop();
            }

            break;
    }
}
