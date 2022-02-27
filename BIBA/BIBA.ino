// [1] IMU: Axis definition: https://axodyne.com/2020/06/arduino-nano33-ble-lsm9ds1-imu/

// Libraries
#include <Arduino_LSM9DS1.h>    // 9DOF IMU (Acceleration, angular rate, magnetometer)
#include <Arduino_LPS22HB.h>    // Pressure sensor
#include <SPI.h>                // Serial peripherial interface (for LCD and SD card)
#include <SD.h>                 // to communicatet with SD card reader
#include <Wire.h>               // Library for I²C communication with EEPROM & BMP180, buffer length was change from 32B to 64B

// Pinout and peri declaration
#define EEPROM_adr 0x50         //Address of 24LC256 eeprom chip
const byte buzPin   =  2;       // the number of the buzzer pin
const byte ledPin   =  3;       // the number of the LED pin

// Data organisation of the EEPROM
const byte  n_page_buffer = 50; // Number of page for the circular memory in the EEPROM in armed mode
const byte  n_page_info   = 2;  // Number of page for the flight info at the begining of the memory
byte        i_page        = 2;

// Variable type definition for EEPROM data pages
union union64bytes{   // 64 bytes variable, in which floats, unsigned long or byte can be written
  byte  b[64];
  float f[16];
  unsigned long ul[16];
  };

// -------------------------------------------------------------
// Flight data declaration and initialization to zero value.
unsigned long t_new = 0;
unsigned long t_old = 0;

float pressure      = 0.0;                // [kPa]
float alt_AGL       = 0.0;                // [m]
float vel_vert      = 0.0;                // [m/s]

float acc_x = 0.0, acc_y= 0.0, acc_z=0.0; // [g]
float w_x   = 0.0, w_y  = 0.0, w_z  =0.0; // [deg/s]
float mag_x = 0.0, mag_y= 0.0, mag_z=0.0; // [µT]

float temperature   = 0.0;                // [°C]
float pressure_bmp  = 0.0;                // [kPa]
byte  mode          = 1 ;

// Default ground level pressure & temperature
float T0  = 15.0;     //[°C]  Temperature at ground level assumed [ACTION: average T0 during arming]
float P0  = 101.3;    //[kPa] Pressure at ground level assumed [ACTION: average P0 during arming]
//

// Flight computer internal variables
float acc_axial = 0.0;              // [g]
int   count     = 0;                // [-] [ACTION]: remove it from global variable and use local variables as counters

// Flight input parameters
float interval            = 50;     // [ms]   Minimal period between two samples.
float acc_LaunchDetect    = 2;      // [g]    Minimal axial acceleration to detect launch.
byte  count_LaunchDetect  = 2;      // [-]    Minimal consecutive loops with axial acceleration above threshold to detect launch.
float acc_BurnoutDetect   = 1.2;    // [g]    Maximal axial acceleration to detect burnout.
byte  count_BurnoutDetect = 2;      // [-]    Minimal consecutive loops with axial acceleration below threshold to detect burnout.
byte  count_ApogeeDetect  = 2;      // [-]    Minimal consecutive loops with negative vertical velocity to detect apogee.
float vel_LandingDetect   = 0.1;    // [m/s]  Maximal absolute vertical velocity to detect landing.
byte  count_LandingDetect = 10;     // [-]    Minimal consecutive loops with absolute vertical velocity below threshold to detect landing.

// Function declaration
void read_sensors();
float calc_altitude(float P, float P0, float T0);
void writeEEPROM_datapage(int deviceaddress, byte i_page,
        unsigned long time_ms, float pressure_kPa, float altitude_m,
        float vertical_velocity_mps, 
        float acc_x_mps2, float acc_y_mps2, float acc_z_mps2,
        float wx_degps,float wy_degps,float wz_degps,
        float magx_uT,float magy_uT,float magz_uT,
        float temperature_C, float pressure_bmp_kPa,
        byte mode);
union union64bytes generate_datapage(unsigned long time_ms, float pressure_kPa, float altitude_m,
        float vertical_velocity_mps, 
        float acc_x_mps2, float acc_y_mps2, float acc_z_mps2,
        float wx_degps,float wy_degps,float wz_degps,
        float magx_uT,float magy_uT,float magz_uT,
        float temperature_C, float pressure_bmp_kPa,
        byte mode);
void readEEPROM(int deviceaddress, unsigned int eeaddress,  
                 unsigned char* data, unsigned int num_chars); 
void FullHealthCheck();
void alarm(unsigned int t_on_ms, unsigned int t_off_ms, unsigned int n_cycles);
void printdata();

void setup() {
  Serial.begin(9600);
  
  pinMode(ledPin, OUTPUT);          // Set the GPIO for the led as output
  pinMode(buzPin, OUTPUT);          // Set the GPIO for the buzzer as output

  //[ACTION] read in EEPROM for last mode and initialise the mode

  alarm(100,100,5);
  // Flight computer health check
  FullHealthCheck;
  
}  
  
void loop() {
  switch(mode){
    //---------------------------------------------------------------------------------------------------
    case 1: // Ground idle
      // As no switch is available for arming the system,
      // the computer will wait for a period of time in idle mode
      alarm(1000,9000,6);// (Buzz for 1s, wait 9s) x6

      // Announce the arming mode will start
      alarm(3000,0,1);            //
      
      mode  = 2;                    // move to Armed mode
      break;
    //---------------------------------------------------------------------------------------------------  
    case 2: // Armed
      // Is it Acquisition time?
      t_new = millis();
      if ((t_new - t_old) >= interval) {
        t_old = t_new; // resets time

          // Read data
          read_sensors();
          alt_AGL =  calc_altitude(pressure,P0,T0);  // [m] Careful, T0 in °C here 
          //[ACTION] vel_vert = (alt_AGL - alt_AGL_old)/(t_new-t_old) /1000; //[m/s] 
                    
          acc_axial = - acc_y;                          // [g] Axis reference from [1]
        
          // Launch detection
          if (acc_axial > acc_LaunchDetect){              // [g]
            count++;
            if (count>count_LaunchDetect) {               //Launch detected
              mode  = 3;
              count = 0;}     
          }
          else{count = 0;}
        }  
      // Save data in EEPROM
      writeEEPROM_datapage(EEPROM_adr, i_page,
        t_new, pressure, alt_AGL,vel_vert, 
        acc_x,acc_y,acc_z, w_x,w_y,w_z, mag_x,mag_y,mag_z,
        temperature, pressure_bmp,mode);            

      i_page = n_page_info + (i_page-n_page_info+1)%n_page_buffer; 

 
      break;
    //---------------------------------------------------------------------------------------------------
    case 3: // Propulsed flight
        // Is it Acquisition time?
        t_new = millis();
        if ((t_new - t_old) >= interval) {
          t_old = t_new; // resets time
  
          // Read data
          read_sensors();
          alt_AGL =  calc_altitude(pressure,P0,T0);  // [m] Careful, T0 in °C here 
          //[ACTION] vel_vert = (alt_AGL - alt_AGL_old)/(t_new-t_old) /1000; //[m/s] 
          
          // Save data in EEPROM
          writeEEPROM_datapage(EEPROM_adr, i_page,
            t_new, pressure, alt_AGL,vel_vert, 
            acc_x,acc_y,acc_z, w_x,w_y,w_z, mag_x,mag_y,mag_z,
            temperature, pressure_bmp,mode);            
          i_page++;
        
          // Burnout detection
          acc_axial = - acc_y;
          if (acc_axial < acc_BurnoutDetect){
            count++;
            if (count>count_BurnoutDetect) {  // Burnout detected if true
              mode  = 4;
              count = 0;
            }  
          }
          else{count = 0;}  
          }
        print_data();
        break;
    //---------------------------------------------------------------------------------------------------   
    case 4: // Ballistic flight
      // Is it Acquisition time?
      t_new = millis();
      if ((t_new - t_old) >= interval) {
        t_old = t_new; // resets time

        // Read data
        read_sensors();
        alt_AGL =  calc_altitude(pressure,P0,T0);  // [m] Careful, T0 in °C here 
        //[ACTION] vel_vert = (alt_AGL - alt_AGL_old)/(t_new-t_old) /1000; //[m/s] 
          
        // Save data in EEPROM
        writeEEPROM_datapage(EEPROM_adr, i_page,
          t_new, pressure, alt_AGL,vel_vert, 
          acc_x,acc_y,acc_z, w_x,w_y,w_z, mag_x,mag_y,mag_z,
          temperature, pressure_bmp,mode);            
        i_page++;

        // Apogee detection
        if (vel_vert < 0){
          count++;
          if (count>count_ApogeeDetect) {     // Apogee detected if true
            count = 0;
            mode  = 5;
            }
        }
        else{count = 0;}  
      }
      print_data();
      break;

    //---------------------------------------------------------------------------------------------------  
    case 5: // Under chute
        t_new = millis();
        if ((t_new - t_old) >= interval) {
          t_old = t_new; // resets time
  
          // Read data
          read_sensors();
          alt_AGL =  calc_altitude(pressure,P0,T0);  // [m] Careful, T0 in °C here 
          //[ACTION] vel_vert = (alt_AGL - alt_AGL_old)/(t_new-t_old) /1000; //[m/s] 
          
          // Save data in EEPROM
          writeEEPROM_datapage(EEPROM_adr, i_page,
            t_new, pressure, alt_AGL,vel_vert, 
            acc_x,acc_y,acc_z, w_x,w_y,w_z, mag_x,mag_y,mag_z,
            temperature, pressure_bmp,mode);            
          i_page++;

          // Landing detection
          if (abs(vel_vert) < vel_LandingDetect){
            count++;
            if (count>count_LandingDetect) {
              // Apogee detected
              count = 0;
              mode  = 5;
              }
          }
          else{count = 0;}  
        }
      print_data();
      break;

    //---------------------------------------------------------------------------------------------------
    case 6: // Landed
      // Save EEPROM Data on microSD
      // [ACTION] write code

      
       break;
  }
}


// -------------------------------------------------------------
// Functions ---------------------------------------------------
// -------------------------------------------------------------
void read_sensors(){
  // Read data
  if (IMU.accelerationAvailable())  {IMU.readAcceleration(acc_x, acc_y, acc_z);}    // [g]      Body non-gravitational acceleration (=1g when still on Earth)
  if (IMU.gyroscopeAvailable())     {IMU.readGyroscope(w_x, w_y, w_z);}             // [deg/s]  Body rates
  if (IMU.magneticFieldAvailable()) {IMU.readMagneticField(mag_x, mag_y, mag_z);}   // [µT]     Magnetic field, /!\ different axis
  pressure = BARO.readPressure();                                                   // [kPa]    Ambiant pressure
}

float calc_altitude(float P, float P0, float T0){
  // formula: https://docs.google.com/presentation/d/1OuJF2bP5jli1vVQzrxGlWD5iXmwQ7Wh_QroukcimZEI/edit#slide=id.g116fb714baa_0_71
  float alt_AGL = (T0+273.15)/0.0065*(1-pow(P/P0,1/05.25864));  // [m] Careful, T0 in °C here
  return alt_AGL;
  }
        

// -------------------------------------------------------------
// Funtions for EEPROM Access
//----------------------------------------------------------------------------
void writeEEPROM_datapage(int deviceaddress, byte i_page,
        unsigned long time_ms, float pressure_kPa, float altitude_m,
        float vertical_velocity_mps, 
        float acc_x_mps2, float acc_y_mps2, float acc_z_mps2,
        float wx_degps,float wy_degps,float wz_degps,
        float magx_uT,float magy_uT,float magz_uT,
        float temperature_C, float pressure_bmp_kPa,
        byte mode)
{  
  // Generate the data page
  union union64bytes data_page;
  data_page = generate_datapage(t_new, pressure, alt_AGL,vel_vert, 
        acc_x,acc_y,acc_z, w_x,w_y,w_z, mag_x,mag_y,mag_z,
        temperature, pressure_bmp,mode);

  if(i_page>=512){i_page=511;};    // if we go over the memory, we keep writing in the last page      
  // Write the data page in the eeprom  
  writeEEPROM_ArrayOf64Bytes( EEPROM_adr,
                              i_page*64,    // address of the first byte of data
                              data_page.b);
} 
//----------------------------------------------------------------------------
void writeEEPROM_ArrayOf64Bytes(int deviceaddress, unsigned int eeaddress, byte data[]) {
// Write an array of 64 bytes
// Requires to change the buffer length of the I²C communication from 32 to 64 bytes in the wire library (wire.h)
// C:\Users\paulb\Documents\ArduinoData\packages\arduino\hardware\avr\1.8.4\libraries\Wire\src
// Not tested on multiple page
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)((eeaddress) >> 8));    // MSB of the address
  Wire.write((int)((eeaddress) & 0xFF));  // LSB of the address
  for(byte i_byte =0;i_byte<64;i_byte++){
    Wire.write((byte) data[i_byte]);      // Write each byte
  }
  Wire.endTransmission();   
  delay(6);  // needs 5ms for page write     
}
//----------------------------------------------------------------------------
union union64bytes generate_datapage(unsigned long time_ms, float pressure_kPa, float altitude_m,
        float vertical_velocity_mps, 
        float acc_x_mps2, float acc_y_mps2, float acc_z_mps2,
        float wx_degps,float wy_degps,float wz_degps,
        float magx_uT,float magy_uT,float magz_uT,
        float temperature_C, float pressure_bmp_kPa,
        byte mode){
  // works for one specific data structure as defined here: https://docs.google.com/presentation/d/1OuJF2bP5jli1vVQzrxGlWD5iXmwQ7Wh_QroukcimZEI/edit#slide=id.g116fb714baa_0_0
  union union64bytes data_page;
  data_page.ul[0]  = time_ms;
  
  data_page.f[1]  = pressure_kPa;
  data_page.f[2]  = altitude_m;
  data_page.f[3]  = vertical_velocity_mps;
  
  data_page.f[4]  = acc_x_mps2;
  data_page.f[5]  = acc_y_mps2;
  data_page.f[6]  = acc_z_mps2;
  
  data_page.f[7]  = wx_degps;
  data_page.f[8]  = wy_degps;
  data_page.f[9]  = wz_degps;
  
  data_page.f[10] = magx_uT;
  data_page.f[11] = magy_uT;
  data_page.f[12] = magz_uT;
  
  data_page.f[13] = temperature_C;
  data_page.f[14] = pressure_bmp_kPa;
  
  data_page.b[60] = mode;
  data_page.b[61] = 0;
  data_page.b[62] = 0;
  data_page.b[63] = 0;

  return data_page;
}
//----------------------------------------------------------------------------
void readEEPROM(int deviceaddress, unsigned int eeaddress,  
                 unsigned char* data, unsigned int num_chars) 
{
  byte i=0;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,num_chars);
 
  while(Wire.available()){
    data[i++] = Wire.read();
    //Serial.println(data[i]);
  }
}
//----------------------------------------------------------------------------
void FullHealthCheck(){
  // If there is an issue, 1s buzz followed by 1s blank will repeat:
  // 2 times: PS22HB Barometer 
  // 3 times: LSM9DS1 IMU
  
  // LPS22HB Barometer // help: https://docs.arduino.cc/tutorials/nano-33-ble-sense/barometric_sensor
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1){
      alarm(1000,1000,2);
      delay(5000);
    }
  } 
  // LSM9DS1 IMU // help: https://docs.arduino.cc/tutorials/nano-33-ble-sense/cheat-sheet#lsm9ds1
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1){
      alarm(1000,1000,3);
      delay(5000);
    }
  }
}
//----------------------------------------------------------------------------
void alarm(unsigned int t_on_ms, unsigned int t_off_ms, unsigned int n_cycles){
  for(unsigned int i_cycle=0;i_cycle<n_cycles;i_cycle++){
    digitalWrite(ledPin, HIGH);   // Turn ON the Led
    digitalWrite(buzPin, HIGH);   // Turn ON the Buzzer
    delay(t_on_ms);               // Wait for t_on 
    digitalWrite(ledPin, LOW);    // Turn OFF the Led
    digitalWrite(buzPin, LOW);    // Turn OFF the Buzzer
    delay(t_off_ms);              // Wait for 250 ms
  }
}

void print_data(){
  Serial.print(mode);
  Serial.print(";");
  Serial.print(acc_axial);
  Serial.print(";");
  Serial.print(acc_x);
  Serial.print(";");
  Serial.print(acc_y);
  Serial.print(";");
  Serial.print(acc_z);
  Serial.println(";");
}
