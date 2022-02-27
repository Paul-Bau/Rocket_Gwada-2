// Inspiration source:https://www.hobbytronics.co.uk/eeprom-page-write

#include <Wire.h>           // remember to change buffer length to 64 bytes
#define EEPROM_adr 0x50     //Address of 24LC256 eeprom chip

union union64bytes{   // 64 bytes variable, in which floats, unsigned long or byte can be written
  byte  b[64];
  float f[16];
  unsigned long ul[16];
  };
  
// Data variable initialisation (like in BIBA)
unsigned long t_new= millis();  // 4 bytes integer

  // Emulate data
float pressure=1342.3;                          // [kPa]
float alt_AGL=130.0;                            // [m]
float vel_vert=40.0;                            // [m/s]

float acc_x=403.0, acc_y=052.20, acc_z=250.50;  // [g]
float w_x=007.470, w_y=053.0, w_z=70.60;        // [deg/s]
float mag_x=03.360, mag_y=60.03, mag_z=650.70;  // [µT]

float temperature =30.0;                        // [°C]
float pressure_bmp =530.0;                      // [kPa]
int   mode=1;

void setup(void){ 
  Wire.begin(); 
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } 


  
  // WRITE IN THE EEPROM FROM HERE------------------------------------------  
  byte i_page = 0;
  writeEEPROM_datapage(EEPROM_adr, i_page,
        t_new, pressure, alt_AGL,vel_vert, 
        acc_x,acc_y,acc_z, w_x,w_y,w_z, mag_x,mag_y,mag_z,
        temperature, pressure_bmp,mode);
  // WRITE IN THE EEPROM TO HERE---------------------------------------------  


  
  // Read the data
  union union64bytes READ_data_page;
  readEEPROM(EEPROM_adr, i_page*64, READ_data_page.b, 64);

  Serial.println("Check 1: Write Vs Read");
  Serial.print("W ");
  //Serial.println(data_page.ul[0]);
  Serial.print("R ");
  Serial.println(READ_data_page.ul[0]);
  Serial.print("\n");
  for (byte i=1;i<15;i++){
    Serial.print("W ");
    //Serial.println(data_page.f[i],2);
    Serial.print("R ");
    Serial.println(READ_data_page.f[i],2);
    Serial.print("\n");
  }
  Serial.print("W ");
  //Serial.println(data_page.b[60]);
  Serial.print("R ");
  Serial.println(READ_data_page.b[60]);
  Serial.print("\n");
}

void loop(){
  // nothing
}
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


// Debbugging tools
/*
void printallpages(int deviceaddress){
  for(byte i_page=0;i_page<1;i_page++){
    for(byte i_float=0;i_float<15;i_float++){
      union ufloat{
        float f;
        byte b[4];
      }myfloat;
      for(byte i_byte=0;i_byte<4;i_byte++){
        int address= 64*i_page + 4*i_float + i_byte;
        myfloat.b[i_byte]= readEEPROM_byte(deviceaddress,address);
      }
//      Serial.print(myfloat.f,DEC);
//      Serial.print("\t");
    }
//    Serial.print("\n");    
  }  
}

void printallpages_HEX(int deviceaddress){
  for(byte i_page=0;i_page<1;i_page++){
    for(byte i_float=0;i_float<15;i_float++){
      for(byte i_byte=0;i_byte<4;i_byte++){
        int address= 64*i_page + 4*i_float + i_byte;
//        Serial.print(readEEPROM_byte(deviceaddress,address),HEX);
//        Serial.print(" ");
      }

//      Serial.print("\t");
    }
    Serial.print("\n");    
  }  
}

////defines the readEEPROM function
byte readEEPROM_byte(int deviceaddress, unsigned int eeaddress ) {
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));      //writes the MSB
  Wire.write((int)(eeaddress & 0xFF));    //writes the LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1);
  rdata = Wire.read();
 // delay(5);
//  Serial.print(rdata,HEX);
//  Serial.print(" ");
  return rdata;
}
*/
