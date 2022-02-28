//Inpsired from: https://www.youtube.com/watch?v=oTXi6kYg0D4

/* For an unknown reason, the file name cannot exceed 9 flights, it worked before hand but not anymore
 * not a big deal but an [ACTION] to correct that would be appreciated. 
 */


#include <SPI.h>
#include <SD.h>

// Pinout and peri declaration
const byte buzPin   = 2;        // the number of the buzzer pin
const byte ledPin   = 3;        // the number of the LED pin
const byte SD_CSPin = 6;        // Chip select of the SD card

File myFile;                    // For the SD card

// Variable type definition for EEPROM data pages
union union64bytes{   // 64 bytes variable, in which floats, unsigned long or byte can be written
  byte  b[64];
  float f[16];
  unsigned long ul[16];
  };
  
// Data variable initialisation (like in BIBA)
unsigned long t_new= millis();  // 4 bytes integer

float pressure=1342.3;                          // [kPa]
float alt_AGL=130.0;                            // [m]
float vel_vert=40.0;                            // [m/s]

float acc_x=403.0, acc_y=052.20, acc_z=250.50;  // [g]
float w_x=007.470, w_y=053.0, w_z=70.60;        // [deg/s]
float mag_x=03.360, mag_y=60.03, mag_z=650.70;  // [µT]

float temperature =30.0;                        // [°C]
float pressure_bmp =530.0;                      // [kPa]
int   mode=1;



void setup() {
  Serial.begin(9600);                           // Remove in the flight code
  while (!Serial){                              // Remove in the flight code
    ; // Wait until serial port to connect      // Remove in the flight code
  }                                             // Remove in the flight code

// -----------------------------------------------------------------------------
  // Health Check
  Serial.print("Initializing SD card...");      // Remove in the flight code
  if (!SD.begin(SD_CSPin)){
    Serial.println("Initialization failed!");   // Remove in the flight code
    while(1);
  }
  Serial.println("Initialisation done.");       // Remove in the flight code

// -----------------------------------------------------------------------------
  // File creation
  // - unique
  // - CSV format
  // - Only from flight 0 to 9
  byte i_flight   = 0;
  char filename[16];
  sprintf(filename, "Flight_%u.csv", i_flight);
  
  while(SD.exists(filename)){                     // Going in the loop if the file name already exists
    Serial.print(filename);                       // Remove from the flight code
    Serial.println(" exists.");                   // Remove from the flight code
    
    i_flight++;                                   // Increment the flight number
    sprintf(filename, "Flight_%u.csv", i_flight); // Update the file name with the incremented flight number
  }
  Serial.print(filename);                         // Remove from the flight code
  Serial.println(" doesn't exist.");              // Remove from the flight code

// -----------------------------------------------------------------------------

  // Open a file and immediatly close it
  Serial.print("Creating ");                    // Remove in the flight code
  Serial.println(filename);                     // Remove in the flight code
  
  myFile = SD.open(filename, FILE_WRITE);       // Create & open a file

  unsigned int n_lines = 3;
  for(byte i_line=0;i_line<n_lines;i_line++){
    Serial.print("i_line: ");
    Serial.println(i_line);
    union union64bytes READ_data_page;
    READ_data_page = generate_datapage(t_new, pressure, alt_AGL,vel_vert, 
        acc_x,acc_y,acc_z, w_x,w_y,w_z, mag_x,mag_y,mag_z,
        temperature, pressure_bmp,mode);
        
    
    // Write the data page in a line of the csv
    // Filewrite: https://www.arduino.cc/en/Reference/FileWrite
    myFile.print(READ_data_page.ul[0]);       // Write time                 
    myFile.print("\t");                       // Tabulation to access next column    
    Serial.print(READ_data_page.ul[0]);       // Remove in the flight code
    Serial.print("\t");                       // Remove in the flight code
    
    for(byte i_col=1; i_col<15;i_col++){           
      myFile.print(READ_data_page.f[i_col]);  // Write all float data
      myFile.print("\t");                     // Tabulation to access next column      
      Serial.print(READ_data_page.f[i_col]);  // Remove in the flight code
      Serial.print("\t");                     // Remove in the flight code
    }
    myFile.print(READ_data_page.b[60]);       // Write mode
    myFile.print("\n");                       // New line to access next line
    Serial.print(READ_data_page.b[60]);       // Remove in the flight code
    Serial.print("\n");                       // Remove in the flight code
  }

  Serial.println("Writing to file successful");
  myFile.close();


  // Read the file
  myFile = SD.open(filename,FILE_READ);
  Serial.println("Reading from file");
  while (myFile.available()){
    Serial.write(myFile.read());
  }
  myFile.close();

}

void loop() {
  // put your main code here, to run repeatedly:
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
