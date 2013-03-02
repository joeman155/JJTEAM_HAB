/*
 * Name:  HOPE
 * Author: Joseph Turner <joeman@leederville.net>
 * Short Description: To operate arduino controller on the High Altitude Balloon and all the
                      gadgets installed.
             * Xbee Serial modems
             * Temperature sensor
             * RTC 
             * Accelometer
             * GPS
             * LinkSprite Camera
             * SD card (older style...not SDHC) < 2GB absolutely required.
          
*
*/


// Load includes
#include <xmodem.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Fat16.h>
#include <Fat16util.h>
#include <Wire.h>


// INITIALISE VARIABLES
// --------------------
// Digital pin assignments

// We want to use Hardware serial port so that we get more reliably radio communications
// i.e. only xbees and antennas to contend with...not serial buffers!!
const short xbee_rx = 0;
const short xbee_tx = 1;

const short ls_rx = 8;
const short ls_tx = 7;

const short gps_rx = 2;
const short gps_tx = 3;

const short status_led = 4;

const short tmp_data_pin = 9;

// All I2C is through A4 and A5, but for the rtc, we specify it for this board
// We have a BMP085 pressure sensor also connected to these pins


// BMP085 Pressure sensor set-up
#define BMP085_ADDRESS 0x77  // I2C address of BMP085
const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1,ac2,ac3;
unsigned int ac4,ac5,ac6;
int b1,b2,mb,mc,md; 
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 
short temperature;
float pressure;



// CAMERA
byte incomingbyte;
SoftwareSerial lsSerial =  SoftwareSerial(ls_rx,ls_tx);  //Configure pin 4 and 5 as soft serial port
long int a=0x0000;
int j=0,k=0,count=0;                     //Read Starting address       
uint8_t MH,ML;
boolean EndFlag=0;
//const unsigned short camera_debug = 0; // 1 = enabled, 0 = disabled
unsigned short picture_freq = 0; // every 30 seconds, get a picture. (assuming cycle delay is 1000)
const char RESET_CAMERA[4] =     {0x56, 0x00, 0x26, 0x00};
const char TAKE_PICTURE[5] =     {0x56, 0x00, 0x36, 0x01, 0x00};
const char STOP_TAKING_PICS[5] = {0x56, 0x00, 0x36, 0x01, 0x03};
const char SET_SPEED[5] =        {0x56, 0x00, 0x24, 0x03, 0x01};
const char GET_SIZE[5] =         {0x56, 0x00, 0x34, 0x01, 0x00};
const char START_READ_ONE[8] =   {0x56, 0x00, 0x32, 0x0c, 0x00, 0x0a, 0x00, 0x00};
const char START_READ_TWO[6] =   {0x00, 0x00, 0x00, 0x20, 0x00, 0x0a};
byte lsData[32];

//SD Set-up
SdCard card;
Fat16 file;


// GPS Set-up
#define GPSBAUD  4800
TinyGPS gps;
SoftwareSerial uart_gps(gps_rx, gps_tx );

// RTC Set-up
#define DS3232_I2C_ADDRESS 0x68
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;


// Misc variables
char temp_string[15];
char temp_string2[3];


// LinkSprite Camera
void SendReadDataCmd();
void setCameraSpeed(uint8_t high_b, uint8_t low_b);


void setup()
{ 
  // Initialise the Xbee Serial Port
  Serial.begin(9600);

  // inidicates that program has started.
  Serial.println("S"); 
  
  // Initialise I2O
  Wire.begin();
  
  pinMode(status_led, OUTPUT); 
  
  // initialize the SD card - CANNOT be SDHC card...must be older type <= 2GB
  if (!card.init()) {
    Serial.println("E0");
    flash_led();   
    return;
  }    
  
  // initialize a FAT16 volume
  if (!Fat16::init(&card)) {
    Serial.println("E1");
    flash_led();
    return;
  }


// Initialise external temperature sensor
  digitalWrite(tmp_data_pin, LOW);
  pinMode(tmp_data_pin, INPUT);      // sets the digital pin as input (logic 1)
  delay(100);


// Initialise Linksprite camera
  lsSerial.begin(38400);
  pinMode(ls_rx, INPUT);
  pinMode(ls_tx, OUTPUT);
  lsSendCommand(RESET_CAMERA, 4);
  delay(4000);                // After reset, wait 4 second to ensure reset is complete.  

 
  // Change camera speed...slow camera down...else pictures become corrupted
  setCameraSpeed(0x56, 0xE4);  // 19200
  lsSerial.end();
  lsSerial.begin(19200);
  delay(100);  
  

// Initialise BMP085 Air pressure sensor
  bmp085Calibration();
  
  
// Output status - via LED
  Serial.print("G"); 
  // Serial.println(FreeRam(), DEC);
  digitalWrite(status_led, HIGH);

}


void loop()
{ 
 

// Initialise GPS
  uart_gps.begin(GPSBAUD);
  delay(100);  
  
  uart_gps.listen();
  short unsigned int i = 0;  // Used to wait listening for gps info...else we miss results.
  while(i < 100)
  {
    delay(25);

    while(uart_gps.available())     // While there is data on the RX pin...
    {
      int c = uart_gps.read();    // load the data into a variable...
      
      if(gps.encode(c))      // if there is a new valid sentence...
      {
        
        i = 100;
        
        long lo, la;
        unsigned long age;
        gps.get_position(&la, &lo, &age);
        Serial.print("La:"); Serial.print(la); 
        Serial.print(",Lo:"); Serial.print(lo);
        Serial.print(",A:"); Serial.print(gps.altitude()/100);
        
        // Same goes for date and time
        int year;
        byte month, day, hour, minute, second, hundredths;
        gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
        // Print data and time
        Serial.print(",D:"); Serial.print(month, DEC); Serial.print("/"); 
        Serial.print(day, DEC); Serial.print("/"); Serial.print(year);
        Serial.print(",T:"); Serial.print(hour, DEC); Serial.print(":"); 
        Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC); 
        Serial.print("."); Serial.println(hundredths, DEC);
  
        break;
      }
      
    }  
  ++i;
  }
uart_gps.end();




  
  
  // Take a picture...if it time to do this.
  lsSerial.listen();
  ++picture_freq;
  if (picture_freq > 5)
  {   
    // Indicate that we are about take a picture...delay expected.
    Serial.println("C");
    
    // Reset picture freq count variable and other variables    
    picture_freq = 0;
    EndFlag = 0;
    a = 0;

    // Send take photo command
    lsSendCommand(TAKE_PICTURE,5);
    
    // Derive full path from date/time  - format is MMDDHHMM
    // This means we comply with nothing longer than 8 characters... DOS restriction 
    readDS3232time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
    
    // Start of file (date)
    if (month < 10) {
      strcpy(temp_string, "0");
      strcat(temp_string, itoa(month, temp_string2, 10));
    } else {
      strcpy(temp_string, itoa(month, temp_string2, 10));
    }
    
    if (dayOfMonth < 10) {
      strcat(temp_string, "0");
    }
    strcat(temp_string, itoa(dayOfMonth, temp_string2, 10));
   
    
    // Second part of filename (time)
    if (hour < 10) {
      strcat(temp_string, "0");
    }
    strcat(temp_string, itoa(hour, temp_string2, 10)); 
    
    if (minute < 10) {
      strcat(temp_string, "0");
    } 
    strcat(temp_string, itoa(minute, temp_string2, 10));
    
    
    int error = 0;
    Serial.print("F:"); Serial.println(temp_string);
    if (!file.open(temp_string, O_CREAT | O_EXCL | O_WRITE)) {
      Serial.println("E2");
      error = 1;
    }
    
    // If no errors above...continue on...
    if (error == 0)
    {
      short int l = 1;
      short int m = 1;
      while(!EndFlag)
      {  
         j=0;
         k=0;
         count=0;
         SendReadDataCmd();

         delay(25);
         
          while(lsSerial.available()>0)
          {
               incomingbyte=lsSerial.read();
               k++;
               if((k>5)&&(j<32)&&(!EndFlag))
               {
               lsData[j]=incomingbyte;
               if((lsData[j-1]==0xFF)&&(lsData[j]==0xD9))      //Check if the picture is over
               EndFlag=1;                           
               j++;
	       count++;
               }
          }
         
          
          for(j=0;j<count;j++)
          {   
              // dataFile.write((byte) lsData[j]);                            
              file.write((byte) lsData[j]);
          }                                        //Send jpeg picture over the serial port

          ++l;
          if (l > 32) {
            Serial.print(".");
            l = 1;
            ++m;
            if (m > 50) {
              Serial.println("");
              m = 1;
            }
          }          
          
      }     
      
     // Finish up writing image.
     file.close();
     if (m >1) Serial.println("");  // Want a new line.
     Serial.println("D");
     lsSendCommand(STOP_TAKING_PICS, 5); 
     

     // Menu.
     long menutimestart = millis();
     Serial.print("T:"); Serial.println(menutimestart);
     Serial.println("U");
     
     
     while(1) {
        while (Serial.available() > 0) {

           // look for the next valid integer in the incoming serial stream:
           short menuopt = Serial.parseInt(); 
        
//           if (menuopt == 1) {  
//             Serial.println("S1");
//           }
    
           if (menuopt == 2) startXmodemSend(temp_string);
    
        }
        if (millis() > menutimestart + 10000) break;
      }  
    }
  }  


  // Get Air pressure information and internal temperature
  temperature = bmp085GetTemperature(bmp085ReadUT()); 
  pressure = bmp085GetPressure(bmp085ReadUP());  

  // M =measurements...pressure, external temp, internal temp.
  Serial.print("M");  // Serial.print(pressure);
  Serial.print((long) pressure, DEC);
  
  // Get outside temperature reading
  Serial.print(","); Serial.print(itoa(getECurrentTemp(), temp_string, 10));
  
  Serial.print(","); // Serial.println(temperature*0.1);  
  Serial.println(itoa(temperature, temp_string,10));
  
  
  // Delay between going back to beginning.
  Serial.println("H");
  
  delay(1000);  
 
}


// LINKSPRITE FUNCTIONS
void lsSendCommand(const char * command, short unsigned int length)
{
     //Clear any data currently in the serial buffer
     lsSerial.flush();

     lsSendCommandNF(command, length);

}

void lsSendCommandNF(const char * command, short unsigned int length)
{
	//Send each character in the command string to the camera through the camera serial port
	for(short unsigned int i=0; i<length; i++){
		lsSerial.print(*command++);
	}

}

void setCameraSpeed(uint8_t high_b, uint8_t low_b)
{
  
      lsSendCommand(SET_SPEED, 5);     
      lsSerial.write((byte) high_b);
      lsSerial.write((byte) low_b);
    
}



//Read data
void SendReadDataCmd()
{
      MH=a/0x100;
      ML=a%0x100;
      lsSerial.flush();
      lsSendCommandNF(START_READ_ONE, 8);  
      lsSerial.write((byte) MH);
      lsSerial.write((byte) ML);   
      lsSendCommandNF(START_READ_TWO, 6);    
      
      a+=0x20;                            //address increases 32£¬set according to buffer size
}



// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}


// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}


// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}


void readDS3232time(byte *second, 
byte *minute, 
byte *hour, 
byte *dayOfWeek, 
byte *dayOfMonth, 
byte *month, 
byte *year)
{
  Wire.beginTransmission(DS3232_I2C_ADDRESS);
  Wire.write(0); // set DS3232 register pointer to 00h
  Wire.endTransmission();  
  Wire.requestFrom(DS3232_I2C_ADDRESS, 7); 
  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f);  
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
}

byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Calculate temperature given ut.
// Value returned will be in units of Kelvin to nearest degree
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  int temp = (int) (0.1 * ((b5 + 8)>>4)); 
  return   273 + temp;
}



void OneWireReset (int Pin) // reset.  Should improve to act as a presence pulse
{
  digitalWrite (Pin, LOW);
  pinMode (Pin, OUTPUT);        // bring low for 500 us
  delayMicroseconds (500);
  pinMode (Pin, INPUT);
  delayMicroseconds (500);
}

void OneWireOutByte (int Pin, byte d) // output byte d (least sig bit first).
{
  byte n;

  for (n=8; n!=0; n--)
  {
    if ((d & 0x01) == 1)  // test least sig bit
    {
      digitalWrite (Pin, LOW);
      pinMode (Pin, OUTPUT);
      delayMicroseconds (5);
      pinMode (Pin, INPUT);
      delayMicroseconds (60);
    }
    else
    {
      digitalWrite (Pin, LOW);
      pinMode (Pin, OUTPUT);
      delayMicroseconds (60);
      pinMode (Pin, INPUT);
    }

    d = d>>1; // now the next bit is in the least sig bit position.
  }
}


byte OneWireInByte (int Pin) // read byte, least sig byte first
{
  byte d, n, b;

  for (n=0; n<8; n++)
  {
    digitalWrite (Pin, LOW);
    pinMode (Pin, OUTPUT);
    delayMicroseconds (5);
    pinMode (Pin, INPUT);
    delayMicroseconds (5);
    b = digitalRead (Pin);
    delayMicroseconds (50);
    d = (d >> 1) | (b<<7); // shift d to right and insert b in most sig bit position
  }
  return (d);
}



int getECurrentTemp ()
{
  int HighByte, LowByte, TReading, Tc_100, sign, whole, fract;

  OneWireReset (tmp_data_pin);
  OneWireOutByte (tmp_data_pin, 0xcc);
  OneWireOutByte (tmp_data_pin, 0x44); // perform temperature conversion, strong pullup for one sec

  OneWireReset (tmp_data_pin);
  OneWireOutByte (tmp_data_pin, 0xcc);
  OneWireOutByte (tmp_data_pin, 0xbe);

  LowByte = OneWireInByte (tmp_data_pin);
  HighByte = OneWireInByte (tmp_data_pin);
  TReading = (HighByte << 8) + LowByte;
  sign = TReading & 0x8000;  // test most sig bit
  if (sign) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  whole = Tc_100 / 100;  // separate off the whole and fractional portions
  fract = Tc_100 % 100;

  return 273 + whole;
}




void startXmodemSend(char *p_file)
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

 // Serial.print("F:"); Serial.println(p_file);
 Serial.println("X");    
  delay(3000);
  int j = XSend(&file, &Serial, p_file);
  if (j == 0) {
    Serial.println("Y");
  } else {
    Serial.println("Z");
  }

}

void flash_led()
{
    while(1)
    {
      digitalWrite(status_led, HIGH);
      delay(500); 
      digitalWrite(status_led, LOW);
      delay(500);       
    } 
}
