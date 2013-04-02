/*

    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <SPI.h>
#include <RFM22.h>a
#include <util/crc16.h>
#include <SdFat.h>
//#include <SdFatUtil.h>
#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EasyTransferI2C.h>
#include <TinyGPS_HJOE.h>

#define DEBUG_ON

/***************************************************
 * Main program
 **************************************************/

#define SET_LED_WHITE 4
#define SET_LED_RED 3
#define SET_LED_BLUE 2
#define SET_LED_GREEN 1
#define SET_LED_OFF 0
//

#define WAIT_ARD_Y    10000

// Define filenames
//
#define SD_LOG_FILE        			"AURA.CSV"
//
// Arduino Pin assignment
//
#define PIN_GPS_RX 0        		//Fixed:Note: RX on Board is connected to RX on GPS Board
#define PIN_GPS_TX 1        		//Fixed:Note: TX on Board is connected to TX on GPS Board
#define PIN_GYRO_INT 2       		//Notes: Gyro Interupt
#define PIN_LED_RED 5        		//Fixed: Red LED
#define PIN_LED_BLUE 6       		//Fixed: Blue LED
#define PIN_LED_GREEN 7      		//Fixed: Blue GREEN

#define PIN_IC2_SDA 20       		//Fixed: SDA
#define PIN_IC2_SLC 21       		//Fixed: SLC
#define PIN_SPI_CS 53        		//Fixed: Card Select for SD Card

#define PIN_TEMP_EX A4       		//Fixed: External Temperature
#define PIN_TEMP_IN A2       		//Fixed: Internal Temperature
#define PIN_LIGHT_SEN A1       		//Fixed: Light Sensor

#define BMP085_ADDRESS 0x77  		// I2C address of BMP085
#define I2C_SLV_MOBILE_ADDRESS 9	//define slave i2c address
#define I2C_SLV_TRANSMITTER_ADDRESS 10	//define slave i2c address

#define aref_voltage 3.3 

//Declare structures

typedef union {
	typedef struct{
	  uint16_t t_count;  			    //2
	  byte 	  year;					//1 		  
	  byte    month;				//1	  
	  byte    day;					//1	  
	  byte    hour;					//1
	  byte    minute;				//1
	  byte    second;				//1
	  long    i_lat;				//4
	  long    i_long;				//4
	  long    i_alt; 				//4	
	  int16_t BMP085_P;				//2		Pressure  Pa  (Convert to Pa /100)
	  int16_t BMP085_T;				//2		Pressure Temp (*x0.1 DegC)
	  int16_t TMP_EXT; 				//2		External Temp
	  byte	  i_L;					//1		Light Sensor
	  byte 	  sats;					//1 ?
	  byte 	  hundredths;			//1
	  long    i_angle;				//4	
	  long    i_Hspeed;				//4		Horizontal speed
	  long    i_Vspeed;				//4		Vertical speed
	  unsigned long age; 			//4
	  unsigned long ihdop;			//4
	  int16_t MPU6050ax;			//2		accel x
	  int16_t MPU6050ay;			//2		accel y
	  int16_t MPU6050az;			//2		accel z
	  int16_t MPU6050gx;			//2		gyro x
	  int16_t MPU6050gy;			//2		gyro y
	  int16_t MPU6050gz; 			//2		gyro z  
	  int16_t TMP_INT;				//2		Internal Temp	
	  long	  BMP085_PFULL;			//4  
	  unsigned long usl_count;		//4	  
	} LOG_DATA_STRUCTURE;
    LOG_DATA_STRUCTURE vals;
	typedef struct {
	  uint16_t t_count;  			    //2
	  byte 	  year;					//1 	  
	  byte    month;				//1	  
	  byte    day;					//1	  
	  byte    hour;					//1
	  byte    minute;				//1
	  byte    second;				//1
	  long    i_lat;				//4
	  long    i_long;				//4
	  long    i_alt; 				//4	  
	  int16_t i_P;					//2		Pressure  Pa  (Convert to KPa)
	  int16_t i_T;					//2		Pressure Temp (*x0.1 DegC)
	  int16_t i_EXT; 				//2		External Temp
	  byte    i_L;					//1		Light Sensor
	  byte 	  sats;					//1 ?
	} IC2DATA_STRUCTURE;
        IC2DATA_STRUCTURE ic2Vals;
} HJPacket;

EasyTransferI2C ET;				//Easy Transfer
HJPacket adx;					//Send to ARDY structure

SdFat 			SD;				//SD
SdFile 			dataFile;		//SDFile
MPU6050 		mpu;			//MPU6050 Gyro
TinyGPS_HJOE 	gps;			//GPS


int			ALGPOLL;
bool		SENDWIRE;
bool		NEWGPSDATA;
//Variables
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

unsigned long elapseMobile;
unsigned long elapseTransmitter;
unsigned long elapseDump;
float 		voltage;

// MPU control/status vars
bool dmpReady = false;  				// set true if DMP init was successful
uint8_t mpuIntStatus;   				// holds actual interrupt status byte from MPU
uint8_t devStatus;      				// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    				// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     				// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; 				// FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint16_t CountRec = 0;

const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;

//short temperature;
//long pressure;

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

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
 
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
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

void datadump() {

    char SDString[120] = "";
    char BufString[30] = "";
    char comm[] = ",";
	
    //usl_count
	sprintf(BufString, "%ld", adx.vals.usl_count);    
	strcat(SDString,BufString);
    strcat(SDString,comm);

	//t_count
	sprintf(BufString, "%d", adx.vals.t_count);    
	strcat(SDString,BufString);
    strcat(SDString,comm);
	
   //time
   	int tmp_year = adx.vals.year + 2000;
    sprintf(BufString, "%04d-%02d-%02dT%02d:%02d:%02d.%02d",tmp_year, adx.vals.month, adx.vals.day,adx.vals.hour, adx.vals.minute, adx.vals.second, adx.vals.hundredths); 
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
	//Lat
    sprintf(BufString, "%ld", adx.vals.i_lat);
    strcat(SDString,BufString);
    strcat(SDString,comm);

 	//Long
    sprintf(BufString, "%ld", adx.vals.i_long);
    strcat(SDString,BufString);
    strcat(SDString,comm);

 	//Alt
    sprintf(BufString, "%ld", adx.vals.i_alt);
    strcat(SDString,BufString);
    strcat(SDString,comm);
 	
	//angle	
    sprintf(BufString, "%ld", adx.vals.i_angle);
    strcat(SDString,BufString);
    strcat(SDString,comm);
 	
	//i_Hspeed	
    sprintf(BufString, "%ld", adx.vals.i_Hspeed);
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
	//i_Vspeed	
    sprintf(BufString, "%ld", adx.vals.i_Vspeed);
    strcat(SDString,BufString);
    strcat(SDString,comm);

	//MPU6050ax
    sprintf(BufString, "%d", adx.vals.MPU6050ax);
    strcat(SDString,BufString);
    strcat(SDString,comm);

	//MPU6050ay
    sprintf(BufString, "%d", adx.vals.MPU6050ay);
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
   //MPU6050az
    sprintf(BufString, "%d", adx.vals.MPU6050az);
    strcat(SDString,BufString);
    strcat(SDString,comm);

   //MPU6050gx
    sprintf(BufString, "%d", adx.vals.MPU6050gx);
    strcat(SDString,BufString);
    strcat(SDString,comm);

   //MPU6050gy
    sprintf(BufString, "%d", adx.vals.MPU6050gy);
    strcat(SDString,BufString);
    strcat(SDString,comm);

   //MPU6050gz
    sprintf(BufString, "%d", adx.vals.MPU6050gz);
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
   //TMP36_INT
    sprintf(BufString, "%d", adx.vals.TMP_INT);
    strcat(SDString,BufString);
    strcat(SDString,comm);

   //TMP36_EXT
    sprintf(BufString, "%d", adx.vals.TMP_EXT);
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
   //Light Sensor
    sprintf(BufString, "%d", adx.vals.i_L);
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
   //Pressure
    sprintf(BufString, "%d", adx.vals.BMP085_P);
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
   //Pressure Temp
    sprintf(BufString, "%d", adx.vals.BMP085_T);
    strcat(SDString,BufString);
    strcat(SDString,comm);

	//Light sats
    sprintf(BufString, "%d", adx.vals.sats);
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
   //age
    sprintf(BufString, "%d", adx.vals.age);
    strcat(SDString,BufString);
    strcat(SDString,comm);
	
   //ihdop
    sprintf(BufString, "%d", adx.vals.ihdop);
    strcat(SDString,BufString);
	
   #ifdef DEBUG_ON	
	Serial.println(SDString);
   #endif 
   
   dataFile.println(SDString);
   dataFile.sync();
 

}

/***************************************************
 * LED STATUS DISPLAY FUNTIONS
 * LED is a Common Anode, therefore the pin is the cathode
 * and must be set LOW to be on, and HIGH to be turned off! 
 **************************************************/
void SET_LED_Status(int stat, int intDelay){

  if (stat == SET_LED_RED) {
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_BLUE, HIGH);  
  } 
  else if (stat == SET_LED_BLUE) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_BLUE, LOW);  
  } 
  else if (stat == SET_LED_GREEN){
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_BLUE, HIGH);  
  } 
  else if (stat == SET_LED_WHITE){
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_BLUE, LOW); 
  } else {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_BLUE, HIGH); 
  }
  if (intDelay > 0 ) delay(intDelay);
}

void SET_LED_Status(int stat){
  SET_LED_Status(stat, 1000);
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
	Serial.begin(9600);
	Serial1.begin(9600); 
	analogReference(EXTERNAL);
	pinMode(PIN_LED_GREEN, OUTPUT);
	pinMode(PIN_LED_RED, OUTPUT);
	pinMode(PIN_LED_BLUE, OUTPUT);
	pinMode(PIN_SPI_CS,OUTPUT);  //Chip Select Pin for the SD Card
	pinMode(10, OUTPUT);

	SET_LED_Status(SET_LED_RED,500);

	//Setup GPS
	gps.init();
	gps.configureUbloxSettings();

	SET_LED_Status(SET_LED_GREEN,500);

	// join I2C bus //start I2C transfer to the Module/Transmitter
	Wire.begin();
	ET.begin(details(adx.ic2Vals), &Wire);

	// Initialise the MPU
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0) {									  	// make sure it worked (returns 0 if so)
	#ifdef DEBUG_ON	
		Serial.println("MPU OK");
	#endif 	
		mpu.setDMPEnabled(true);							    // turn on the DMP, now that it's ready
		attachInterrupt(0, dmpDataReady, RISING);			    // enable Arduino interrupt detection
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();				// get expected DMP packet size for later comparison
	} else {
	#ifdef DEBUG_ON	
		Serial.println("MPU not working!!");
	#endif 	
	}

	bmp085Calibration();
	
	SET_LED_Status(SET_LED_BLUE,500);   

	//Connect to the SD Card
	if(!SD.init(SPI_HALF_SPEED, PIN_SPI_CS)) {
		SET_LED_Status(SET_LED_RED,500);
		SET_LED_Status(SET_LED_OFF);
	#ifdef DEBUG_ON	
		Serial.println("SD not working!!");
	#endif 
		return;
	} else {
	#ifdef DEBUG_ON	
		Serial.println("SD OK");
	#endif 	
	}

	boolean  bolFileexists = false;							  			//Check if Log file exists
	if (SD.exists(SD_LOG_FILE)) bolFileexists = true;
		dataFile.open(SD_LOG_FILE, O_CREAT | O_WRITE | O_APPEND);	    //Open Logfile
	
	if (!dataFile.isOpen()) {
		SET_LED_Status(SET_LED_WHITE,500);
		SET_LED_Status(SET_LED_RED,500);
		return;
	}

	SET_LED_Status(SET_LED_OFF,0);  
	SET_LED_Status(SET_LED_WHITE,500);
	SET_LED_Status(SET_LED_GREEN,500);
	elapseMobile = millis();
	elapseTransmitter = millis();
	elapseDump = millis();
	ALGPOLL = 0;
	NEWGPSDATA = false;
	
	adx.vals.t_count = 0;
	adx.vals.usl_count = 0;
	adx.vals.year = 0;
	adx.vals.month = 0;
	adx.vals.day = 0;
	adx.vals.hour = 0;
	adx.vals.minute = 0;
	adx.vals.second = 0;
	adx.vals.i_lat = 0;
	adx.vals.i_long = 0;
	adx.vals.i_alt = 0;
	adx.vals.TMP_INT = 0;
	adx.vals.TMP_EXT = 0;
	adx.vals.i_L = 0;
	adx.vals.sats = 0;
	adx.vals.hundredths = 0;
	adx.vals.i_angle = 0;
	adx.vals.i_Hspeed = 0;
	adx.vals.i_Vspeed = 0;
	adx.vals.age = 0;
	adx.vals.ihdop = 0;
	adx.vals.MPU6050ax = 0;
	adx.vals.MPU6050ay = 0;
	adx.vals.MPU6050az = 0;
	adx.vals.MPU6050gx = 0;
	adx.vals.MPU6050gy = 0;
	adx.vals.MPU6050gz = 0;
	adx.vals.BMP085_P = 0;
	adx.vals.BMP085_T = 0;
}

void loop() {

	adx.vals.usl_count++;
	
		//flip flop between TMP's to allow voltages to settle
	if (ALGPOLL == 0) {
	  voltage = analogRead(PIN_TEMP_IN) * aref_voltage;
	  voltage = ((voltage/1024.0) - 0.5) * 1000;
	  adx.vals.TMP_INT = (int)voltage;	
	  ALGPOLL++;
	}
	else if (ALGPOLL == 1) {  
	  voltage = analogRead(PIN_TEMP_EX) * aref_voltage;
	  voltage = ((voltage/1024.0) - 0.5) * 1000;
	  adx.vals.TMP_EXT = (int)voltage;
	  ALGPOLL++;		  
	}
	else if (ALGPOLL == 2) {  
	   voltage = analogRead(PIN_LIGHT_SEN);
	   voltage = (voltage/1024) * 250;
	   adx.vals.i_L = (byte)voltage;
	  ALGPOLL++;	  

	}
	if (ALGPOLL >2) ALGPOLL=0;
	
    //if (feedgps()) NEWGPSDATA =true;
	mpu.getMotion6(&adx.vals.MPU6050ax, &adx.vals.MPU6050ay, &adx.vals.MPU6050az, &adx.vals.MPU6050gx, &adx.vals.MPU6050gy, &adx.vals.MPU6050gz);
	adx.vals.BMP085_T = bmp085GetTemperature(bmp085ReadUT());
	adx.vals.BMP085_PFULL = bmp085GetPressure(bmp085ReadUP());
	adx.vals.BMP085_P = (int16_t)(bmp085GetPressure(bmp085ReadUP())/100);
 
	byte lcount = 0;
	while (!NEWGPSDATA && lcount < 255) {
		NEWGPSDATA = feedgps();
		lcount++
	}
		
	if (NEWGPSDATA) {
		int tmp_year = 0;
		gps.crack_datetime(&tmp_year, &adx.vals.month, &adx.vals.day,&adx.vals.hour, &adx.vals.minute, &adx.vals.second, &adx.vals.hundredths, &adx.vals.age);
		adx.vals.year = tmp_year - 2000;
		gps.get_position(&adx.vals.i_lat, &adx.vals.i_long, &adx.vals.age);
		if (gps.altitude() != TinyGPS_HJOE::GPS_INVALID_ALTITUDE && gps.altitude() >= 0) adx.vals.i_alt = gps.altitude(); 
		adx.vals.i_angle = gps.course();
		adx.vals.i_Hspeed = gps.speed(); 
		adx.vals.sats = gps.satellites();
		adx.vals.ihdop = gps.hdop();
		SET_LED_Status(SET_LED_BLUE,0);	
	} else {
		SET_LED_Status(SET_LED_GREEN,10);
	}
	
	//flip flop between I2C's to avoid both on one loop
	if (!SENDWIRE && (millis() - elapseTransmitter) >10000) {
		adx.vals.t_count++;
		ET.sendData(I2C_SLV_TRANSMITTER_ADDRESS);
		elapseTransmitter = millis();
	}
	
	if (SENDWIRE && (millis() - elapseMobile) >10000) {
		adx.vals.t_count++;
		ET.sendData(I2C_SLV_MOBILE_ADDRESS);		
		elapseMobile = millis();
	} 
	datadump();
	SET_LED_Status(SET_LED_OFF,0);
	//elapseDump = millis();
	NEWGPSDATA = false;
	SENDWIRE = !SENDWIRE;
}

static bool feedgps() {
  while (Serial1.available()) {
    if (gps.encode(Serial1.read()))
      return true;
	}
  return false;
}
