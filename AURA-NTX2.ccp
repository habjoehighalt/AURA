/*
 HABJOE  Arduino Flight script
 HABJOE - Andrew Myatt and Joseph Myatt March/2013


KEYWORDS: ARDUINO MINIPRO, HIGH ALTITUDE BALLOON, NTX2, AI2C, WIRING

This code is in the public domain.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
This software is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

RECEIVES PACKET OF INFORMATION VIA I2C INTERFACE AND SENDS IT VIA NTX2 TRANSMITTER.

#include "Wire.h"
#include "I2Cdev.h"
#include <EasyTransferI2C.h>
#include <String.h>
#include <util/crc16.h>

/***************************************************
 * Main program
 **************************************************/

#define LED_ON 1
#define LED_OFF 0
//
// Arduino Pin assignment
//
static int PIN_NTX_TX = 8;        //Note: NTX Transmitter out
static int PIN_LED_GREEN = 13;    //Fixed: GREEN

 /***********************************************************************************************************************************
 * Data definitions
 * 
 *
 */

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
     	 
EasyTransferI2C ET; 			   //Easy Transfer
IC2DATA_STRUCTURE i2cVals;		             
 
/*********************************************************
 * User settings
 */

static char USR_PAYLOADNAME[]		= "$$$$AURA";								// Include the $$ as well.
char 		telemetryString[80];  	
#define I2C_SLAVE_ADDRESS 10

/***********************************************************************************************************************************
 * user functions
 * 
 * 
 */
 
 /*****************************************
 * datadump
 */  
void datadump() {
	//Need to be made specific to the actual payload being used.

	sprintf(telemetryString, "$$AURA,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%d,%d,%d,%d", 
	i2cVals.t_count,cVals.hour, i2cVals.minute, i2cVals.second,i2cVals.i_lat,
	i2cVals.i_long,i2cVals.i_alt,i2cVals.i_P,i2cVals.i_T, i2cVals.i_EXT,
	i2cVals.i_L,i2cVals.sats);

	unsigned int CHECKSUM = gps_CRC16_checksum(telemetryString);  // Calculates the checksum for this datastring
	char checksum_str[6];
	sprintf(checksum_str, "*%04X\n", CHECKSUM);
	strcat(telemetryString,checksum_str);	
}
void setup() {
	pinMode(PIN_LED_GREEN, OUTPUT);
	pinMode(PIN_NTX_TX,OUTPUT);

	LED_Status(LED_ON,0);
	Serial.begin(19200); 
	Wire.begin(I2C_SLAVE_ADDRESS);																// Uses the I2C interface to get the telemetry values.
	ET.begin(details(i2cVals), &Wire);
	//define handler function on receiving data ps. you don't want to put anything in this that takes any time.!!!
	Wire.onReceive(receive);	 

	for (int i = 0; i<10;i++){
		LED_Status(LED_ON,200);
		LED_Status(LED_OFF,100);
	}

}

void receive(int numBytes) {
// Don't put anything in this function as it stops the Mobile phone loop working correctlu	
}

void LED_Status(int stat, int intDelay){
  if (stat == LED_ON){
    digitalWrite(PIN_LED_GREEN, HIGH);
  } 
  else {
    digitalWrite(PIN_LED_GREEN, LOW); 
  }
  if (intDelay > 0 ) delay(intDelay);
}

void LED_Status(int stat){
  LED_Status(stat, 1000);
}

/***************************************************
 * NTX2 Transmit functions
 * 
 **************************************************/

void rtty_txstring ( char * string) {

  //Simple function to sent a char at a time to rtty_txbyte function. 
  char c;
  c = *string++;    
  while (c != '\0'){
    rtty_txbyte (c);
    c = *string++;
  }
  rtty_txbyte (c);

}

void rtty_txbyte (char c){
  /* Simple function to sent each bit of a char to 
   ** rtty_txbit function. 
   ** NB The bits are sent Least Significant Bit first
   **
   ** All chars should be preceded with a 0 and 
   ** proceded with a 1. 0 = Start bit; 1 = Stop bit
   **/

  int i;
  rtty_txbit (0); // Start bit
  // Send bits for for char LSB first 

  for (i=0;i<7;i++) {// Change this here 7 or 8 for ASCII-7 / ASCII-8
    if (c & 1) rtty_txbit(1); 
    else rtty_txbit(0); 
    c = c >> 1;
  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit){
  if (bit) {
    digitalWrite(PIN_NTX_TX, HIGH); 
	} else {
    digitalWrite(PIN_NTX_TX, LOW);
	}

  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
  delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.
}

uint16_t gps_CRC16_checksum (char *string){
  size_t i;
  uint16_t crc;
  uint8_t c;
  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  } 
  return crc;
}



void loop() {
	LED_Status(LED_OFF,0);
	if(ET.receiveData()) {
		LED_Status(LED_ON,0);
		datadump();		
		Serial.println(telemetryString);
		rtty_txstring("$$");
		rtty_txstring(telemetryString);
	}
}

