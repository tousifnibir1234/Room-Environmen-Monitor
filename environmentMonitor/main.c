/*
 * hotash.c
 *
 * Created: 9/7/2019 4:35:34 AM
 * Author : nibir
 */ 

#define F_CPU 1000000UL // 16 MHz clock speed
#include<math.h>
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include "LCD16x2_4bit.h"
#include "UART.h"
//#include "lcd.h"
#ifndef DHT_SETTINGS_H_INCLUDED
#define DHT_SETTINGS_H_INCLUDED


//----- Configuration --------------------------//
#define DHT_Type	DHT22        //DHT11 or DHT22
#define DHT_Pin		C, 0
//----------------------------------------------//
#endif

#ifndef IO_MACROS_H_INCLUDED
#define IO_MACROS_H_INCLUDED


///for gsm module
#define BAUD 9600
#define BAUDRATE ((16*F_CPU)/(BAUD*16UL)-1)
unsigned char rxdata,a,cmd,b;
unsigned int z;
unsigned char message[15];
unsigned char cmd1[]={"AT"};
unsigned char cmd2[]={"AT+CMGF=1"};
unsigned char cmd3[]={"AT+CMGS="};
unsigned char cmd4[100];
unsigned char cmd5[]={"8801557338468"};
unsigned char atcmd[] = "AT\r";
unsigned char atcall[] = "ATD+8801557338468;\r";
int callState = 0;
unsigned char mobile_num[]="8801870214264";

unsigned char Command_CMGF[]="AT+CMGF=1";

unsigned char Command_AT[]="AT";

unsigned char msg[]="Danger! The patient exceed the maximum allowable rate.";

///////////***************************RECEIVE********/////////////////

#include <avr/interrupt.h>
#include <stdbool.h>
#define SREG   _SFR_IO8(0x3F)


char status_flag = 0;	
volatile int buffer_pointer;   
char message_received[60];		//Return values of sensor - Must be pointers or more preferably size-one arrays



///////////****************************************



void usartinit()
{
	UBRRH=00;
	UBRRL=12;
	UCSRB|=(1<<RXEN)|(1<<TXEN);
	UCSRC|=(1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);
	
}
unsigned char rxvalue(void)
{
	while(!(UCSRA&(1<<RXC)));
	{
		rxdata=UDR;
		return rxdata;
	}
}
void send(){
	
	for(z=0;cmd1[z]!='\0';z++)
	{
		UDR = cmd1[z];
		_delay_ms(96);
	}
	UDR = ('\r');
	_delay_ms(480);
	for(z=0;cmd2[z]!='\0';z++)
	{
		UDR = cmd2[z];
		_delay_ms(96);
	}
	UDR = ('\r');
	_delay_ms(480);
	for(z=0;cmd3[z]!='\0';z++)
	{
		UDR = cmd3[z];
		_delay_ms(96);
	}
	UDR = ('"');
	_delay_ms(96);
	
	for(z=0;cmd5[z]!='\0';z++)
	{
		UDR = cmd5[z];
		_delay_ms(96);
	}
	UDR = ('"');
	_delay_ms(96);
	UDR = ('\r');
	_delay_ms(480);
	for(z=0;cmd4[z]!='\0';z++)
	{
		UDR = cmd4[z];
		_delay_ms(96);
	}
	UDR = (26);
	_delay_ms(96);
	
}
void uart_transmit (unsigned char data)
{
	while (!( UCSRA & (1<<UDRE)));
	UDR = data;
}
void uart_Send(unsigned char *str, int n){
	int z;
	for(z=0;z<n;z++)
	{
		UDR = str[z];
		while (!(UCSRA & (1<<UDRE)));
	}
	
}
void Call(){
   // lcd_clear();
	//lcd_print("in call");
	//Lcd4_Write_String("In Call...");
	uart_Send(atcmd,4);
	_delay_ms(500);
	//lcd_print("in call2");
	uart_Send(atcall,20);
	_delay_ms(2000);
}

void send_msg(){

	Lcd4_Clear();
	
	Lcd4_Write_String("In Message...");
	
	char CrLf[]={13,10,0};

	char ctrlz[]={26,0};

	//send AT command

	printf("%s",Command_AT);

	puts(CrLf);



	//set message to text mode

	printf("%s",Command_CMGF);

	puts(CrLf);

	_delay_ms(1000);

	//set phone no.

	printf("AT+CMGS=\"%s\"",mobile_num);

	puts(CrLf);

	_delay_ms(1000);



	//send the msg

	printf("%s",msg);

	puts(ctrlz);

	_delay_ms(1000);

}


/*
||
||  Filename:	 		IO_Macros.h
||  Title: 			    IO manipulation macros
||  Author: 			Efthymios Koktsidis
||	Email:				efthymios.ks@gmail.com
||  Compiler:		 	AVR-GCC
||	Description:		This library contains macros for
||						easy port manipulation (similar
||						to Arduino).
||
||	Demo:
|| 1.	#define LED		A, 0		|| 6. 	PinModeToggle(BUTTON);
|| 2.	#define BUTTON	A, 1		|| 7. 	DigitalWrite(LED, LOW);
|| 3.								|| 8. 	DigitalWrite(LED, HIGH);
|| 4. 	PinMode(BUTTON, OUTPUT);	|| 9. 	DigitalLevelToggle(LED);
|| 5. 	PinMode(LED, OUTPUT);		||10.	int a = DigitalRead(BUTTON);
||
*/
//#include <avr/io.h>

//----- I/O Macros -----
//Macros to edit PORT, DDR and PIN
#define PinMode(			x, y)	( 		y 			?	_SET(DDR, x)	:	_CLEAR(DDR, x)		)
#define DigitalWrite(		x, y)	( 		y 			?	_SET(PORT, x)	:	_CLEAR(PORT, x)		)
#define DigitalRead(		x)		(						_GET(PIN, x)							)
#define PinModeToggle(		x)		(						_TOGGLE(DDR, x)							)
#define DigitalLevelToggle(	x)		(						_TOGGLE(PORT, x)						)

//General use bit manipulating commands
#define BitSet(		x, y)			(	x |=	 (1UL<<y)			)
#define BitClear(	x, y)			(	x &=	(~(1UL<<y))			)
#define BitToggle(	x, y)			(	x ^=	 (1UL<<y)			)
#define BitCheck(	x, y)			(	x &		 (1UL<<y)	? 1 : 0	)

//Access PORT, DDR and PIN
#define PORT(	port)				(_PORT(	port))
#define DDR(	port)				(_DDR(	port))
#define PIN(	port)				(_PIN(	port))

#define _PORT(	port)				(PORT##	port)
#define _DDR(	port)				(DDR##	port)
#define _PIN(	port)				(PIN##	port)

#define _SET(	type, port, bit)	(	BitSet(		(type##port),	bit)	)
#define _CLEAR(	type, port, bit)	(	BitClear(	(type##port),	bit)	)
#define _TOGGLE(type, port, bit)	(	BitToggle(	(type##port),	bit)	)
#define _GET(	type, port, bit)	(	BitCheck(	(type##port),	bit)	)

//Definitions
#define Input		0
#define Output		!Input
#define Low			0
#define High		!Low
#define False		0
#define True		!False
//------------------
#endif

#ifndef DHT_H_INCLUDED
#define DHT_H_INCLUDED
/*
||
||  Filename:	 		DHT.h
||  Title: 			    DHTxx Driver
||  Author: 			Efthymios Koktsidis
||	Email:				efthymios.ks@gmail.com
||  Compiler:		 	AVR-GCC
||	Description:
||	This library can drive DHT11 and DHT22 sensors.
||
*/

//------ Headers ------//
#include <inttypes.h>
#include <util/delay.h>
//#include <avr/io.h>

//----------------------//

//----- Auxiliary data -------------------//
#define DHT11						 1
#define DHT22						 2
#define DHT_ReadInterval			1500

#define __DHT_Delay_Setup			2000

enum DHT_Status_t
{
	DHT_Ok,
	DHT_Error_Humidity,
	DHT_Error_Temperature,
	DHT_Error_Checksum,
	DHT_Error_Timeout
};
//-----------------------------------------//

//----- Prototypes---------------------------//
void DHT_Setup(void);
enum DHT_Status_t DHT_status(void);
void DHT_ReadRaw(uint8_t Data[4]);
void DHT_ReadTemperature(double *Temperature);
void DHT_ReadHumidity(double *Humidity);
void DHT_Read(double *Temperature, double *Humidity);
double DHT_ConvertToFahrenheit(double Temperature);
double DHT_ConvertToKelvin(double Temperature);
//-------------------------------------------//
#endif

//----- Auxiliary data ----------//
enum DHT_Status_t __DHT_STATUS;

#if (DHT_Type == DHT11)
#define __DHT_Temperature_Min	0
#define __DHT_Temperature_Max	50
#define __DHT_Humidity_Min		20
#define __DHT_Humidity_Max		90
#define __DHT_Delay_Read		50
#elif (DHT_Type == DHT22)
#define __DHT_Temperature_Min	-40
#define __DHT_Temperature_Max	80
#define __DHT_Humidity_Min		0
#define __DHT_Humidity_Max		100
#define __DHT_Delay_Read		20
#endif
//-------------------------------//

//----- Prototypes ----------------------------//
static double DataToTemp(uint8_t Data3, uint8_t Data4);
static double DataToHum(uint8_t Data1, uint8_t Data2);
//---------------------------------------------//

//----- Functions -----------------------------//
//Setup sensor.
void DHT_Setup(void)
{
	_delay_ms(__DHT_Delay_Setup);
	__DHT_STATUS = DHT_Ok;
}

//Get sensor status.
enum DHT_Status_t DHT_status(void)
{
	return (__DHT_STATUS);
}

//Read raw buffer from sensor.
void DHT_ReadRaw(uint8_t Data[4])
{
	uint8_t buffer[5] = {0, 0, 0, 0, 0};
	uint8_t retries, i;
	int8_t j;
	__DHT_STATUS = DHT_Ok;
	retries = i = j = 0;

	//----- Step 1 - Start communication -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Request data
		DigitalWrite(DHT_Pin, Low);			//DHT_PIN = 0
		PinMode(DHT_Pin, Output);			//DHT_PIN = Output
		_delay_ms(__DHT_Delay_Read);
		
		//Setup DHT_PIN as input with pull-up resistor so as to read data
		DigitalWrite(DHT_Pin, High);		//DHT_PIN = 1 (Pull-up resistor)
		PinMode(DHT_Pin, Input);			//DHT_PIN = Input

		//Wait for response for 20-40us
		retries = 0;
		while (DigitalRead(DHT_Pin))
		{
			_delay_us(2);
			retries += 2;
			if (retries > 60)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
	}
	//----------------------------------------

	//----- Step 2 - Wait for response -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Response sequence began
		//Wait for the first response to finish (low for ~80us)
		retries = 0;
		while (!DigitalRead(DHT_Pin))
		{
			_delay_us(2);
			retries += 2;
			if (retries > 100)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
		//Wait for the last response to finish (high for ~80us)
		retries = 0;
		while(DigitalRead(DHT_Pin))
		{
			_delay_us(2);
			retries += 2;
			if (retries > 100)
			{
				__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
				break;
			}
		}
	}
	//--------------------------------------

	//----- Step 3 - Data transmission -----
	if (__DHT_STATUS == DHT_Ok)
	{
		//Reading 5 bytes, bit by bit
		for (i = 0 ; i < 5 ; i++)
		for (j = 7 ; j >= 0 ; j--)
		{
			//There is always a leading low level of 50 us
			retries = 0;
			while(!DigitalRead(DHT_Pin))
			{
				_delay_us(2);
				retries += 2;
				if (retries > 70)
				{
					__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
					j = -1;								//Break inner for-loop
					i = 5;								//Break outer for-loop
					break;								//Break while loop
				}
			}

			if (__DHT_STATUS == DHT_Ok)
			{
				//We read data bit || 26-28us means '0' || 70us means '1'
				_delay_us(35);							//Wait for more than 28us
				if (DigitalRead(DHT_Pin))				//If HIGH
				BitSet(buffer[i], j);				//bit = '1'

				retries = 0;
				while(DigitalRead(DHT_Pin))
				{
					_delay_us(2);
					retries += 2;
					if (retries > 100)
					{
						__DHT_STATUS = DHT_Error_Timeout;	//Timeout error
						break;
					}
				}
			}
		}
	}
	//--------------------------------------


	//----- Step 4 - Check checksum and return data -----
	if (__DHT_STATUS == DHT_Ok)
	{
		if (((uint8_t)(buffer[0] + buffer[1] + buffer[2] + buffer[3])) != buffer[4])
		{
			__DHT_STATUS = DHT_Error_Checksum;	//Checksum error
		}
		else
		{
			//Build returning array
			//data[0] = Humidity		(int)
			//data[1] = Humidity		(dec)
			//data[2] = Temperature		(int)
			//data[3] = Temperature		(dec)
			//data[4] = Checksum
			for (i = 0 ; i < 4 ; i++)
			Data[i] = buffer[i];
		}
	}
	//---------------------------------------------------
}

//Read temperature in Celsius.
void DHT_ReadTemperature(double *Temperature)
{
	double waste[1];
	DHT_Read(Temperature, waste);
}

//Read humidity percentage.
void DHT_ReadHumidity(double *Humidity)
{
	double waste[1];
	DHT_Read(waste, Humidity);
}

//Read temperature and humidity.
void DHT_Read(double *Temperature, double *Humidity)
{
	uint8_t data[4] = {0, 0, 0, 0};

	//Read data
	DHT_ReadRaw(data);
	
	//If read successfully
	if (__DHT_STATUS == DHT_Ok)
	{
		//Calculate values
		*Temperature = DataToTemp(data[2], data[3]);
		*Humidity = DataToHum(data[0], data[1]);
		
		//Check values
		if ((*Temperature < __DHT_Temperature_Min) || (*Temperature > __DHT_Temperature_Max))
		__DHT_STATUS = DHT_Error_Temperature;
		else if ((*Humidity < __DHT_Humidity_Min) || (*Humidity > __DHT_Humidity_Max))
		__DHT_STATUS = DHT_Error_Humidity;
	}
}

//Convert temperature from Celsius to Fahrenheit.
double DHT_ConvertToFahrenheit(double Temperature)
{
	return (Temperature * 1.8 + 32);
}

//Convert temperature from Celsius to Kelvin.
double DHT_ConvertToKelvin(double Temperature)
{
	return (Temperature + 273.15);
}

//Convert temperature data to double temperature.
static double DataToTemp(uint8_t Data2, uint8_t Data3)
{
	double temp = 0.0;
	
	#if (DHT_Type == DHT11)
	{
		temp = Data2;
	
	}
	
	
	#elif (DHT_Type == DHT22)
	//(Integral<<8 + Decimal) / 10
	temp = (BitCheck(Data2, 7) ? ((((Data2 & 0x7F) << 8) | Data3) / (-10.0)) : (((Data2 << 8) | Data3) / 10.0));
	#endif
	
	
	return temp;
}

static double DataToHum(uint8_t Data0, uint8_t Data1)
{
	double hum = 0.0;
	
	#if (DHT_Type == DHT11)
	hum = Data0;
	#elif (DHT_Type == DHT22)
	//(Integral<<8 + Decimal) / 10
	hum = ((Data0<<8) | Data1) / 10.0;
	#endif
	
	return hum;
}
//---------------------------------------------//
void doCall(){
	Call();
}
int main(void)
{
	lcdinit();					/* initialize LCD */
	lcd_clear();
	ADMUX = 0b01000101; // REFS1:0 = 01    -> AVCC as reference,
	ADCSRA = 0b10000001;
	
	
	
	char outstr[15];
	usartinit();
	double temp[1], hum[1];		//Return values of sensor - Must be pointers or more preferably size-one arrays
	//doCall();
	
	DHT_Setup();
	
	//Loop
	while (1 == 1)
	{
		//Read from sensor
		lcdinit();
		lcd_clear();
		
		DHT_Read(temp, hum);
		_delay_ms(1000);
		lcd_gotoxy(0,0);
		lcd_print("Humidity=");
		lcd_gotoxy(10,0);
		dtostrf(hum[0] , 2 , 2 , outstr);
		lcd_print(outstr);
		lcd_print("%");
		
		if(hum[0]>99.00|| temp[0]>36.00 )
		{
			Call();
			_delay_ms(1000);
			//char * str="humidity over limit";
			
			cmd4[0]='N';
			cmd4[1]='e';
			cmd4[2]='e';
			cmd4[3]='d';
			cmd4[4]='_';
			cmd4[5]='A';
			cmd4[6]='c';
			cmd4[7]='t';
			cmd4[8]='i';
			cmd4[9]='o';
			cmd4[10]='n';
			
			send();
			//_delay_ms(1000);
		}
		
	
		//cmd4[0]='g';
		//send();
		

		
		
		lcd_gotoxy(0,1);
		lcd_print("Temp=");
		dtostrf(temp[0],2,2,outstr);
		lcd_gotoxy(6,1);
		
		lcd_print(outstr);
		lcd_print("C");
		_delay_ms(1000);
		
		
		//________START
		//_delay_ms(2000);
		lcd_clear();
		
		lcd_gotoxy(1,0);
		//Lcd4_Set_Cursor(1,0);
		lcd_print("Sound Level");
	
		ADCSRA |= (1 << ADSC);
		while(ADCSRA & (1 << ADSC)){
			
		}
		
		double final_result =13 * log(5.0 *200* ADC/ 1024.0 ) ;
		
		char outstr[15];
		dtostrf(final_result , 5 , 3 , outstr);
		
		lcd_gotoxy(2,1);
		lcd_print(outstr);
		lcd_print("dB");
		if(final_result>80 )
		{
			//Call();
			_delay_ms(1000);
			//char * str="humidity over limit";
			
			cmd4[0]='S';
			cmd4[1]='o';
			cmd4[2]='u';
			cmd4[3]='n';
			cmd4[4]='d';
			cmd4[5]='C';
			cmd4[6]='o';
			cmd4[7]='n';
			cmd4[8]='t';
			cmd4[9]='r';
			cmd4[10]='o';
			cmd4[11]='l';

			
			send();
			//_delay_ms(1000);
		}
		_delay_ms(1300);
		
		
		//Lcd4_Shift_Right();
		//________END__________
	
		
		//Sensor needs 1-2s to stabilize its readings
		//_delay_ms(1000);
		
	}
	
	
	
	
}

