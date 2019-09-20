/*
 * Flow_Demo_Total_Code.cpp
 *
 * Created: 9/14/2019 8:04:12 PM
 * Author : Vishal Dhandhukia
 */ 

#define F_CPU 1000000UL //8MHz internal clock with divider set at 8 (1MHz)
#define BAUD 9600 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include <util/delay.h>

char String[]="Hello World";    //String[] is in fact an array but when we put the text between the " " symbols the compiler threats it as a String and automatically puts the null termination character in the end of the text


#define ADC_PIN			0
#define I2C_DATA		0


void ADC_Init(void) //Timer Initialization
{
	ADCSRA |= 0b10000011; //Enable ADC, prescale 8 for 125kHz, auto trigger
	ADCSRA &= 0b11100011; //Enable ADC, prescale 8 for 125kHz, auto trigger
	ADCSRB &= 0b11111000; //Trigger source = free running 
	ADMUX |= 0b01100000; //Avcc reference, Left shift, ADC0. Set 0b01100001 for Channel 2, our sword sensor 
	ADMUX &= 0b01110000; //Avcc reference, Left shift, ADC0
	DIDR0 = 0b00000001; //Remove input buffer from channel 1 (not needed?)
}

/* This function just keeps the reading code out of the loop itself.
 * It takes the analog pin number as a parameter and returns the
 * analog reading on that pin as a result.*/
uint16_t day_read(uint16_t adcx);
uint16_t sword_read(uint16_t adcx);

uint16_t day_read(uint16_t adcx) // sets the daytime sensor to read from channel 0
{
	ADMUX	&=	0b11110000; //Select ADC0
	ADMUX	|=	0b01100000; //Select AVcc ref
	_delay_ms(1);
	ADCSRA |= 0b01000000; 	//This starts the conversion.
	while ( (ADCSRA & _BV(ADSC)) ); //idle loop
	return ADCH; 	//Finally, we return the converted value to the calling function.
}

uint16_t sword_read(uint16_t adcx) //sets the sword sensor to read from channel 1
{
	ADMUX	&=	0b11110001; //Select ADC1
	ADMUX	|=	0b01100001; //Select AVcc ref. 
	_delay_ms(1);
	ADCSRA |= 0b01000000; 	//This starts the conversion.
	while ( (ADCSRA & _BV(ADSC)) ); //idle loop
	return ADCH; 	//Finally, we return the converted value to the calling function.
}

uint16_t current_read(uint16_t adcx) //sets the sword sensor to read from channel 2
{
	ADMUX	&=	0b11110011; //Select ADC2
	ADMUX	|=	0b01100011; //Select AVcc ref.
	_delay_ms(1);
	ADCSRA |= 0b01000000; 	//This starts the conversion.
	while ( (ADCSRA & _BV(ADSC)) ); //idle loop
	return ADCH; 	//Finally, we return the converted value to the calling function.
}

void PMW_Init(void) //Timer Initialization
{
	OCR0A = 128; //50% duty cycle (8 bit - 255)
	TCCR0A |= 0b10000011; //Clear on match, Fast PWM, OC0B disabled
	TCCR0A &= 0b10001111; //Clear on match, Fast PWM, OC0B disabled
	TCCR0B |= 0b00000001; //No prescaling
	TCCR0B &= 0b00110001; //No prescaling
}

void Disable_SwordLED(void) //Disable Sword LED 
{
	PORTC &= 0b11111011; 
	_delay_ms(500);
} 

void Enable_SwordLED(void) //Enable Sword LED
{
	PORTC |= 0b00000100;
	_delay_ms(500);
}

void UART0_TX(unsigned char data)
{
	while ( !( UCSR0A & (1<<UDRE0)) ); // Wait for empty transmit buffer
	UDR0 = data; // Put data into buffer, sends the data
}

unsigned char UART0_RX(void){
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

void UART0_putstring(char* StringPtr){
	
	while(*StringPtr != 0x00){  //Here we check if there is still more chars to send, this is done checking the actual char and see if it is different from the null char
	UART0_TX(*StringPtr);		//Using the simple send function we send one char at a time
	StringPtr++;}				//We increment the pointer so we can read the next char
}

void LCD_Init(void)
{
	UBRR0H = 0;
	UBRR0L = 12;			//1 MHz with with 2x division on U2X0
	UCSR0A |= 0b00000010;	//Enable 2x transmit speed
	UCSR0B = 0b00001000;	//Enable TX output
	UCSR0C = 0b00000110;	//Async, no parity, 1 stop, 8 bit mode
}

void LCD_Home(void) //Sends LCD cursor home************************************
{
	UART0_TX(0xFE); //Control Character
	UART0_TX(0x80); //Position 0
}

void LCD_Line2(void) //Sends LCD cursor to line 2******************************
{
	UART0_TX(0xFE); //Control Character 
	UART0_TX(0xC0);  //Position 64 on the LCD (line 2)
}

void LCD_Clear(void) //Clears LCD**********************************************
{
	UART0_TX(0xFE); //Control Character 
	UART0_TX(0x01); //Clear display
}

void LCD_SetSplash(void) //Sets Splash Screen**********************************
{
	UART0_TX(0x7C); //Control Character
	UART0_TX(0x0A); //Set splash
}

void LCD_LightOff(void) //Turns Off Backlight 0%*******************************
{
	UART0_TX(0x7C); //Control Character
	UART0_TX(0x80); //Backlight off
}

void LCD_LightOn(void) //Turns On Backlight 100%*******************************
{
	UART0_TX(0x7C); //Control Character
	UART0_TX(0x9D); //Backlight on
}

void Power_Off(void) //Turn off System
{
	PORTB |= 0b00000001;
}

int main(void)
{
	/*********************************************************************************
	***************************************MAIN***************************************
	*********************************************************************************/
	DDRC = 0b00100100; //Data direction register setup 0=Input, 1=Output
	DDRB = 0b11000111; //Data direction register setup 0=Input, 1=Output
	DDRD = 0b11111111; //Data direction register setup 0=Input, 1=Output
	//SETTING PORT C DDR SEEMED TO INTERFERE WITH THE ADC FUNCTIONALITY
	_delay_ms(5);
	PORTD = 0b00000000; //Initialize port to 0, 
	PORTB = 0b00000000; //Initialize port to 0
	PORTC = 0b00000000; //Initialize port to 0, 
	
	_delay_ms(5);
	PMW_Init();
	ADC_Init();
	LCD_Init();
	_delay_ms(5);
	LCD_Clear();
	LCD_Home();
	
	int x = 0; //32768 is 15 bits - 2's comp makes anything over over 32k negative 65536=0
	float y = 0;
	int speed = 0;
	char snum[5];
	char snum2[5]; 
	int data = 0;
	int bit = 0;
	int i;
	int j;
	int state = 0; 
	int timeout = 0;

		
	UART0_putstring("FLOW DIAG TEST - VISHAL");    //Pass the string to the USART_putstring function and sends it over the serial	
	_delay_ms(2000)	;
	LCD_Clear();
	LCD_Home();
	_delay_ms(100)	;
		
	while (1)
	{
		while (state == 0)
		{
			
		
		if (day_read(ADC_PIN) < 10)
		{
			LCD_Line2();
			UART0_putstring("State 1 Night");
			_delay_ms(2000);
			timeout = 0;
			state = 1;
			
		}
		else 
		{
			timeout++; 
			if (timeout > 20)
			{
				Power_Off(); //System timeout and shut down
				LCD_Home();
				UART0_putstring("Power Off      ");
				_delay_ms(2000);
			}
			Disable_SwordLED();
			float ambient = sword_read(NULL);
			Enable_SwordLED();
			float netlight = sword_read(NULL);
			Disable_SwordLED();
			float ratio = ambient/netlight; 
			LCD_Line2();
			itoa(ratio, snum, 10);
			UART0_putstring(snum);
			UART0_putstring(" ratio");
			if (ratio < 0.8) // if less than, no blade is detected
			{
				LCD_Home();
				UART0_putstring("Open Clamp   ");
				_delay_ms(2000);
				// system physically opens clamp. Place function here
				
				Disable_SwordLED();
				ambient = sword_read(NULL);
				Enable_SwordLED();
				netlight = sword_read(NULL);
				Disable_SwordLED();
				float ratio = ambient/netlight;
				if (ratio < 0.8) // if less than, no blade is detected
				{
					LCD_Home();
					UART0_putstring("Not Inserted   ");
					_delay_ms(1000);
				}
				else
				{
					LCD_Home();
					UART0_putstring("Close Clamp   ");
					_delay_ms(2000);
					// system physically closes clamp. Place function here
				}
				
				
			}
			else 
			{
				LCD_Home();
				UART0_putstring("Blade present   ");
				_delay_ms(1000);
			}
		}
		}
		while (state == 1)
		{
			LCD_Line2();
			UART0_putstring("State 1            ");
			_delay_ms(1000);
		}
}
}


