#include<avr/io.h>

#include<stdio.h>

#include<util/delay.h>



void UART_init(void){

	UCSRA = 0x00;

	UCSRB = 0b00011000;

	UCSRC = 0b10110110;

	UBRRL = 0x33;

	UBRRH = 0x00;

}



void UART_send(unsigned char data){

	while((UCSRA & (1<<UDRE)) == 0x00) ;

	UDR = data;



}

unsigned char UART_receive(void){

	while((UCSRA & (1<<RXC)) == 0x00);

	return UDR;

}



void UART_Transmit(unsigned int data){

	while(!(UCSRA & (1<<UDRE)));

	UCSRB &= ~(1<<TXB8);

	if( data & 0x0100)

		UCSRB |= (1<<TXB8);



	UDR = data;

}



unsigned int UART_Receive(void){



	unsigned char status, resh, resl;



	while(!(UCSRA & (1<<RXC)));



	status = UCSRA;

	resh = UCSRB;

	resl = UDR;



	if(status & ((1<<FE)|(1<<DOR)|(1<<PE)))

		return -1;



	return ((resh<<8) | resl);

}

/*void main(void){



	unsigned char a;

	UART_init();

	stdout = fdevopen(UART_send,NULL);

	stdin = fdevopen(NULL, UART_receive);



	//while(1){

		printf("AT\r");

		_delay_ms(100);

		printf("ATD+8801521300786;\r");

		_delay_ms(2000);

		//scanf("%s",&a);

	//}

	while(1);

}*/