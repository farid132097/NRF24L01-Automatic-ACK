



//Transmitter sample code
#define  RF_OWN_ADDR  0x11  //TX address
#include <avr/io.h>
#include <util/delay.h>
#include "nRF24L01P.h"


int main(void){

RF_START(2); //channel 

uint8_t len=1;
while(1){
           
	       uint8_t buf[32]="ABCDEFGHIJKLMNOPQRSTUVWXYZ";
           RF_TX(buf, len, 0x12, RF_ACK_REQ, 10);
           _delay_ms(100);
		   len++;
		   if(len>26){len=1;}
		   
        }
}



/*
//Reciver sample code
#define  RF_OWN_ADDR  0x12  //RX address
#include <avr/io.h>
#include <util/delay.h>
#include "nRF24L01P.h"


#define  BAUD              19200         
#define  UBRR_VAL          F_CPU/16/BAUD-1
#define  UART_START()      UBRR0H=0;UBRR0L=UBRR_VAL;UCSR0B=0x18;UCSR0C=6;

void UART_TX(uint8_t *data, uint8_t len){
for(uint8_t i=0;i<len;i++){
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0=data[i];
  while(!(UCSR0A & (1<<TXC0)));
  }
}

int main(void){

UART_START();
RF_START(2); //channel

while(1){
           
	      
	     
	       uint8_t rx_buf[32],rx_len=0;
	       if(RF_RX(rx_buf, &rx_len))      //automatically handles if ack is requested
		    {
			   UART_TX(rx_buf,rx_len);     //transmit via uart
			   UART_TX((uint8_t *)"\n",1); //set line ending if necessary
			}
	       _delay_ms(1);
		   
        }
}*/
