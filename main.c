



//Transmitter sample code
#define  RF_OWN_ADDR  0x11 
#include <avr/io.h>
#include <util/delay.h>
#include "nRF24L01P.h"

int main(void){

CLKPR = 0x80;CLKPR = 0x00;

RF_START(2);

while(1){
	
		   uint8_t arry[26]="ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		   if(RF_TX(arry,cnt,__CONFIG_DATA_TYPE_GD_gv|__CONFIG_ACK_bm,0x12,5)){
		     PORTD&=~(1<<2);
		     _delay_ms(5);
		     PORTD|=(1<<2);
		   }
		   cnt++;
		   if(cnt>20){cnt=1;}
		   
		   _delay_ms(1000);
		   
        }
}




