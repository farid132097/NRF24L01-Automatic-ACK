

/*

+-----------------------------------------------------------------------------------------------------------------+
|    PIN Configurations:                                                                                          |
+-----------------------------------------------------------------------------------------------------------------+
NRF SCK  -> ATmega328P PB5 (Hardware SPI pin, can't be changed)
NRF MISO -> ATmega328P PB4 (Hardware SPI pin, can't be changed)
NRF MOSI -> ATmega328P PB3 (Hardware SPI pin, can't be changed)
NRF CSN  -> ATmega328P PC1 (Configureable, see below definitions)
NRF CE   -> ATmega328P PC0 (Configureable, see below definitions)



+-----------------------------------------------------------------------------------------------------------------+
|    RF FUNCTIONS USED:                                                                                           |
+-----------------------------------------------------------------------------------------------------------------+
void    RF_START(set_rf_channel) 
void    RF_SLEEP(void)
void    RF_WAKE_UP(void)
uint8_t RF_TX_BASIC(tx_buf, set_tx_data_len, set_auto_ack_req, set_rx_addr)
uint8_t RF_RX_BASIC(rx_buf, get_rx_data_len, get_auto_ack_req, rx_timeout, rx_buf_flush)
uint8_t RF_TX(tx_buf, set_tx_data_len, set_rx_addr, set_auto_ack_req, tx_retry)
uint8_t RF_RX(rx_buf, get_rx_data_len)



+-----------------------------------------------------------------------------------------------------------------+
|    Function  Prototypes Used:                                                                                   |
+-----------------------------------------------------------------------------------------------------------------+
|  type   |       name         |         comment                                                                  |
+-----------------------------------------------------------------------------------------------------------------+
uint8_t      set_rf_channel       single byte (max value 127)
uint8_t      tx_buf               uint8_t type array (must be 32 bytes long)
uint8_t      rx_buf               uint8_t type array (must be 32 bytes long)
uint8_t      set_tx_data_len      single byte (max value 26)
*uint8_t     get_rx_data_len      8 bit pointer (use &variable)
uint8_t      set_auto_ack_req     boolean input (set 0 to disable / set 1 to enable auto acknowledgement)
*uint8_t     get_auto_ack_req     boolean output (use &variable, gets 1 when auto ack is requested else gets 0)
uint8_t      set_rx_addr          device address which will receive data; single byte (max value 255)
uint16_t     rx_timeout           set timeout in 0.1ms resolution (max value 65535)
uint16_t     rx_buf_flush         boolean input (to flush rx buffer every time before checking new data set it 1)
uint8_t      tx_retry             max retries when no acknowledgement received (max value 255)
*/



#include <avr/io.h>
#include <util/delay.h>

#define  RF_CSN_DDR        DDRC   /* Change CSN Data direction if necessary */
#define  RF_CSN_PORT       PORTC  /* Change CSN port if necessary */
#define  RF_CSN            1      /* Change CSN pin number if necessary */

#define  RF_CE_DDR         DDRC   /* Change CE Data direction if necessary */
#define  RF_CE_PORT        PORTC  /* Change CE port if necessary */
#define  RF_CE             0      /* Change CE pin number if necessary */






/*====================================================================================*/
#define  RF_SCK_DDR        DDRB   /* Do not change */
#define  RF_SCK_PORT       PORTB  /* Do not change */
#define  RF_SCK            5      /* Do not change */

#define  RF_MISO_DDR       DDRB   /* Do not change */
#define  RF_MISO_PORT      PORTB  /* Do not change */
#define  RF_MISO           4      /* Do not change */

#define  RF_MOSI_DDR       DDRB   /* Do not change */
#define  RF_MOSI_PORT      PORTB  /* Do not change */
#define  RF_MOSI           3      /* Do not change */

#define  MCU_SS_DDR        DDRB   /* Do not change */
#define  MCU_SS_PORT       PORTB  /* Do not change */
#define  MCU_SS            2      /* Do not change */

#define  RF_GENERAL_CALL   0x00   /* Do not change */
#define  RF_ACK_WAIT_MS    10     /* Do not change */
#define  RF_ACK_MULTI_FACT 7      /* Do not change */
#define  RF_REG_CHECK_US   100    /* Do not change */
#define  RF_REG_READ       0x01   /* Do not change */
#define  RF_REG_WRITE      0x00   /* Do not change */
#define  RF_MODE_PWR_DOWN  0x02   /* Do not change */
#define  RF_MODE_RX        0x01   /* Do not change */
#define  RF_MODE_TX        0x00   /* Do not change */
#define  RF_ACK_REQ        0x01   /* Do not change */
#define  RF_NO_ACK_REQ     0x00   /* Do not change */
#define  CRC_LSBYTE_POS    31     /* Do not change */
#define  CRC_MSBYTE_POS    30     /* Do not change */
#define  LEN_BYTE_POS      29     /* Do not change */
#define  RX_ADDR_BYTE_POS  28     /* Do not change */
#define  OWN_ADDR_BYTE_POS 27     /* Do not change */
#define  CONFIG_BYTE_POS   26     /* Do not change */



#define  RF_ACK_TMOUT_RX   RF_ACK_WAIT_MS*RF_ACK_MULTI_FACT
#define  RF_CSN_LOW()      RF_CSN_PORT&=~(1<<RF_CSN)
#define  RF_CSN_HIGH()     RF_CSN_PORT|=(1<<RF_CSN)
#define  RF_CE_LOW()       RF_CE_PORT&=~(1<<RF_CE)
#define  RF_CE_HIGH()      RF_CE_PORT|=(1<<RF_CE)

#define  RF_Enable()       RF_SCK_DDR|=(1<<RF_SCK);RF_MISO_DDR&=~(1<<RF_MISO);\
                           RF_MOSI_DDR|=(1<<RF_MOSI);MCU_SS_DDR|=(1<<MCU_SS);\
                           RF_CSN_DDR|=(1<<RF_CSN);RF_CE_DDR|=(1<<RF_CE);\
			               RF_CSN_PORT|=(1<<RF_CSN);RF_CE_PORT&=~(1<<RF_CE);\
			               SPCR=(1<<SPE)|(1<<MSTR);SPSR=(1<<SPI2X);

#define  RF_Disable()      SPCR=0x00;RF_SCK_DDR|=(1<<RF_SCK);\
                           RF_MISO_DDR|=(1<<RF_MISO);RF_MOSI_DDR|=(1<<RF_MOSI);\
			               MCU_SS_DDR|=(1<<MCU_SS);RF_CSN_DDR|=(1<<RF_CSN);\
			               RF_CE_DDR|=(1<<RF_CE);\
			               RF_SCK_PORT&=~(1<<RF_SCK);RF_MISO_PORT&=~(1<<RF_MISO);\
			               RF_MOSI_PORT&=~(1<<RF_MOSI);MCU_SS_PORT&=~(1<<MCU_SS);\
                           RF_CSN_PORT&=~(1<<RF_CSN);RF_CE_PORT&=~(1<<RF_CE);\
/*====================================================================================*/


typedef struct{
uint8_t tpid;
uint8_t rpid;
}rf_params;

rf_params rf;


uint8_t SPI_TRX(uint8_t data){
SPDR = data;
uint16_t ticks=0;
while(!(SPSR & (1 << SPIF))){_delay_us(1);ticks++;if(ticks>5000){break;}}
return SPDR;
}


uint16_t CRC16(uint16_t crc, uint8_t data){
crc=crc^((uint16_t)data<<8);
for(uint8_t i=0;i<8;i++){
  if(crc & 0x8000){crc=(crc<<1)^0x1021;}
  else{crc<<=1;}
  }
return crc;
}

uint16_t CRC_CHECK(uint8_t *buf, uint8_t len){
uint16_t crc=0;
for(uint8_t i=0;i<len;i++){crc=CRC16(crc,buf[i]);}
return crc;
}


void RF_RW_REG(uint8_t reg, uint8_t rw, uint8_t *data, uint8_t len){
RF_CSN_LOW();
if(rw==0){ reg|=0x20; SPI_TRX(reg);for(uint8_t i=0;i<len;i++){SPI_TRX(data[i]);}}
else     { SPI_TRX(reg);for(uint8_t i=0;i<len;i++){data[i]=SPI_TRX(0xFF);}}
RF_CSN_HIGH();
}

void RF_PWR(uint8_t state){
uint8_t buf[2];
if     (state==RF_MODE_PWR_DOWN){buf[0]=0x00;}
else if(state==RF_MODE_RX){buf[0]=0x73;RF_CE_HIGH();}
else if(state==RF_MODE_TX){buf[0]=0x72;RF_CE_LOW();}
RF_RW_REG(0x00,RF_REG_WRITE,buf,1);
}

void RF_SLEEP(void){
RF_PWR(RF_MODE_PWR_DOWN);
RF_Disable();
}

void RF_WAKE_UP(void){
RF_Enable();
RF_PWR(RF_MODE_RX);
}

void RF_START(uint8_t channel){
rf.tpid=0;
rf.rpid=0;
RF_Enable();
uint8_t rf_config[20]={0x00,0x00,0x03,0x01,0x00,channel,0x26,0x70,0x20,0x20,
                       0x00,0x00,0x00,0x00,0x00,0x00,'A','A',0xE1,0xE2};
uint8_t index=0,bytes=1,buf[5];
buf[1]='C';buf[2]='K';
for(uint8_t addr=0;addr<=0xE2;addr++){
  if     (addr==0x08){addr=0x11;}
  else if(addr==0x17){addr=0x1C;}
  else if(addr==0x1E){addr=0x0A;bytes=5;}
  else if(addr==0x0B){addr=0x10;bytes=5;}
  else if(addr==0x11){addr=0xE1;bytes=0;}
  buf[0]=rf_config[index];
  RF_RW_REG(addr,RF_REG_WRITE,buf,bytes);
  index++;
 }
RF_PWR(RF_MODE_RX);
}



uint8_t RF_TX_BASIC(uint8_t *buf, uint8_t len, uint8_t config, uint8_t rx_addr){
RF_PWR(RF_MODE_TX);
uint8_t  temp[32],temp_len=(len & 0x1F); uint16_t crc=0;
RF_RW_REG(0xE1,RF_REG_WRITE,temp,0);
for(uint8_t i=0;i<temp_len;i++){temp[i]=buf[i];}
temp[OWN_ADDR_BYTE_POS]=RF_OWN_ADDR;
temp[RX_ADDR_BYTE_POS]=rx_addr;temp[LEN_BYTE_POS]=len;
temp[CONFIG_BYTE_POS]=config;
crc=CRC_CHECK(temp,30);
temp[CRC_MSBYTE_POS]=(crc>>8);
temp[CRC_LSBYTE_POS]=crc;
RF_RW_REG(0xA0,RF_REG_WRITE,temp,32);
RF_CE_HIGH();
temp[0]=0;
while(!(temp[0]&(1<<4))){RF_RW_REG(0x17,RF_REG_READ,temp,1);_delay_us(RF_REG_CHECK_US);}
RF_CE_LOW();
return crc;
}


uint8_t RF_RX_BASIC(uint8_t *buf, uint8_t *len, uint8_t *config, uint16_t timeout, uint8_t flush){
RF_PWR(RF_MODE_RX);
uint8_t dummy[2];
if(flush){RF_RW_REG(0xE2,RF_REG_WRITE,dummy,0);}
uint16_t ticks=0,sts=0;

while(ticks<timeout){
sts=0; uint8_t temp[2]; uint16_t crc=0;
RF_RW_REG(0x17,RF_REG_READ,temp,1);
if(!(temp[0] & 0x01))
    {
       RF_RW_REG(0x61,RF_REG_READ,buf,32);
	   crc=CRC_CHECK(buf,30);
	   uint16_t calc_crc=buf[CRC_MSBYTE_POS];
	   calc_crc=calc_crc<<8;
	   calc_crc|=buf[CRC_LSBYTE_POS];
       if(crc==calc_crc){*len=(buf[LEN_BYTE_POS] & 0x1F);*config=buf[CONFIG_BYTE_POS];sts=1;break;}
    }
  _delay_us(RF_REG_CHECK_US);
  ticks++;
 }
return sts;
}


uint8_t RF_TX_BASIC_ACK(uint8_t *tbuf, uint8_t tlen, uint8_t rx_addr, uint16_t retry){
rf.tpid++;
if(rf.tpid>7){rf.tpid=0;}
uint8_t  sts=0,temp_len=0,temp_pid=(rf.tpid<<5),rbuf[32],ack=0;
uint16_t rty=0;

while(rty<retry)
 {
    RF_TX_BASIC(tbuf,tlen|temp_pid,RF_ACK_REQ,rx_addr);
    if(RF_RX_BASIC(rbuf,&temp_len,&ack,RF_ACK_TMOUT_RX,0))
     {
        if((rbuf[RX_ADDR_BYTE_POS]==RF_OWN_ADDR)&&(rf.tpid==(rbuf[LEN_BYTE_POS]>>5)))
		 {
	        sts=1;
	        break;
	     }
     }
    else{rty++;}
  }
return sts;
}

uint8_t RF_RX(uint8_t *rbuf, uint8_t *rlen, uint16_t tmout){
uint8_t sts=0,temp_len=0,temp_pid=0,tbuf[32],tlen=0,ack=0;
if(RF_RX_BASIC(rbuf,&temp_len,&ack,tmout,0))
 {
    if((rbuf[RX_ADDR_BYTE_POS]==RF_OWN_ADDR)||(rbuf[RX_ADDR_BYTE_POS]==RF_GENERAL_CALL))
	 {
        if(ack==RF_ACK_REQ)
		 {
		    _delay_us(500);
            temp_pid=(rbuf[LEN_BYTE_POS]>>5);
	        RF_TX_BASIC(tbuf,tlen|(temp_pid<<5),RF_NO_ACK_REQ,rbuf[OWN_ADDR_BYTE_POS]);
		    *rlen=temp_len;
	        if(temp_pid!=rf.rpid){sts=1;}
	        rf.rpid=temp_pid;
		  }
		else if(ack==RF_NO_ACK_REQ){*rlen=temp_len; sts=1;}
	 }
 }
return sts;
}

uint8_t RF_TX(uint8_t *tbuf, uint8_t tlen, uint8_t rx_addr,uint8_t ack_req, uint16_t retry){
uint8_t sts=0;
if(ack_req==RF_ACK_REQ)        { sts=RF_TX_BASIC_ACK(tbuf,tlen, rx_addr, retry);}
else if(ack_req==RF_NO_ACK_REQ){ for(uint16_t i=0;i<retry;i++){RF_TX_BASIC(tbuf,tlen,RF_NO_ACK_REQ,rx_addr);}}
return sts;
}
