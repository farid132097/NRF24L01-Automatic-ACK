

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


/*=======================Pin Definitions (Configerable)=================================*/

#define  __RF_CSN_DDR              DDRC   
#define  __RF_CSN_PORT             PORTC  
#define  __RF_CSN                  1      

#define  __RF_CE_DDR               DDRC  
#define  __RF_CE_PORT              PORTC 
#define  __RF_CE                   0


/*====================Pin Definitions (Can't be changed)=================================*/
#define  __RF_SCK_DDR              DDRB   
#define  __RF_SCK_PORT             PORTB 
#define  __RF_SCK                  5      

#define  __RF_MISO_DDR             DDRB   
#define  __RF_MISO_PORT            PORTB 
#define  __RF_MISO                 4     

#define  __RF_MOSI_DDR             DDRB   
#define  __RF_MOSI_PORT            PORTB  
#define  __RF_MOSI                 3      

#define  __MCU_SS_DDR              DDRB  
#define  __MCU_SS_PORT             PORTB  
#define  __MCU_SS                  2     


/*====================Don't Change Anything to the Below Section=================================*/
#define  __RF_GENERAL_CALL         0x00   
#define  __RF_ACK_WAIT_MS          10    
#define  __RF_ACK_MULTI_FACT       7      
#define  __RF_REG_CHECK_US         100   

#define  __RF_REG_READ             0x01  
#define  __RF_REG_WRITE            0x00  

#define  __RF_MODE_PWR_DOWN        0x02 
#define  __RF_MODE_RX              0x01  
#define  __RF_MODE_TX              0x00  

#define  __CRC_LSBYTE_POS          31   
#define  __CRC_MSBYTE_POS          30     
#define  __LEN_BYTE_POS            29    
#define  __RX_ADDR_BYTE_POS        28    
#define  __OWN_ADDR_BYTE_POS       27     
#define  __CONFIG_BYTE_POS         26    

#define  __CONFIG_ACK_bp           0    /*Config ACK Bit Position*/
#define  __CONFIG_DATA_TYPE_gp     1    /*Config Data Type Group Position, 3 Bit, Starts from 1*/
#define  __CONFIG_DATA_TYPE_UGD    0x01 /*Config Data Type->Unacknowledged General Data*/
#define  __CONFIG_DATA_TYPE_AGD    0x02 /*Config Data Type->Acknowledged General Data*/
#define  __CONFIG_DATA_TYPE_ASD    0x03 /*Config Data Type->Acknowledged Sync Data*/
#define  __CONFIG_DATA_TYPE_AAD    0x04 /*Config Data Type->Acknowledged ACK Data*/
#define  __CONFIG_DATA_TYPE_ABD    0x07 /*Config Data Type->Acknowledged Boot Data*/

#define  __CONFIG_ACK_bm           (1<<__CONFIG_ACK_bp)
#define  __CONFIG_DATA_TYPE_gm     (0x07<<__CONFIG_DATA_TYPE_gp)  /*3 Bit data type space*/
#define  __CONFIG_DATA_TYPE_UGD_gv (__CONFIG_DATA_TYPE_UGD<<__CONFIG_DATA_TYPE_gp) /*Config Data Type->Unacknowledged General Data group value*/
#define  __CONFIG_DATA_TYPE_AGD_gv (__CONFIG_DATA_TYPE_AGD<<__CONFIG_DATA_TYPE_gp) /*Config Data Type->Acknowledged General Data group value*/
#define  __CONFIG_DATA_TYPE_ASD_gv (__CONFIG_DATA_TYPE_ASD<<__CONFIG_DATA_TYPE_gp) /*Config Data Type->Acknowledged Sync Data group value*/
#define  __CONFIG_DATA_TYPE_AAD_gv (__CONFIG_DATA_TYPE_AAD<<__CONFIG_DATA_TYPE_gp) /*Config Data Type->Acknowledged ACK Data group value*/
#define  __CONFIG_DATA_TYPE_ABD_gv (__CONFIG_DATA_TYPE_ABD<<__CONFIG_DATA_TYPE_gp) /*Config Data Type->Acknowledged Boot Data group value*/

#define  __RF_ACK_TMOUT_RX         __RF_ACK_WAIT_MS*__RF_ACK_MULTI_FACT
#define  __RF_CSN_LOW()            __RF_CSN_PORT&=~(1<<__RF_CSN)
#define  __RF_CSN_HIGH()           __RF_CSN_PORT|=(1<<__RF_CSN)
#define  __RF_CE_LOW()             __RF_CE_PORT&=~(1<<__RF_CE)
#define  __RF_CE_HIGH()            __RF_CE_PORT|=(1<<__RF_CE)


/*====================Don't Change Anything to the Below Section=================================*/
#define  __RF_Enable()             __RF_SCK_DDR|=(1<<__RF_SCK);     __RF_MISO_DDR&=~(1<<__RF_MISO);\
                                   __RF_MOSI_DDR|=(1<<__RF_MOSI);   __MCU_SS_DDR|=(1<<__MCU_SS);\
                                   __RF_CSN_DDR|=(1<<__RF_CSN);     __RF_CE_DDR|=(1<<__RF_CE);\
			                       __RF_CSN_PORT|=(1<<__RF_CSN);    __RF_CE_PORT&=~(1<<__RF_CE);\
			                         SPCR=(1<<SPE)|(1<<MSTR);         SPSR=(1<<SPI2X);

/*====================Don't Change Anything to the Below Section=================================*/
#define  __RF_Disable()              SPCR=0x00;                     __RF_SCK_DDR|=(1<<__RF_SCK);\
                                   __RF_MISO_DDR|=(1<<__RF_MISO);   __RF_MOSI_DDR|=(1<<__RF_MOSI);\
			                       __MCU_SS_DDR|=(1<<__MCU_SS);     __RF_CSN_DDR|=(1<<__RF_CSN);\
			                       __RF_CE_DDR|=(1<<__RF_CE);\
			                       __RF_SCK_PORT&=~(1<<__RF_SCK);   __RF_MISO_PORT&=~(1<<__RF_MISO);\
			                       __RF_MOSI_PORT&=~(1<<__RF_MOSI); __MCU_SS_PORT&=~(1<<__MCU_SS);\
                                   __RF_CSN_PORT&=~(1<<__RF_CSN);   __RF_CE_PORT&=~(1<<__RF_CE);\




/*====================Not Recommended for user to use=================================*/
typedef struct{
uint8_t __tpid;
uint8_t __rpid;
uint8_t __mode;
}rf_params;
rf_params __RF;


/*====================Not Recommended for user to use=================================*/
uint8_t __SPI_TRX(uint8_t data){
SPDR = data;
uint16_t ticks=0;
while(!(SPSR & (1 << SPIF))){_delay_us(1);ticks++;if(ticks>5000){break;}}
return SPDR;
}


/*====================Not Recommended for user to use=================================*/
uint16_t __CRC16(uint16_t crc, uint8_t data){
crc=crc^((uint16_t)data<<8);
for(uint8_t i=0;i<8;i++){
  if(crc & 0x8000){crc=(crc<<1)^0x1021;}
  else{crc<<=1;}
  }
return crc;
}


/*====================Not Recommended for user to use=================================*/
uint16_t __CRC_CHECK(uint8_t *buf, uint8_t len){
uint16_t crc=0;
for(uint8_t i=0;i<len;i++){crc=__CRC16(crc,buf[i]);}
return crc;
}


/*====================Not Recommended for user to use=================================*/
void __RF_RW_REG(uint8_t reg, uint8_t rw, uint8_t *data, uint8_t len){
__RF_CSN_LOW();
if(rw==0){ reg|=0x20; __SPI_TRX(reg);for(uint8_t i=0;i<len;i++){__SPI_TRX(data[i]);}}
else     { __SPI_TRX(reg);for(uint8_t i=0;i<len;i++){data[i]=__SPI_TRX(0xFF);}}
__RF_CSN_HIGH();
}


/*====================Not Recommended for user to use=================================*/
void __RF_PWR(uint8_t state){
uint8_t buf[2];
if     (state==__RF_MODE_PWR_DOWN){buf[0]=0x00;}
else if(state==__RF_MODE_RX){buf[0]=0x73;__RF_CE_HIGH();}
else if(state==__RF_MODE_TX){buf[0]=0x72;__RF_CE_LOW();}
__RF_RW_REG(0x00,__RF_REG_WRITE,buf,1);
}



void RF_PWR_SLEEP(void){
if(__RF.__mode!=__RF_MODE_PWR_DOWN){__RF_PWR(__RF_MODE_PWR_DOWN);}
__RF_Disable();
}



void RF_PWR_WAKE_UP(void){
__RF_Enable();
if((__RF.__mode!=__RF_MODE_RX)||(__RF.__mode!=__RF_MODE_TX)){__RF_PWR(__RF_MODE_RX);}
}



void RF_START(uint8_t channel){
__RF.__tpid=0;
__RF.__rpid=0;
__RF.__mode=__RF_MODE_RX;
__RF_Enable();
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
  __RF_RW_REG(addr,__RF_REG_WRITE,buf,bytes);
  index++;
 }
__RF_PWR(__RF_MODE_RX);
}



uint8_t RF_TX_BASIC(uint8_t *buf, uint8_t len, uint8_t config, uint8_t rx_addr){
if(__RF.__mode!=__RF_MODE_TX){__RF_PWR(__RF_MODE_TX);__RF.__mode=__RF_MODE_TX;}
uint8_t  temp[32],temp_len=(len & 0x1F); uint16_t crc=0;
__RF_RW_REG(0xE1,__RF_REG_WRITE,temp,0);
for(uint8_t i=0;i<temp_len;i++){temp[i]=buf[i];}
temp[__OWN_ADDR_BYTE_POS]=RF_OWN_ADDR;
temp[__RX_ADDR_BYTE_POS]=rx_addr;temp[__LEN_BYTE_POS]=len;
temp[__CONFIG_BYTE_POS]=config;
crc=__CRC_CHECK(temp,30);
temp[__CRC_MSBYTE_POS]=(crc>>8);
temp[__CRC_LSBYTE_POS]=crc;
__RF_RW_REG(0xA0,__RF_REG_WRITE,temp,32);
__RF_CE_HIGH();
temp[0]=0;
while(!(temp[0]&(1<<4))){__RF_RW_REG(0x17,__RF_REG_READ,temp,1);_delay_us(__RF_REG_CHECK_US);}
__RF_CE_LOW();
return crc;
}



uint8_t RF_RX_BASIC(uint8_t *buf, uint8_t *len, uint8_t *config, uint16_t timeout, uint8_t flush){
if(__RF.__mode!=__RF_MODE_RX){__RF_PWR(__RF_MODE_RX);__RF.__mode=__RF_MODE_RX;}
uint8_t dummy[2];
if(flush){__RF_RW_REG(0xE2,__RF_REG_WRITE,dummy,0);}
uint16_t ticks=0,sts=0;

while(ticks<timeout){
sts=0; uint8_t temp[2]; uint16_t crc=0;
__RF_RW_REG(0x17,__RF_REG_READ,temp,1);
if(!(temp[0] & 0x01))
    {
       __RF_RW_REG(0x61,__RF_REG_READ,buf,32);
	   crc=__CRC_CHECK(buf,30);
	   uint16_t calc_crc=buf[__CRC_MSBYTE_POS];
	   calc_crc=calc_crc<<8;
	   calc_crc|=buf[__CRC_LSBYTE_POS];
       if(crc==calc_crc){*len=(buf[__LEN_BYTE_POS] & 0x1F);*config=buf[__CONFIG_BYTE_POS];sts=1;break;}
    }
  _delay_us(__RF_REG_CHECK_US);
  ticks++;
 }
return sts;
}



uint8_t RF_TX(uint8_t *tbuf, uint8_t tlen, uint8_t config, uint8_t rx_addr, uint16_t retry){
__RF.__tpid++;
if(__RF.__tpid>7){__RF.__tpid=0;}
uint8_t  sts=0,temp_len=0,temp_pid=(__RF.__tpid<<5),rbuf[32];
uint16_t rty=0;

if(__CONFIG_ACK_bm&config)
 {
  while(rty<retry)
   {
      RF_TX_BASIC(tbuf,tlen|temp_pid,config,rx_addr);
      if(RF_RX_BASIC(rbuf,&temp_len,&config,__RF_ACK_TMOUT_RX,0))
       {
          if((rbuf[__RX_ADDR_BYTE_POS]==RF_OWN_ADDR)&&(__RF.__tpid==(rbuf[__LEN_BYTE_POS]>>5))&&((config&__CONFIG_DATA_TYPE_gm)==__CONFIG_DATA_TYPE_AAD_gv))
		   {
	          sts=1;
	          break;
	       } 
       }
      else{rty++;}
    }
 }
else
 {
  RF_TX_BASIC(tbuf,tlen|temp_pid,config,rx_addr);
 }
return sts;
}



uint8_t RF_RX(uint8_t *rbuf, uint8_t *rlen, uint16_t tmout){
uint8_t sts=0,temp_len=0,temp_pid=0,tbuf[32],tlen=2,config=0;
if(RF_RX_BASIC(rbuf,&temp_len,&config,tmout,0))
 {
    if((rbuf[__RX_ADDR_BYTE_POS]==RF_OWN_ADDR)||(rbuf[__RX_ADDR_BYTE_POS]==__RF_GENERAL_CALL))
	 {
        if((config & __CONFIG_ACK_bm)==__CONFIG_ACK_bm)
		 {
		    _delay_us(500);
            temp_pid=(rbuf[__LEN_BYTE_POS]>>5);
	        RF_TX_BASIC(tbuf,tlen|(temp_pid<<5),__CONFIG_DATA_TYPE_AAD_gv,rbuf[__OWN_ADDR_BYTE_POS]);
		    *rlen=temp_len;
	        if(temp_pid!=__RF.__rpid){sts=(config&__CONFIG_DATA_TYPE_gm)>>1;}
	        __RF.__rpid=temp_pid;
		  }
		else {*rlen=temp_len; sts=1;}
	 }
 }
return sts;
}

