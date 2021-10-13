/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio
#include <EEPROM.h>
#include <IRLibRecvPCI.h>
#include <IRLibSendBase.h> //We need the base code
#include <IRLib_HashRaw.h> //Only use raw sender

IRsendRaw mySender;
IRrecvPCI MyReceiver(2);//create instance of receiver using pin 2


#define START_ADDRESS_ON_SIGNAL  0 //Starting address of On sinal in EEPROM
#define START_ADDRESS_OFF_SIGNAL 512 //Starting address of Off sinal in EEPROM

#define UART_RECV_ON    '1'
#define UART_RECV_OFF   '2'
#define UART_RECV_STOP  '3'

#define UART_SEND_ON    '4'
#define UART_SEND_OFF   '5'



extern recvGlobal_t recvGlobal;
//RECV_BUF_LENGTH = 500 defined by Global IR library
volatile uint8_t g_ui8compressedSignal[450];


//volatile uint8_t g_ui8onSignal[RECV_BUF_LENGTH];
//volatile uint8_t g_ui8offSignal[RECV_BUF_LENGTH];


volatile uint16_t g_countOn;
volatile uint16_t g_countOff;

void SaveOnSignal_IntoEEPROM(void);
void SaveOffSignal_IntoEEPROM(void);
//void RestoreFromEEPROM(void);
void RestoreOn_FromEEPROM(void);
void RestoreOff_FromEEPROM(void);

void Compress(uint8_t * p_ui8dest, uint16_t * p_countDest , uint16_t * p_ui16source, uint16_t * p_countSource );
void Extract(uint8_t * p_ui8source,uint16_t * p_countSource, uint16_t * p_ui16dest, uint16_t * p_countDest );

bool ReceiveOnSignal(void);
bool ReceiveOffSignal(void);

void OnAirConditioner(void);
void OffAirConditioner(void);

void setup() {
	Serial.begin(9600);
	MyReceiver.setFrameTimeout(15000);
	Serial.print(RECV_BUF_LENGTH);
}


/////////////////////////////////////////////////////////////////////
void loop() {
	uint8_t tem_rx_UART;
	if(Serial.available()){
		tem_rx_UART = Serial.read();
		switch (tem_rx_UART){
			case UART_RECV_ON:
			ReceiveOnSignal();
			break;
			case UART_RECV_OFF:
			ReceiveOffSignal();
			break;
			case UART_SEND_ON:
			OnAirConditioner();
			break;
			case UART_SEND_OFF:
			OffAirConditioner();
			break;
			default:
			Serial.print("Invalid UART receiving value\n");
			break;
		}
		
		//flush all receiving data by 4 below lines
		Serial.flush();
		while(Serial.available()>0){
			tem_rx_UART = Serial.read();
		}
	}
}

///////////////////////////////////////////////
void Compress(uint8_t * p_ui8dest, uint16_t * p_countDest , uint16_t * p_ui16source, uint16_t * p_countSource ){
    *p_countDest=0;
    for(uint16_t i = 1; i <= (*p_countSource) ; ++i, ++p_ui16source){ 
        if ( (*p_ui16source) > 0x0F00 ){// special value, just remain it.

            *p_ui8dest = (uint8_t) ( ((uint16_t) (((*p_ui16source) >> 4) | 0xF000)) >>8 ); //Most significant sign will always be F (hex)
            ++p_ui8dest;
            ++(*p_countDest);

            *p_ui8dest =    (uint8_t)  ((*p_ui16source) >> 4);
            ++p_ui8dest;
            ++(*p_countDest);

        }
        else{  // smaller than or equal to 0x0EFF
            *p_ui8dest = (uint8_t) ((*p_ui16source) >> 4); //Most significant sign will not be F (hex)
            ++p_ui8dest;
            ++(*p_countDest);
        }
    }
}

void Extract(uint8_t * p_ui8source,uint16_t * p_countSource, uint16_t * p_ui16dest, uint16_t * p_countDest){
    *p_countDest =0;
    for(uint16_t i = 1; i<= ((*p_countSource));){

        if (    ((*p_ui8source) & 0xF0) == 0xF0 ){ //0x Fx special value, no compressed

            *p_ui16dest = ( ((uint16_t) ((*p_ui8source) & 0x0F))  << 12);
            ++p_ui8source;
            ++i;

            *p_ui16dest |= ((((uint16_t)(*p_ui8source)) << 4) | 0x0008 );//prevent from rounding down (8= 0xF/2)
            ++p_ui16dest;
            ++p_ui8source;
            ++(*p_countDest);
            ++i;
        }
        else {
           *p_ui16dest =   (uint16_t)  ( ((*p_ui8source)<<4) | 0x0008 ); //prevent from rounding down (8= 0xF/2)
           if ((*p_ui16dest)==8){
                *p_ui16dest = 0;
           }
            ++p_ui16dest;
            ++p_ui8source;
            ++(*p_countDest) ;
            ++i;
        }

    }
}
//////////////////////////////////////////////////////////////////
bool ReceiveOnSignal(void){
	MyReceiver.enableIRIn();
	Serial.print("Start receiving on\n");
	while(1){
		if (MyReceiver.getResults()) {//wait until it returns true
			Compress((uint8_t*) &g_ui8compressedSignal[0],(uint16_t*)&g_countOn,
			(uint16_t*) &(recvGlobal.recvBuffer[1]), (uint16_t*) &(recvGlobal.recvLength));

			for(int i=0; i<=recvGlobal.recvLength;  i++){
				Serial.print(recvGlobal.recvBuffer[i]);
				Serial.print(", ");
			}

			Serial.print(recvGlobal.recvLength);

			
			
			delay(2000);
			if(g_countOn>500){
				Serial.print("\n compressed signal is too big > 500byte, can't not save it into EEPROM \n");
			}
			else {
				SaveOnSignal_IntoEEPROM();
				Serial.print("\n Receiving On signal successful \n");
				break;
			}
		}
		//Serial.print("break fault ");
		//blink
		if(Serial.available()){ //STOP
			uint8_t tem_rx_UART;
			if( (tem_rx_UART = Serial.read()) == UART_RECV_STOP) {
				break;
			}
		}
	}
	MyReceiver.disableIRIn();
	Serial.print("Stop receiving on\n");
}

bool ReceiveOffSignal(void){
	MyReceiver.enableIRIn();
	Serial.print("Start receiving off\n");
	while(1){
		if (MyReceiver.getResults()) {//wait until it returns true
			Compress((uint8_t*) g_ui8compressedSignal,(uint16_t* )&g_countOff,
			(uint16_t *) &(recvGlobal.recvBuffer[1]), (uint16_t*) &(recvGlobal.recvLength));

			for(int i=0; i<=recvGlobal.recvLength;  i++){
				Serial.print(recvGlobal.recvBuffer[i]);
				Serial.print(", ");
			}
			if(g_countOff>500){
				Serial.print("\n compressed signal is too big > 500byte, can't not save it into EEPROM \n");
			}
			else {
				SaveOffSignal_IntoEEPROM();
				Serial.print("\n Receiving Off signal successful \n");
				break;
			}
		}
		//Serial.print("break fault ");
		//blink
		if(Serial.available()){ //STOP
			uint8_t tem_rx_UART;
			if( (tem_rx_UART = Serial.read()) == UART_RECV_STOP) {
				break;
			}
		}
	}
	MyReceiver.disableIRIn();
	Serial.print("Stop receiving off\n");
}


//void OnAirConditioner(void){}
//void OffAirConditioner(void){}



void OnAirConditioner(void){
	Serial.print("On air conditioner\n");
	RestoreOn_FromEEPROM();
	Extract((uint8_t *)&g_ui8compressedSignal[0],(uint16_t *)&g_countOn,
	(uint16_t*) &(recvGlobal.recvBuffer[1]),(uint16_t *) &(recvGlobal.recvLength) );

	mySender.send((uint16_t*) &(recvGlobal.recvBuffer[1]),recvGlobal.recvLength,38);
}

void OffAirConditioner(void){
	Serial.print("Off air conditioner\n");
	RestoreOff_FromEEPROM();
	Extract((uint8_t *)&g_ui8compressedSignal[0],(uint16_t *)&g_countOff,
	(uint16_t*) &(recvGlobal.recvBuffer[1]),(uint16_t *) &(recvGlobal.recvLength) );

	Serial.print("\n");
	for(int i=0; i<=recvGlobal.recvLength;  i++){
		Serial.print(recvGlobal.recvBuffer[i]);
		Serial.print(", ");
	}

	mySender.send((uint16_t*) &(recvGlobal.recvBuffer[1]),recvGlobal.recvLength,38);
}
////////////////////////////////////////////////
void SaveOnSignal_IntoEEPROM(void){
	for(uint16_t i =0; i<g_countOn; i++){
		EEPROM[i+ START_ADDRESS_ON_SIGNAL] = g_ui8compressedSignal[i];
	}
	EEPROM[512 - 2] = (uint8_t) (g_countOn>>8);
	EEPROM[512 - 1] = (uint8_t) (g_countOn>>0);
	Serial.print("Save on signal\n");
}

void SaveOffSignal_IntoEEPROM(void){
	for(uint16_t i =0; i < g_countOff; i++){
		EEPROM[i+ START_ADDRESS_OFF_SIGNAL] = g_ui8compressedSignal[i];
	}
	EEPROM[1024 - 1] = (uint8_t) (g_countOff>>8);
	EEPROM[1024 - 2] = (uint8_t) (g_countOff>>0);
	Serial.print("Save off signal\n");
}

void RestoreOn_FromEEPROM(void){
	
	g_countOn  = (uint16_t) EEPROM[512 - 2];
	g_countOn  = (g_countOn<<8)| ((uint16_t) EEPROM[512 - 1]);
	
	for(uint16_t i =0; i<g_countOn; i++){
		g_ui8compressedSignal[i] = EEPROM[i+ START_ADDRESS_ON_SIGNAL];
	}
	Serial.print("Restore on signal from EEPROM\n");
}

void RestoreOff_FromEEPROM(void){
	
	g_countOff = (uint16_t) EEPROM[1024 - 1];
	g_countOff = (g_countOff << 8)| ((uint16_t) EEPROM[1024 - 2]);
	
	for(uint16_t i =0; i<g_countOff; i++){
		g_ui8compressedSignal[i] = EEPROM[i+ START_ADDRESS_OFF_SIGNAL];
	}
	
	Serial.print("Restore off signal from EEPROM\n");
}