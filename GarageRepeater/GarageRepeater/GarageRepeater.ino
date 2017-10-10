#define MY_RADIO_NRF24
#define MY_NODE_ID			7
#define MY_RF24_PA_LEVEL	RF24_PA_MAX
#define MY_BAUD_RATE		115200
#define MY_REPEATER_FEATURE 
#define MY_SPECIAL_DEBUG

#include <avr/pgmspace.h>
//#include <avr/wdt.h>
#include <SPI.h>
//#include <TimerOne.h>

#include <myRPlibs.h>
#include <myRPLDR.h>
#include <myRPDS18b20.h>
#include <myRPPIR.h>
#include <myRPContact.h>
#include <MySensors.h> 

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)

#define NODE_NAME		"Garage Repeater"
#define NODE_VER		"1.00" STR(MY_RF24_PA_LEVEL)

//region Hardware

#define ONE_WIRE_BUS	A5
#define PIN_PIR			A4	//2
#define PIN_DOOR		A3	//3
#define PIN_LUX			A1

//endregion Hardware

//bool tick1s = false;
//bool tick10ms = false;

void before()
{
	new RpPir(PIN_PIR);
	new RpDs18b20(ONE_WIRE_BUS);
	new RpLdr(PIN_LUX);
	new RpContact(PIN_DOOR);

	rp_before();
}

/*volatile byte ms10Count = 0;
void TenMsSecTick() {
	
	tick10ms = true;

	ms10Count++;
	if(ms10Count == 100) {
		ms10Count = 0;
		tick1s = true;
	}
}*/

void presentation()  
{
	sendSketchInfo(NODE_NAME, NODE_VER);
	
	rp_presentation();
}

void setup()
{
	//Timer1.initialize();
	//Timer1.attachInterrupt(TenMsSecTick, 10*1000L);
}

void loop()
{
	rp_loop();

	/*if(tick10ms) {
		tick10ms = false;
	}

	if(tick1s) {
		
		tick1s=false;
	}*/

	rp_loop_end();
}

void receive(const MyMessage &message) {
	rp_receive(message);
}
