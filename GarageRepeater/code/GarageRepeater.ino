#define MY_RADIO_NRF24
#define MY_NODE_ID			7
#define MY_RF24_PA_LEVEL	RF24_PA_HIGH
#define MY_BAUD_RATE		115200
#define MY_REPEATER_FEATURE 
#define MY_SPECIAL_DEBUG

#include <avr/pgmspace.h>
//#include <avr/wdt.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <TimerOne.h>
//#include <RPlibs\rpsensors.h>
#include <MySensors.h> 

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)

#define NODE_NAME		"Garage Repeater"
#define NODE_VER		"0.02" STR(MY_RF24_PA_LEVEL)


#define CHILD_ID_LIGHT_SENSOR	5
#define CHILD_ID_DOOR			20
#define CHILD_ID_PIR			21
#define CHILD_ID_TEMP			30

#define MAX_ATTACHED_DS18B20	4

//region Hardware

#define ONE_WIRE_BUS	A5
#define PIN_PIR			A4	//2
//#define PIN_PIR			2
#define PIN_DOOR		A3	//3
//#define PIN_DOOR		3
#define PIN_LUX			A1

#define EE_MAP_OFFSET	0


//endregion Hardware

// mySensors variables
bool isMetric = true;
MyMessage luxMsg(CHILD_ID_LIGHT_SENSOR, V_LIGHT_LEVEL);
MyMessage tempMsg(CHILD_ID_TEMP, V_TEMP);
MyMessage pirMsg(CHILD_ID_PIR, V_TRIPPED);
MyMessage doorMsg(CHILD_ID_DOOR, V_ARMED);

MyMessage var2Msg(CHILD_ID_PIR, V_VAR2);

// Temp variables
byte numSensors = 0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float lastTemperature[MAX_ATTACHED_DS18B20];
float avgTemp[MAX_ATTACHED_DS18B20];
uint32_t lastTempSend[MAX_ATTACHED_DS18B20];
byte prevSkippedTemp[MAX_ATTACHED_DS18B20];
byte mapTempId[MAX_ATTACHED_DS18B20];
byte dtt = 4;

// PIR
byte prev_pir;
uint32_t lastSendPir;
uint16_t pirDelay = 10;

// DOOR
byte prev_door;

// LUX
byte dtLux= 100;
uint32_t luxValue = 0;
uint32_t prevLuxValue;

#define MAX_REPEATS	5

uint32_t now;
bool tick1s = false;
bool tick10ms = false;

void myresend(MyMessage &msg /*, byte repeats = 5*/)
{
	byte repeat = 1;
	boolean sendOK = false;
	
	while ((sendOK == false) && (repeat < MAX_REPEATS)) {
		sendOK = send(msg);
		if(!sendOK) {
			repeat++; 
		} else {
			break;
		}	
		wait(20*(1<<repeat));
	}	
}

const int v2Lx[] /*PROGMEM*/   = {  1024, 0,
					  112, 21,
					  78 ,  70,
					  42 ,  150,
					  26 , 230,
					  20 , 380,
					  12 , 1200,
					  7 , 2100,
					  0, 5000
};
#define MAXLXDEF (sizeof(v2Lx)/sizeof(int)/2)

int sensorToLx(int value) {
	//int lightLevel = (2500/((value*0.0048828125)-500))/10; 
	//float a = -0.27442;
	//float b = 61.3721;
	float a = -1.6477;
	float b = 7.873;
	float v0 = value*0.0048828125;
	float v = 4.75;
	float R1 = 30*1000;

	//Serial.print(pow(10,3));
	//Serial.print(", v0: ");
	//Serial.print(v0);
	float R = R1/(v/v0 - 1);
	/*Serial.print(", R: ");
	Serial.print(R);
	Serial.print(", log10: ");
	Serial.print(a*log10(R)+b);
	Serial.print(", log10f: ");
	Serial.print(a*log10f(R)+b);
	Serial.print(", pow: ");
	Serial.println(pow(10,a*log10(R)+b));*/

	int lightLevel = pow(10,a*log10(R)+b);
	return lightLevel;
	//112 -> 21lx
			  //78 -> 70lx
			  //42 -> 160
			  //26 ->330lx
			  //20 ->680
			  //12 ->1500
			  //7 -> 2100
	//return value;
	for (byte i = 0; i < MAXLXDEF; i++)
	{
		byte idx=i*2;
		if(value >= v2Lx[idx]) {
			//displayInt = pgm_read_word_near(charSet + k);
			return map(value, v2Lx[idx-2], v2Lx[idx], v2Lx[idx-2 + 1], v2Lx[idx-2+3]);
		}
		
	}
	return 0;
}
//byte temp_mode = -1;
void startMeasureDS() { 

	for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
		bool tempOvertime = ((now - lastTempSend[i]) > 30UL*60*1000UL);
		// Fetch and round temperature to one decimal
		float temperature = static_cast<float>(static_cast<int>((isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;		

		// Only send data if temperature has changed and no error
		if (temperature != -127.00 && temperature != 85.00) {

			if(avgTemp[i]==0) {
				avgTemp[i]=temperature;
			}
			avgTemp[i] = (avgTemp[i]*dtt + temperature)/(dtt + 1);

			if(round(avgTemp[i]*10) != round(lastTemperature[i]*10) || tempOvertime) {

				if(prevSkippedTemp[i]>2 || (abs(lastTemperature[i] - avgTemp[i])>0.19) || tempOvertime) {
					// Send in the new temperature
					myresend(tempMsg.setSensor(/*CHILD_ID_TEMP+i*/ mapTempId[i]).set(avgTemp[i],1));
					//if(temp_mode == numSensors - 1) {
						// update send time after last sensor 
						lastTempSend[i] = now;
					//}
					// Save new temperatures for next compare
					lastTemperature[i] = avgTemp[i];
					prevSkippedTemp[i]=0;
				} else {
					prevSkippedTemp[i]++;
				}
			}
		}
	}
	// Fetch temperatures from Dallas sensors
	sensors.requestTemperatures();
	//temp_mode++;
}



void before()
{
	pinMode(PIN_DOOR, INPUT_PULLUP); 
	pinMode(PIN_PIR, INPUT_PULLUP); 

	digitalWrite(PIN_LUX, HIGH);	// pull up analog pin

	sensors.begin();
	sensors.setWaitForConversion(false);
	numSensors = sensors.getDeviceCount();
	sensors.requestTemperatures();	

	Serial.print("Temps: ");
	Serial.println(numSensors);

	for (int i=0; i<MAX_ATTACHED_DS18B20; i++) {      
		mapTempId[i] = CHILD_ID_TEMP+i;
		byte id = loadState(EE_MAP_OFFSET + i);
		if(id!=0 && id !=255) {
			mapTempId[i]=id;
		}
		Serial.print("Map temp sensor: ");
		Serial.print(i);
		Serial.print(" -> ");
		Serial.println(mapTempId[i]);
	}
}
//volatile int loopCount = 0;
volatile byte ms10Count = 0;
void TenMsSecTick() {
	
	tick10ms = true;

	//Serial.print("Tick: ");
	//Serial.print(now);
	//Serial.print(", loops: ");
	//Serial.println(loopCount);
	//loopCount = 0;  
	ms10Count++;
	if(ms10Count == 100) {
		ms10Count = 0;
		tick1s = true;
	}
}

void presentation()  
{
	sendSketchInfo(NODE_NAME, NODE_VER);
	
	present(CHILD_ID_LIGHT_SENSOR, S_LIGHT_LEVEL);
	
	present(CHILD_ID_DOOR, S_DOOR);
	present(CHILD_ID_PIR, S_MOTION);

	for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
		present(/*CHILD_ID_TEMP + i*/ mapTempId[i], S_TEMP);
	}

	isMetric = getControllerConfig().isMetric;
}

void setup()
{
	Timer1.initialize();
	Timer1.attachInterrupt(TenMsSecTick, 10*1000L);
}

void loop()
{
	//loopCount++;
	now = millis();	

	if(tick10ms) {
		byte trip;
		trip = digitalRead(PIN_PIR)>0?1:0;

		if(trip != prev_pir) {
			if((trip == 1) || ((millis() - lastSendPir) > (uint32_t)pirDelay*1000)) {			
				myresend(pirMsg.set(trip));
				lastSendPir = millis();
				prev_pir = trip;
			}
		} else lastSendPir = millis();

		trip = digitalRead(PIN_DOOR)>0?1:0;

		if(trip != prev_door) {
			myresend(doorMsg.set(trip));
			prev_door = trip;		
		}
		tick10ms = false;
	}

	uint32_t sensor100 = (uint32_t)(analogRead(PIN_LUX));//*100UL;
	//uint32_t sensor100 = static_cast<uint32_t>(analogRead(PIN_LUX))*100UL;
	//luxValue = (luxValue*dtLux + sensor100) / (dtLux + 1);
	luxValue = sensor100;

	if(tick1s) {

		startMeasureDS();
		int vLux;
		/*if(prevLuxValue != luxValue/100) {
			vLux = sensorToLx(luxValue/100);
			if(vLux != sensorToLx(prevLuxValue)) {
				myresend(luxMsg.set(vLux));
				prevLuxValue = luxValue/100;
			}
		}*/
		if(prevLuxValue != luxValue) {
			vLux = sensorToLx(luxValue);
			/*if(vLux != sensorToLx(prevLuxValue)) {
				myresend(luxMsg.set(vLux));
				prevLuxValue = luxValue;
			}*/
			prevLuxValue = luxValue;
		}

		tick1s=false;
		Serial.print(", Tick: ");
		
		Serial.print(now);
		Serial.print(", vLux: ");
		Serial.print(vLux);
		Serial.print(", luxValue: ");
		Serial.println(luxValue);
	}

	//wait(1000);
}

void receive(const MyMessage &message) {

	if (message.type==V_VAR2){
		//no = message.sensor - WATER_ID;
		char input[26];
		char *p;
		strcpy(input, message.data);

		p =  strtok(input , ",");
		byte sensor_no = atoi(p );  
		byte id;

		p =  strtok(NULL , ","); 
		id = atoi(p );
		if(id != 0) { 
			Serial.println("SET id"); 
			
			//mapTempId[sensor_no] = id;
			//saveState(sensor_no, id);
		} else {
			id = mapTempId[sensor_no];
		}
		Serial.print(" sensor: ");
		Serial.print(sensor_no);
		Serial.print(" -> ");
		Serial.print(id);

		p =  strtok(NULL , ","); 
		if(p != NULL) {
			Serial.print(" - ");
			Serial.print(p);
		}

		
		Serial.println();
	}
}
