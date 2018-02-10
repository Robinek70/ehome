#define MY_RADIO_NRF24
//#define MY_NODE_ID		 9
#define MY_RF24_PA_LEVEL	RF24_PA_HIGH
//#define MY_REPEATER_FEATURE 
#define MY_BAUD_RATE		57600
#define MY_SPECIAL_DEBUG


//#define MY_SIGNING_SOFT
//#define MY_SIGNING_SOFT_RANDOMSEED_PIN A6
//#define MY_SIGNING_REQUEST_SIGNATURES

#include <avr/pgmspace.h>
//#include <avr/wdt.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <MySensors.h> 


#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)

#define NODE_NAME		"Dimm. Light Switch"
#define NODE_VER		"3.08" STR(MY_RF24_PA_LEVEL)
//MY_RF24_PA_LEVEL

#define CHILD_ID_LIGHT_SENSOR	0
#define CHILD_ID_DIMMER			10
#define CHILD_ID_TRIP			20
#define CHILD_ID_TEMP			30
#define CHILD_ID_HUM			40

#define MIN_VALUE   0
#define MAX_VALUE   100

#define SWITCH_OFF	0
#define SWITCH_ON	1

#define ONE_WIRE_BUS A5	//A5
#define MAX_ATTACHED_DS18B20 4

#define TRIP_PIN1	A1
#define TRIP_PIN2	A2

#define TIMER2_START TCCR2B |= (1<<CS21);// | (1<<CS20); 
#define TIMER2_STOP TCCR2B &= ~((1<<CS22) |(1<<CS21) | (1<<CS20)); 

#define INT_NO	0

//F_CPU == 16000000
// 8MHz
#define COUNTDOWN_TIMER  255-100+1

// 16MHz
//#define COUNTDOWN_TIMER 255 - 200 + 1

//#define L1  (1<<PC3)
//#define L2  (1<<PC4)

#define CFG_DIMMER_ON				0x01
//#define CFG_SWITCH_STORE_LEVEL			0x02
//#define CFG_SWITCH_ALWAYS_FULL			0x04
//#define CFG_HUMIDITY				0x08
#define CFG_PIR1					0x10
#define CFG_PIR2					0x20
//#define CFG_P2_AS_DS				0x40
#define CFG_LUX						0x80

#define EE_CONFIG		0					// 1 byte
#define EE_SWITCHES		EE_CONFIG + 1		// 1 byte
#define EE_PIR_DELAY	EE_SWITCHES + 1		// 2 bytes / int
#define EE_DTT			EE_PIR_DELAY + 2	// 1 byte
#define EE_FADE			EE_DTT + 1			// 1 byte
#define EE_MINWAVES		EE_FADE + 1			// 1 byte
#define EE_PING_UNIT	EE_MINWAVES + 1		// 1 byte
#define EE_PING_TIME	EE_PING_UNIT + 1	// 1 byte

#define EE_SW_DATA		100

#define EE_SW_SAVED		0
#define EE_SW_INVERT	1
#define EE_SW_DATA_SIZE	5

struct SAVEDDATA {
	byte savedLevel;
	byte invertSwitch;
	byte reserverd1;
	byte reserverd2;
	byte reserverd3;
};

struct LIGHTHW {
	volatile uint8_t* rejestr;
	volatile uint8_t* port;
	byte  pin;

	//volatile uint8_t* line_rejestr;
	volatile uint8_t* line_port;
	byte line_pin;

	byte savedLevel;
	byte invertSwitch;

	byte current;
	byte targetLevel;
	

	byte lineState;
	byte prevLineState;	

	byte switchPosition;	// 1 - on; 0 - off
	byte switchStatus;	
	

	byte prevImpState;
	byte prevImpStateCounter;
	byte currentImpState;

	SAVEDDATA data;
};

#define LIGHT_PORT	PORTC

LIGHTHW /*PROGMEM*/ lights_hw[] = {
  {&DDRC, &PORTC, (1<<PC3), /*&DDRD,*/ &PIND, (1<<PD4), 100, 0, 100, 100},
  {&DDRC, &PORTC, (1<<PC4), /*&DDRD,*/ &PIND, (1<<PD3), 100, 0, 100, 100},
};

#define MAXLIGHTS (sizeof(lights_hw)/sizeof(LIGHTHW))

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float lastTemperature[MAX_ATTACHED_DS18B20];
float avgTemp[MAX_ATTACHED_DS18B20];
byte prevSkippedTemp[MAX_ATTACHED_DS18B20];
byte numSensors=0;
boolean metric = true;
byte dtt = 4;		//EE
byte minWaves = 20;		//EE
volatile byte ms10_flag  = 0;
volatile byte ms10_cnt  = 0;
volatile byte s1_cnt = 0;
volatile byte s5_cnt = 0;
int32_t luxValue = 0;
int prevLuxValue = 0;
byte dtLux = 50;

byte MaxLights = 1;	// EE
byte setByRemote = 0;
int8_t temp_mode = 0;
float lastTempSend = 0;
uint32_t lastPing = 0;
byte pingIntervalValue = 5;
byte pingIntervalUnit = 'M';

//PIR
uint32_t lastSendPir1 = 0;
uint32_t lastSendPir2 = 0;
byte prev_trip1;
byte prev_trip2;
byte fadeSpeed = 2;	// EE
int pirDelay = 0;	// EE
byte cfg;			// EE

#define isLIGHTFULL		(!(cfg & CFG_DIMMER_ON))
#define isLUX			(cfg & CFG_LUX)
#define isPIR1			(cfg & CFG_PIR1)
#define isPIR2			(cfg & CFG_PIR2)

MyMessage luxMsg(CHILD_ID_LIGHT_SENSOR, V_LIGHT_LEVEL);
MyMessage switchMsg(CHILD_ID_TEMP, V_LIGHT);
MyMessage tempMsg(CHILD_ID_TEMP, V_TEMP);
MyMessage pirMsg(CHILD_ID_TRIP, V_TRIPPED);
MyMessage var2Msg(CHILD_ID_DIMMER, V_VAR2);


void myresend(MyMessage &msg, byte repeats = 5)
{
	byte repeat = 1;
	boolean sendOK = false;

	while ((sendOK == false) && (repeat < repeats)) {
		sendOK = send(msg);
		if(!sendOK) {
			Serial.print(50*(1<<repeat));
			Serial.print(", try again: ");
			Serial.println(repeat);			
			
			repeat++; 
		} else {
			//pingErrors=0;
			break;
		}	
		wait(20*(1<<repeat));
	}
	if(!sendOK){
		//pingErrors++;
	}	
}

void EEPROMWriteInt(int p_address, int p_value)
     {
     byte lowByte = ((p_value >> 0) & 0xFF);
     byte highByte = ((p_value >> 8) & 0xFF);

	 saveState(p_address, lowByte);
     saveState(p_address + 1, highByte);
     }

unsigned int EEPROMReadInt(int p_address)
     {
     byte lowByte = loadState(p_address);
     byte highByte = loadState(p_address + 1);

     return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
     }

void EEReadInt(int pos, int* data) {
	int tmp = EEPROMReadInt(pos);
	if(tmp != -1) *data = tmp;
}

void EEReadByte(int pos, byte* data) {
	byte tmp = loadState(pos);
	if(tmp != 255) *data = tmp;
}

inline uint32_t calcTimestamp(char u, byte v) {
			uint32_t multiple = u=='S'?1:(u=='M'?60:(u=='H'?60UL*60:(u=='D'?24UL*60*60:1)));
			return multiple * v * ((u=='U')?1:1000);
}

byte tmp1, tmp2, tmp4;
int tmp3;

void before() {
	MaxLights = 1;
	pinMode(INT_NO + 2, INPUT);    //  Int0

	for( byte i=0;i<MAXLIGHTS;i++) {
		LIGHTHW* light_hw = &lights_hw[i];

		*(light_hw->rejestr) |= light_hw->pin;	// out mode
		*(light_hw->port) |= light_hw->pin;		//OFF - HIGH level

		EEReadByte(EE_SW_DATA + EE_SW_SAVED + i*EE_SW_DATA_SIZE, &(light_hw->savedLevel));
		EEReadByte(EE_SW_DATA + EE_SW_INVERT + i*EE_SW_DATA_SIZE, &(light_hw->invertSwitch));
		//light_hw->savedLevel = loadState(EE_SW_DATA + EE_SW_SAVED + i*EE_SW_DATA_SIZE);
		//light_hw->invertSwitch = loadState(EE_SW_DATA + EE_SW_INVERT + i*EE_SW_DATA_SIZE);
		light_hw->prevLineState = light_hw->lineState;
	}



	pinMode(5, OUTPUT); // test pin, SHA

	EEReadByte(EE_CONFIG, &cfg);
	EEReadByte(EE_SWITCHES, &MaxLights);
	EEReadInt(EE_PIR_DELAY, &pirDelay);
	EEReadByte(EE_DTT, &dtt);
	EEReadByte(EE_FADE, &fadeSpeed);
	EEReadByte(EE_MINWAVES, &minWaves);
	EEReadByte(EE_PING_UNIT, &pingIntervalUnit);
	EEReadByte(EE_PING_TIME, &pingIntervalValue);
	if(pingIntervalValue==0) {
		pingIntervalValue=1;
		pingIntervalUnit='H';
	}

	

	/*EEReadByte(EE_CONFIG, &tmp1);
	EEReadByte(EE_SWITCHES, &tmp2);
	EEReadInt(EE_PIR_DELAY, &tmp3);
	EEReadByte(EE_DTT, &tmp4);*/

	if(isLIGHTFULL) {
		fadeSpeed = 100;
	}

	//Serial.begin(57600);
	//Serial.println("Before...");

	sensors.begin();
	sensors.setWaitForConversion(false);
	numSensors = sensors.getDeviceCount();
	sensors.requestTemperatures();	
}

void presentation()  
{
	sendSketchInfo(NODE_NAME, NODE_VER);

	if(isLUX) {
		present(CHILD_ID_LIGHT_SENSOR, S_LIGHT_LEVEL);
	}

	if(isPIR1)
		present(CHILD_ID_TRIP, S_MOTION);

	if(isPIR2)
		present(CHILD_ID_TRIP + 1, S_MOTION);

	for(byte i=0;i<MaxLights;i++) {
		present(CHILD_ID_DIMMER + i, S_DIMMER);
	}

	for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
		present(CHILD_ID_TEMP + i, S_TEMP);
	}
	//metric = getConfig().isMetric;
	metric = getControllerConfig().isMetric;
}

void sendRequests() {
	for(byte i=0;i<MaxLights;i++) {
		  request(CHILD_ID_DIMMER + i, V_LIGHT);
	  }
}

void setup() {
  Serial.println("Setup...");

  TIMER2_STOP;

  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));  // normal overflow
  TCCR2A &= ~((1<<COM2B0) | (1<<COM2B1));

  TIMER2_START;
  TCNT2 = COUNTDOWN_TIMER;

  TIMSK2 |= (1 << TOIE2); // overflow timer2 interupt enable

  attachInterrupt (INT_NO, myint0, CHANGE);
  sei();
  
  Serial.println("Running...");

  sendRequests();
}

volatile byte count;
//volatile byte current = 100;


void myint0()
{
	TIMER2_STOP;
	TCNT2 = COUNTDOWN_TIMER;//255-200+1;
	TIMER2_START;
	count=0;
	for(byte i = 0;i<MaxLights;i++) {
		byte cL = lights_hw[i].current;
		byte tL = lights_hw[i].targetLevel;
		byte newValue = cL;
		if(cL != tL) {
			if(cL<tL) {
				newValue = cL + fadeSpeed;
				if(newValue > tL) newValue = tL;
			} else {
				//newValue = cL - fadeSpeed;//<cL?fadeSpeed:cL;
				newValue = (cL > fadeSpeed) ? (cL - fadeSpeed):MIN_VALUE;
				if(newValue < tL) newValue = tL;
				//newValue = (tL > fadeSpeed) ? (cL - fadeSpeed) : tL;
			}
		}
		byte pin = lights_hw[i].pin;
		if(newValue == MIN_VALUE) {
			LIGHT_PORT &= ~(pin); // Low - On
		} else {  // Turn On immediately if 100%
			LIGHT_PORT |= (pin); // High - Off
		}
		lights_hw[i].current = newValue;
	}

	ms10_flag = 1;

	if(++ms10_cnt>99) {  // 1s tick
		ms10_cnt=0;
		s1_cnt = 1;
	}
}

ISR(TIMER2_OVF_vect)          // timer compare interrupt service routine
{
	TCNT2 = COUNTDOWN_TIMER;//255-200+1;
	count++;

	for(byte i=0; i<MaxLights;i++){ 
		byte pin = lights_hw[i].pin;
		if(count > 40) { // don't off, required for Low Power LEDs, impuls should be longer than one tick
			LIGHT_PORT |= pin;		// High - Off
		}
		if(count > 92) // proper values are 0 - 96
			return;

		if(count == lights_hw[i].current) {  // wlacz	
			LIGHT_PORT &= ~pin;	// Low - On
		}
		
		// line detection
		if(count == 50) {
			if(PIND & (1<<PD2)) {  // first half wave
				lights_hw[i].currentImpState = *lights_hw[i].line_port & lights_hw[i].line_pin; 

				if(lights_hw[i].prevImpState == lights_hw[i].currentImpState) {
					if(lights_hw[i].prevImpStateCounter < minWaves) {
						lights_hw[i].prevImpStateCounter++;
					} else {
						lights_hw[i].lineState = lights_hw[i].currentImpState;
					}
				} else {
					lights_hw[i].prevImpStateCounter = 0;
					lights_hw[i].prevImpState = lights_hw[i].currentImpState;
				}
			}
		}
	}
}

byte started = 0;
uint32_t now;
byte tempOvertime;

void loop() {
	if(!started) {
		
		/*send(var2Msg.setSensor(CHILD_ID_DIMMER + 0).set(hwCPUFrequency()));
		send(var2Msg.setSensor(CHILD_ID_DIMMER + 0).set(hwCPUVoltage()));
		send(var2Msg.setSensor(CHILD_ID_DIMMER + 0).set("---"));*/
		reportCfg() ;
		started=1;
	}
	now = millis();
	tempOvertime = ((now - lastTempSend) > 30UL*60*1000UL);
	byte anyDimm = 0;
	for(byte i=0;i<MaxLights;i++) {
		LIGHTHW* light = &lights_hw[i];
		byte switch_status = light->switchStatus;

		// final light switch position; 1 - on; 0 - off
		light->switchPosition = switchPosition(light);

		if(light->prevLineState != light->lineState) {
			light->prevLineState = light->lineState;

			//myresend( varDimmMsg.setSensor(CHILD_ID_DIMMER + i).set(light->lineState?"line ON":"line OFF") );
			myresend( var2Msg.set(light->lineState?"line ON":"line OFF") );
		}

		if(light->switchPosition) {
			// switch PRESSED/ACTIVE
			anyDimm = (light->targetLevel != MIN_VALUE ) && (light->targetLevel != MAX_VALUE);
			if(switch_status == SWITCH_OFF) {
				switch_status = SWITCH_ON;

				if((isLIGHTFULL) || (light->targetLevel == MAX_VALUE)) {
					light->targetLevel = MIN_VALUE;	// 100% light value
				}
				else {
					light->targetLevel = light->savedLevel;
				}
				myresend(switchMsg.setSensor(CHILD_ID_DIMMER + i).set(1));
			} 
		} else {
			// switch NOT pressed/active
			if(switch_status == SWITCH_ON) {
				switch_status = SWITCH_OFF;
				
				light->savedLevel = light->targetLevel; 
				// zapisaæ w eeprom jesli wartosc sie zmienila
				if(loadState(EE_SW_DATA + EE_SW_SAVED + i*EE_SW_DATA_SIZE) != light->targetLevel /*&& !setByRemote*/){
				  
				  saveState(EE_SW_DATA + EE_SW_SAVED + i*EE_SW_DATA_SIZE,light->savedLevel);
				  //setByRemote = 0;
				  //logToConsole("Saved");
			  }

				light->targetLevel = MAX_VALUE;
				myresend(switchMsg.setSensor(CHILD_ID_DIMMER + i).set(0));
			}
		}

		light->switchStatus = switch_status;
	}	

	if(s1_cnt) {  // 1s tick
		s1_cnt = 0;
		s5_cnt++;
		if(s5_cnt >= 2) {  // 5s tick
			s5_cnt = 0;

			//send(var2Msg.setSensor(CHILD_ID_DIMMER + 0).set(anyDimm));
			//send(var2Msg.setSensor(CHILD_ID_DIMMER + 0).set(tempOvertime));

			if(numSensors > 0) {
				if(!anyDimm || tempOvertime) {
					startMeasureDS();				
				}
			}

			if(isLUX) {
				if(prevLuxValue != luxValue/100) {
					int vLux = sensorToLx(luxValue/100);
					if(vLux != sensorToLx(prevLuxValue)) {
						myresend(luxMsg.set(vLux));
						prevLuxValue = luxValue/100;
					}
				}
			}
		}  // 5s tick
	}	// 1s
	
	if(ms10_flag) {
		ms10_flag = 0;
		if(isLUX) {
			int sensor = analogRead(A0);
			luxValue = (luxValue*dtLux + sensor*100L) / (dtLux + 1);
		}
	}
	byte trip;
	if(isPIR1) {
		trip = digitalRead(TRIP_PIN1)>0?1:0;

		if(trip != prev_trip1) {
			if((trip == 1) || ((millis() - lastSendPir1) > (uint32_t)pirDelay*1000)) {
				lastSendPir1 = millis();
				myresend( var2Msg.set(trip?"move ON":"move OFF") );
				myresend(pirMsg.setSensor(CHILD_ID_TRIP).set(trip));
				prev_trip1 = trip;
			}
		} else lastSendPir1 = millis();
	}
	if(isPIR2) {
		trip = digitalRead(TRIP_PIN2)>0?1:0;
		if(trip != prev_trip2) {
			if((trip == 1) || ((millis() - lastSendPir2) > (uint32_t)pirDelay*1000)) {
				lastSendPir2 = millis();
				myresend(pirMsg.setSensor(CHILD_ID_TRIP+1).set(trip));
				prev_trip2 = trip;
			}
		} else lastSendPir2 = millis();
	}

	
	if(millis() > lastPing + calcTimestamp((char)pingIntervalUnit, pingIntervalValue) /*60UL*1000*pingInterval*/ /*PING_INTERVAL*/) {
	  //request(CHILD_ID_DIMMER, V_LIGHT);
	  //_sendRoute(MyMessage &message)
	  lastPing = millis();
	  //if(!_sendRoute(build(_msgTmp, _nc.nodeId, 0, CHILD_ID_DIMMER, C_REQ, V_LIGHT, false).set(""))) {
		//pingErrors++;
		//lastPing -= PINGERROR_INTERVAL + 10*1000;
	  //}
	  
	 // if(pingErrors>MAX_PING_ERRORS) {
	//	  Serial.println("No Pings. Reboot.");
	//	  while(1);	// watchdog
	 // }
	  
	  
	  /*for(byte i=0;i<MaxLights;i++) {
		  request(CHILD_ID_DIMMER + i, V_LIGHT);
	  }*/
	  sendRequests();
    }
}

byte switchPosition(struct LIGHTHW* light) {

	/*if(light->prevImpState == light->currentImpState) {
		if(light->prevImpStateCounter < minWaves) {
			light->prevImpStateCounter++;
		} else {
			light->lineState = light->currentImpState; 
		}
	} else {
		light->prevImpStateCounter = 0;
		light->prevImpState = light->currentImpState;
	}*/

	if(light->invertSwitch) {
		return light->lineState?0:1;
	}

	return light->lineState?1:0;	  
}

boolean switchTo(byte idx, byte onOff) {
	  if(onOff == lights_hw[idx].switchPosition){
		return false;
	  }

	  lights_hw[idx].invertSwitch = !lights_hw[idx].invertSwitch;

	  storeSwitchPosition(idx);

	  return true;
}

void storeSwitchPosition(byte idx) {
	saveState(EE_SW_DATA + EE_SW_INVERT + idx*EE_SW_DATA_SIZE,lights_hw[idx].invertSwitch);
}

/*const int v2Lx[]  = {  1024, 0,
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
*/
#define L_a_	-1.6477
#define L_b_	7.873

#define RP_LDR_V_REF	4.	// reference V
#define RP_LDR_VCC		4.	// Vcc
#define RP_LDR_R1		80.*1000	// R1 - PullUp value

#define RP_LDR_R(_adc_)	(RP_LDR_R1/(RP_LDR_VCC/(adc *(RP_LDR_V_REF/1023.)) - 1))

inline int sensorToLx(int adc) {
	int lightLevel = pow(10,L_a_*log10(RP_LDR_R(adc))+L_b_);
	return lightLevel;
}

void startMeasureDS() { 
	if(temp_mode == -1) {
		sensors.requestTemperatures();
		temp_mode = 0;
		return;
	}

	if(temp_mode >= numSensors) {
		temp_mode = -1;
		
		return;
	}

	int8_t i = temp_mode;
	//for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {

	// Fetch and round temperature to one decimal
	float temperature = static_cast<float>(static_cast<int>((metric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;		

	// Only send data if temperature has changed and no error
	if (temperature != -127.00 && temperature != 85.00) {

		if(avgTemp[i]==0) avgTemp[i]=temperature;
		avgTemp[i] = (avgTemp[i]*dtt + temperature)/(dtt + 1);
		//if((int)(avgTemp[i]*10) != (int)(lastTemperature[i]*10)) {
		if(round(avgTemp[i]*10) != round(lastTemperature[i]*10) || tempOvertime) {

			if(prevSkippedTemp[i]>2 || (abs(lastTemperature[i] - avgTemp[i])>0.19) || tempOvertime) {
				// Send in the new temperature
				myresend(tempMsg.setSensor(CHILD_ID_TEMP+i).set(avgTemp[i],1));
				if(temp_mode == numSensors - 1) {
					// update send time after last sensor 
					lastTempSend = now;
				}
				// Save new temperatures for next compare
				lastTemperature[i] = avgTemp[i];
				prevSkippedTemp[i]=0;
			} else {
				prevSkippedTemp[i]++;
			}
		}
	}
	//}
	// Fetch temperatures from Dallas sensors
	//sensors.requestTemperatures();
	temp_mode++;
}

//#define FS(x) (__FlashStringHelper*)(x)
const char cCfg[] PROGMEM  =  {"Cfg:"};
const char cPirDelay[] PROGMEM  =  {"PIR Delay (d):"};
const char cdtt[] PROGMEM  =  {"dtT (t):"};
const char cLights[] PROGMEM  =  {"Switches (s):"};
const char cTempSens[] PROGMEM  =  {"Temps:"};
const char cDimmer[] PROGMEM  =  {"Dimmer (0):"};
const char cPir1[] PROGMEM  =  {"PIR1 (4):"};
const char cPir2[] PROGMEM  =  {"PIR2 (5):"};
const char cLux[] PROGMEM  =  {"Lux (7):"};
const char cFade[] PROGMEM  =  {"Fade (f):"};
const char cMinWaves[] PROGMEM  =  {"MinWaves (w):"};
const char cSaved[] PROGMEM  =  {"Saved:"};
const char cInverted[] PROGMEM  =  {"Inverted:"};
const char cPing[] PROGMEM  =  {"PING:"};

char buffer[20];

#define onoff(x) ((char*)(((x)>0)?"On":"Off"))

void reportCfg() {
	//cfg=128+16+1;
	myF(cCfg, cfg);
	byte len = strlen(buffer);
	buffer[len++]='b';
	itoa(cfg,buffer+len,2);
	myresend(var2Msg.set(buffer));

	myresend( var2Msg.set(myF(cDimmer,onoff(cfg & CFG_DIMMER_ON)) ));
	myresend( var2Msg.set(myF(cPir1,onoff(cfg & CFG_PIR1)) ));
	myresend( var2Msg.set(myF(cPir2,onoff(cfg & CFG_PIR2)) ));
	myresend( var2Msg.set(myF(cLux,onoff(cfg & CFG_LUX)) ));

	
	myresend(var2Msg.set(myF(cPirDelay, pirDelay)));
	myresend(var2Msg.set(myF(cdtt, dtt)));
	myresend(var2Msg.set(myF(cLights, MaxLights)));
	myresend(var2Msg.set(myF(cFade, fadeSpeed)));
	myresend(var2Msg.set(myF(cMinWaves, minWaves)));
	myresend(var2Msg.set(myF(cTempSens, numSensors)));
	myF(cPing, (char)pingIntervalUnit);
	itoa(pingIntervalValue,buffer+strlen(buffer),10);
	
	//sprintf(buffer, "PING:%c%d", (char)pingIntervalUnit, pingIntervalValue);
	myresend(var2Msg.set(buffer));

	myresend(var2Msg.set(myF(cTempSens, numSensors)));

	for( byte i=0;i<MaxLights;i++) {
		LIGHTHW* light_hw = &lights_hw[i];
		myresend(var2Msg.setSensor(CHILD_ID_DIMMER + i).set(myF(cSaved, light_hw->savedLevel)));
		myresend(var2Msg.set(myF(cInverted, light_hw->invertSwitch)));
	}
}

char* myF(const char* s, char v) {
	char *p = buffer;
	for(char c; (c = pgm_read_byte(s));s++) *(p++)=c;
	*(p++) = v;
	*p = '\0';

	return buffer;
}
char* myF(const char* s, byte v) {
	char *p = buffer;
	for(char c; (c = pgm_read_byte(s));s++) *(p++)=c;
	//myF(s);
	//byte len = strlen(buffer);
	itoa(v,p,10);

	return buffer;
}
char* myF(const char* s, int v) {
	char *p = buffer;
	for(char c; (c = pgm_read_byte(s));s++) *(p++)=c;
	itoa(v,p,10);

	return buffer;
}
char* myF(const char* s, char* t) {
	char *p = buffer;
	for(char c; (c = pgm_read_byte(s));s++) *(p++)=c;
	strcpy(p,t);

	return buffer;
}
void (*softReset) (void) = 0;

byte mystrncmp(const char* flash, const char * s, byte count) {
	byte i=0;
	for(char c; (c = pgm_read_byte(flash)) && i<count;flash++,s++,i++) {
		if(c!=*s) {
			return 0;
		}
	}
	return i==count;
}

void receive(const MyMessage &message) {

	byte idx  = message.sensor - CHILD_ID_DIMMER;
	if (message.type == V_DIMMER) {
		int requestedLevel = atoi( message.data );

		//requestedLevel *= ( message.type == V_LIGHT ? 100 : 1 );

		byte onOff = requestedLevel > 0 ? 1 : 0;	// todo:  wysylac tylko gdy zmiana	  

		Serial.print( "%=" );
		Serial.print( requestedLevel );
		requestedLevel = map(requestedLevel, 100, 0, MIN_VALUE, MAX_VALUE);
		Serial.print( "Changing level to " );
		Serial.print( requestedLevel );
		Serial.print( ", from " ); 

		Serial.println( lights_hw[idx].current);
		myresend( var2Msg.setSensor(message.sensor).set(onOff?"dimmer ON":"dimmer OFF") );
		switchTo(idx, onOff);

		lights_hw[idx].targetLevel = requestedLevel;	  
		lights_hw[idx].savedLevel = requestedLevel;	  
	}
	if (message.type == V_LIGHT) {
		int onOff = atoi( message.data );
		myresend( var2Msg.setSensor(message.sensor).set(onOff?"light ON":"light OFF") );
		if(!switchTo(idx, onOff)){
			return;
		}
	}
	if (message.type == V_VAR1 || message.type == V_CUSTOM) {
	  if(strlen(message.data) == 0) {
		  /// EMPTY
		  reportCfg();
		  return;
	  }
	  const char* data = message.data;
	  const char* s = &message.data[0];
	  int d1 = atoi( &data[1] );
	  
	  if(*s == 'b') {		  
		  s++;
		  cfg = 0;
		  for (; *s!='\0'; s++)
		  { 
			  cfg *= 2; 
			  if (*s == '1') cfg++;
		  }
		  saveState(EE_CONFIG, cfg);
	  } else if( *s== 'p') {
		  hwWriteConfig(EEPROM_PARENT_NODE_ID_ADDRESS, d1);
	  } else if( *s== 'i') {
		  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, d1);;
	  } else if( *s== 'd') {
		  pirDelay = d1;
		  EEPROMWriteInt(EE_PIR_DELAY, pirDelay);
	  } else if( *s== 't') {
		  dtt = d1;
		  saveState(EE_DTT, dtt);
	  } else if( *s== 's') {
		  MaxLights = d1;
		  saveState(EE_SWITCHES, MaxLights);
	  } else if( *s== 'f') {
		  fadeSpeed = d1;
		  saveState(EE_FADE, fadeSpeed);
	  } else if( *s== 'w') {
		  minWaves = d1;
		  saveState(EE_MINWAVES, minWaves);
	  } else if( mystrncmp(cPing, s,5)) {
			pingIntervalUnit = data[5];
			pingIntervalValue = atoi(&data[6]);
			saveState(EE_PING_UNIT, pingIntervalUnit);
			saveState(EE_PING_TIME, pingIntervalValue);
	  }	
	  else if ((*s>='0') && (*s<='9')) {
		  cfg = atoi( message.data );
		  saveState(EE_CONFIG, cfg);
	  }
	  //hwReboot();

	  softReset();
	} 
	if (message.type == V_VAR2) {
		int v = atoi( message.data );
		lights_hw[0].targetLevel = v;
		//lights_hw[0].current = v;
	} 
}

