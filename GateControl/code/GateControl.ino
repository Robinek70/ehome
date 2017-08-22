#define MY_DEBUG
#define MY_RADIO_NRF24
//#define MY_NODE_ID			10
#define MY_NODE_ID			222
//#define RF24_PA_LEVEL 		RF24_PA_MAX
#define RF24_PA_LEVEL 		RF24_PA_MIN
#define MY_REPEATER_FEATURE

#include <MySensors.h> 
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>

//const char NODE_NAME[] = "Gate Control";
#define	NODE_NAME	"Gate Control"
//const char NODE_VER[] = "1.43.1";
#define	NODE_VER	"2.01.3"

#define GATE_INDICATOR_PIN	2
#define GATE_OPEN_PIN		4
#define DOOR_OPEN_PIN		5
#define DOOR_INDICATOR_PIN	6
#define DOOR_RING_PIN		7
#define LIGHT_PIN			8

#define TEST_LED_PIN		3

#define FOTO_PIN			A1
#define ONEWIRE_PIN			A2 

#define GATE_OPEN_ID		0
#define DOOR_OPEN_ID		1
#define DOOR_RING_ID		2
#define LIGHT_ID			3
#define GATE_STATUS_ID		10

#define FOTO_ID		5
#define TEMP_ID		20

#define DURATION_GATE_PULSE		1000	//ms
#define DELAY_GATE_STATUS		30000UL	//ms
#define MAX_ATTACHED_DS18B20	1
#define TEMP_REPORT_INTERVAL	5000	//ms

MyMessage gateOpenMsg(GATE_OPEN_ID, V_TRIPPED);
MyMessage doorOpenMsg(DOOR_OPEN_ID, V_TRIPPED);
//MyMessage gateStatusMsg(GATE_STATUS_ID, V_TRIPPED);
MyMessage doorRingMsg(DOOR_RING_ID, V_TRIPPED);
MyMessage tempMsg(TEMP_ID,V_TEMP);
MyMessage fotoMsg(FOTO_ID, V_LIGHT_LEVEL);

bool is_gate_opened;
bool is_door_openning;
bool is_door_bell;
bool prev_status;
bool prev_ring;
bool prev_door_opening;

bool open_locker;
bool report_gate;
uint32_t start_door_time;
uint32_t start_gate_time;
byte duration_door_open = 10;

uint32_t current_time;
uint32_t gate_last_send;
uint32_t temp_last_send;
byte numSensors;
byte isMetric;
float lastTemperature[MAX_ATTACHED_DS18B20];
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);
int prev_foto;
uint16_t avg = 0;
byte dt = 30;

void myresend(MyMessage &msg, int repeats = 5)
{
	int repeat = 1;
	//int repeatdelay = 0;
	boolean sendOK = false;

	while ((sendOK == false) && (repeat < repeats)) {
		sendOK = send(msg);
		if(!sendOK) {
			sendOK = false;
			Serial.print("try again: ");
			Serial.println(repeat);
			//repeatdelay += 100;
		} 
		repeat++; 
		wait(repeat>>7);
	}
}

void before() {
	sensors.begin();
	sensors.setWaitForConversion(false);
	numSensors = sensors.getDeviceCount();
	sensors.requestTemperatures();

	Serial.print("DS18B20 sensors:");
	Serial.println(numSensors);

	duration_door_open = loadState(0);
	if(duration_door_open == 0xff) duration_door_open = 8;
	log();
}

void presentation()  
{ 
	// Send the sketch version information to the gateway
	sendSketchInfo(NODE_NAME, NODE_VER, true);
	delay(100);
	present(GATE_OPEN_ID, S_DOOR, "Brama");
	present(DOOR_OPEN_ID, S_DOOR, "Bramka");
	present(DOOR_RING_ID, S_LIGHT, "Dzwonek");
	present(LIGHT_ID, S_LIGHT, "Swiatlo");
	present(FOTO_ID, S_LIGHT_LEVEL);
	for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   		
		present(TEMP_ID+i, S_TEMP);
	}

	isMetric = getControllerConfig().isMetric;
}

void setup()
{	
	pinMode(GATE_INDICATOR_PIN, INPUT_PULLUP);

	pinMode(GATE_OPEN_PIN, OUTPUT);
	pinMode(DOOR_OPEN_PIN, OUTPUT);
	pinMode(DOOR_INDICATOR_PIN, INPUT_PULLUP);
	pinMode(DOOR_RING_PIN, INPUT_PULLUP);

	pinMode(TEST_LED_PIN, OUTPUT);

	readState();
	prev_status = is_gate_opened;
	prev_door_opening = is_door_openning;
	prev_ring = is_door_bell;
}
bool first_loop=true;

void loop()
{
	if(first_loop) {
		wait(2000);
		Serial.println("First: ");
		myresend(gateOpenMsg.set(is_gate_opened ? 1 : 0));
		first_loop=0;
	}
	current_time = millis();

	readState();

	/*is_gate_opened = digitalRead(GATE_INDICATOR_PIN) == HIGH;
	is_door_openning = digitalRead(DOOR_INDICATOR_PIN) == HIGH;
	is_door_bell = digitalRead(DOOR_RING_PIN) == LOW;*/
	//Serial.print(is_gate_opened);
	byte impuls = is_gate_opened? 100:0 ;
    avg = (avg * dt + impuls)/(dt+1);
	is_gate_opened = avg > 1;
	//Serial.print(", Avg: ");
	//Serial.println(avg);

	if(current_time > gate_last_send + 2000) {
		if(is_gate_opened != prev_status) {
			Serial.println("roznica gate 2000: ");
			prev_status = is_gate_opened;
			myresend(gateOpenMsg.set(is_gate_opened ? 1 : 0));
			//send(gateOpenMsg.set(is_gate_opened ? 1 : 0));
			start_gate_time = millis();
			report_gate = true;
			//digitalWrite(TEST_LED_PIN, is_gate_opened ? HIGH : LOW);
			gate_last_send = current_time;
		}

		if(is_door_bell != prev_ring){
			prev_ring = is_door_bell;
			myresend(doorRingMsg.set(is_door_bell ? 1 : 0));
			gate_last_send = current_time;
		}
		
		if(is_door_openning != prev_door_opening){
			Serial.println("roznica door 2000: ");
			prev_door_opening = is_door_openning;
			myresend(doorOpenMsg.set(is_door_openning ? 1 : 0));
			gate_last_send = current_time;
		}		
	}

	if(open_locker && (current_time > start_door_time + 1000UL * duration_door_open)) {
		openDoor(false);
	}
	if(report_gate && (current_time > start_gate_time + DELAY_GATE_STATUS)) {
		Serial.println("delayed try send");
		report_gate=false;
		myresend(gateOpenMsg.set(is_gate_opened ? 1 : 0));
	}

	if(current_time > temp_last_send + TEMP_REPORT_INTERVAL) {
		int foto = analogRead(FOTO_PIN);
		int vLux = sensorToLx(foto);

		if(prev_foto != vLux) {
			myresend(fotoMsg.set(vLux));
			prev_foto = vLux;
		}
		
		startMeasureDS();
		temp_last_send = current_time;
	}
	wait(40);
}

void readState() {
	is_gate_opened = digitalRead(GATE_INDICATOR_PIN) == HIGH;
	is_door_openning = digitalRead(DOOR_INDICATOR_PIN) == HIGH;
	is_door_bell = digitalRead(DOOR_RING_PIN) == LOW;
}

void log() {
	Serial.print("Door open time: ");
	Serial.println(duration_door_open);
}

void receive(const MyMessage &message) {
	if(first_loop) {
		Serial.println("First receive");
	}
	Serial.print("Destination: ");
	Serial.print(message.destination);
	Serial.print(", type: ");
	Serial.print(message.type);
	Serial.print(", Sensor: ");
	Serial.print(message.sensor);
	Serial.println();
	//  byte no;
	if (message.type == V_VAR1){
		duration_door_open = message.getULong();
		
		saveState(0, duration_door_open);
	}

	if (message.type == V_TRIPPED || message.type == V_STATUS) {


		bool status;
		switch(message.sensor) {
		case GATE_OPEN_ID:
			status = message.getBool();
			Serial.print(", trying gate: ");
			Serial.println(status);
			if(status != is_gate_opened && (millis() - start_gate_time >3000UL)) {
				// start moving gate
				Serial.print(", PROCESSING...");
				digitalWrite(GATE_OPEN_PIN, HIGH);
				delay(DURATION_GATE_PULSE);
				digitalWrite(GATE_OPEN_PIN, LOW);

			}
			start_gate_time = millis();
			report_gate = true;
			break;
		case DOOR_OPEN_ID:
			status = message.getBool();
			Serial.print(", trying door: ");
			Serial.println(status);
			if(status) {
				Serial.print(", OPENING DOOR...");
				start_door_time = millis();
			}
			openDoor(status);
			break;
		case LIGHT_ID:
			status = message.getBool();
			Serial.print("Light: ");
			Serial.println(status);
			digitalWrite(LIGHT_PIN, status?HIGH:LOW);
			break;
		}
	}
	Serial.println();
}

void openDoor(bool open) {
	open_locker = open;
	digitalWrite(DOOR_OPEN_PIN, open ? HIGH : LOW);
	Serial.print("Open door: ");
	Serial.println(open);
	myresend(doorOpenMsg.set(open ? 1 : 0));
}

float dtt = 3;
void startMeasureDS() {
	for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {

		// Fetch and round temperature to one decimal
		float temperature = static_cast<float>(static_cast<int>((isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;

		
		if (temperature != -127.00 && temperature != 85.00) {

			if(lastTemperature[i] == 0) 
				lastTemperature[i] = temperature;

			temperature = (lastTemperature[i]*dtt + temperature)/(dtt + 1);

			if (round(temperature*10) != round(lastTemperature[i]*10)) {
				// Send in the new temperature
				myresend(tempMsg.setSensor(TEMP_ID+i).set(temperature,1));	
				lastTemperature[i] = temperature;
			}
		}
	}
	// Fetch temperatures from Dallas sensors
	sensors.requestTemperatures();
}

//value,Lx
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
			/*Serial.print(value);
			Serial.print(',');
			Serial.print(v2Lx[idx-2]);
			Serial.print(',');
			Serial.print(v2Lx[idx]);
			Serial.print(',');
			Serial.print(v2Lx[idx-2 + 1]);
			Serial.print(',');
			Serial.println(v2Lx[idx-2+3]);*/
			return map(value, v2Lx[idx-2], v2Lx[idx], v2Lx[idx-2 + 1], v2Lx[idx-2+3]);
		}
		
	}
	return 0;
}