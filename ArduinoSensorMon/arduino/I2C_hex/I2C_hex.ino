// Include Emon Libraryd
#include "EmonLib.h"

#include <Wire.h>
#include "ringBufS.h"
#include <string.h>
#include "Timer.h"
#include "DHT.h"
#define DEBUG
#define PRINT_HEX
#define TIMER_SENSOR_PRINT
/* GPIO assignment */
#define ANALOG_CURRENT_PIN	7 
#define DHTPIN 2     // what digital pin we're connected to

//jwd: these defines should be moved to header file, common b/w pi and arduino
// jwd: convert to hex, keep here for reference
#define SENSOR	0x01 
#define READ    0x01
#define ENABLE_TIMER  0x03
#define DISABLE_TIMER	0x04
#define TEST_EVENT 0x05
#define SET_ALARM_THRESHOLD 0X06

#define EVENT		0x02
#define LOOPBACK	0x03

#define CURRENTPROBE	0x01
#define DISTSENSOR1	0x02
#define TEMP_HUMIDITY	0x03 // jwd should I separate?

#define SLAVE_ADDRESS	0x04
#define INT_TO_PI	4
#define I2C_READY_TO_PI	7

#define EVENT_SIZE 12
#define MAX_EVENTS 4

#ifdef DEBUG
// Update the following if needed…
#define sprint(ARGS...) Serial.print(ARGS)
#else
#define sprint(ARGS...) do {} while (0)
#endif
#ifdef DEBUG
#define sprintln(ARGS...) Serial.println(ARGS)
#else
#define sprintln(ARGS...) do {} while (0)
#endif

/*
 * next steps:
 * define event format
 <START_BYTE> <length> <EVENT> <reason: SENSOR> <type: alarm> <data....>
 for distance: return height value that generated alarm
 for current: return current value (Irms current, that is) and duration of run; current isn't that imporant, though, really

 * determine how to do timing: i.e. how long did pump run?  then send an event for "pump ran for xxxx seconds; use "millis() command"
 grab stamp on pump on, on pump off grab again and subtract
 * set threshold commands; for now, can just hard code; in all honest, this is overkill anyway
 * if alarm is received by pi, determine if email should be sent
 */


//burden resistor (ohms) = (AREF * CT TURNS) / (2√2 * max primary current)
// AREF: i.e. 4.7V, as measured input to circuit
// CT TURNS: i.e. 2000, from data sheet of CT
// max primary current: chose ~33A, even though CT is rated to 60A
// see https://learn.openenergymonitor.org/electricity-monitoring/ct-sensors/interface-with-arduino

// current constant = (# turns for CT) ÷ (burden resistor) 
// using a 100ohm resistor, so:

/*
 * get event:
 * bd <size> <EVENT> <subcommand> <type> 
 * 0xbd <size> EVENT=0X2 SENSOR=0X01 type=specific ID <extra data?>
 *
 */

#define CURRENT_CONSTANT	20.0
#define LINE_VOLTAGE		120.0
// see https://learn.openenergymonitor.org/electricity-monitoring/ctac/ct-and-ac-power-adaptor-installation-and-calibration-theory?redirected=true

EnergyMonitor emon1;  // for current sensor
DHT dht(DHTPIN, DHT22); // for temp/humidity sensor

#define RX_BUF_SIZE 16 
#define TX_BUF_SIZE 16
#define EVT_BUF_SIZE 32
ringBufS rxBuffer;
ringBufS txBuffer;
ringBufS evtBuffer;

#define START_BYTE	0xBD
#define LOOP_BACK_BYTE	0xDB
// jwd change to enum
#define GET_START_BYTE	0x01	
#define GET_LENGTH_BYTE	0x02	
#define GET_DATA_BYTE	0x04	
#define GET_CS_BYTE	0x05	
#define PARSE_COMMAND	0x06	
#define LOOP_BACK_GET_LENGTH 0x07	
#define LOOP_BACK_BYTES	     0x08	

#define MAX_DATA_REQUEST 16
#define COMMAND 0
#define SUB_COMMAND 1
unsigned char receive_read_count = 0;
unsigned char receive_byte_index = 0;
unsigned char receive_state = GET_START_BYTE;
unsigned char xor_cs = 0;
unsigned char receive_buffer[MAX_DATA_REQUEST]; // jwd maybe too large
// sensor flags
#define TIMER_ENABLED	(1<<0)
#define INT_ENABLED	(2<<0)

#define DIST_RUN_AVG_SIZE 5
struct sample_data {
	int distance_x_100;
	struct sample_data *next;
};

struct running_avg {
	struct	sample_data list[DIST_RUN_AVG_SIZE];
	struct sample_data *curr;	
};
struct running_avg distance_running_avg;
struct sensor_data {
	Timer		timer;
	unsigned int	pollingInterval_ms;
	int		timerHandle;
	unsigned int	low_threshold;
	unsigned int	high_threshold;
	unsigned char	flags;
};
struct distance_sensor {
	unsigned int distance;
	int 	runningAvg_x_100;
	struct sensor_data sensor;
	/* should thresholds and runavg sample be stored here?*/
};
struct distance_sensor distanceSensor;

struct current_sensor {
	unsigned int	lastCurrentReadingx1000;
	struct sensor_data sensor;
	/* what about the other constants?*/
};
struct current_sensor currentSensor;
#define TRIGGER_PIO		5
#define ECHO_PIN		6
int sensorEvent=0;

void SetupRingBuffer(struct running_avg* avg_data) {
	int NumElements = sizeof(avg_data->list)/sizeof(sample_data);
	int i;

	avg_data->curr=avg_data->list;

	for (i=1;i<NumElements;i++) {
		avg_data->curr->distance_x_100=0;
		avg_data->curr->next = avg_data->curr+1;
		avg_data->curr = avg_data->curr->next;
	}
	avg_data->curr->distance_x_100=0;
	avg_data->curr->next=avg_data->list;
} 

int updateRunningAverage(struct running_avg *avg_data, int new_distance){
	struct sample_data *avgRunner;
	int NumElements = sizeof(avg_data->list)/sizeof(sample_data);
	int i,sum=0;

	avg_data->curr->distance_x_100=new_distance; 
	avgRunner = avg_data->curr; 

	for (i=0;i<NumElements;i++) { 
		sum+=avgRunner->distance_x_100; 
		avgRunner=avgRunner->next; 
	} 	
	avg_data->curr=avg_data->curr->next;
	
	return (sum/NumElements);
}

float getDistance(void) {
	long duration;
	float distance;
//	noInterrupts();
	/* water level */
	digitalWrite(TRIGGER_PIO, LOW);  // Added this line
	delayMicroseconds(2); // Added this line
	digitalWrite(TRIGGER_PIO, HIGH);
	delayMicroseconds(10); // Added this line
	digitalWrite(TRIGGER_PIO, LOW);
	duration = pulseIn(ECHO_PIN, HIGH);
	//interrupts();
	distance = ((duration/2) / 29.1)/2.54;// jwd  - change to bounded while to check for valid value?
	if (distance >= 200 || distance <= 0) 
		sprintln("Out of range");


#ifdef TIMER_SENSOR_PRINT
	sprint(distance);
	sprintln("\"");
#endif
	return distance;
} 
#define EVENT_COUNT 5
struct event_data {
	unsigned char buf[EVENT_SIZE];
	int size;
	struct event_data *next;
};
struct event_data eventList[EVENT_COUNT];
struct event_data *currEvtPtr = eventList;
struct event_data *eventReadPtr;
struct event_data *eventWritePtr;
int eventCount=0;

unsigned char get_xor_cs(unsigned char *buf, int size){
	unsigned char cs=0;
	while (size--) 
		cs^=*buf++;
	return cs;
}

void initEventBuffer() {
	//set next of last elment to beginning of buffer, init read pointer, write pointer
	int i;

	eventWritePtr = eventList;
	eventReadPtr = NULL;

	// set up the linked list; can experiment with mallocing intead....later
	for (i=0; i < (EVENT_COUNT-1); i++) {
		eventList[i].next = &eventList[i+1];
	}
	// point last element to first
	eventList[EVENT_COUNT-1].next = eventList;

}
int addEvent(unsigned char reason, unsigned char type, unsigned char *dataBuf, int size) {
	int rc = -1;
	//<START_BYTE> <length> <EVENT> <reason: SENSOR> <type: alarm> <data....>
	// sizeofData = 1 (START_BYTE) + 1( LENGTH) + 1 (EVENT) + 1 (reason) + 1 (type) + size of event data

	eventWritePtr->buf[0] = START_BYTE;
	eventWritePtr->buf[1] = size + 3; // jwd : 4 includes EVENT, reason, type
	eventWritePtr->buf[2] = EVENT;
	eventWritePtr->buf[3] = reason;
	eventWritePtr->buf[4] = type;
	// jwd more sloppy
#define HEADER_SIZE  5
#define CS_SIZE	1
	eventWritePtr->size = size + HEADER_SIZE + CS_SIZE;

	// buffer size must accommodate data + the 3 byte header
	if ((eventWritePtr->size) <= EVENT_SIZE) { // buffer is large enough to hold data
		memcpy(&eventWritePtr->buf[HEADER_SIZE],dataBuf,size);
		eventWritePtr->buf[eventWritePtr->size-1]=get_xor_cs(eventWritePtr->buf,eventWritePtr->size-1);
		
		if (!eventCount ) { // is list empty?
			eventReadPtr = eventWritePtr;
			eventWritePtr = eventWritePtr->next;
		}
		else if (eventCount == EVENT_COUNT) { // event buffer full?
			eventReadPtr = eventReadPtr->next;
			eventWritePtr = eventWritePtr->next;
		}
		else {
			eventWritePtr = eventWritePtr->next;
		}

		if (eventCount < EVENT_COUNT)
			eventCount++;
		rc = eventWritePtr->size;
		GenerateInterrupt(1);
	}
	return rc;
}

void getEvent(){
	if (!eventCount) {
		char unsigned buf[] = {(unsigned char)START_BYTE,1,(unsigned char)EVENT,0};
		buf[sizeof(buf)-1] = get_xor_cs(buf,sizeof(buf)-1);
		sprintln("load event");	
		RingBufWriteBlock(buf,4,&txBuffer);// jwd sloppy	
	}
	else {// if (eventReadPtr) 
		RingBufWriteBlock(eventReadPtr->buf,eventReadPtr->size,&txBuffer);	
		eventCount --;
		eventReadPtr = eventReadPtr->next;
		if (!eventCount)
			GenerateInterrupt(0);
	}
}

void distanceUpdate(void) {
	struct sensor_data *sensor=&distanceSensor.sensor;
	unsigned char buf[2];
	unsigned int value = distanceSensor.distance = (unsigned int) (getDistance()*100);
	int avg = updateRunningAverage(&distance_running_avg,value);
	sprint("dist: ");sprint(value);sprint("  avg:  ");sprintln(avg);
	
	if ((sensor->high_threshold && (avg > sensor->high_threshold)) ||
			(sensor->low_threshold && (avg < sensor->low_threshold))) {

		buf[0]=	(unsigned char)((avg >> 8) & 0xFF);
		buf[1]=	(unsigned char)(avg & 0xff);

		addEvent(SENSOR,DISTSENSOR1,buf,sizeof(buf));
	}
} 

float getCurrent(void) {
	/* current sensor */
#if 1
	float Irms;
//	noInterrupts();
	Irms  = emon1.calcIrms(1480);  // Calculate Irms only
//interrupts();
#else
	float Irms = emon1.calcIrms(1480);  // Calculate Irms only
#endif


#ifdef TIMER_SENSOR_PRINT
	sprint(Irms*LINE_VOLTAGE);           // Apparent power
	sprint(" ");
	sprintln(Irms);             // Irms
#endif
	/*
	 *    if turning on, record start time
	 *    if turning off, record stop time and make sure there's a "valid" start before calculating difference and providing bad data
	 *       send event that includes run duration in milliseconds
	 *
	 *    data to include when issuing event: run time
	 *
	 *
	 */
	return Irms;
}

void currentUpdate(void) {
	struct sensor_data *sensor=&currentSensor.sensor;
	unsigned char buf[4];
	unsigned int value = currentSensor.lastCurrentReadingx1000 =(unsigned int) (getCurrent()*1000);
	static unsigned char recordingCurrentDuration = 0;
	static unsigned long starttime;

	if (sensor->high_threshold && (value > sensor->high_threshold) && !recordingCurrentDuration) {
		starttime = millis();
    sprint("start recording time: ");sprintln(starttime);
		recordingCurrentDuration = 1;
	}
	else if (recordingCurrentDuration && sensor->low_threshold && (value < sensor->low_threshold)){
		unsigned long stoptime = millis();
		unsigned long duration = stoptime - starttime;
    sprint("stop recording time:"); sprintln(stoptime);
		sprint("stop recording duration. duration ="); sprintln(duration);
		recordingCurrentDuration = 0;
		buf[0]=	(unsigned char)(( duration >> 24) & 0xFF);
		buf[1]=	(unsigned char)(( duration >> 16) & 0xFF);
		buf[2]=	(unsigned char)(( duration >> 8) & 0xFF);
		buf[3]=	(unsigned char)(duration & 0xff);
		addEvent(SENSOR,CURRENTPROBE,buf,sizeof(buf));
	}
}

float getTemp(void) {
	float temp;
//	noInterrupts();
	temp = dht.readTemperature(true);
	//interrupts();

#ifdef TIMER_SENSOR_PRINT
	sprint("temperature, F: ");
	sprintln(temp);             // Irms
#endif
	return temp;
}
float getHumidity(void) {
	float humidity;
//	noInterrupts();
	humidity = dht.readHumidity();
	//interrupts();
#ifdef TIMER_SENSOR_PRINT
	sprint("humidity: ");
	sprintln(humidity);             // Irms
#endif
	return humidity;
}
void RingBufWriteBlock(unsigned char *buf, int size, ringBufS *ringBuf) {
	int i;
//	noInterrupts();
	for (int i=0; i<size; i++) {
#ifdef PRINT_HEX
    sprint("^");sprintln(buf[i],HEX); // jwd remove me to reduce debug spam
#endif    
		ringBufS_put(ringBuf,buf[i]);
	}

	if (!ringBufS_empty(ringBuf))  // jwd debug this needs to come out
		digitalWrite(I2C_READY_TO_PI,1);
	//interrupts();
}

// jwd should check all commands for size/bounds checking; not doing this yet
void HandleSensorCommand(unsigned char *bufPtr) {
	switch(bufPtr[0]) {
		case READ: // read current value
			switch (bufPtr[1]) {
				case CURRENTPROBE:
					{
						unsigned char buf[]={START_BYTE,0, // fill in size later
							(unsigned char) SENSOR,
							(unsigned char) READ,
							(unsigned char) bufPtr[1],
							(unsigned char)((currentSensor.lastCurrentReadingx1000 >> 8) & 0xFF),
							(unsigned char)(currentSensor.lastCurrentReadingx1000 & 0xFF),
							0}; // fill in cs later
						buf[1]=sizeof(buf)-3;
						buf[sizeof(buf)-1] = get_xor_cs(buf,sizeof(buf));
						RingBufWriteBlock(buf,sizeof(buf),&txBuffer);
					}
					break;
				case DISTSENSOR1:
					{
						unsigned char buf[]={START_BYTE,0, // fill in size later
							(unsigned char) SENSOR,
							(unsigned char) READ,
							(unsigned char) bufPtr[1],
							(unsigned char)((distanceSensor.distance >> 8) & 0xFF),
							(unsigned char)(distanceSensor.distance & 0xFF),
							0}; // fill in cs later

						buf[1]=sizeof(buf)-3;
						buf[sizeof(buf)-1] = get_xor_cs(buf,sizeof(buf));

						RingBufWriteBlock(buf,sizeof(buf),&txBuffer);
					}
					break;
				case TEMP_HUMIDITY:
					{
						unsigned int temperature = (unsigned int)(getTemp()*10);
						unsigned int humidity = (unsigned int)(getHumidity()*9);
						unsigned char buf[]={START_BYTE,0, // fill in size later
							(unsigned char) SENSOR,
							(unsigned char) READ,
							(unsigned char) bufPtr[1],
							(unsigned char)((temperature >> 8) & 0xFF),
							(unsigned char)( temperature & 0xFF),

							(unsigned char)((humidity >> 8) & 0xFF),
							(unsigned char)( humidity & 0xFF),
							0}; // fill in cs later

						buf[1]=sizeof(buf)-3;
						buf[sizeof(buf)-1] = get_xor_cs(buf,sizeof(buf));

						RingBufWriteBlock(buf,sizeof(buf),&txBuffer);
					}
					break;
				default:
					sprint("Unsupported sensor: ");sprintln(bufPtr[1]);
					break;
			}
			break;
		case ENABLE_TIMER: // should provide polling interval in ms, threshold low,high
			{
				sensor_data *sensor=NULL;
				void (*timerCB)(void);
				switch (bufPtr[1]) {
					case CURRENTPROBE:
						sensor = &currentSensor.sensor;
						timerCB = currentUpdate;
						break;
					case DISTSENSOR1:
						sensor = &distanceSensor.sensor;
						timerCB = distanceUpdate;
						break;
					default:
						sprint("Unsupported sensor: ");sprintln(bufPtr[1]);
						break;
				}
				if (sensor) {
					/* stop timer if running already; can only keep track of one running timer  */
					if (sensor && sensor->timerHandle >= 0) {
						sensor->timer.stop(sensor->timerHandle);
						sensor->timerHandle = -1;
					}
					sensor->pollingInterval_ms = (bufPtr[2] << 8) + bufPtr[3];
					sensor->timerHandle = sensor->timer.every(sensor->pollingInterval_ms, timerCB);
					sprint("polling interval: ");sprintln(sensor->pollingInterval_ms);
					sprint("tick started id=");
					sprintln(sensor->timerHandle);
				}
				break;
			}
		case DISABLE_TIMER: // stops timer via handle
			{
				sensor_data *sensor=NULL;
				switch (bufPtr[1]) {
					case CURRENTPROBE:
						sensor = &currentSensor.sensor;
						break;
					case DISTSENSOR1:
						sensor = &distanceSensor.sensor;
						break;
					default:
						sprint("Unsupported sensor: ");sprintln(bufPtr[1]);
						break;
				}
				if (sensor && sensor->timerHandle >= 0) {
					sensor->timer.stop(sensor->timerHandle);
					sensor->timerHandle = -1;
				}
				break;
			}
#ifdef DEBUG
		case TEST_EVENT: // stops timer via handle
			{
				unsigned char buf[4];
				int size;
				switch (bufPtr[1]) {
					case CURRENTPROBE:
						buf[0]=	(unsigned char)((currentSensor.lastCurrentReadingx1000 >> 8) & 0xFF);
						buf[1]=	(unsigned char)(currentSensor.lastCurrentReadingx1000 & 0xFF);
						size=2;	
						break;
					case DISTSENSOR1:
						buf[0]=	(unsigned char)((distanceSensor.distance >> 8) & 0xFF);
						buf[1]=	(unsigned char)(distanceSensor.distance & 0xff);
						size=2;	
						break;
					default:
						sprint("Unsupported sensor: ");sprintln(bufPtr[1]);
						break;
				}
				addEvent(SENSOR,bufPtr[1],buf,size);
				break;
			}
#endif
		case SET_ALARM_THRESHOLD: // should provide polling interval in ms, threshold low,high
			{
				Serial.println("SET_ALARM_THRESHOLD");
				sensor_data *sensor=NULL;
				switch (bufPtr[1]) {
					case CURRENTPROBE:
						sprintln("CURRENTPROBE");
						sensor = &currentSensor.sensor;
						break;
					case DISTSENSOR1:
						sprintln("DISTSENSOR1");
						sensor = &distanceSensor.sensor;
						break;
					default:
						sprint("Unsupported sensor: ");sprintln(bufPtr[1]);
						break;
				}
				if (sensor) {
					// jwd need to bounds check; do this in timer enable as well
					sensor->low_threshold = (bufPtr[2] << 8) + bufPtr[3];
					sensor->high_threshold = (bufPtr[4] << 8) + bufPtr[5];

					sprint("low_threshold: ");sprintln(sensor->low_threshold);
					sprint("high_threshold: ");sprintln(sensor->high_threshold);
				}
				break;
			}
		default:
			break;
	}
}

void receiveData(int byteCount){ 
	while( Wire.available()) {
		if (!ringBufS_full(&rxBuffer)) {
			unsigned char rc = Wire.read();
#ifdef PRINT_HEX
				sprint(">");sprintln(rc,HEX);
#endif       
			ringBufS_put (&rxBuffer, rc);
		}
		else {
			sprintln("RX Buffer Overflow");
			// generate an error event here; for now, just deplete data
			Wire.read();
		}
	}
}	

void sendData(void) {
	if(!ringBufS_empty(&txBuffer)) {
		Wire.write(ringBufS_get(&txBuffer));
		if(ringBufS_empty(&txBuffer)) 
			digitalWrite(I2C_READY_TO_PI,0); 
	}
}

void GenerateInterrupt(int state){
	digitalWrite(INT_TO_PI, state);
}
void initSensor(struct sensor_data *sensor) {

	sensor->low_threshold=0;
	sensor->high_threshold=0;
	sensor->flags=0;
	sensor->timerHandle=-1;
}

void setup() {
  Serial.begin(57600);
	// initialize i2c as slave
	Wire.begin(SLAVE_ADDRESS);
	// define callbacks for i2c communication
	pinMode(INT_TO_PI, OUTPUT);
	pinMode(I2C_READY_TO_PI, OUTPUT);
	digitalWrite(INT_TO_PI, 0);
	digitalWrite(I2C_READY_TO_PI, 0);

	// init temp/humidity sensor
	dht.begin();

	// init rng buffers
	ringBufS_init (&rxBuffer,RX_BUF_SIZE);
	ringBufS_init (&txBuffer,TX_BUF_SIZE);
	ringBufS_init (&evtBuffer,EVT_BUF_SIZE);
	// setup event ring
	initEventBuffer();
	// init running avg for distance
	SetupRingBuffer(&distance_running_avg);

	// init sensors
	initSensor(&currentSensor.sensor);
	initSensor(&distanceSensor.sensor);

	// initialize i2c as slave
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);
	sprintln("I2C Ready!");
	// init current monitor	
	emon1.current(ANALOG_CURRENT_PIN, CURRENT_CONSTANT);             // Current: input pin, calibration.

	// setup distance sensor pins
	pinMode(TRIGGER_PIO, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
}

void parseCommand(void) {
	// main command byte stored in offset 0
	switch (receive_buffer[COMMAND])
	{
		case SENSOR:
			HandleSensorCommand(&receive_buffer[SUB_COMMAND]);
			break;
		case EVENT:
			getEvent(); // Sensor event should cache value that causes event, only do one event at a time per sensor
			break;
		default:
			sprint("unsupported command"); sprintln(receive_buffer[COMMAND],HEX);
			break;
	}
}
unsigned char getByte(ringBufS *rb){
	unsigned char rc;
	//noInterrupts();
	rc =ringBufS_get(rb);
	//interrupts();
	return rc;
}

int zeroCount=0;
#define ZERO_RESET_COUNT 16
// callback for received data

void(*resetFunc) (void) = 0; //declare reset function @ address 0

void parseData(ringBufS *buffer){
	static byte index = 0;
	while(!ringBufS_empty(&rxBuffer))  {
		if (receive_state ==  GET_START_BYTE) {
			unsigned char rc;

			receive_byte_index = 0;
			if ((rc=getByte(&rxBuffer)) == START_BYTE) {
				receive_state =	GET_LENGTH_BYTE;
				xor_cs = START_BYTE;
				zeroCount=0;
			}
			else if (rc == LOOP_BACK_BYTE) {
				receive_state = LOOP_BACK_GET_LENGTH;
			}
			else if (rc == 0){
				zeroCount++;
				if (zeroCount >= ZERO_RESET_COUNT)
					resetFunc();

			}
			else {
				sprint("invalid char. expecting 0xBD, found "); sprintln(rc,HEX);
			}
		}
#if 0
		else if (receive_state == LOOP_BACK_GET_LENGTH) {
			sprintln("LPL");
			receive_read_count=ringBufS_get(&rxBuffer);
			ringBufS_put(&txBuffer,LOOP_BACK_BYTE);
			ringBufS_put(&txBuffer,receive_read_count);
			xor_cs = LOOP_BACK_BYTE^receive_read_count;
			receive_state = LOOP_BACK_BYTES;
		}
		else if (receive_state == LOOP_BACK_BYTES) {
			sprintln("LPB");
			unsigned char rc=ringBufS_get(&rxBuffer);
			if (++receive_byte_index == receive_read_count) 
				receive_state = GET_START_BYTE;
			xor_cs^=rc;
			ringBufS_put(&txBuffer,rc);
		}
#endif
		else if (receive_state == GET_LENGTH_BYTE) {
			if ((receive_read_count=getByte(&rxBuffer)) <= MAX_DATA_REQUEST) {
				xor_cs^=receive_read_count;
				receive_state =	GET_DATA_BYTE;
			}
			else {
				sprint("invalid byte count: "); sprintln(receive_read_count,HEX);
				receive_read_count = 0; // jwd is this necessary?
				receive_state =	GET_START_BYTE;
			}
		}
		else if (receive_state == GET_DATA_BYTE) {

			unsigned char rc=getByte(&rxBuffer);
			receive_buffer[receive_byte_index++] = rc;	
			xor_cs^=rc;
			if (receive_byte_index == receive_read_count) {
				receive_state = GET_CS_BYTE;
			}
		}
		else if (receive_state == GET_CS_BYTE) {
			unsigned char rc=getByte(&rxBuffer);

			if (xor_cs == rc) {
				// cs good
				receive_state = PARSE_COMMAND;
			}
			else {
				sprint("Checksum Error. Expected: "); sprint(xor_cs,HEX);
				sprint("  Received:  "); sprint(rc,HEX);
				// jwd send an error here
				// look for valid byte again
				receive_state = GET_START_BYTE;
			}
		}
		else
			sprintln("nada"); // jwd set error event here

		if (receive_state == PARSE_COMMAND) { 
			parseCommand();
			receive_state = GET_START_BYTE; // jwd may change if use CS		
		}
	}
}

void loop() {
	if (distanceSensor.sensor.timerHandle >= 0)
		distanceSensor.sensor.timer.update();
	if (currentSensor.sensor.timerHandle >= 0)
		currentSensor.sensor.timer.update();

	while(!ringBufS_empty(&rxBuffer)) {
		parseData(&rxBuffer);
	}
	delay(1); // jwd i know, this is VERY VERY BAD
}
