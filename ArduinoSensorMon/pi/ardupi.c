#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/signal.h>
#include "wiringPi.h"
#include "wiringPiI2C.h"
#include "requests.h"

#define VERSION 1.0
//#define PRINT_I2C
//#define PRINT_SENSOR

//#define LOG_FILE_FOLDER "/home/pi/logs/"
//#define ERROR_LOG		"error.txt"
#define ERROR_LOG		stdout
//#define LAST_SUMP_DATA_FILE	"/var/tmp/sump_info.txt"
//#define LAST_TEMP_HUMIDITY_FILE	"/var/tmp/temp_humidity.txt"
#define LAST_SUMP_DATA_FILE	"/var/www/html/rt_stats/sump_info.txt"
#define LAST_TEMP_HUMIDITY_FILE	"/var/www/html/rt_stats/temp_humidity.txt"
#define TEMP_HUMIDITY_LOG	"temp_humidity.txt"
#define DISTANCE_LOG		"sump_distance.txt"
#define SUMP_CYCLE_STAT_FILE	"sump_cycle_stat.txt"

#ifdef PRINT_I2C 
#define printI2C(ARGS...) printf(ARGS)
#else
#define printI2C(ARGS...) do {} while (0)
#endif 
#ifdef PRINT_SENSOR 
#define printSensor(ARGS...) printf(ARGS)
#else
#define printSensor(ARGS...) do {} while (0)
#endif 

#define DISTANCE_INTERVAL	1
#define CURRENT_INTERVAL	1
#define TEMP_HUMIDITY_INTERVAL	5

#define START_BYTE      0xBD
#define LOOP_BACK_BYTE  0xDB

#define SENSOR  0x01 
#define READ    0x01
#define ENABLE_TIMER  0x03
#define DISABLE_TIMER   0x04
#define SET_ALARM_THRESHOLD 0x6
#define GET_EVENT		2

#define GET_DISTANCE		3
#define GET_TEMP_HUMIDITY	4
#define GET_CURRENT		5 
#define INIT    6
#define EVENT           0x02
#define LOOPBACK        0x03

#define CURRENTPROBE    0x01
#define DISTSENSOR1     0x02
#define TEMP_HUMIDITY   0x03
#define USE_CS          (1 << 0)
#define REQUEST_REPLY   (1 << 1)
/*
   1) DONE  next step: because of issues I'm having, will add interrupt pin that stays high when arduino has data available
   add simple wrapper that does not read unless pin is high, wait in 1ms loop until byte available
   can later change to interrupt
   2) DONE use timer to grab data from sensors via read command 

   DONE set up arduino interrupt to perform get event upon interrupt   change to level?
   working, but what I DO NOT like:
   FIXED I think...I have to wait 1s between commands. if I do this, seems ok, but I hate it
   FIXED I changed to poll I want to change to be interrupted whenever a byte is available, going to move forward with it like it is though

   add reset to arduino when program starts. need to add PIO

   DONE send threshold commands automatically
   DONE enable sensor polling automatically
   DONE periodically read all sensors
   tie into main program
   DONE   save off data to file
   publish web page with data
   make work with 128x64 display
   change errors to log to file
 */
/* recall that wiring pi gpio's are strange. remember to run "gpio readall" to get the wiringpi designated pio */
#define ARDUINO_INT_PIO	7 // gpio 4 according to pinout
#define ARDUINO_I2C_READY 0 // gpio 17 according to pinout

#define SENSOR_TO_BOTTOM_OF_WELL_INCHES 34

char *logFolder; 
char *emailAddressTo=NULL;
char *emailAddressFrom=NULL;
int arduinoI2Cfd;
int GetEvent(void);
void UpdateSumpStats(void);
int SendCmd(const char command, const char subcommand, const char type,
		const char *dataBuf, const char dataSize);
void setupAlarmThresholds(void);
int getResponse(char *buf, int bufSize);
unsigned long sumpRunTime_ms=0;
unsigned long sumpRunCycles=0;
struct request_list reqList;
// jwd move timer stuff to different file
char sequenceID=0;
int verbose=1; // jwd change me
/*
   for saving samples, combine sump distance with pump data. i.e.:
   <time> <water height> <sump cycle count> <sump total run time>
   log on: predetermined interval and alarm for either dist or pump on
   for temp/humidity:
   <time> <temperature in F> <humidity>

 */

int sendmail(const char *to, const char *from, const char *subject, const char *message) {
	int retval = -1;
	static int warn_once=0;
	FILE *mailpipe;

	if (!to || !from) {
		if (!warn_once) {
			printf("Error: must specify email \"to\" and \"from\" addresses\n");
			warn_once=1;
		}
	}

	else if ((mailpipe = popen("/usr/lib/sendmail -t", "w")) != NULL) {
		fprintf(mailpipe, "To: %s\n", to);
		fprintf(mailpipe, "From: %s\n", from);
		fprintf(mailpipe, "Subject: %s\n\n", subject);
		fwrite(message, 1, strlen(message), mailpipe);
		fwrite(".\n", 1, 2, mailpipe);
		pclose(mailpipe);
		retval = 0;
	}
	if (verbose) printf("email: %s %s\n",subject,message);

	return retval;
}

char * FormatTime(const time_t time, char *timeStr,const int bufsize) {
	struct tm *info;

	if (timeStr) {
		info = localtime( &time );
		strftime(timeStr, bufsize, "%Y-%m-%d %H:%M:%S", info);
	}
	return timeStr;
}

char * FormatDate(const time_t time, char *dateStr,const int bufsize) {
	struct tm *info;
	if (dateStr) {
		info = localtime( &time );
		strftime(dateStr, bufsize, "%Y-%m-%d", info);
	}
	return dateStr;
}

int SaveSample(const time_t time, char *buf, unsigned int bufSize, char *basePath, char *fileName) {
	int Output_fd, rc=-1; char buffer [128]; char timeBuf[128]; char fullLogFileName[128];
	struct stat st = {0};
	char dateBuf[11];

	// yeah yeah yeah. ugly, but gotta get something in


	// copy base path to file name buf
	if (!basePath || ! strncpy(fullLogFileName,basePath,sizeof(fullLogFileName))) {
		printf("Error getting basepath\n");
	}
	else if(!FormatDate(time,dateBuf,sizeof(dateBuf))) {
		printf("Error creating date string\n");
	}
	else if (!strncat(fullLogFileName,dateBuf,sizeof(fullLogFileName))) {
		printf("Error adding date to path\n");
	}
	else if (!strncat(fullLogFileName,"/",sizeof(fullLogFileName))) {
		printf("Error adding \"/\" to path\n");
	}
	else if ((stat(fullLogFileName, &st) == -1) && (mkdir(fullLogFileName, 0777)==-1)) {
		mkdir(fullLogFileName, 0777);
		printf("Error creating date folder\n");
	}
	else if (!strncat(fullLogFileName,fileName,sizeof(fullLogFileName))) {
		printf("Error adding file name to path\n");
	}
	else if((Output_fd = open(fullLogFileName, O_WRONLY | O_CREAT| O_APPEND, 0644)) < 0 ) {
		printf("Error opening %s for write\n",fullLogFileName);
	}
	else if (!FormatTime(time,timeBuf,sizeof(timeBuf))) {
		printf("Error creating time string\n");
	}
	else {
		snprintf(buffer, sizeof(buffer),"%s %s", timeBuf,buf);
		rc = write (Output_fd, &buffer,strlen(buffer));
		close (Output_fd);
	}
	return rc;
}

void err_abort(int status, char* message) {
	fprintf(stderr, "%s\n", message);
	exit(status);
}

void errno_abort(char* message) {
	perror(message);
	exit(EXIT_FAILURE);
}

void current_thread(union sigval arg) {
	add_request(&reqList,GET_CURRENT, &reqList.request_mutex, &reqList.got_request);
}
void distance_thread(union sigval arg) {
	add_request(&reqList,GET_DISTANCE, &reqList.request_mutex, &reqList.got_request);
}
void temp_humidity_thread(union sigval arg) {
	add_request(&reqList,GET_TEMP_HUMIDITY, &reqList.request_mutex, &reqList.got_request);
}

void create_timer(unsigned i,void (*func)(union sigval arg)) {
	timer_t timer_id;
	int status;
	struct itimerspec ts;
	struct sigevent se;
	/*
	 * Set the sigevent structure to cause the signal to be
	 * delivered by creating a new thread.
	 */
	se.sigev_notify = SIGEV_THREAD;
	se.sigev_value.sival_ptr = &timer_id;
	se.sigev_notify_function = func;
	se.sigev_notify_attributes = NULL;

	ts.it_value.tv_sec = i;
	ts.it_value.tv_nsec = 0;
	ts.it_interval.tv_sec = i;
	ts.it_interval.tv_nsec = 0;

	status = timer_create(CLOCK_REALTIME, &se, &timer_id);
	if (status == -1) errno_abort("Create timer");

	// TODO maybe we'll need to have an array of itimerspec
	status = timer_settime(timer_id, 0, &ts, 0);
	if (status == -1) errno_abort("Set timer");
}

void arduinoInterrupt(void) {
	add_request(&reqList,GET_EVENT, &reqList.request_mutex, &reqList.got_request);
}

int i2cWriteBlock(char * buf, int bytesToWrite, const int fd) {
	int rc=0;
	int bytesWritten=0;
	while((bytesToWrite--) && ((rc=wiringPiI2CWrite (fd,*buf++)) >= 0)) {
		bytesWritten++;
	}
#ifdef PRINT_I2C
	{
		int i;
		char *bufStart=buf-bytesWritten;
		for (i=0; i< bytesWritten;i++) {
			printI2C("%02x@",*bufStart++);
		}
		printI2C("\n");
	}
#endif	
	if(rc<0) {
		printf("writeblock failed rc=%d\n",rc);
	}
	if(rc>=0) rc = bytesWritten;
	return rc;
}

char xor_cs (char *buf, int size) {
	int i;
	char cs=0;
	for (i=0; i<(size);i++) {
		cs^=buf[i];
	}
	return cs;
}

int getArduinoI2Cdata(char *buf, int size, const int fd) {
	int rc=0, bytesRead = 0;
	int retryCount=0;
#define MAX_RETRY 500	
	while (size && (rc >= 0) && (retryCount < MAX_RETRY)) {
		if(!digitalRead(ARDUINO_I2C_READY)) {
			retryCount++;
			usleep(2000); // wait 2ms for more data
		}
		else if((size--) && ((rc=wiringPiI2CRead(fd)) >= 0)) {
			*buf++ = (char)rc;
			printI2C("*%02x",rc);
			bytesRead++;
			retryCount=0;
		}
	}

	if (retryCount >= MAX_RETRY) {
		printf("I2c Error: timed out waiting for data ready\n");
		rc = -2;
	} else if (rc < 0) {
		printf("Error reading from I2C device. rc = %d %s\n",rc,strerror(errno));
	}
	else if (rc >= 0) {
		rc = bytesRead;
	}
	return rc;
}

int GetEvent(void) {
	int rc;
	char getEvent[]={START_BYTE,sequenceID++,1,EVENT,0};
	char rspBuf[32]; // jwd fixme

	getEvent[sizeof(getEvent)-1]= xor_cs(getEvent,sizeof(getEvent)-1);
	rc = i2cWriteBlock(getEvent, sizeof(getEvent), arduinoI2Cfd);
	if (rc >=0) {
		rc = getResponse(rspBuf, sizeof(rspBuf));

		if (rc >= 0) {
			if (rspBuf[2] == DISTSENSOR1) {
				int strSize;
				char buf[64];
				int distance = ((rspBuf[3] << 8) | rspBuf[4]);
				float waterHeight;

				// sensor measures distance to water. convert to height from bottom of well
				waterHeight = SENSOR_TO_BOTTOM_OF_WELL_INCHES - (float)distance/100.0; 
				strSize=snprintf(buf,sizeof(buf)-1,"water height: %.2f\" sump run time: %.2fs sump cycles: %lu\n",
						waterHeight,sumpRunTime_ms/1000.,sumpRunCycles);
				printSensor("%s",buf);
				SaveSample(time(NULL),buf,strSize,logFolder,DISTANCE_LOG);
				sendmail(emailAddressTo,emailAddressFrom,"Sump Monitor WARNING!!\n",buf);
			}
			else if (rspBuf[2] == CURRENTPROBE) {
				unsigned long time_on = (rspBuf[3] << 24) | (rspBuf[4] << 16) | (rspBuf[5] << 8) | rspBuf[6];
				// jwd grab timestamp here to indicate "last time ran"
				sumpRunTime_ms+=time_on;
				sumpRunCycles++;
				printSensor("sump on: %lums total: %lums sump cycle count: %lu\n",time_on,sumpRunTime_ms,sumpRunCycles);
				UpdateSumpStats();
			}
		}
	}
	return rc;
}

int writeToTempFile(char *buf, char *fileName) {
	int tmpFileFD = open(fileName, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (tmpFileFD < 0) {
		printf("failed to open %s",fileName);
	}
	else {
		char timeBuf[128]; char dateBuf[11];
		if(!FormatDate(time(NULL),dateBuf,sizeof(dateBuf))) {
			printf("Error creating date string\n");
		}
		else if (!FormatTime(time(NULL),timeBuf,sizeof(timeBuf))) {
			printf("Error creating time string\n");
		}
		else {
			write (tmpFileFD,timeBuf,strlen(timeBuf)); write(tmpFileFD," ",1);

		}
		write (tmpFileFD, buf,strlen(buf));
		close (tmpFileFD);
	}
	return tmpFileFD;
}

int ReadSensorData(int sensorID) {
	int rc=0;
	char rspBuf[32]; // jwd fixme

	if ((rc=SendCmd(SENSOR,READ,sensorID,0,0)) < 0) {
		printf("Error sending read for sensor: %d\n",sensorID);
	}
	else if ((rc=getResponse(rspBuf, sizeof(rspBuf)))< 0) {
		printf("Error reading response for sensor: %d\n",sensorID);
	}
	else {
		/* parse sensor specific data here or send to caller? maybe send to caller and do interesting stuff there*/
		switch(sensorID) {
			case DISTSENSOR1:
				{
					int strSize;
					char buf[64];
					int distance = ((rspBuf[3] << 8) | rspBuf[4]);
					float waterHeight;
					// sensor measures distance to water. convert to height from bottom of well
					waterHeight = SENSOR_TO_BOTTOM_OF_WELL_INCHES - (float)distance/100.0; 
					strSize=snprintf(buf,sizeof(buf)-1,"water height: %.2f\" sump run time: %.2fs sump cycles: %lu\n",
							waterHeight,sumpRunTime_ms/1000.,sumpRunCycles);
					printSensor("%s",buf);
					SaveSample(time(NULL),buf,strSize,logFolder,DISTANCE_LOG);

					writeToTempFile(buf, LAST_SUMP_DATA_FILE);
				}	
				break;
			case CURRENTPROBE:
				{
					#ifdef PRINT_SENSOR
					int current = ((rspBuf[3] << 8) | rspBuf[4]);
					printSensor("sump current: %dmA\n",current);
					#endif
				}
				break;
			case TEMP_HUMIDITY:
				{
					int strSize;
					char buf[64];
					float temperature = ((int16_t)((rspBuf[3] << 8) | rspBuf[4]))/100.0;
					float humidity = ((int16_t)((rspBuf[5] << 8) | rspBuf[6]))/100.0;

					if ((humidity == 0.0) && (temperature == 0.)) {
						printf("error reading temp/humidty sensor, discard value\n");
						rc = -1;
						break;
					}
					strSize=snprintf(buf,sizeof(buf)-1,"temperature: %.2fF humidity: %.2f%%\n",
							temperature,humidity);
					printSensor("%s",buf);
					SaveSample(time(NULL),buf,strSize,logFolder,TEMP_HUMIDITY_LOG);
					writeToTempFile(buf, LAST_TEMP_HUMIDITY_FILE);
				}	
				break;
			default:
				break;
		}
	}
	return rc;
}

int getResponse(char *buf, int bufSize) {
	int rc;
	char dataSize, rxCS, SB,sequenceID;

	if (((rc=getArduinoI2Cdata(&SB, 1, arduinoI2Cfd)) < 0) || (SB != START_BYTE)) {// 1) validate start byte
		if (rc >=0 ) {
			printf("searching for 0x%02x but received 0x%02x\n",(char)START_BYTE,SB);	
		}
		else {
			printf("Error reading from I2C device\n");
		}
	}
	else if ((rc=getArduinoI2Cdata(&sequenceID, 1, arduinoI2Cfd)) < 0) {// 3) get sequence 
		printf("Error reading sequence ID: rc = %d\n",rc);	
	}
	else if ((rc=getArduinoI2Cdata(&dataSize, 1, arduinoI2Cfd)) < 0) {// 3) get data size
		printf("Error reading data size: rc = %d\n",rc);	
	}
	else if ((dataSize > bufSize) || ((rc=getArduinoI2Cdata(buf,dataSize,arduinoI2Cfd)) < 0)) {
		if (rc < 0) {
			printf("Error block reading from device: rc = %d\n",rc);
		}
		else {
			printf("invalid size: expected < bufSize: %d received dataSize: %d\n",bufSize,dataSize);
		}
	}
	else if((rc=getArduinoI2Cdata(&rxCS,1, arduinoI2Cfd)) < 0) {
		printf("Error receiving cheksum from device: rc =  %d\n",rc);
	}
	else {
		char calcCS=0;
		int i;

		calcCS = START_BYTE^(char)dataSize^sequenceID;
		for(i=0;i<dataSize; i++) {
			calcCS^=buf[i];
		}
		if (calcCS != rxCS) {
			printf("bad checksum. expected 0x%02x but received 0x%02x\n",rxCS,calcCS);
			rc = -2; // create define?
		}
	}
	if (rc >= 0) rc = dataSize;
	return rc;
}

int SendCmd(const char command, const char subcommand, const char type,
		const char *dataBuf, const char dataSize) {
	// <start byte> <size> <sensor> <sensor read> <sensor id>
	// <start byte> <size> <command> <subcommand > <type> <data> <size>
	int rc;
	int sendBufSize;
	char cs;
	char *sendBuf;
#define CS_SIZE		1
#define SEQUENCE_BYTE_SIZE	1
#define START_BYTE_SIZE	1
#define DATA_SIZE_BYTE	1
	sendBufSize = START_BYTE_SIZE + SEQUENCE_BYTE_SIZE + DATA_SIZE_BYTE + sizeof(command)+sizeof(subcommand)
		+sizeof(type)+dataSize+CS_SIZE;
	sendBuf = (char *)malloc(sendBufSize);
	if (sendBuf) {
		sendBuf[0]=START_BYTE;
		sendBuf[1]=sequenceID++;
		sendBuf[2]=sizeof(command)+sizeof(subcommand)+sizeof(type)+dataSize;
		sendBuf[3]=command;
		sendBuf[4]=subcommand;
		sendBuf[5]=type;
		if (dataBuf) memcpy(&sendBuf[6],dataBuf,dataSize); // jwd rc?
		cs = xor_cs(sendBuf,sendBufSize - 1);
		sendBuf[sendBufSize-1] = cs;
		rc = i2cWriteBlock(sendBuf,sendBufSize, arduinoI2Cfd);
		if (rc <0) {
			printf("SendCmd write failed\n");
		}
		free(sendBuf);
	}
	else {
		printf("failed to allocate memory\n");	
		rc = -1;
	}
	return rc;
}

int EnableSensor(int sensorID, int timerDuration_ms) {
	char timerBufTime[2];	

	timerBufTime[0] = (timerDuration_ms >> 8) & 0xFF; 
	timerBufTime[1] = timerDuration_ms & 0xFF; 

	return SendCmd(SENSOR,ENABLE_TIMER,sensorID,timerBufTime,sizeof(timerBufTime)); 
}
int StopSensor(int sensorID) {
	return 	SendCmd(SENSOR,DISABLE_TIMER,sensorID,0,0); 	
}

void initSensors(void)
{
	setupAlarmThresholds();
	EnableSensor(DISTSENSOR1,710);	usleep(250000);
	EnableSensor(CURRENTPROBE,200); usleep(250000);
}

void handle_event(struct request* a_request)
{
	int pioVal;
	int retryCounter=0;
#define RETRY_MAX 5
	if (!a_request) {
		printf("no valid request\n");
	}
	else {
		switch (a_request->number){
			case INIT:
				initSensors();
				break;
			case GET_EVENT:
				printf("Get Event: ");
				while ((pioVal = digitalRead(ARDUINO_INT_PIO))==1) {
					while ((retryCounter++ < RETRY_MAX) && (GetEvent()<0)) {
						printf("retry get event: %d\n",retryCounter);
						usleep(10000);
					}
				}
				break;
			case GET_DISTANCE:
				while ((retryCounter++ < RETRY_MAX) && (ReadSensorData(DISTSENSOR1)<0)) {
					printf("retry get distance retry: %d\n",retryCounter);
					usleep(10000);
				}
				break;
			case GET_TEMP_HUMIDITY:
				while ((retryCounter++ < RETRY_MAX) && (ReadSensorData(TEMP_HUMIDITY)<0)) {
					printf("retry get temp humidity retry: %d\n",retryCounter);
					usleep(10000);
				}
				break;
			case GET_CURRENT:
				while ((retryCounter++ < RETRY_MAX) && (ReadSensorData(CURRENTPROBE)<0)) {
					printf("retry get current retry: %d\n",retryCounter);
					usleep(10000);
				}
				break;
			default:
				printf("Not a valid event\n");
				break;
		}
	}
}
#define SUMP_CYCLE_FIELD	"sump_cycle"
#define SUMP_TIME_FIELD		"sump_run_time"
void setupSumpData(void) {
	FILE 	 *pFile;
	char fullLogFileName[128];

	if (!logFolder || !strncpy(fullLogFileName,logFolder,sizeof(fullLogFileName))) {
		printf(" Error getting log folder\n");
	}
	else if(!strncat(fullLogFileName,"/",sizeof(fullLogFileName))) {
		printf("Error adding \"/\" to path\n");
	}
	else if (!strncat(fullLogFileName,SUMP_CYCLE_STAT_FILE,sizeof(fullLogFileName))) {
		printf("Error getting sump cycle stat file name\n");
	}
	else if((pFile = fopen(fullLogFileName, "r")) == NULL ) {
		printf("Warning: %s does not exist, initializing stats to zeros.\n",fullLogFileName); 
	}
	else {
		char tag[16];
		while( fscanf(pFile,"%s",tag) != EOF ) {
			if( strncmp(tag,SUMP_CYCLE_FIELD,strlen(SUMP_CYCLE_FIELD)) == 0 ) {
				if (fscanf(pFile,"%lu",&sumpRunCycles) != 1) {
					printf("error storing saved sump cycle");
				}
			}
			else if( strncmp(tag,SUMP_TIME_FIELD,strlen(SUMP_TIME_FIELD)) == 0 ) {
				if (fscanf(pFile,"%lu",&sumpRunTime_ms) != 1) {
					printf("error storing saved sump run time");
				}
			}
		}
		fclose(pFile);
	}
	printf("Initial Sump Cycle Count: %lu, Initial Sump Run Time: %lums\n",sumpRunCycles,sumpRunTime_ms);
}
void UpdateSumpStats(void) {
	FILE 	 *pFile;
	char fullLogFileName[128];

	if (!logFolder || !strncpy(fullLogFileName,logFolder,sizeof(fullLogFileName))) {
		printf(" Error getting log folder\n");
	}
	else if(!strncat(fullLogFileName,"/",sizeof(fullLogFileName))) {
		printf("Error adding \"/\" to path\n");
	}
	else if (!strncat(fullLogFileName,SUMP_CYCLE_STAT_FILE,sizeof(fullLogFileName))) {
		printf("Error getting sump cycle stat file name\n");
	}
	else if((pFile = fopen(fullLogFileName, "w+")) == NULL ) {
		printf("Error creating or opening %s\n",fullLogFileName); 
	}
	else {
		fprintf(pFile,"%s:\t%lu\n%s:\t%lu\n",SUMP_CYCLE_FIELD,sumpRunCycles,SUMP_TIME_FIELD,sumpRunTime_ms);
		fclose(pFile);
	}
}

int setupLogDir(char *logDir) {
	int rc;
	struct stat st = {0};
	if ( (rc=stat(logDir, &st)) >= 0) { 
		// directory already created
	}
	else if ((rc=mkdir(logDir, 0777))  == -1) { 
		printf("error creating directory %s\n",logDir);
	}
	return rc;
}
void setupAlarmThreshold(char sensorid, unsigned int th_low, unsigned int th_high) {
	char buf[4];

	buf[0]=(th_low>>8)&0xFF;
	buf[1]=th_low&0xFF;
	buf[2]=((th_high)>>8)&0xFF;
	buf[3]=th_high&0xFF; 

	SendCmd(SENSOR, SET_ALARM_THRESHOLD,sensorid,buf,sizeof(buf));
}

void setupAlarmThresholds(void) {
#define CURRENT_LOW_TH_ma  100
#define CURRENT_HIGH_TH_ma 1000	
#define DIST_LOW_TH_inches 0
#define DIST_HIGH_TH_inches 24.58	

	// jwd put me back; right now it's constant alarmage	setupAlarmThreshold(DISTSENSOR1,DIST_LOW_TH_inches*100.,DIST_HIGH_TH_inches*100.);
	setupAlarmThreshold(CURRENTPROBE,CURRENT_LOW_TH_ma,CURRENT_HIGH_TH_ma);
}

void arduinoReset(int fd){
	char flushBuf[32];
	memset(flushBuf,0,sizeof(flushBuf));
	i2cWriteBlock(flushBuf,sizeof(flushBuf), arduinoI2Cfd);
	sleep(1);
}

void PrintSyntax( char *executableName ) {
	printf("\n  %s Version %.2f\n",executableName,VERSION);
	printf("\n  Options:");
	printf("\n  -t : set email notification \"to\" address");
	printf("\n  -f : set email notification \"from\" address");
	printf("\n  -l : local data directory");
	printf("\n  -v : set verbose mode");
	printf("\n  -? : program description\n");
}
int main( int argc, char * argv[] ) {
	const int deviceI2CAddress = 0x04;
#define RSP_BUF_SIZE 16	

	if ( (argc > 1) &&  (((argv[1][0] == '/') || (argv[1][0] == '-')) && (argv[1][1] == '?'))) {
		PrintSyntax(argv[0]);
		exit(-1);
	}

	/* now check for options */
	/* add 1 to index to account for program name*/

	if (argc > 1) { // don't attempt to read if no argument exists
		int numOptions=1;
		while (argc > numOptions) {
			switch (argv[numOptions][1]) {
				/* check to see if user wants associated file names at the top of */
				/* each column */

				case 't':
					if ( argc >= numOptions) {
						numOptions++;
						emailAddressTo = argv[numOptions];
						printf("Sending email notifications to: %s\n",emailAddressTo);
						numOptions++;
					}
					else {
						PrintSyntax(argv[0]);
						exit(-1);
					}
					break;
				case 'f':
					if ( argc >= numOptions) {
						numOptions++;
						emailAddressFrom = argv[numOptions];
						printf("Sending email notifications from: %s\n",emailAddressFrom);
						numOptions++;
					}
					else {
						PrintSyntax(argv[0]);
						exit(-1);
					}
					break;
				case 'l':
					if ( argc >= numOptions) {
						numOptions++;
						logFolder = argv[numOptions];
						printf("Using %s for log directory\n",logFolder);
						if (setupLogDir(logFolder) < 0){
							exit(-1);
						}

						numOptions++;
					}
					else {
						PrintSyntax(argv[0]);
						exit(-1);
					}
					break;
				default:
					printf("\n%c%c is not a valid option",argv[numOptions][0],argv[numOptions][1]);
					PrintSyntax(argv[0]);
					exit(-1);
					break;
			}
		}
	}
	else {
		PrintSyntax(argv[0]);
		exit(-1);
	}

	sendmail(emailAddressTo,emailAddressFrom,"sump monitor info: started","monitoring started");
	/* set up ring buffer for running avg calc */
	setupSumpData();

	pthread_t arduinoIntThread;
	initRequestList (&reqList, handle_event);
	pthread_create(&arduinoIntThread,NULL, handle_requests_loop,(void *)&reqList);

	arduinoI2Cfd = wiringPiI2CSetup (deviceI2CAddress);

	if (wiringPiSetup () < 0) {
		fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
		return -1;
	}
	/* setup up pin that indicates data is ready to be read from arduino */
	pinMode(ARDUINO_I2C_READY,INPUT);
	/* reset arduino so we start in known state */
	arduinoReset(arduinoI2Cfd);

	// set Pin 4 to generate an interrupt on rising edge and attach arduinoInterrupt() to the interrupt
	if ( wiringPiISR (ARDUINO_INT_PIO, INT_EDGE_RISING, &arduinoInterrupt) < 0 ) {
		fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
		return -1;
	}

	/* make sure we get any pending events */
	add_request(&reqList,GET_EVENT, &reqList.request_mutex, &reqList.got_request);

	add_request(&reqList,INIT, &reqList.request_mutex, &reqList.got_request);

	/* start timers */
	create_timer(DISTANCE_INTERVAL,distance_thread);
	create_timer(CURRENT_INTERVAL,current_thread);
	create_timer(TEMP_HUMIDITY_INTERVAL,temp_humidity_thread);
	(void)pthread_join(arduinoIntThread,NULL);
}
