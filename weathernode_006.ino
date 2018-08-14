
/************************************************************************************
 Ver 0006

 Implements HTTP Update and subsribes to nodered looking for update. If Update == 1, then 
 calls checkForUpdate.

 For some reason, it would not work with Upgrade vs Update and I had to add this to main loop:


if (!client.connected()) {
      reconnect();
    }

    
    Serial.println(F("done with if not client.connect"));

    // Added these 4 lines for Update subscription to work, Upgrade never worked
    delay(500);
    client.subscribe("wemosTestClient/setSleepTime");
    client.loop();
    client.subscribe("wemosTestClient/Update");
    client.loop();
 
 {
  "firmware": [
    {
      "version": "0.0.0.2",
      "platforms": "2",
      "customers": "B&T",
      "description": "Home Weather Nodes",
      "url": "http://HomePi/WeatherNodes/WeatherNode_0.0.0.2",
      "build_epoch": "",
      "md5": "26800fc59dc5091d06e6c01556f0e551",
      "prereq": "0.0.0.1"


For now, just use flat txt with version string, add other items and format in json later

Using this link as a starting point (already had it working in POC sketch httpUpdate4.ino
https://www.bakke.online/index.php/2017/06/02/self-updating-ota-firmware-for-esp8266/

that example uses MAC address to identify the unique sketch, I'll try using the 
pubsub ClientID and using node red to tell the device when to check for an update. this should
default to "do not check" on bootup. later add md5 checksum check -
  if the checksum is the same, do not upgrade and send message "versions same,not upgrading"
  
07/27/2018 
Creating WeatherNode_0003, this marks where we have a working POC OTA described above.
For now it checks updates every time through the loop, this version will start cleaning up 
the code then add node-red and pubsub callbacks to initiate the upgrade

Upto 0005 due to testing and cleanup

Steps to upgrade:
1: Create new build with changes and updated version string
2: Export binary and copy it to directory as (example)
pi@MesaPi1:/var/www/html/weathernodes/wemosTestClient $ ls wemosTestClient.bin
wemosTestClient.bin
example: firmware image trying to upload http://192.168.100.238/weathernodes/wemosTestClient/wemosTestClient.bin
NOTE: May change this to  include version string in build ie wemostTestClient_0004.bin
Later will have one weatherNode build once config file is done.
3: change .version file to updated version string

  
 **************************************************************************************/



#include <ESP8266WiFi.h>
#include <PubSubClient.h>  // for mqtt
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>  // This library is already built in to the Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <ESP8266mDNS.h> 
#include <ArduinoOTA.h>
#include <WiFiUdp.h> 
#include <TimeLib.h>
#include <NewPing.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
//#include <string.h>
//#include "WifiConfig.h"  Maybe try editing this to add wemos ?
#include <NtpClientLib.h>
#include <cactus_io_BME280_I2C.h>
//#define DEBUG 1
#define NTP_ON 1
//#define SERIAL_ON
extern "C" {
#include "gpio.h"
}

extern "C" {
#include "user_interface.h"
}

//***************** Define which station and platform we are compling for **************
#define Wemos 1
//#define TBOffice  1
//#define BethOffice 1
//#define Liv_Patio  1
//#define Danube 1  // The two outside temp sensor module
//***************** End define which station compiling for ****************

//************************ PIN DEFINITIONS **************************************
static const byte LCD_PIN =  D0;     // gate pin to control LCD was D3 10K ohm resistor
static const byte DST_PIN =  D5;     // gate pin to control distance sensor was D4 10K ohm resistor
static const byte SCL_PIN =  D1;     // i2c SCL
static const byte SDA_PIN =  D2;     // i2c SDA
static const byte ECHO_PIN = D7;     // USS ECHO Pin
static const byte TRIG_PIN = D8;     // USS TRIG Pin
static const byte ONEB_PIN = D6;     // One Wire buss 4.75K ohm resistor
static const byte PWR_PIN =  D3;     // Batt Charging yes/no was D0
static const byte DNE_PIN =  D4;     // Batt Charged yes/no was D0
static const byte BATT_PIN = A0;     // Batt Level 220k ohm resistor
//*********************** END PIN DEFINITIONS *****************************************




//****************** LCD & Dist Sensor transistor gate pins  **********************
//static const byte DST_PIN = D0;  // pin to drive gate pin of 2n222 for ultrasonic sensors - swapped with LCD pin for Ver L
//static const byte LCD_PIN = D4;   // pin to drive gate pin of 2n222 for LCD on/off - swapped with USS pin for Ver L
//********************* End gate pin section **************************************


//********************* BTTN_PIN Push LCD Section ***************************
//static const byte BTTN_PIN = D8; // our BTTN_PIN pin
//unsigned long BTTN_PINPushedMillis; // when BTTN_PIN was released
unsigned long lcdTurnedOnAt; // when lcd was turned on
int turnOnDelay = 500; // wait to turn on LCD
int turnOffDelay = 5000; // turn off LED after this time
int turnOnLcdDelay = 500; // wait to turn on LED
int turnOffLcdDelay = 5000; // turn off LED after this time
bool lcdReady = false; // flag for when BTTN_PIN is let go
bool lcdState = false; // for LCD is on or not.
//**************** END BTTN_PIN Push LCD Section ********************************


//*********************** HC-S04 UltraSonic Sensor ********************************#
//static const byte TRIG_PIN = D8;
//#define ECHO_PIN     D5
//static const byte ECHO_PIN = D5;
unsigned long sensorInRangeMillis; // when in sensor range
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define MAX_DIST 60
#define MIN_DIST 20
unsigned long pingSpeed = 2500; //How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;      // Holds the next ping time
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximu
//************************* END HC-S04 UltraSonic Sensor **************************#



//********************* Global BME sensor values *************************
// FIXME - Do I really need to save globals? Just create lcdPrintBme()
float lastBmePressure;
float lastBmeTempF;
float lastBmeHumidity;
char tubTempFString[6];
char outTmpFString[6];

//************************* End BME sensor globals ******************

//******************** Battery Charging Status Pins and State Variables ***********
//static const byte PWR_PIN = D3;   // Indicates if power is being supplied to charger board
//static const byte DNE_PIN = D1;  // Indicates if battery is fully charged
//************************* End Battery Charging Status **************************


//*********************** NTP SECTION *******************************
static const byte timeZone = -8; // PST

// Start NTP only after IP network is connected
void onSTAGotIP(WiFiEventStationModeGotIP ipInfo) {
  Serial.printf("Got IP: %s\r\n", ipInfo.ip.toString().c_str());
}

// Manage network disconnection
void onSTADisconnected(WiFiEventStationModeDisconnected event_info) {
  Serial.printf("Disconnected from SSID: %s\n", event_info.ssid.c_str());
  Serial.printf("Reason: %d\n", event_info.reason);
  NTP.stop();

}
#ifdef NTP_ON
void processSyncEvent(NTPSyncEvent_t ntpEvent) {
  if (ntpEvent) {
    Serial.print(F("Time Sync error: "));
    if (ntpEvent == noResponse)
      Serial.println("NTP server not reachable");
    else if (ntpEvent == invalidAddress)
      Serial.println(F("Invalid NTP server address"));
  }
  else {
    Serial.print(F("Got NTP time: "));
    Serial.println(NTP.getTimeDateString(NTP.getLastNTPSync()));
  }
}

boolean syncEventTRIGed = false; // True if a time event has been TRIGed
NTPSyncEvent_t ntpEvent; // Last TRIGed event
#endif
//***************************** END NTP GLOBAL SECTION ********************************


//************************** IP, HOSTNAME, VERSION, WIFI  ********************************
// Static IP Addressing
#ifdef Wemos
IPAddress ip(192, 168, 100, 139);
#elif defined TBOffice
IPAddress ip(192,168,100,181);
#elif defined BethOffice
IPAddress ip(192,168,100,123);
#elif defined Liv_Patio
IPAddress ip(192,168,100,137);
#endif
IPAddress gateway(192, 168, 100, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 100, 1);
const char* ssid = "BlueZebra";
const char* password = "jY7bzy2XJv";
//const char* IpAddr;
const char* Version = "0006";
//const int Version = 0001;
#ifdef Wemos
const char* ahostname = "Wemos";
#elif defined TBOffice 
const char* ahostname = "TbOffice_Test5";
#elif defined BethOffice 
const char* ahostname = "BethOffice";
#elif defined Liv_Patio 
const char* ahostname = "Liv_Patio";
#endif

const char* fwUrlBase = "http://192.168.100.238/weathernodes/";

//********************** End IP, HOSTNAME, VERSION, WIFI Defines*********************************



//*************************** ADC VOLTAGE ***********************************
// Unique for each chip, have to manually calibrate
//#define ADC_ADJUST 179.33  // Using a 220K resistor  .07 to low
//#define ADC_ADJUST 169.33  // Using a 220K resistor  .2 to high
#define ADC_ADJUST 175.33  // Using a 220K resistor, 4.02 = 4.02 = exact at 4.15 V - now about .2 high
//*************************** END ADC VOLTAGE ******************************


//********************** PUBSUB AND NODERED SETUP ********************************
#ifdef Wemos
WiFiClient(wemosTestClient);
PubSubClient client(wemosTestClient);
const char* Client = "wemosTestClient/";
#elif defined TBOffice
WiFiClient TbOfficeClient;
PubSubClient client(TbOfficeClient);
const char* Client = "TbOfficeClient/";
#endif
// RPI ADDRESS (MQTT BROKER - MOSQUITO)
const char* mqtt_server = "192.168.100.238";
//********************** END PUBSUB AND NODERED SETUP **************************

//************************ WiFi MILLIS & SLEEP DEFFINITIONS **********************
bool awake = true;
unsigned long previousMillis = 0;
unsigned long sleepStartMillis = 0;
//int sleeptime = 40; // initial sleep seconds, was byte
byte sleeptime = 40; // range 1 - 255 or about 3 minutes sleeptime
byte numReadings = 2;
//************************ END MILLIS SLEEP DEF"S ********************************

 
// *********************** ONE WIRE SETUP **************************************************
#ifdef Wemos
  DeviceAddress outAddr={0x28,0xFF,0x4F,0x57,0x73,0x16,0x04,0xBC}; // real wemos
  //DeviceAddress outAddr={0x28,0xFF,0x3C,0x20,0x72,0x16,0x04,0x1A};   // test wemos
#elif defined TBOffice
  DeviceAddress tubAddr = {0x28, 0xFF, 0x8D, 0xE7, 0x73, 0x16, 0x03, 0x0F};
  DeviceAddress outAddr = {0x28, 0xFF, 0xCC, 0x88, 0x72, 0x16, 0x03, 0x8A}; // Clone address's
#elif defined BethOffice
  //DeviceAddress outAddr={0x28,0xFF,0x3C,0x20,0x72,0x16,0x04,0x1A};  // Beth office outside old short
  DeviceAddress outAddr={0x28,0x75,0x64,0x37,0x08,0x00,0x00,0x88}; // Beth office outside new long
#elif defined Liv_Patio
  DeviceAddress outAddr={0x28,0xBA,0x14,0xB4,0x07,0x00,0x00,0x050};  // Patio address
  DeviceAddress tubAddr={0x28,0xA2,0x61,0xC3,0x06,0x00,0x00,0x90};  //  tub address
#endif
//DeviceAddress tubAddr = {0x28, 0xFF, 0x8D, 0xE7, 0x73, 0x16, 0x03, 0x0F}; // Clone address's
//DeviceAddress outAddr={0x28,0xBA,0x14,0xB4,0x07,0x00,0x00,0x050};  // out living address
//DeviceAddress tubAddr={0x28,0xA2,0x61,0xC3,0x06,0x00,0x00,0x90};
//DeviceAddress tubAddr={0x28,0xFF,0x3C,0x20,0x72,0x16,0x04,0x1A};
//DeviceAddress tubAddr={0x28,0xA7,0x61,0xC3,0x06,0x00,0x00,0x89};
//DeviceAddress outAddr = {0x28, 0xFF, 0xCC, 0x88, 0x72, 0x16, 0x03, 0x8A}; // Clone address's

//#define ONEB_PIN D4 // D2 on Wemos
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire SensorsPin(ONEB_PIN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature Sensors(&SensorsPin);



//**************************** END ONE WIRE SETUP ********************************************


//********************** I2C FOR LCD AND BME ******************************************
#define BME_ADJUST -1
LiquidCrystal_I2C lcd(0x27, 20, 4);
// Create the BME280 Object
BME280_I2C bme(0x76);  // I2C using address 0x76


//********************** END I2C SETUP *****************************************

/****************** PUBSUB  MQTT/NODE RED FUNCTIONS ***************************************
 This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
 Change the function below to add logic to your program, so when a device publishes a message to a topic that
 your ESP8266 is subscribed you can actually do something
 ***********************************************************************************************/

/*************************************************************************************
 *    takes two args, client and topic
 *    client = ahostname
 *    topic = the topic to publish
 *    
 *************************************************************************************/
void publishTopic(char topic[])
{
  char Topic[32];       // Make it big enough
  //char* Topic = ahostname;

  //strcpy(Topic, client);
  strcpy(Topic, Client);
  strcat(Topic, topic);
  client.publish(Topic, topic);
  
}


//************************ HTTP OTA UPDATE  ***********************************************

void checkForUpdates() {
  //String mac = getMAC();
  //Use pubsub Client instead of MAC
  String ClientID = String(Client);
  String fwURL = String( fwUrlBase );
  //fwURL.concat( mac );
  fwURL.concat(ClientID);
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );

  Serial.println( F("Checking for firmware updates."));
  //Serial.print( "MAC address: " );
  Serial.print(F( "ClientID: "));
  Serial.println( ClientID );
  //Serial.println( mac );
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );

  HTTPClient httpClient;
  httpClient.begin( fwVersionURL );
  int httpCode = httpClient.GET();
  if( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();

    Serial.print( "Current firmware version: " );
    //Serial.println( FW_VERSION );
    Serial.println(Version);
    Serial.print( "Available firmware version: " );
    Serial.println( newFWVersion );

    int newVersion = newFWVersion.toInt();
    int oldVersion = atoi(Version);
    
    if( newVersion > oldVersion ) {
      Serial.println( "Preparing to update" );

      String fwImageURL = fwURL;
      
      fwImageURL.concat(Client);
      fwImageURL.remove(fwImageURL.length()-1);
      fwImageURL.concat( ".bin" );
      Serial.print(F("firmware image trying to upload "));
      Serial.println(fwImageURL);
      
      t_httpUpdate_return ret = ESPhttpUpdate.update( fwImageURL );

      switch(ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
      }
    }
    else {
      Serial.println( "Already on latest version" );
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
  }
  httpClient.end();
}
//************************** END HTTP OTA UPDATE **************************************


//************************ PubSub Callbacks - Reconnect etc *******************************************

void callback(char* topic, byte* message, unsigned int length) {

  char TOPIC[64];
  char sleepTime[5];
  char temp[4];
  char *ret;
  byte count = 0; 
  
  Serial.println();
  Serial.print(F("!!!!!!!Message arrived on topic: "));
  Serial.println(topic);
  Serial.println(F("Building message"));
  Serial.print(F("Message =  "));
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
   }

  Serial.println();

  strcpy(TOPIC, Client);

 /*********************** setSleepTime section ********************************/
   //strcat(TOPIC, "Upgrade");
   strcat(TOPIC, "setSleepTime");
   
   if (strcmp(topic, TOPIC)== 0 ) {
    
    //Serial.println(F("!!!*** Matched on Upgrade ********!!!"));
    Serial.println(F("!!!*** Matched on setSleepTime ********!!!"));
    
    Serial.print(F("Current sleeptime = "));
    Serial.println(sleeptime);
    Serial.print(F("changing sleeptime to: "));
    for (int i = 0; i < length; i++) {
        sleepTime[i] = (char)message[i];
        count++;
     }
     
    sleepTime[count] = '\0';
    Serial.println(sleepTime);
    sleeptime = atoi(sleepTime);  // Change global sleeptime 
  
  }  // endif strcmp(topic,TOPIC) == 0) {
    Serial.print(F("Just before checking for upgrade, topic = "));
    Serial.println(topic);
    
    /********************  Upgrade section *************************/

    strcpy(TOPIC, "");
    strcpy(TOPIC, Client);
    //strcat(TOPIC, "setSleepTime");
    strcat(TOPIC, "Update");

    Serial.print(F("In Upgrade section, looking to match on "));
   // Serial.print(F("In setSleepTime section, looking to match on "));
    Serial.println(TOPIC);

    if (strcmp(topic, TOPIC) == 0 ) {
    //Serial.println(F("!!!*** Matched on setSleepTime *******!!!"));
    Serial.println(F("!!!*** Matched on Update *******!!!"));
    if((char)message[0] == '1') {
      checkForUpdates();
    }
    }
    else {
      Serial.print(F("TOPIC we are looking for is "));  
      Serial.println(TOPIC);
      Serial.print(F("Topic we recieved is "));
      Serial.println(topic);
      
    }
  
   //else ( Serial.println(F("Did not match on any callback functions other than Sleeptime!!")));
   }
//}


void reconnect() {

  char Topic[64];
  strcpy(Topic,Client);
  //strcat(Topic, "#");

  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect(Client)) {
      Serial.print(F("!!!!! In recconect, connected to "));
      Serial.println(Client);
      Serial.print(F("Building Topic with Client "));
      Serial.println(Topic);
      Serial.println(Client);
      /****************  Subscribe to Topic setSleeptime **********************/
      strcat(Topic,"setSleepTime");
      Serial.print(F("now added setSleepTime, Topic = "));
      Serial.println(Topic);
      client.subscribe(Topic);
      client.loop();
      //client.subscribe(Topic);  // Don't know why but I need to subscribe twice for setSleeptime but I do
      Serial.print(F("subscribed to "));
      Serial.println(Topic);
     // client.connect(Client);
      /*************************Subscribe to Topic Upgrade ***********************************/
      delay(100);
      client.loop();
      
      Serial.println(F("Attempting to subscribe to Upgrade Topic"));
      strcpy(Topic, Client);
      strcat(Topic, "Update");
       Serial.print(F("now added Upgrade, Topic = "));
      Serial.println(Topic);
      //client.subscribe(Topic);
      Serial.print(F("Subscribing to topic "));
      Serial.println(Topic);
        if (client.subscribe(Topic)){
          client.loop();
         // if (client.subscribe("wemosTestClient/Upgrade")){
           Serial.print(F("Successfully subscribed to "));
           Serial.println(Topic);
           //client.connect(Client);
           }
      else {
        Serial.print(F("FAILED SUBSCRIBE to "));
        Serial.println(Topic);    
      
      }
     // client.subscribe(Topic);
      
            
    }
  
    else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    } 
    
  }
}

//******************* END PUBSUB FUNCTIONS ************************************


//******************* GET BATTERY LEVEL  **************************************
float getBatteryLevel() {
  //float level = analogRead(A0) / ADC_ADJUST;
  float level = analogRead(BATT_PIN) / ADC_ADJUST;
#ifdef SERIAL_ON
  Serial.print("Battery level: "); Serial.print(level); Serial.println("%");
 #endif
return level;

}

//************************ END GET BATTERY LEVEL *********************************


//*********************** OTA SETUP ******************************************
//  Can I move stuff from Setup to here?
//************************* END OTA SETUP ***************************************



//********************************** LCD Functions ***************************

void lcdOn(void)
{
  digitalWrite(LCD_PIN, HIGH); // added this using transistor
  lcd.init();
  lcd.backlight();
  //lcd.display();

}

void lcdOff(void)
{
 lcd.noDisplay();
 lcd.noBacklight();
 digitalWrite(LCD_PIN, LOW);  // added this using transistor
}

//**************************** LCD Setup *********************************
//                    Run's once in setup
//     Display hostname, version and ip info then turns off display
//***********************************************************************

void lcdSetup(void)
{
  //digitalWrite(LCD_PIN, HIGH);
  //lcd.init();   // initializing the LCD
 // lcd.backlight(); // Enable or Turn On the backlight
  //lcd.display();
  lcdOn();
  lcd.setCursor(0, 0);
  lcd.print(F("Hostname"));
  lcd.print(ahostname);
  lcd.setCursor(0, 1);
  lcd.print(F("Ver: "));
  lcd.print(Version);
  /*
  #ifdef Danube
  lcd.setCursor(0, 2);
  lcd.print(F("I'm a Danube!"));
  #else
  lcd.print(F("I'm just a Nile"));
  #endif
  */
  lcd.setCursor(0, 3);
  lcd.print(F("Ip "));
  lcd.print (WiFi.localIP());
  delay(5000);
  lcdOff();
}


//*********************** lcdDisplayTemps2 *********************
//             Displays Sensor readings when BTTN_PIN pressed
//**************************************************************
void lcdDisplayTemps2()
{
  // Add LCD state machine stuff here
  digitalWrite(LCD_PIN, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("B&T Home TMP Guage"));
  //lcd.setCursor(0,1);
 // lcd.print("Tub Temp: ");
  //lcd.print(tubTempFString);
  //lcd.print(" *F");
  lcd.setCursor(0, 1);
  lcd.print(F("Out Temp: "));
  lcd.print(outTmpFString);
  lcd.print(F(" *F"));
  lcd.setCursor(0, 2);
  lcd.print(F("In  Temp: "));
  lcd.print(lastBmeTempF);
  lcd.print(F(" *F"));
  lcd.setCursor(0, 3);
  lcd.print(F("P:  "));
  //lcd.print(bme.getPressure_MB()/33.8693);
  lcd.print(lastBmePressure);
  lcd.print(F(" H: "));
  lcd.print(lastBmeHumidity);
}
//}

//***************** END LCD FUNCTIONS **********************************
#ifdef SERIAL_ON
void serialPrintReadings(void)
{
  serialPrintNTP();
  serialPrintBme();
  serialPrintHeap();
  serialPrintResetReason();
  serialPrintVcc();
}
#endif

void publishReadings(void)
{
  // MQTT Publish readings
  publishBme();
  publishVcc();
  publishStaInfo();
  publishNTP();
  publishHeap();
  publishResetReason();
  publishDallasTemps();
  checkChargerStatus();
  publishSleepTime();
//void publishImAwake();
  publishSleepState(0);
}

#ifdef SERIAL_ON
void serialPrintBme(void)
{
  bme.readSensor();
  Serial.print(bme.getPressure_MB() / (33.8639)); Serial.print(F("\t\t"));
  Serial.print(bme.getHumidity()); Serial.print(F("\t\t"));
  Serial.print(bme.getTemperature_C()); Serial.print(F(" *C\t"));
  Serial.print(bme.getTemperature_F()); Serial.println(F(" *F\t"));
}

#endif

#ifdef SERIAL_ON
void serialPrintNTP(void)
{
  Serial.print(NTP.getTimeDateString()); Serial.print(F(" "));
  Serial.print(NTP.isSummerTime() ? "Summer Time. " : "Winter Time. ");
  Serial.print(F("WiFi is "));
  Serial.print(WiFi.isConnected() ? "connected" : "not connected"); Serial.print(F(". "));
  Serial.print(F("Uptime: "));
  Serial.print(NTP.getUptimeString()); Serial.print(F(" since "));
  Serial.println(NTP.getTimeDateString(NTP.getFirstSync()).c_str());
  Serial.println("Free Heap      = " + String(ESP.getFreeHeap()));
  Serial.println("Last Reset Reason      = " + String(ESP.getResetReason()));
}

#endif

#ifdef SERIAL_ON
void serialPrintHeap(void)`

{
  Serial.println("Free Heap      = " + String(ESP.getFreeHeap()));
}
#endif

#ifdef SERIAL_ON
void serialPrintResetReason(void)
{
  Serial.println("Last Reset Reason      = " + String(ESP.getResetReason()));
}
#endif

#ifdef SERIAL_ON

void serialPrintVcc(void)
{
  char VCC[7];
  float vdd = getBatteryLevel();
  dtostrf(vdd, 6, 2, VCC);
  Serial.print(F("VCC voltage = "));
  Serial.print(VCC);
  Serial.println(F(" Vdc\t"));

}
#endif
 
void publishVcc(void)
{
  char VCC[7];
  char Topic[32];

  strcpy(Topic, Client);
  strcat(Topic, "VCC");
  
  float vdd = getBatteryLevel();
  dtostrf(vdd, 6, 2, VCC);
  //client.publish("/TbOfficeClient/VCC", VCC);
  client.publish(Topic, VCC);
}

void publishSleepTime(void) {
  char sleepTime[12];
  char Topic[32];
  char temp[4];

  strcpy(sleepTime, itoa(sleeptime, temp, 10));
  strcpy(Topic, Client);
  strcat(Topic, "sleepTime");
  
  //client.publish(Topic, itoa(sleeptime/1000, sleepTime, 10));
  //client.publish(Topic, itoa(sleeptime, sleepTime, 10));
  client.publish(Topic,sleepTime);
  Serial.print(F("sleeptime now equals "));
  Serial.println(sleeptime);
 // client.publish(Topic, sleeptime);
  
}


void pubTopic(char Topic[], char Payload[])
 {
  char topic[32];
  char payload[32];
  strcpy(topic, Client);
  strcpy(payload, Payload);
  strcat(topic, Topic);
  client.publish(topic, payload);
  }

  //FIXME - make pubTopioc work!
char* makeTopic(char* value)
  {
 // char topic[32];
  char* topic;
  strcpy(topic, Client);
  strcat(topic, value);
  return(topic);
  }


 

void publishStaInfo(void)
{
  char Topic[32];

  // Publish version
  strcpy(Topic, Client);
  strcat(Topic, "Version");
  client.publish(Topic, Version);
  
  // Publish Hostname
  strcpy(Topic,Client);
  strcat(Topic, "Hostname");
  client.publish(Topic, ahostname);

  // Publish IP
  strcpy(Topic,Client);
  strcat(Topic, "IP");
  client.publish(Topic,  WiFi.localIP().toString().c_str());
  
  // Publish MAC
  strcpy(Topic,Client);
  strcat(Topic, "MAC");
  client.publish(Topic, WiFi.macAddress().c_str());
 
}


void publishBme(void)
{
  char Topic[32];

  //strcpy(Topic, Client);
  //strcat(Topic, "Version");
  // supposedly BME re-uses the last reading so take 2 readings
  bme.readSensor();
  bme.readSensor();
  bme.readSensor();
  // Get floats from BME
  float P = bme.getPressure_MB() / 33.8639;
  float T = bme.getTemperature_F();
  float H = bme.getHumidity();
  T = T - 2;  // BME adjust temp

  // Create character strings for MQTT publish
  static char bmeTemperatureF[7];
  static char bmePressure[7];
  static char bmeHumidity[7];

  // Convert Float (returned from bme.readsensor)  to String to publish
  dtostrf(T, 6, 2, bmeTemperatureF);
  dtostrf(P, 6, 2, bmePressure);
  dtostrf(H, 6, 2, bmeHumidity);

  //String currDate = NTP.getTimeDateString();

  strcpy(Topic, Client);
  strcat(Topic, "temperature");
  client.publish(Topic, bmeTemperatureF);

  strcpy(Topic, Client);
  strcat(Topic, "humidity");
  client.publish(Topic, bmeHumidity);

  strcpy(Topic, Client);
  strcat(Topic, "pressure");
  client.publish(Topic, bmePressure);
  
  // Save last bme readings for LCD display when wifi not up
  lastBmePressure = P;
  lastBmeTempF = T;
  lastBmeHumidity = H;

}


void publishNTP(void)
{

  char Topic[32];
  
  //  NTP returns type String, bummer!
  String currDate = NTP.getTimeDateString();
  String upTime = NTP.getUptimeString();
  String bootDate = (NTP.getTimeDateString(NTP.getFirstSync()).c_str());

  // Build Topic for Pubsub publish, Start with Client and add topic then publish Topic, value
  
  //char Uptime[50]; 
  //char CurrTime[50]; 
  //char BootTime[50];

  // uptime
  strcpy(Topic,Client);
  strcat(Topic, "upTime");
  client.publish(Topic, upTime.c_str()); 
  
  // Current time
  strcpy(Topic,Client);
  strcat(Topic, "currTime");
  client.publish(Topic, currDate.c_str()); 
  
  // Boot Time
  strcpy(Topic,Client);
  strcat(Topic, "bootTime");
  client.publish(Topic, bootDate.c_str()); 
}


void publishHeap(void)
{
  char freeHeap[8];
  char Topic[32];

  float FH = ESP.getFreeHeap();
  dtostrf(FH, 9, 2, freeHeap);
  
  strcpy(Topic, Client);
  strcat(Topic, "freeHeap");
 
  client.publish(Topic, freeHeap);

}



void publishResetReason(void)
{
  char Topic[32];
  String lastResetReason = (ESP.getResetReason().c_str());
  
  strcpy(Topic, Client);
  strcat(Topic, "lastResetReason");
  
  client.publish(Topic,lastResetReason.c_str());
  
 
}

// Let NodeRed know we are awake
/*
void publishImAwake(void)
{
  char Topic[32];
  
  strcpy(Topic, Client);
  strcat(Topic, "ImAwake");
  
  client.publish(Topic,"ImAwake");

  #ifdef SERIAL_ON
  Serial.print(F("Topic = "));
  Serial.println(Topic);
  Serial.println(F("Awake message = ImAwake"));
  #endif
}
*/

// Template for config change config options
void publishSleepState(int state)
{
  char Topic[32];
  
  strcpy(Topic, Client);
  strcat(Topic, "SleepState");

  if (state == 0){
     client.publish(Topic,"Awake");
     #ifdef SERIAL_ON
        Serial.print(F("Topic = "));
        Serial.println(Topic);
        Serial.println(F("Sleep State message = Awake"));
     #endif
  }
  else {
     client.publish(Topic,"Sleeping");
     #ifdef SERIAL_ON
        Serial.print(F("Topic = "));
        Serial.println(Topic);
        Serial.println(F("Sleep State message = Sleeping"));
     #endif
 }

 // #ifdef SERIAL_ON
 // Serial.print(F("Topic = "));
  //Serial.println(Topic);
 // Serial.println(F("Sleep State message = Awake"));
 // #endif
  
}

void publishImAsleep(void)
{
  char Topic[32];
  
  strcpy(Topic, Client);
  strcat(Topic, "ImAwake");
  
  client.publish(Topic,"ImAsleep");

 // #ifdef SERIAL_ON
  Serial.print(F("Topic = "));
  Serial.println(Topic);
  Serial.println(F("Awake message = ImAwake"));
 // #endif
  
}
/*
void publishGetSleepTime(void)
{
  char Topic[32];
  
  strcpy(Topic, Client);
  strcat(Topic, "getSleepTime");
  
  client.publish(Topic,"getSleepTime");

 // #ifdef SERIAL_ON
  Serial.print(F("Topic = "));
  Serial.println(Topic);
  Serial.println(F("message = getSleepTime"));
 // #endif
   
}
*/

void publishDallasTemps(void)
{
  char Topic[32];
  //strcpy(Topic, Client);
  float tempC;
  float tempF;

  char tubTempCString[6];
  //char tubTempFString[6];

  char outTmpCString[6];
  //char outTmpFString[6];

  Sensors.requestTemperatures();
  do {
   //#ifdef Tub_Sensor
   
   #ifdef Danube  // We have a second outside temp one wire sensor
      // Tub Temp
      tempC = Sensors.getTempC(tubAddr);
      dtostrf(tempC, 2, 2, tubTempCString);
      tempF = Sensors.getTempF(tubAddr);
      dtostrf(tempF, 3, 2, tubTempFString);
     #endif
    
    // Outside Temp
    tempC = Sensors.getTempC(outAddr);
    dtostrf(tempC, 2, 2, outTmpCString);
    tempF = Sensors.getTempF(outAddr);
    dtostrf(tempF, 3, 2, outTmpFString);

  } while (tempC == 85.0 || tempC == (-127.0));

  #ifdef Danube 
  strcpy(Topic, Client);
  strcat(Topic, "Tub/temperature");
  //client.publish("/TbOfficeClient/Tub/temperature", tubTempFString);
  client.publish(Topic, tubTempFString);
   
  #endif
  strcpy(Topic, Client);
  strcat(Topic, "Out/temperature");
  client.publish(Topic, outTmpFString);
}


unsigned int checkDistance()
{
  digitalWrite(DST_PIN, HIGH);
  delay(800);
  //Serial.println(F("Turning DST_PIN HIGH"));
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  uS = sonar.ping();
  unsigned int Tmp = uS / US_ROUNDTRIP_CM;  // US_ROUNDTRIP_CM included in lib ?


  //***** Find a way to ping x times, break if Distance in range 
  if ( Tmp >= MAX_DIST || Tmp <= MIN_DIST) {
    //Serial.print(F("sonar distance = "));
   //Serial.println(Tmp);
    //Serial.println(F("Out of Range, LCD is OFF"));
   // Serial.println(F("DST PIN = LOW"));
    //digitalWrite(DST_PIN, LOW);
    lcdReady = false;
  }
  else {
    sensorInRangeMillis = millis();
    lcdReady = true;
    //Serial.println(F("Set lcdReady to True, DST PIN to LOW"));
    //digitalWrite(DST_PIN, LOW);
  }
  digitalWrite(DST_PIN, LOW);
  return (Tmp);
  
}


/**************************************************************************
                             SETUP!
 **************************************************************************/
void setup() {
 //strcpy(sleeptime, "40");
  //pinMode(BTTN_PIN, INPUT_PULLUP);    // D8
  pinMode(DST_PIN, OUTPUT);
  pinMode(PWR_PIN, INPUT_PULLUP);   // D3
  pinMode(DNE_PIN, INPUT_PULLUP);  // D2
  pinMode(LCD_PIN, OUTPUT);
  digitalWrite(LCD_PIN, HIGH);
  digitalWrite(DST_PIN, LOW); // Just added this 12/31
  //Wire.begin(SDA_PIN, SCL_PIN);
 // Wire.begin();
  //lcdSetup();
  //strcpy(sleeptime, 40);
  Serial.begin(115200);
  delay(10);



  //***************************  OTA *****************************************************
  //*********** Start OTA stuff  Then finish  wifi connect ******************
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);                                                    

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("TrentOffice");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println(F("Starting OTA"));
  });
  ArduinoOTA.onEnd([]() {
    Serial.println(F("\nEnding OTA"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  //************************************** END OTA IN SETUP *****************************
  Wire.begin();
  // I2C setuf for SDA= D7,                                                                                                                  = D8
  //Wire.begin(SDA_PIN, SCL_PIN);
  //lcdSetup();
  WiFi.hostname( ahostname );
  Sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement

  // **************** WIFI connect section ***************
  Serial.println();
  Serial.print(F("Connecting to "));                                                                   
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);   // try uncommenting if OTA does not work
  WiFi.persistent(false);
  WiFi.config(ip, gateway, subnet, dns);
  WiFi.begin(ssid, password);

  Serial.print(F("Connecting")); //vdd == getBatteryLevel();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("."));                                      
    delay(200);
  }
  // Added this to circumvent Event stuff in VH
  NTP.begin("pool.ntp.org", timeZone, true);
  NTP.setInterval(63);

                     
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println(F("Connection Failed! Rebooting..."));
    delay(5000);
    ESP.restart();
  }

  lcdSetup();
  ArduinoOTA.begin();

  Serial.println(F(""));
  Serial.println(F("WiFi connected"));



  // Printing the ESP IP address
  Serial.println(WiFi.localIP());

  Serial.print(F("Version: "));
  Serial.println(Version);

  //BME280 to serial
  Serial.println(F("Bosch BME280 Barometric Pressure - Humidity - Temp Sensor | cactus.io"));

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    // while (.8);
  }

  bme.setTempCal(BME_ADJUST);  // Adjust up or down

  Serial.println(F("Pressure\tHumdity\t\tTemp\t\tTemp"));


  // MQTT setup
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.println(F(" In setup, just set lcdState to 1"));

}



void checkChargerStatus(void)
{
  char Topic[32];
  
  // Charging State
  strcpy(Topic, Client);
  strcat(Topic,"pwrState");
  if (digitalRead(PWR_PIN) == LOW)
  {
    client.publish(Topic, "Charging");
   }
  else 
  {
    client.publish(Topic,"Not Charging");
  }

   // Battery Charged 
  strcpy(Topic, Client);
  strcat(Topic,"dneState");
  
  if (digitalRead(DNE_PIN) == LOW)
  {
    client.publish(Topic,"Charged");
   }
  else 
  {
    client.publish(Topic, "Not Charged");
     }
}



 /*****************************************************************************
                               MAIN LOOP
******************************************************************************/
void loop() {


  //Serial.println(F("At very top of loop"));
  unsigned long currentMillis = millis();
  unsigned int dS;
  sleepStartMillis = millis();
  //previousMillis = millis();
#ifdef NTP_ON
  if (syncEventTRIGed) {
    processSyncEvent(ntpEvent);
    syncEventTRIGed = false;
  }
#endif


  //************************ BTTN_PIN Push LCD Stuff **************************
 /*
  // check the BTTN_PIN
  if (digitalRead(BTTN_PIN) == LOW) {
    // update the time when BTTN_PIN was pushed
    BTTN_PINPushedMillis = currentMillis;
    //Serial.println("BTTN_PIN Pushed");
    lcdReady = true;
  }
  // make sure this code isn't checked until after BTTN_PIN has been let go
  if (lcdReady) {
    //this is typical millis code here:
    if ((unsigned long)(currentMillis - BTTN_PINPushedMillis) >= turnOnDelay) {
      // okay, enough time has passed since the BTTN_PIN was let go.
      //digitalWrite(LED, HIGH);
      // setup our next "state"
      lcdOn();
      lcdDisplayTemps2();
      lcdState = true;
      // save when the LED turned on
      lcdTurnedOnAt = currentMillis;
      // wait for next BTTN_PIN press
      lcdReady = false;
    }
  }

  // see if we are watching for the time to turn off LED
  if (lcdState) {
    // okay, lcd on, check for now long
    if ((unsigned long)(currentMillis - lcdTurnedOnAt) >= turnOffDelay) {
      lcdOff();
      lcdState = false;
      //digitalWrite(LED, LOW);
    }
  }

*/
 /**************************** Arm waving stuff **********************************/

 //sensorInRangeMillis = currentMillis;
 //uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
 //dS = uS / US_ROUNDTRIP_CM;
 //dS = getDistance();  // this worked

 if (millis() >= pingTimer) { //pingSpeed millis since last ping; do another ping
 // Serial.println(F("ABOUT TO DO CHECKDISTANCE"));
  //digitalWrite(DST_PIN, LOW);
  pingTimer += pingSpeed;   // set the next ping time
  dS = checkDistance();
  
 }
// make sure this code isn't checked until after the range sensor is activated
 if (lcdReady) {

  //Serial.println(F("!!!!!!!!!!!!!!IN IF(LCDREADY !!!!!!!!!!!!!!"));
   //this is typical millis code here:
    // FIXME!!
  // if ((unsigned long)(currentMillis - sensorInRangeMillis) <= turnOnLcdDelay) {
  //if ((unsigned long)(currentMillis - sensorInRangeMillis) >= 0) {
     // okay, enough time has passed since the button was let go.
     //digitalWrite(ULT_LED, HIGH);
     // setup our next "state"
     lcdState = true;
     // save when the LED turned on
     //sensorInRangeMillis = currentMillis;
     lcdTurnedOnAt = currentMillis;
     // wait for next arm wave
     lcdReady = false;
    // Serial.println(F("!!!!!!!!!! In IF ENOUGH TIME HAS PASSED !!!!!!!!!!!!!!!"));
     lcdOn();
     lcdDisplayTemps2();
 //  }
 }
  
 // see if we are watching for the time to turn off LCD
 if (lcdState) {
   // okay, lcd on, check for how long
   if ((unsigned long)(currentMillis - lcdTurnedOnAt) >= turnOffLcdDelay) {
     lcdState = false;
     //digitalWrite(ULT_LED, LOW);
     lcdOff();
   }
 }

  //******************* END LCD STUFF ************************************

  int x;  // for loop counter
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  //Serial.println(F("Just before if awake is true"));
  if (awake == true)
  {
   // Serial.println(F("awake = true, getting to work"));
    Serial.println("Free Heap      = " + String(ESP.getFreeHeap(), DEC));

    Serial.println(F("starting Wifi"));
    Serial.print(F("Connecting to "));
    Serial.println(ssid);
    // try commenting if OTA does not work

    //WiFi.config(ip, gateway, subnet, dns);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(F("."));
    }
   // Serial.println(F("awake set to true, then OTA handle and recconect"));
   // ArduinoOTA.handle();
   // Serial.println(F("done OTA.handle"));
    if (!client.connected()) {
      reconnect();
    }
    Serial.println(F("done with if not client.connect"));
    delay(500);
    client.subscribe("wemosTestClient/setSleepTime");
    client.loop();
    //delay(500);
    client.subscribe("wemosTestClient/Update");
    client.loop();
       
   //client.connect(Client);
    
    Serial.println(F("done with client.connect to "));
    //Serial.println(Client);
    Serial.println(F("starting Client.loop()"));
    client.loop();

    // This delay added so subscribe works, it used to work without it!
    delay(100);
    
    Serial.println("Free Heap      = " + String(ESP.getFreeHeap(), DEC));

    // Get x readings then sleep for 60 seconds
   
    for (x = 1; x <= numReadings; x++)
    {
      yield();
      // Serial Print data


//FIXME: Why is this not using previous defined functions?
  #ifdef SERIAL_ON
      serialPrintNTP();
      serialPrintBme();
      serialPrintHeap();
      serialPrintResetReason();
      serialPrintVcc();
      yield();
  #endif
      // MQTT Publish
      publishReadings();
      //checkForUpdates();

      //lcdDisplayTemps();
     // Serial.print(F(" End loop number: "));
     // Serial.println(x);
      yield();
    }// end for loop

    //******* reduced from delay(5000) to delay(3000) for OTA WDT
    delay(3000);
    publishSleepState(1);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);

    Serial.print(F("************ forceing modem sleep "));
    WiFi.forceSleepBegin();
    delay(1);
   // Serial.print(sleeptime);
    Serial.println(F("************"));
    Serial.print(F("Current sleeptime =  "));
    Serial.println(sleeptime);
    //publishImAsleep();
    //sleepStartMillis = millis();
    WiFi.forceSleepBegin();
    delay(1);
   // Serial.println(F("**************** Setting awake to False ***************"));
    awake = false;
    //e2 = WiFi.onStationModeDisconnected(onSTADisconnected);
  }// end if awake = true

  else {
    delay(1000);
   
    /*
    Serial.println(F("*************** Still sleeping but checking TIME **********************"));
    Serial.print(F("sleeping for  = "));
    //Serial.print(sleeptime / 1000);
    Serial.print(sleeptime);
    Serial.print(F(" seconds, Time Elapsed = "));
    //Serial.print((currentMillis - previousMillis)/1000);
    Serial.print((sleepStartMillis - previousMillis) / 1000);
    Serial.println(F(" seconds"));
    Serial.println(F("***********************************************************************"));
  }
  */

  //******************** WAKING UP ************************************
  //WiFi.persistent(false);
  //WiFi.mode(WIFI_OFF);
  // WiFi.mode(WIFI_STA);

  // had to add 10 seconds to sleeptime, not sure why
  if ((unsigned long)(currentMillis - previousMillis) >= (sleeptime * 1000 + 10000)) {
  //if ((unsigned long)(sleepStartMillis - previousMillis) >= sleeptime) {
  //  Serial.println(F("**************Time to Wake up and get to work!******************"));
   // Serial.println(F("************** awake = TRUE ******************"));
    awake = true;
    //count_wake += 1;
   // Serial.println(F("**************** Waking up WiFi ***************"));
    
    WiFi.forceSleepWake();
    //if(!client.loop())
    // THE SECTION IN reconnect() function should match the device name
    //client.connect("TbOfficeClient");

    previousMillis = millis();
  }

}   // end loop
}
