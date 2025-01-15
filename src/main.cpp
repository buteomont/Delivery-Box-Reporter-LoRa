#include <Arduino.h>

/**
 * This is an ESP8266 program to measure distance with an VL53L0X
 * infrared ranger, and report over LoRa whether or not some object
 * is within a specified distance window. Its purpose is to send a
 * notification when a package is placed in or removed from a
 * delivery box.
 * 
 * It utilizes the ESP8266's  
 * sleep mode to maximize battery life.  It will wake up at least
 * once per hour to let you know it's still alive.
 *
 * Configuration is done via serial connection.  Enter:
 *  broker=<broker name or address>
 *  port=<port number>   (defaults to 1883)
 *  topicroot=<topic root> (something like buteomont/gate/package/ - must end with / and 
 *  "present", "distance", "analog", or "voltage" will be added)
 *  user=<mqtt user>
 *  pass=<mqtt password>
 *  ssid=<wifi ssid>
 *  wifipass=<wifi password>
 *  mindistance=<minimum presence distance>
 *  maxdistance=<maximum presence distance>
 *  sleepTime=<seconds to sleep between measurements> (set to zero for continuous readings)
 * 
 *  ****** RYLR998 Commands ********
 * 
 *  This is a comprehensive list of AT commands for the RYLR998 LoRa module:
 * 
 * - Basic Commands
 * AT: Test if the module can respond to commands
 * AT+RESET: Perform a software reset of the module
 *
 * Configuration Commands
 * AT+MODE=<Parameter>[,<RX time>,<Low speed time>]: Set the wireless work mode
 *   Where:
 *   <Parameter> ranges from 0 to 2:
 *     0: Transceiver mode (default)
 *     1: Sleep mode
 *     2: Smart receiving mode for power saving
 *   <RX time>: 30ms to 60000ms (default is 1000ms)
 *     Used in Smart receiving mode to set the active receiving time
 *   <Low speed time>: 30ms to 60000ms
 *     Used in Smart receiving mode to set the low-power state duration
 * AT+IPR: Set the UART baud rate
 * AT+BAND: Set RF frequency in Hz
 * AT+PARAMETER=<Spreading Factor>,<Bandwidth>,<Coding Rate>,<Programmed Preamble>: Set the RF parameters 
 *   (spreading factor, bandwidth, coding rate, preamble)
 *   - Parameters
 *     - Spreading Factor (SF): Range 5-11 (default 9)
 *       Higher SF improves sensitivity but increases transmission time
 *       SF7 to SF9 at 125kHz, SF7 to SF10 at 250kHz, and SF7 to SF11 at 500kHz
 *     - Bandwidth (BW): 7-9
 *       7: 125 KHz (default)
 *       8: 250 KHz
 *       9: 500 KHz
 *       Smaller bandwidth improves sensitivity but increases transmission time
 *     - Coding Rate (CR): 1-4 (default 1)
 *       1: 4/5
 *       2: 4/6
 *       3: 4/7
 *       4: 4/8
 *       Lower coding rate is faster
 *     - Programmed Preamble: Default 12
 *       When NETWORKID=18, can be set from 4 to 24
 *       For other NETWORKID values, must be set to 12
 *       Larger preamble reduces data loss but increases transmission time
 * AT+ADDRESS: Set the ADDRESS ID of the module (0 to 65535)
 * AT+NETWORKID: Set the network ID (3 to 15, or 18 (default))
 * AT+CPIN: Set the domain password
 *   - The password must be exactly 8 characters long
 *   - Valid range: 00000001 to FFFFFFFF (hexadecimal)
 *   - Only modules using the same password can recognize and communicate with each other
 * AT+CRFOP: Set the RF output power (0 to 22 dBm)
 *
 * - Communication Commands
 * AT+SEND=<Address>,<Payload Length>,<Data>: Send data to the appointed address (250 bytes max)
 *   Use address 0 to broadcast to all addresses.
 *   For payloads greater than 100 bytes, the suggested setting is "AT+PARAMETER=8,7,1,12"
 *
 * - Query Commands
 * AT+UID?: Inquire module ID
 * AT+VER?: Inquire the firmware version
 *
 * - Other Commands
 * AT+FACTORY: Reset all parameters to manufacturer defaults
 *
 * - Response Formats
 * +RCV=<Address>,<Length>,<Data>,<RSSI>,<SNR>: Shows received data actively
 * +OK: Indicates successful command execution
 * +ERR: Indicates an error, followed by an error code
 * +READY: Reset was successful and waiting for a command
 *
 * You can also add a question mark at the end of most commands
 * to query their current settings. 
 *
 * - The Error Codes that can be returned are:
 * +ERR=1: There is no "enter" or 0x0D 0x0A (carriage return and line feed) at the end of the AT Command.
 * +ERR=2: The head of the AT command is not an "AT" string.
 * +ERR=4: Unknown command or the data to be sent does not match the actual length.
 * +ERR=5: The data to be sent does not match the actual length.
 * +ERR=10: TX is over time (transmission timeout).
 * +ERR=12: CRC error.
 * +ERR=13: TX data exceeds 240 bytes.
 * +ERR=14: Failed to write flash memory.
 * +ERR=15: Unknown failure.
 * +ERR=17: Last TX was not completed.
 * +ERR=18: Preamble value is not allowed.
 * +ERR=19: RX failed, Header error.
 * 
 * The data from the RYLR998 before processing is like this:
 * +RCV=3,46,{"DISTANCE":8123,"ISPRSENT":0,"BATTERY":3.41},-47,12
 *      a b  c                                              d   e
 * a=source address
 * b=data length
 * c=raw data
 * d=RSSI
 * e=SNR
 *
 * This program will receive a JSON object from the RYLR998 object that looks something like this:
 * {"address":2,"rssi":-23,"snr":3,"data":{"distance":8123,"ispresent":0,"battery":3.41}}
 * and send each field to the MQTT broker over WiFi.


 */

#define VERSION "24.11.09.0"  //remember to update this after every change! YY.MM.DD.REV
 
//#include <ESP8266WiFi.h>
#include "user_interface.h"
#include <EEPROM.h>
#include <VL53L0X.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <LoRa.h>
#include "RYLR998.h"
#include "delivery_reporter_lora.h"

VL53L0X sensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RYLR998 lora(LORA_RX_PIN, LORA_TX_PIN);
StaticJsonDocument<250> doc;

//WiFiClient wifiClient;
// PubSubClient mqttClient(wifiClient);

// These are the settings that get stored in EEPROM.  They are all in one struct which
// makes it easier to store and retrieve.
typedef struct 
  {
  unsigned int validConfig=0; 
  int mindistance=0;  // Item is present if distance is greater than this
  int maxdistance=400;// and distance is less than this
  int sleeptime=DEFAULT_SLEEP_TIME; //seconds to sleep between distance checks
  bool debug=false;
  bool displayenabled=true;    //enable the display
  bool invertdisplay=false;   //rotate display 180 degrees
  uint16_t loRaTargetAddress=DEFAULT_LORA_TARGET_ADDRESS;
  int loRaAddress=DEFAULT_LORA_ADDRESS;
  int loRaNetworkID=DEFAULT_LORA_NETWORK_ID;
  uint32_t loRaBand=DEFAULT_LORA_BAND; //(915000000);
  byte loRaSpreadingFactor=DEFAULT_LORA_SPREADING_FACTOR;
  byte loRaBandwidth=DEFAULT_LORA_BANDWIDTH;
  byte loRaCodingRate=DEFAULT_LORA_CODING_RATE;
  byte loRaPreamble=DEFAULT_LORA_PREAMBLE;
  uint32_t loRaBaudRate=DEFAULT_LORA_BAUD_RATE; //both for RF and serial comms
  uint8_t loRaPower=DEFAULT_LORA_POWER; //dbm
  } conf;

conf settings; //all settings in one struct makes it easier to store in EEPROM
boolean settingsAreValid=false;

String commandString = "";     // a String to hold incoming commands from serial
bool commandComplete = false;  // goes true when enter is pressed

unsigned long doneTimestamp=0; //used to allow publishes to complete before sleeping

//This is true if a package is detected. It will be written to RTC memory 
// as "wasPresent" just before sleeping
bool isPresent=false;

//This is the distance measured on this pass. It will be written to RTC memory just before sleeping
int distance=0;

boolean rssiShowing=false; //used to redraw the RSSI indicator after clearing display
String lastMessage=""; //contains the last message sent to display. Sometimes need to reshow it

//We should report at least once per hour, whether we have a package or not.  This
//will also let us retrieve any outstanding MQTT messages.  Since the internal millis()
//counter is reset every time it wakes up, we need to save it before sleeping and restore
//it when waking up. To keep from killing our flash memory, we'll store it in the RTC
//memory, which is kept alive by the battery or power supply.
typedef struct
  {
  unsigned long nextHealthReportTime=0;//the RTC for the next report, regardless of readings
  unsigned long rtc=0;        //the RTC maintained over sleep periods
  bool wasPresent=false;      //Package present on last check
  bool presentReported=false; //MQTT Package Present report was sent
  bool absentReported=false;  //MQTT Package Removed report was sent
  long rssi=-99;              //The signal strength
  } MY_RTC;
  
MY_RTC myRtc;

ADC_MODE(ADC_VCC); //so we can use the ADC to measure the battery voltage

/* Like delay() but checks for serial input */
void myDelay(ulong ms)
  {
  ulong doneTime=millis()+ms;
  while(millis()<doneTime)
    {
    checkForCommand();
    delay(10);
    }
  }

// Configure LoRa module
void configureLoRa()
  {
  if (settingsAreValid)
    {
    lora.begin(settings.loRaBaudRate);
    lora.setAddress(settings.loRaAddress);
    lora.setNetworkID(settings.loRaNetworkID);
    lora.setBand(settings.loRaBand);
    lora.setRFPower(settings.loRaPower);
    lora.setBaudRate(settings.loRaBaudRate);
    lora.setParameter(settings.loRaSpreadingFactor, 
                      settings.loRaBandwidth, 
                      settings.loRaCodingRate, 
                      settings.loRaPreamble);
    }
  }


void show(String msg)
  {
  if (settings.displayenabled)
    {
    lastMessage=msg; //in case we need to redraw it

    if (settings.debug)
      {
      Serial.print("Length of display message:");
      Serial.println(msg.length());
      }
    display.clearDisplay(); // clear the screen
    display.setCursor(0, 0);  // Top-left corner

    if (msg.length()>20)
      {
      display.setTextSize(1);      // tiny text
      }
    else if (msg.length()>7 || rssiShowing) //make room for rssi indicator
      {
      display.setTextSize(2);      // small text
      }
    else
      {
      display.setTextSize(3);      // Normal 1:1 pixel scale
      }
    display.println(msg);
    display.display(); // move the buffer contents to the OLED
    }
  }

void show(uint16_t val, String suffix)
  {
  if (settings.displayenabled)
    {
    String msg=String(val)+suffix;
    show(msg);
    }
  }

void initSensor()
  {
  if (settings.debug)
    {
    Serial.println("Initializing sensor...");
    }

  // pinMode(PORT_XSHUT,OUTPUT);
  digitalWrite(PORT_XSHUT,HIGH); //Enable the sensor

  uint8 retry=0;

  // If the initialization fails, print the error message and then "fix it"
  // every 10 seconds until it quits failing.
  while (++retry)
    {
    if (sensor.init()) 
      {
      if (settings.debug)
        {
        Serial.println("VL53L0X init OK!");
        show("Sensor\nOK");
        }
      break;
      } 
    else 
      {
      if (retry==1)
        {
        Serial.println("Error initializing VL53L0X sensor!");
        show("Sensor\nFailure");
        retry=1;
        }
      Serial.print(MAX_HARDWARE_FAILURES-retry);
      Serial.print(". ");
      Serial.println("fix it!");
      myDelay(5000); // Gimme time to read it
      }
    if (retry>=MAX_HARDWARE_FAILURES)
      ESP.reset();
    }
  }

void initSerial()
  {
  Serial.begin(115200);
  Serial.setTimeout(10000);
  
  while (!Serial); // wait here for serial port to connect.
  if (settings.debug)
    {
    Serial.println();
    Serial.println("Serial communications established.");
    }
  }

void initSettings()
  {
  system_rtc_mem_read(64, &myRtc, sizeof(myRtc)); //load the last saved timestamps from before our nap
  EEPROM.begin(sizeof(settings)); //fire up the eeprom section of flash
  commandString.reserve(200); // reserve 200 bytes of serial buffer space for incoming command string

  loadSettings(); //set the values from eeprom 

  if (settingsAreValid)
    lora.setdebug(settings.debug); //should mirror the main class


  if (settings.maxdistance <= 0) //then this must be the first powerup
    {
    Serial.println("\n*********************** Resetting All EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  }

//Take a measurement
int getDistance()
  {  
  distance=sensor.readRangeSingleMillimeters();
  if (distance) 
    {
    if (settings.debug)
      {
      Serial.print("Inst. Dist. (mm): ");
      Serial.println(distance);
      }
    } 
  else 
    {
    Serial.println("Ranging test failed!");
    }
  return distance?distance:-1;
  }

void initDisplay()
  {
  pinMode(PORT_DISPLAY,OUTPUT); //port for display power
  if (settings.displayenabled)
    {
    if (settings.debug)
      {
      Serial.println("Initializing display");
      }
    digitalWrite(PORT_DISPLAY,HIGH); //turn it on
    myDelay(1000); //let the voltage stabilize

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
      {
      Serial.println(F("SSD1306 allocation failed"));
      myDelay(5000);
      ESP.reset();  //try again
      }
    display.setRotation(settings.invertdisplay?2:0); //make it look right
    display.clearDisplay();       //no initial logo
    display.setTextSize(3);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font

    if (settings.debug)
      show("Init");
    }
  else
    {
    Serial.println("Display is disabled.");
    digitalWrite(PORT_DISPLAY,LOW); //turn it off
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS); // this seems to be the only way to get i2c to work
    //Wire.begin(SDA_PIN, SCL_PIN); //normally handled by the display driver
    }
  }

void setup() 
  {
  pinMode(PORT_XSHUT,OUTPUT);
  digitalWrite(PORT_XSHUT,LOW); //Let it finish booting

  initSerial();

  initSettings();

  if (settingsAreValid)
    {      
    //initialize everything
    initDisplay();
    initSensor(); //sensor should be initialized after display because display sets up i2c
    configureLoRa();

    //Get a measurement and compare the presence with the last one stored in EEPROM.
    //If they are the same, no need to phone home. Unless an hour has passed since
    //the last time home was phoned. 
    distance=measure(); 
    show(distance," mm");

    isPresent=distance>settings.mindistance 
                && distance<settings.maxdistance;
    int analog=readBattery();
    
    Serial.print("**************\nThis measured distance: ");
    Serial.print(distance);
    Serial.println(" mm ");

    Serial.print("Package is ");
    Serial.println(isPresent?"present":"absent");
    
    if (settings.debug)
      {
      Serial.print("Last RSSI was ");
      Serial.println(myRtc.rssi);

      Serial.print("Analog input is ");
      Serial.println(analog);

      Serial.print("Battery voltage: ");
      Serial.println(convertToVoltage(analog));
      }

    sendOrNot(); //decide whether or not to send a report

    if (settings.displayenabled)
      {
      myDelay(3000); //give someone a chance to read the value
      }
    }
  else
    {
    showSettings();
    }
  }


void loop()
  {
  checkForCommand(); // Check for input in case something needs to be changed to work
  if (settingsAreValid && settings.sleeptime==0) //if sleepTime is zero then don't sleep
    {
    distance=measure();
    isPresent=distance>settings.mindistance 
              && distance<settings.maxdistance;
    show(distance," mm");
    report();
    myDelay(1000);
    } 
  else if (settingsAreValid                        //setup has been done and
          && millis()-doneTimestamp>PUBLISH_DELAY) //waited long enough for report to finish
    {
    unsigned long nextReportSecs=(myRtc.nextHealthReportTime-myMillis())/1000;

    Serial.print("Next report in ");
    Serial.print(nextReportSecs/60);
    Serial.print(" minutes and ");
    Serial.print(nextReportSecs%60);
    Serial.println(" seconds.");

    //RTC memory is weird, I'm not sure I understand how it works on the 8266.
    //Reset the health report if it's way wrong
    if (myRtc.nextHealthReportTime-myMillis()>ONE_HOUR)
      {
      Serial.println("------------Fixing bogus health report time-------------");
      myRtc.nextHealthReportTime=myMillis();
      }

    //save the wakeup time so we can keep track of time across sleeps
    myRtc.rtc=myMillis()+settings.sleeptime*1000;
    myRtc.wasPresent=isPresent; //this presence flag becomes the last presence flag
    saveRTC(); //save the timing before we sleep 
    
    digitalWrite(PORT_XSHUT,LOW);   //turn off the TOF sensor
    if (settings.displayenabled)
      {
      digitalWrite(PORT_DISPLAY,LOW); //turn off the display only if it is enabled
      }

    unsigned long goodnight=min((unsigned long)settings.sleeptime,nextReportSecs);// whichever comes first
    Serial.print("Sleeping for ");
    Serial.print(goodnight);
    Serial.println(" seconds");
    ESP.deepSleep(goodnight*1000000, WAKE_RF_DEFAULT); 
    } 
  }

/**
 * This routine will decide if a report needs to be sent, and send it if so.
 * The decision is based on whether or not a package was detected for two
 * successive checks. If two successive checks show that the package is 
 * present, or two succesive checks show that the package is not present,
 * then send the report once.  Don't send another report until two successive
 * checks show the opposite or until an hour has passed, whichever comes first.
 * The truth table is:
 * Last |This |Present |Absent  |Send It and
 * Check|Check|Msg Sent|Msg Sent|Do This 
 * -----+-----+--------+--------+-------------------------------
 * No   | No  | N/A    | False  | Yes, set "Absent Sent"=true, "Present Sent"=false
 * No   | No  | N/A    | True   | No
 * No   | Yes | N/A    | N/A    | No
 * No   | Yes | N/A    | N/A    | No
 * Yes  | No  | N/A    | N/A    | No
 * Yes  | No  | N/A    | N/A    | No
 * Yes  | Yes | False  | N/A    | Yes, set "Present Sent"=true, "Absent Sent"=false
 * Yes  | Yes | True   | N/A    | No
 */
void sendOrNot()
  {
  if (myMillis()>myRtc.nextHealthReportTime
      ||((!myRtc.wasPresent && !isPresent) && !myRtc.absentReported)
      ||((myRtc.wasPresent && isPresent) && !myRtc.presentReported))
    {      
    // ********************* attempt to connect to Wifi network
    report();

    if (isPresent)
      {
      myRtc.presentReported=true;
      myRtc.absentReported=false;
      }
    else
      {
      myRtc.absentReported=true;
      myRtc.presentReported=false;
      }
    
    doneTimestamp=millis(); //this is to allow the publish to complete before sleeping
    if (myMillis()>myRtc.nextHealthReportTime)
      {
      myRtc.rtc=millis(); //122024dep reset this to keep it from overflowing in 49 days
      }
    myRtc.nextHealthReportTime=myMillis()+ONE_HOUR;
    myDelay(5000); //wait for any incoming messages
    }
  }

/* Draw a dot at a point on the screen, and increment to the next position */
void makeDot(uint8_t *position)
  {
  display.fillCircle(*position,SCREEN_HEIGHT-DOT_RADIUS*2,DOT_RADIUS,WHITE);
  display.display();
  *position+=DOT_RADIUS*2+DOT_SPACING;
  }

/*
 * This returns the elapsed milliseconds, even if we've been sleeping
 */
unsigned long myMillis()
  {
  return millis()+myRtc.rtc;
  }

// Read the distance SAMPLE_COUNT times and return the dominant value
int measure()
  {
  int vals[SAMPLE_COUNT];
  int answer=0,answerCount=0;
  uint8_t dotPosition=DOT_RADIUS; //where to start drawing the sampling dots

  //get samples
  for (int i=0;i<SAMPLE_COUNT;i++)
    {
    makeDot(&dotPosition);
    vals[i]=getDistance();

    // Turn off the LED
    digitalWrite(LED_BUILTIN,LED_OFF);
    
    delay(50); //give it some space
    }

  //find the most common value within the sample set
  //This code is not very efficient but hey, it's only 10 values
  for (int i=0;i<SAMPLE_COUNT-1;i++) //using SAMPLE_COUNT-1 here because the last one can only have a count of 1
    {
    int candidate=vals[i];
    int candidateCount=1;  
    for (int j=i+1;j<SAMPLE_COUNT;j++)
      {
      if (candidate==vals[j])
        {
        candidateCount++;
        }
      }
    if (candidateCount>answerCount)
      {
      answer=candidate;
      answerCount=candidateCount;
      }
    }
  return answer;
  }



void showSettings()
  {
  Serial.print("mindistance=<minimum presence distance in cm> (");
  Serial.print(settings.mindistance);
  Serial.println(")");
  Serial.print("maxdistance=<maximum presence distance in cm> (");
  Serial.print(settings.maxdistance);
  Serial.println(")");
  Serial.print("sleeptime=<seconds to sleep between measurements> (");
  Serial.print(settings.sleeptime);
  Serial.println(")");
  Serial.print("debug=1|0 (");
  Serial.print(settings.debug);
  Serial.println(")");
  Serial.print("displayenabled=1|0 (");
  Serial.print(settings.displayenabled);
  Serial.println(")");
  Serial.print("invertdisplay=1|0 (");
  Serial.print(settings.invertdisplay);
  Serial.println(")");
  Serial.print("loRaTargetAddress=<Target LoRa module's address 0-65535> (");
  Serial.print(settings.loRaTargetAddress);
  Serial.println(")");
  Serial.print("loRaAddress=<LoRa module's address 0-65535> (");
  Serial.print(settings.loRaAddress);
  Serial.println(")");
  Serial.print("loRaBand=<Freq in Hz> (");
  Serial.print(settings.loRaBand);
  Serial.println(")");
  Serial.print("loRaBandwidth=<bandwidth code 7-9> (");
  Serial.print(settings.loRaBandwidth);
  Serial.println(")");
  Serial.print("loRaCodingRate=<Coding rate code 1-4> (");
  Serial.print(settings.loRaCodingRate);
  Serial.println(")");
  Serial.print("loRaNetworkID=<Network ID 3-15 or 18> (");
  Serial.print(settings.loRaNetworkID);
  Serial.println(")");
  Serial.print("loRaSpreadingFactor=<Spreading Factor 5-11> (");
  Serial.print(settings.loRaSpreadingFactor);
  Serial.println(")");
  Serial.print("loRaPreamble=<4-24, see docs> (");
  Serial.print(settings.loRaPreamble);
  Serial.println(")");
  Serial.print("loRaBaudRate=<baud rate> (");
  Serial.print(settings.loRaBaudRate);
  Serial.println(")");
  Serial.print("loRaPower=<RF power in dbm> (");
  Serial.print(settings.loRaPower);
  Serial.println(")");

  Serial.println("\n*** Use NULL to reset a setting to its default value ***");
  Serial.println("*** Use \"factorydefaults=yes\" to reset all settings  ***\n");
  
  Serial.print("\nSettings are ");
  Serial.println(settingsAreValid?"complete.":"incomplete.");
  }

void showSub(char* topic, bool subgood)
  {
  if (settings.debug)
    {
    Serial.print("++++++Subscribing to ");
    Serial.print(topic);
    Serial.print(":");
    Serial.println(subgood);
    }
  }

  
/*
 * Check for configuration input via the serial port.  Return a null string 
 * if no input is available or return the complete line otherwise.
 */
String getConfigCommand()
  {
  if (commandComplete) 
    {
    Serial.println(commandString);
    String newCommand=commandString;

    commandString = "";
    commandComplete = false;
    return newCommand;
    }
  else return "";
  }

bool processCommand(String cmd)
  {
  const char *str=cmd.c_str();
  char *val=NULL;
  char *nme=strtok((char *)str,"=");
  if (nme!=NULL)
    val=strtok(NULL,"=");

  //Get rid of the carriage return
  if (val!=NULL && strlen(val)>0 && val[strlen(val)-1]==13)
    val[strlen(val)-1]=0; 

  if (nme==NULL || val==NULL || strlen(nme)==0 || strlen(val)==0)
    {
    showSettings();
    return false;   //not a valid command, or it's missing
    }
  else if (strcmp(val,"NULL")==0) //to nullify a value, you have to really mean it
    {
    strcpy(val,"");
    }
  else if (strcmp(nme,"mindistance")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.mindistance=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"maxdistance")==0)
    {
    if (!val)
      strcpy(val,"400");
    settings.maxdistance=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"sleeptime")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.sleeptime=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"loRaTargetAddress")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaTargetAddress=atoi(val);
    saveSettings();
    }
  else if (strcmp(nme,"loRaAddress")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaAddress=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"loRaBand")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaBand=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"loRaBandwidth")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaBandwidth=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"loRaCodingRate")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaCodingRate=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"loRaNetworkID")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaNetworkID=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"loRaSpreadingFactor")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaSpreadingFactor=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"loRaPreamble")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaPreamble=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"loRaBaudRate")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaBaudRate=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"loRaPower")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.loRaPower=atoi(val);
    configureLoRa();
    saveSettings();
    }
  else if (strcmp(nme,"debug")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.debug=atoi(val)==1?true:false;
    lora.setdebug(settings.debug);
    saveSettings();
    }
  else if ((strcmp(nme,"factorydefaults")==0) && (strcmp(val,"yes")==0)) //reset all eeprom settings
    {
    Serial.println("\n*********************** Resetting EEPROM Values ************************");
    initializeSettings();
    saveSettings();
    delay(2000);
    ESP.restart();
    }
  else if (strcmp(nme,"displayenabled")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.displayenabled=atoi(val)==1?true:false;
    saveSettings();
    }
  else if (strcmp(nme,"invertdisplay")==0)
    {
    if (!val)
      strcpy(val,"0");
    settings.invertdisplay=atoi(val)==1?true:false;
    display.setRotation(settings.invertdisplay?2:0); //go ahead and do it
    saveSettings();
    }
  else
    {
    showSettings();
    return false; //command not found
    }
  return true;
  }

void initializeSettings()
  {
  settings.validConfig=0; 
  settings.mindistance=0;
  settings.maxdistance=400;
  settings.sleeptime=DEFAULT_SLEEP_TIME;
  settings.displayenabled=true;
  settings.invertdisplay=false;
  settings.loRaTargetAddress=DEFAULT_LORA_TARGET_ADDRESS;
  settings.loRaAddress=DEFAULT_LORA_ADDRESS;
  settings.loRaNetworkID=DEFAULT_LORA_NETWORK_ID;
  settings.loRaBand=DEFAULT_LORA_BAND;
  settings.loRaSpreadingFactor=DEFAULT_LORA_SPREADING_FACTOR;
  settings.loRaBandwidth=DEFAULT_LORA_BANDWIDTH;
  settings.loRaCodingRate=DEFAULT_LORA_CODING_RATE;
  settings.loRaPreamble=DEFAULT_LORA_PREAMBLE;
  settings.loRaBaudRate=DEFAULT_LORA_BAUD_RATE;
  settings.loRaPower=DEFAULT_LORA_POWER;
  }

void checkForCommand()
  {
  if (Serial.available())
    {
    incomingData();
    String cmd=getConfigCommand();
    if (cmd.length()>0)
      {
      processCommand(cmd);
      }
    }
  }

int readBattery()
  {
  int raw=ESP.getVcc(); //This commandeers the ADC port
  if (settings.debug)
    {
    Serial.print("Raw voltage count:");
    Serial.println(raw);
    }
  return raw;
  }

float convertToVoltage(int raw)
  {
  int vcc=map(raw,0,FULL_BATTERY_COUNT,0,FULL_BATTERY_VOLTS);
  if (settings.debug)
    Serial.println("Mapped "+String(raw)+" to "+String(vcc));
  float f=((float)vcc)/100.0;
  return f;
  }


/************************
 * Do the LoRa thing
 ************************/
void report()
  {
  doc["distance"]=distance;
  doc["battery"]=(float)convertToVoltage(readBattery());
  doc["isPresent"]=isPresent;
  if (publish())
    Serial.println("Sending data successful.");
  else
    Serial.println("Sending data failed!");
  }

boolean publish()
  {
  String json;
  serializeJson(doc,json);
  Serial.println("Publishing "+json);
  return lora.send(settings.loRaTargetAddress, json);
  }

  
/*
*  Initialize the settings from eeprom and determine if they are valid
*/
void loadSettings()
  {
  EEPROM.get(0,settings);
  if (settings.validConfig==VALID_SETTINGS_FLAG)    //skip loading stuff if it's never been written
    {
    settingsAreValid=true;
    if (settings.debug)
      {
      Serial.println("Loaded configuration values from EEPROM");
//      showSettings();
      }
    }
  else
    {
    Serial.println("\nSkipping load from EEPROM, device not configured.");    
    settingsAreValid=false;
    }
  }

/*
 * Save the settings to EEPROM. Set the valid flag if everything is filled in.
 */
boolean saveSettings()
  {
  if (true)
    {
    Serial.println("Settings deemed complete");
    settings.validConfig=VALID_SETTINGS_FLAG;
    settingsAreValid=true;
    }
  else
    {
    Serial.println("Settings still incomplete");
    settings.validConfig=0;
    settingsAreValid=false;
    }
        
  EEPROM.put(0,settings);
  return EEPROM.commit();
  }

/*
 * Save the pan-sleep information to the RTC battery-backed RAM
 */
void saveRTC()
  {
  system_rtc_mem_write(64, &myRtc, sizeof(myRtc)); 
  }

  
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void incomingData() 
  {
  while (Serial.available()) 
    {
    // get the new byte
    char inChar = (char)Serial.read();
    Serial.print(inChar); //echo it back to the terminal

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it 
    if (inChar == '\n') 
      {
      commandComplete = true;
      }
    else
      {
      // add it to the inputString 
      commandString += inChar;
      }
    }
  }
