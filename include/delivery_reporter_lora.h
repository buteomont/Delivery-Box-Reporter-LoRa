// Pins
#define MAX_DIST 8100 // Anything over 8100 mm is "out of range"

#define LED_ON LOW
#define LED_OFF HIGH
#define PORT_XSHUT D8 //needs a pull-down resistor on the esp8266
#define PORT_DISPLAY D7
#define SDA_PIN D2
#define SCL_PIN D1

#define MAX_HARDWARE_FAILURES 20
#define VALID_SETTINGS_FLAG 0xDAB0
#define JSON_MESSAGE_SIZE 50
#define LORA_RX_PIN D5
#define LORA_TX_PIN D6
#define DEFAULT_SLEEP_TIME 0
#define DEFAULT_LORA_TARGET_ADDRESS 1
#define DEFAULT_LORA_ADDRESS 3
#define DEFAULT_LORA_NETWORK_ID 18
#define DEFAULT_LORA_BAND 915000000
#define DEFAULT_LORA_POWER 22
#define DEFAULT_LORA_SPREADING_FACTOR 8
#define DEFAULT_LORA_BANDWIDTH 7
#define DEFAULT_LORA_CODING_RATE 1
#define DEFAULT_LORA_PREAMBLE 12
#define DEFAULT_LORA_BAUD_RATE 115200
#define JSON_STATUS_SIZE SSID_SIZE+PASSWORD_SIZE+USERNAME_SIZE+MQTT_TOPIC_SIZE+150 //+150 for associated field names, etc
#define PUBLISH_DELAY 400 //milliseconds to wait after publishing to MQTT to allow transaction to finish
#define WIFI_TIMEOUT_SECONDS 20 // give up on wifi after this long
//#define MAX_CHANGE_PCT 2 //percent distance change must be greater than this before reporting
#define FULL_BATTERY_COUNT 3686 //raw A0 count with a freshly charged 18650 lithium battery 
#define FULL_BATTERY_VOLTS 412 //4.12 volts for a fully charged 18650 lithium battery 
#define ONE_HOUR 3600000 //milliseconds
#define SAMPLE_COUNT 5 //number of samples to take per measurement 

#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 32      // OLED display height, in pixels
#define OLED_RESET    -1      // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C   //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define DOT_RADIUS    2       // radius of activity dot
#define DOT_SPACING   4       // spacing between dots

#define RSSI_DOT_RADIUS 2       // Radius of the little dot at the bottom of the wifi indicator

// Error codes copied from the MQTT library
// #define MQTT_CONNECTION_REFUSED            -2
// #define MQTT_CONNECTION_TIMEOUT            -1
// #define MQTT_SUCCESS                        0
// #define MQTT_UNACCEPTABLE_PROTOCOL_VERSION  1
// #define MQTT_IDENTIFIER_REJECTED            2
// #define MQTT_SERVER_UNAVAILABLE             3
// #define MQTT_BAD_USER_NAME_OR_PASSWORD      4
// #define MQTT_NOT_AUTHORIZED                 5

//WiFi status codes
//0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
//1 : WL_NO_SSID_AVAILin case configured SSID cannot be reached
//3 : WL_CONNECTED after successful connection is established
//4 : WL_CONNECT_FAILED if password is incorrect
//6 : WL_DISCONNECTED if module is not configured in station mode

//prototypes
//PubSubClient callback function prototype.  This must appear before the PubSubClient constructor.
// void incomingMqttHandler(char* reqTopic, byte* payload, unsigned int length);

unsigned long myMillis();
bool processCommand(String cmd);
void checkForCommand();
int measure();
int getDistance();
void showSettings();
void showSub(char* topic, bool subgood);
void initializeSettings();
int readBattery();
void report();
boolean publish();
void loadSettings();
boolean saveSettings();
void saveRTC();
void serialEvent(); 
void sendOrNot();
char* generateMqttClientId(char* mqttId);
float convertToVoltage(int raw);
void setup(); 
void loop();
void incomingData();