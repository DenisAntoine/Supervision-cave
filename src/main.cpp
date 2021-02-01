
/*
 *  
 */
 
///////////////////////////////////////////// 
#include <Arduino.h> 
#include <SPI.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RunningAverage.h>
////////////////////////////////////////////
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
  //needed for library
  #include <DNSServer.h>
  #include <ESP8266WebServer.h>

  // From v1.1.0
  #include <ESP8266WiFiMulti.h>
  ESP8266WiFiMulti wifiMulti;

  #include <FS.h>

  #define USE_LITTLEFS      true
  
  #if USE_LITTLEFS
    #include <LittleFS.h>
    FS* filesystem = &LittleFS;
    #define FileFS    LittleFS
    #define FS_Name       "LittleFS"
  #else
    FS* filesystem = &SPIFFS;
    #define FileFS    SPIFFS
    #define FS_Name       "SPIFFS"
  #endif
  #define ESP_getChipId()   (ESP.getChipId())

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
char configFileName[] = "/config.json";


  // You only need to format the filesystem once
//#define FORMAT_FILESYSTEM       true
#define FORMAT_FILESYSTEM         false

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// From v1.1.0
#define MIN_AP_PASSWORD_SIZE    8

#define SSID_MAX_LEN            32
//From v1.0.10, WPA2 passwords can be up to 63 characters long.
#define PASS_MAX_LEN            64

typedef struct
{
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw  [PASS_MAX_LEN];
}  WiFi_Credentials;

typedef struct
{
  String wifi_ssid;
  String wifi_pw;
}  WiFi_Credentials_String;

#define NUM_WIFI_CREDENTIALS      2

typedef struct
{
  WiFi_Credentials  WiFi_Creds [NUM_WIFI_CREDENTIALS];
} WM_Config;

WM_Config         WM_config;

#define  CONFIG_FILENAME              F("/wifi_cred.dat")

// Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected
bool initialConfig = false;
//////

// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESP_WiFiManager.h>
#define USE_AVAILABLE_PAGES     false

// From v1.0.10 to permit disable/enable StaticIP configuration in Config Portal from sketch. Valid only if DHCP is used.
// You'll loose the feature of dynamically changing from DHCP to static IP, or vice versa
// You have to explicitly specify false to disable the feature.
//#define USE_STATIC_IP_CONFIG_IN_CP          false

// Use false to disable NTP config. Advisable when using Cellphone, Tablet to access Config Portal.
// See Issue 23: On Android phone ConfigPortal is unresponsive (https://github.com/khoih-prog/ESP_WiFiManager/issues/23)
#define USE_ESP_WIFIMANAGER_NTP     false

// Use true to enable CloudFlare NTP service. System can hang if you don't have Internet access while accessing CloudFlare
// See Issue #21: CloudFlare link in the default portal (https://github.com/khoih-prog/ESP_WiFiManager/issues/21)
#define USE_CLOUDFLARE_NTP          false

// New in v1.0.11
#define USING_CORS_FEATURE          true
//////

// Use USE_DHCP_IP == true for dynamic DHCP IP, false to use static IP which you have to change accordingly to your network
#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
  // Force DHCP to be true
  #if defined(USE_DHCP_IP)
    #undef USE_DHCP_IP
  #endif
  #define USE_DHCP_IP     true
#else
  // You can select DHCP or Static IP here
  #define USE_DHCP_IP     true
  //#define USE_DHCP_IP     false
#endif

#if ( USE_DHCP_IP || ( defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP ) )
// Use DHCP
  #warning Using DHCP IP
  IPAddress stationIP   = IPAddress(0, 0, 0, 0);
  IPAddress gatewayIP   = IPAddress(192, 168, 1, 1);
  IPAddress netMask     = IPAddress(255, 255, 255, 0);
#else
  // Use static IP
  #warning Using static IP
  
  #ifdef ESP32
    IPAddress stationIP   = IPAddress(192, 168, 2, 232);
  #else
    IPAddress stationIP   = IPAddress(192, 168, 2, 186);
  #endif
  
  IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
  IPAddress netMask     = IPAddress(255, 255, 255, 0);
#endif

#define USE_CONFIGURABLE_DNS      true

IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(8, 8, 8, 8);

#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager
#define MQTT_SERVER_MAX_LEN             40
#define MQTT_SERVER_PORT_LEN            6
char mqtt_server  [MQTT_SERVER_MAX_LEN];
char mqtt_port    [MQTT_SERVER_PORT_LEN]    = "1883";



// Function Prototypes
uint8_t connectMultiWiFi(void);


//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback(void)
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

bool loadFileFSConfigFile(void)
{
  //clean FS, for testing
  //FileFS.format();

  //read configuration from FS json
  Serial.println("Mounting FS...");

  if (FileFS.begin())
  {
    Serial.println("Mounted file system");

    if (FileFS.exists(configFileName))
    {
      //file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = FileFS.open(configFileName, "r");

      if (configFile)
      {
        Serial.print("Opened config file, size = ");
        size_t configFileSize = configFile.size();
        Serial.println(configFileSize);

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[configFileSize + 1]);

        configFile.readBytes(buf.get(), configFileSize);

        Serial.print("\nJSON parseObject() result : ");

        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get(), configFileSize);

        if ( deserializeError )
        {
          Serial.println("failed");
          return false;
        }
        else
        {
          Serial.println("OK");
         
          if (json["mqtt_server"])
            strncpy(mqtt_server, json["mqtt_server"], sizeof(mqtt_server));

          if (json["mqtt_port"])  
            strncpy(mqtt_port,   json["mqtt_port"], sizeof(mqtt_port));
        }

        //serializeJson(json, Serial);
        serializeJsonPretty(json, Serial);
        configFile.close();
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
    return false;
  }
  return true;
}

bool saveFileFSConfigFile(void)
{
  Serial.println("Saving config");

  DynamicJsonDocument json(1024);

  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"]   = mqtt_port;

  File configFile = FileFS.open(configFileName, "w");

  if (!configFile)
  {
    Serial.println("Failed to open config file for writing");
  }

  serializeJsonPretty(json, Serial);
  serializeJson(json, configFile);
  configFile.close();
  //end save
}

void heartBeatPrint(void)
{
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.print("H");        // H means connected to WiFi
  else
    Serial.print("F");        // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(" ");
  }
}

void check_WiFi(void)
{
  if ( (WiFi.status() != WL_CONNECTED) )
  {
    Serial.println("\nWiFi lost. Call connectMultiWiFi in loop");
    connectMultiWiFi();
  }
}  

void check_status(void)
{
  static ulong checkstatus_timeout  = 0;
  static ulong checkwifi_timeout    = 0;
  static ulong mqtt_publish_timeout = 0;
  static ulong current_millis;

#define WIFICHECK_INTERVAL    1000L
#define HEARTBEAT_INTERVAL    10000L
#define PUBLISH_INTERVAL      60000L

  current_millis = millis();
  
  // Check WiFi every WIFICHECK_INTERVAL (1) seconds.
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
  {
    check_WiFi();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }

  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = current_millis + HEARTBEAT_INTERVAL;
  }

  // Check every PUBLISH_INTERVAL (60) seconds.
  if ((current_millis > mqtt_publish_timeout) || (mqtt_publish_timeout == 0))
  {
    //faire un truc pour publier
    mqtt_publish_timeout = current_millis + PUBLISH_INTERVAL;
  }
}

void loadConfigData(void)
{
  File file = FileFS.open(CONFIG_FILENAME, "r");
  LOGERROR(F("LoadWiFiCfgFile "));

  if (file)
  {
    file.readBytes((char *) &WM_config, sizeof(WM_config));
    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}
    
void saveConfigData(void)
{
  File file = FileFS.open(CONFIG_FILENAME, "w");
  LOGERROR(F("SaveWiFiCfgFile "));

  if (file)
  {
    file.write((uint8_t*) &WM_config, sizeof(WM_config));
    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}


uint8_t connectMultiWiFi(void)
{
#if ESP32
  // For ESP32, this better be 0 to shorten the connect time
  #define WIFI_MULTI_1ST_CONNECT_WAITING_MS       0
#else
  // For ESP8266, this better be 2200 to enable connect the 1st time
  #define WIFI_MULTI_1ST_CONNECT_WAITING_MS       2200L
#endif

#define WIFI_MULTI_CONNECT_WAITING_MS           100L
  
  uint8_t status;

  LOGERROR(F("ConnectMultiWiFi with :"));
  
  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass );
  }

  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
  {
    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
    {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
    }
  }
  
  LOGERROR(F("Connecting MultiWifi..."));

  WiFi.mode(WIFI_STA);

#if !USE_DHCP_IP    
  #if USE_CONFIGURABLE_DNS  
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
    WiFi.config(stationIP, gatewayIP, netMask, dns1IP, dns2IP);  
  #else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    WiFi.config(stationIP, gatewayIP, netMask);
  #endif 
#endif

  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ( ( i++ < 10 ) && ( status != WL_CONNECTED ) )
  {
    status = wifiMulti.run();

    if ( status == WL_CONNECTED )
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if ( status == WL_CONNECTED )
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP() );
  }
  else
    LOGERROR(F("WiFi not connected"));

  return status;
}

  ////////////////////////////////////////

#define PIN_FLOW D5
#define PIN_ECHO D7 // Echo Pin
#define PIN_TRIG D6 // Trigger Pin
#define ONE_WIRE_BUS D4  // DS18B20 pin 

//Sonde de temperature--------------
#define TEMP_PRECISION 9
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
DeviceAddress insideThermometer, outsideThermometer; 

long duration, distance; // Duration used to calculate distance
RunningAverage MoyGlissDistance(10);

unsigned long currentMillis = 0;
unsigned long previousMillisFlow = 0;
unsigned long intervalFlow = 1000;
unsigned long previousMillisDist = 0;
unsigned long intervalDist = 1000;


float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;

float flowRate;
float flowLitres;
float totalLitres;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
  Serial.println("Compte!");
}

// Mesure unitaire de la distance ultrason
float mesureDistance() // lance 5 mesures de la distance et retourne la moyenne glisante
{
  for (int i = 0; i <= 5; i++) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    float duration = pulseIn(PIN_ECHO, HIGH);
    //Serial.print("duree :");
    //Serial.println(duration);

    float distance = (duration / 2) / 2910; //duree aller retour divisee par vitesse son
    //Serial.print("Distance :");
    //Serial.println(distance);
    MoyGlissDistance.addValue(distance);
  }

  return MoyGlissDistance.getAverage();
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\nStarting AutoConnectAP using " + String(FS_Name));
  Serial.println(" on " + String(ARDUINO_BOARD));

  if (FORMAT_FILESYSTEM) 
    FileFS.format();

      // Format FileFS if not yet
    if (!FileFS.begin()){
    Serial.print(FS_Name);
    Serial.println(F(" failed! AutoFormatting."));
    FileFS.format();
  }
  loadFileFSConfigFile();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  
  ESP_WMParameter custom_mqtt_server("mqtt_server", "mqtt_server", mqtt_server, MQTT_SERVER_MAX_LEN + 1);
  ESP_WMParameter custom_mqtt_port  ("mqtt_port",   "mqtt_port",   mqtt_port,   MQTT_SERVER_PORT_LEN + 1);

  unsigned long startedAt = millis();

  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESP_WiFiManager ESP_wifiManager;
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESP_WiFiManager ESP_wifiManager("AutoConnectAP");
  //set config save notify callback
  ESP_wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  
  ESP_wifiManager.addParameter(&custom_mqtt_server);
  ESP_wifiManager.addParameter(&custom_mqtt_port);

 
  ESP_wifiManager.setDebugOutput(true);

  //reset settings - for testing
  //ESP_wifiManager.resetSettings();

  //set custom ip for portal
  ESP_wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 100, 1), IPAddress(192, 168, 100, 1), IPAddress(255, 255, 255, 0));
  ESP_wifiManager.setMinimumSignalQuality(-1);
  ESP_wifiManager.setConfigPortalChannel(0);
  
#if !USE_DHCP_IP    
  #if USE_CONFIGURABLE_DNS  
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
    ESP_wifiManager.setSTAStaticIPConfig(stationIP, gatewayIP, netMask, dns1IP, dns2IP);  
  #else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    ESP_wifiManager.setSTAStaticIPConfig(stationIP, gatewayIP, netMask);
  #endif 
#endif

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESP_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif

  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.println("Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  if (Router_SSID != "")
  {
    ESP_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println("Got stored Credentials. Timeout 120s");
  }
  else
  {
    Serial.println("No stored Credentials. No timeout");
  }

  String chipID = String(ESP_getChipId(), HEX);
  chipID.toUpperCase();

  // SSID and PW for Config Portal
  String AP_SSID = "ESP_" + chipID + "_AutoConnectAP";
  String AP_PASS = "MyESP_" + chipID;

  // From v1.1.0, Don't permit NULL password
  if ( (Router_SSID == "") || (Router_Pass == "") )
  {
    Serial.println("We haven't got any access point credentials, so get them now");

    initialConfig = true;

    // Starts an access point
    if ( !ESP_wifiManager.startConfigPortal(AP_SSID.c_str(), AP_PASS.c_str()) )
      Serial.println("Not connected to WiFi but continuing anyway.");
    else
      Serial.println("WiFi connected...yeey :)");

    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));
    
    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESP_wifiManager.getSSID(i);
      String tempPW   = ESP_wifiManager.getPW(i);
  
      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);  

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    saveConfigData();
  }
  else
  {
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }

  startedAt = millis();

  if (!initialConfig)
  {
    // Load stored data, the addAP ready for MultiWiFi reconnection
    loadConfigData();

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    if ( WiFi.status() != WL_CONNECTED ) 
    {
      Serial.println("ConnectMultiWiFi in setup");
     
      connectMultiWiFi();
    }
  }

  Serial.print("After waiting ");
  Serial.print((float) (millis() - startedAt) / 1000L);
  Serial.print(" secs more in setup(), connection result is ");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("connected. Local IP: ");
    Serial.println(WiFi.localIP());
  }
  else
    Serial.println(ESP_wifiManager.getStatus(WiFi.status()));
  //read updated parameters
  
  strncpy(mqtt_server, custom_mqtt_server.getValue(), sizeof(mqtt_server));
  strncpy(mqtt_port, custom_mqtt_port.getValue(),     sizeof(mqtt_port));

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveFileFSConfigFile();
  }


  // Capteur ultrason Trigger en sortie et Echo en entrée
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  MoyGlissDistance.clear();
  
  // Flow Sensor
  pinMode(PIN_FLOW, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  previousMillisFlow = 0;
  previousMillisDist = 0;
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), pulseCounter, FALLING);

// Demarrage library onewire et sondes de temp
  DS18B20.begin();
  DS18B20.getAddress(outsideThermometer, 0);
  DS18B20.getAddress(insideThermometer, 1);
  DS18B20.setResolution(outsideThermometer, TEMP_PRECISION);
  DS18B20.setResolution(insideThermometer, TEMP_PRECISION);
 }


void loop() {
 currentMillis = millis();
 // Mesure temperature et Volume cuve
  
  if (currentMillis - previousMillisDist > intervalDist) {
    // mesure de la distance et volume de cuve
    detachInterrupt(digitalPinToInterrupt(PIN_FLOW));
    float hauteur = 1.07 - mesureDistance();
    float volumel = 1000 * hauteur * (9.50460 - hauteur * (0.84256 + hauteur * 6.63434)); // formules magiques
    
    
    //Temperatures
    // request to all devices on the bus
    DS18B20.requestTemperatures();
    float tempOC = DS18B20.getTempC(outsideThermometer);
    float tempIC = DS18B20.getTempC(insideThermometer); 
    previousMillisDist = millis();
    
    Serial.print("Temp Out C: ");
    Serial.println(tempOC);
    Serial.print("Volume: ");
    Serial.println(volumel);
    Serial.print("Temp In C: ");
    Serial.println(tempIC);
    Serial.print("Hauteur: ");
    Serial.println(hauteur);
    // publier le tout

    attachInterrupt(digitalPinToInterrupt(PIN_FLOW), pulseCounter, FALLING);
  }
  else if (currentMillis - previousMillisFlow > intervalFlow) {
    pulse1Sec = pulseCount;
    pulseCount = 0;
    
    flowRate = ((1000.0 / (millis() - previousMillisFlow)) * pulse1Sec) / calibrationFactor;
    previousMillisFlow = millis();
    flowLitres = (flowRate / 60);
    totalLitres += flowLitres;
    
    // Print the flow rate for this second in litres / minute
    Serial.println("Flow rate: ");
    Serial.print(float(flowRate));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space
    }
  check_status();
}