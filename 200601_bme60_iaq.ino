/*
 * Room Sensor Node for Temperature, Pressure, Humidity, IAQ
 * Sends and receives values via MQTT
 * Bosch BME680 sensor
 * Uses the Bosch Sensortec Environmetal Cluster for Arduino to compute IAQ values
 * https://github.com/BoschSensortec/BSEC-Arduino-library
 * Basd on BSEC Arduino library basic.ino example script.
 * Copyright K.Schmolders 06/2020
 */

// wifi credentials stored externally and .gitignore
//all wifi credential and MQTT Server importet through wifi_credential.h
#include "wifi_credentials.h"
//Load all variables
#include "config.h"

//required for MQTT
#include <ESP8266WiFi.h>
//required for OTA updater
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
//end OTA requirements
#include <PubSubClient.h>

#include <EEPROM.h>
#include "bsec.h"

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

#define STATE_SAVE_PERIOD  UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

//timer
int timer_update_state_count;

//MQTT (see also wifi_credentials)
WiFiClient espClient;
PubSubClient client(espClient);

//OTA
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

//BSEC Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

//General Helper Functions declarations
void setup_wifi();
void send_status();
void sendSensorValues();
void reconnect();

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

//storing and sending sensor readouts
float bme680_temperature, bme680_pressure, bme680_humidity, bme680_iaq, bme680_iaq_acc, bme680_co2, bme680_voc;

//for Serial.println
String output;

//callback function for MQTT client
void callback(char* topic, byte* payload, unsigned int length) {
   payload[length]='\0'; // Null terminator used to terminate the char array
   String message = (char*)payload;
 
   Serial.print("Message arrived on topic: [");
   Serial.print(topic);
   Serial.print("]: ");
   Serial.println(message);
   
   //get last part of topic 
   char* cmnd = "test";
   char* cmnd_tmp=strtok(topic, "/");
 
   while(cmnd_tmp !=NULL) {
     cmnd=cmnd_tmp; //take over last not NULL string
     cmnd_tmp=strtok(NULL, "/"); //passing Null continues on string
     //Serial.println(cmnd_tmp);    
   }
    if (!strcmp(cmnd, "status")) {
        Serial.print("Received status request. sending status");
        send_status();
    }
    else if (!strcmp(cmnd, "reset")) {
        Serial.print(F("Reset requested. Resetting..."));
        //software_Reset();
    }
}

void setup(void)
{
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
  Serial.begin(115200);
  Wire.begin();

  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

  loadState();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  // Print the header
  output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
  Serial.println(output);

    //INIT TIMERS
    timer_update_state_count=millis();

    //WIFI and MQTT
    setup_wifi(); // Connect to wifi 
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

// Function that is looped forever
void loop(void)
{
    //handle Wifi
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

   //http Updater for OTA
   httpServer.handleClient(); 

   //send status update via MQTT as configured
   if(millis()-timer_update_state_count > timer_update_state) {
    //addLog_P(LOG_LEVEL_INFO, PSTR("Serial Timer triggerd."));
    timer_update_state_count=millis();
    sendSensorValues();
   }
  
  unsigned long time_trigger = millis();
  if (iaqSensor.run()) { // If new data is available
    output = String(time_trigger);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure / 1e2);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance / 1e3);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output);

    //make sensor values globally available
    bme680_temperature = iaqSensor.temperature;
    bme680_pressure = iaqSensor.pressure / 1e2;
    bme680_humidity = iaqSensor.humidity;
    bme680_iaq = iaqSensor.staticIaq; //use the IAQ reading for indoor static use case
    bme680_iaq_acc = iaqSensor.iaqAccuracy;
    //bme680_co2 = iaqSensor.co2Equivalent; //equivalents not usable.
    //bme680_voc = iaqSensor.breathVocEquivalent;

    updateState();
  } else {
    checkIaqSensorStatus();
  }
}

//sends module status via MQTT
void send_status()
 {
   char outTopic_status[50];
   char msg[50];
   //IP Address
   strcpy(outTopic_status,outTopic);
   strcat(outTopic_status,"ip_address");
   
   //ESP IP
   WiFi.localIP().toString().toCharArray(msg,50);
   client.publish(outTopic_status,msg ); 
 }

//send Sensor Values via MQTT
 void sendSensorValues(){
    char outTopic_status[50];
    char msg[50];
    Serial.println("Sending Sensor values.");

   //roomtemp from BME680
    strcpy(outTopic_status,outTopic);
    dtostrf(bme680_temperature,2,2,msg); 
    strcat(outTopic_status,"temperature");
    client.publish(outTopic_status, msg);

   //BME680 Humidity
    strcpy(outTopic_status,outTopic);
    dtostrf(bme680_humidity,2,2,msg); 
    strcat(outTopic_status,"humidity");
    client.publish(outTopic_status, msg);

    //BME680 Pressure
    strcpy(outTopic_status,outTopic);
    dtostrf(bme680_pressure,2,2,msg); 
    strcat(outTopic_status,"pressure");
    client.publish(outTopic_status, msg);
 
    //BME680 IAQ
    strcpy(outTopic_status,outTopic);
    dtostrf(bme680_iaq,2,2,msg); 
    strcat(outTopic_status,"iaq");
    client.publish(outTopic_status, msg);

    //BME680 CO2
    //strcpy(outTopic_status,outTopic);
    //dtostrf(bme680_co2,2,2,msg); 
    //strcat(outTopic_status,"co2");
    //client.publish(outTopic_status, msg);

    //BME680 VOC
    //strcpy(outTopic_status,outTopic);
    //dtostrf(bme680_voc,2,2,msg); 
    //strcat(outTopic_status,"voc");
    //client.publish(outTopic_status, msg);

     //IP Address
    strcpy(outTopic_status,outTopic);
    strcat(outTopic_status,"ip_address");
    WiFi.localIP().toString().toCharArray(msg,50);
    client.publish(outTopic_status,msg ); 
 }

void reconnect() {
   // Loop until we're reconnected
   while (!client.connected()) {
     Serial.print("Attempting MQTT connection...");
     // Attempt to connect
     if (client.connect(mqtt_id)) {
       Serial.println("connected");
       client.publish(outTopic, "sensor node iaq station booted");
       
       //send current Status via MQTT to world
       sendSensorValues();
       // ... and resubscribe
       client.subscribe(inTopic);
 
     } else {
       Serial.print("failed, rc=");
       Serial.print(client.state());
       Serial.println(" try again in 5 seconds");      
       delay(5000);
     }
   }
 }


void setup_wifi() {
   delay(10);
   // We start by connecting to a WiFi network
   Serial.println();
   Serial.print("Connecting to ");
   Serial.println(ssid);
   WiFi.persistent(false);
   WiFi.mode(WIFI_OFF);
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
   }
     
   Serial.println("");
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());
 
   httpUpdater.setup(&httpServer);
   httpServer.begin();
 }

//BSEC helper functions---------------------
// Helper function definitions
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
  iaqSensor.status = BSEC_OK;
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}
