#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
// Replace with your SIM800L GSM module settings
#define SerialMon Serial
#define SerialAT Serial2

SoftwareSerial gpsSerial(21, 4);

// Create a TinyGPS++ object
TinyGPSPlus gps;

const char apn[] = "";

// Replace with your MQTT broker details
const char* mqttServer = "";  // IP address or host name
const int mqttPort = 1883;
const char* mqttTopic = "";
const char* mqttClientId = "mqtt-explorer-b5ea49f9";
const char* mqttUsername = "";
const char* mqttPassword = "";

TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqttClient(gsmClient);

bool publishingEnabled = false;
bool isNetworkConnected;
bool gpsValid = false;
unsigned long previousPublishTime = 0;
const unsigned long publishInterval = 5000;  // Publish interval in milliseconds (5 seconds)
//sd card
const int chipselect = 5;
DynamicJsonDocument jsonDoc(256);

void setup() {
  // Start serial communication
  SerialMon.begin(115200);
  SerialAT.begin(9600);
  gpsSerial.begin(9600);
  delay(3000);

  // Connect to the GSM network
  SerialMon.println("Initializing modem...");
  modem.restart();
  int signalStrength = modem.getSignalQuality();
  SerialMon.print("Modem Signal Strength: ");
  SerialMon.println(signalStrength);
  SerialMon.print("Connecting to network...");
  isNetworkConnected = modem.isNetworkConnected();
  SerialMon.print("Network Status: ");
  SerialMon.println(isNetworkConnected ? "Network Connected" : "Network Not Connected");

  while (!modem.gprsConnect(apn)) {
    SerialMon.println("Failed");
    delay(1000);
  }
  SerialMon.println("Network Connected");
  isNetworkConnected = modem.isNetworkConnected();
  SerialMon.print("Network Status: ");
  SerialMon.println(isNetworkConnected ? "Network Connected" : "Network Not Connected");

  // Connect to MQTT broker
  mqttClient.setServer(mqttServer, mqttPort);
  //connectToMqtt();
}

void sendmsg() {
  if (gpsValid) {
    Serial.print("Number of Satellites in view :");
    Serial.println(float(gps.satellites.value()));
    char lat[10], lng[11];
    JsonObject gpsdata = jsonDoc.createNestedObject("gps");
    gpsdata["latitude"] = gps.location.lat();
    gpsdata["longitude"] = gps.location.lng();
    dtostrf(gps.location.lat(), 9, 6, lat);
    dtostrf(gps.location.lng(), 10, 6, lng);
    
      int d=float(gps.date.day());
      int m=float(gps.date.month());
      int y=float(gps.date.year());
      float h=float(gps.time.hour());
      float mi=float(gps.time.minute());
      float s=float(gps.time.second());
    String location = String(lat) + "," + String(lng);
    String Date=String(d)+"/"+String(m)+"/"+String(y);
    String time = String(h)+" : "+String(mi)+" : "+String(s);
    //mqttClient.publish(mqttTopic, location.c_str());
    Serial.println("Location:");
    Serial.println(location);
    Serial.println("Date :");
    Serial.println(Date);
    Serial.println("Time :");
    Serial.println(time);
    jsonDoc["latitude"] = gps.location.lat();
    jsonDoc["longitude"] = gps.location.lng();
    jsonDoc["Date"] = Date;
    jsonDoc["Time"] = time;
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    mqttClient.publish(mqttTopic, jsonString.c_str());

  } else {
    Serial.println("Invalid GPS data");
    Serial.print("Number of Satellites in view :");
    Serial.println(float(gps.satellites.value()));
  }
}

void loop() {
  // Handle GSM and MQTT connections
  if (!connectToMqtt()) {
    SerialMon.println("Lost network connection. Storing data in SD card.");
    
    ESP.restart();
  }
  mqttClient.loop();

  // Read GPS data
  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Publish GPS location every 5 seconds
      if (gps.location.isUpdated() && millis() - previousPublishTime >= publishInterval) {
        previousPublishTime = millis();
        gpsValid = true;
        sendmsg();
      }
    }
  }
}

bool connectToMqtt() {
  bool ret = false;
  auto iniTime = 0;

  mqttClientId = mqttClientId + millis();
  if (MQTT_CONNECTED == mqttClient.state()) {
    return true;
  }

  mqttClient.setServer(mqttServer, mqttPort);
  iniTime = millis();
  do {
    ret = mqttClient.connect(mqttClientId, mqttUsername, mqttPassword);
    delay(1000);
  } while ((ret == false) && ((millis() - iniTime) <= 60));


  if (true == ret && !publishingEnabled) {
    Serial.println("Publishing enabled");
    sendmsg();
    publishingEnabled = true;
  }
  return ret;
}