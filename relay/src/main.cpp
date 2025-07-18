#include <Arduino.h>

#include <WiFi.h>
#include "PubSubClient.h" // Include the PubSubClient library for MQTT

#include "../include/credentials.h" // Define WIFI_SSID, WIFI_PASSWORD, MQTT_USER and MQTT_PASSWD

#define RELAY_PIN 16
#define LED_PIN 23

#define LOOP_PERIOD 10000 // in microseconds

// MQTT Broker  
const char* mqtt_broker = "10.1.14.50"; // Local broker IP address
const int mqtt_port = 1883; 
const char* mqtt_clientid = "ESP32_Client_Relay";
const char* sub_topic = "test/esp32/relay-ac/relay/state";

WiFiClient espClient;
PubSubClient client(espClient);

// Others
unsigned long startAttemptTime;

// Cadencement 
int counterForPrinting;
int desiredPrintingInterval = 2000; // in milliseconds
const int printingPeriodicity = 1000 * desiredPrintingInterval / LOOP_PERIOD; // The variables will be sent to the serial link one out of printingPeriodicity loop runs. Every printingPeriodicity * LOOP_PERIOD microseconds
unsigned long current_time, previous_time, initial_time;

String mainLoopBuffer = "";
String callbackBuffer = "";

int ValueToSend;

void callback(char* topic, byte* payload, unsigned int length) {
  callbackBuffer = "Message received :\n";
  callbackBuffer += " - Topic: " + String(topic) + "\n";
  callbackBuffer += " - Payload: ";
  String msg;

  if (String(topic) == "test/esp32/relay-ac/relay/state"){
    for (int i = 0; i < length; i++) {
      msg += (char)payload[i];
    }
    callbackBuffer += String(msg) + "\n";
    callbackBuffer += "Setting LED to status: ";
    if (msg == "OPEN"){ 
      digitalWrite(RELAY_PIN, HIGH); // Turn on the relay
      digitalWrite(LED_PIN, HIGH); // Turn on the LED
      callbackBuffer += "OPEN\n";
    }
    else if (msg == "CLOSE"){ 
      digitalWrite(RELAY_PIN, LOW); // Turn off the relay
      digitalWrite(LED_PIN, LOW); // Turn off the LED
      callbackBuffer += "CLOSE\n";
    }
  }
}

void setup() {
  delay(5000);
  // Serial initialisation
  Serial.begin(115200);
  Serial.println();
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) { // Wait 10 seconds max
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Succesfully connected to Wi-Fi !");
    Serial.println(" - IP Address : " + String(WiFi.localIP()));
  } else {
    Serial.println("Failed to connect to Wi-Fi : connection timed out");
    Serial.println("Restarting device...");
    ESP.restart();  // Or enter deep sleep
  }

  // Connect to MQTT broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  Serial.print("Connecting to MQTT broker");
  startAttemptTime = millis();
  while (!client.connect(mqtt_clientid, MQTT_USER, MQTT_PASSWD) && millis() - startAttemptTime < 30000) { // Wait 30 seconds max
    Serial.print(".");
    delay(500);
  }

  if (client.connected()) {      //user id must be unique in case of many users under one topic to avoid conflict/crosstalk information
    Serial.println("Succesfully connected to MQTT !");
    client.subscribe(sub_topic);            //SUBSCRIBE TOPIC
  } else {
    Serial.println("Failed to connect to MQTT broker with state : " + String(client.state()));
    Serial.println("Restarting device...");
    ESP.restart();  // Or enter deep sleep
  }

  // Relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // relais éteint au démarrage

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED éteinte au départ

  counterForPrinting = 0;
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_clientid, MQTT_USER, MQTT_PASSWD)) {
      Serial.println("connected");
      client.subscribe(sub_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  // Keeping MQTT connection alive
  if (!client.connected()) { reconnectMQTT(); }
  client.loop(); 

  mainLoopBuffer = "";
  
  // Cadencement
  previous_time = current_time;
  current_time = micros();

  // Affichage
  counterForPrinting++;
  if (counterForPrinting > printingPeriodicity) {  // Reset the counter and print
    Serial.println("---------" + String(millis())+ "---------");
    Serial.println(mainLoopBuffer);
    Serial.println(callbackBuffer);
    counterForPrinting = 0; // Reset counter
  }

  // Cadencement
  unsigned int sleep_time = LOOP_PERIOD - (micros() - current_time);
  if (sleep_time > 0 && sleep_time < LOOP_PERIOD) delayMicroseconds(sleep_time); // On patiente le temps restant pour respecter la fréquence d'itération (SUPPOSE QUE LES INSTRUCTIONS SONT RÉALISABLES DURANT LA PERIODE)
}

