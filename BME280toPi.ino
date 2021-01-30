#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1025.7)

Adafruit_BME280 bme;

float temperature, humidity, pressure, altitude;

const char* ssid = "WiFi";
const char* password = "password";

// MQTT
const char* mqtt_server = "192.168.0.8";  // IP of the MQTT broker
const char* humidity_topic = "boat/saloon/humidity";
const char* temperature_topic = "boat/saloon/temperature";
const char* pressure_topic = "boat/saloon/pressure";
const char* altitude_topic = "boat/saloon/altitude";
const char* mqtt_username = "username"; // MQTT username
const char* mqtt_password = "password"; // MQTT password
const char* clientID = "clientId"; // MQTT client ID

// MQTT
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient);

// Connect to the MQTT broker via WiFi
void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, password);

  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void setup() {
  Serial.begin(9600);
  bme.begin(0x76);
  delay(100);
}

void loop() {
  connect_MQTT();
  Serial.setTimeout(2000);
  handle_OnConnect();
}

void handle_OnConnect() {
  temperature = bme.readTemperature() * 9/5 + 32;
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 3386.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *F");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("Altitude: ");
  Serial.println(altitude);

  String hs="Humidity: "+String((float)humidity)+" % ";
  String ts="Temperature: "+String((float)temperature)+" C ";
  String ps="Pressure: "+String((float)pressure)+" inches ";
  String as="Altitude: "+String((float)altitude)+" m ";

if (client.publish(temperature_topic, String(ts).c_str())) {
    Serial.println("Temperature sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("Temperature failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(temperature_topic, String(ts).c_str());
  }

  // PUBLISH to the MQTT Broker (topic = Humidity, defined at the beginning)
  if (client.publish(humidity_topic, String(hs).c_str())) {
    Serial.println("Humidity sent!");
  }
  // Again, client.publish will return a boolean value depending on whether it succeded or not.
  // If the message failed to send, we will try again, as the connection may have broken.
  else {
    Serial.println("Humidity failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(humidity_topic, String(humidity).c_str());
  }
  client.disconnect();  // disconnect from the MQTT broker
  delay(1000*60);       // print new values every 1 Minute
}
