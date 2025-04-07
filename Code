#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <sps30.h>
#include <DHT.h>
#include <ArduinoJson.h>

// Wi-Fi Credentials
const char* ssid = "Absolute_1stFloor";
const char* password = "Abs@2018";

// MQTT Credentials (HiveMQ Cloud)
const char* mqttServer = "0a8926ff283644b5894dedaed07f99a2.s1.eu.hivemq.cloud";
const int mqttPort = 8883;
const char* mqttUser = "testing";
const char* mqttPassword = "Test@123";
const char* mqttClientID = "ESP32SPS30";
const char* mqttTopic = "ampl/sps30/data"; 

// DHT Sensor Setup
#define DHTPIN 26
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Secure Wi-Fi Client
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Timers
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 30000;  // 30 seconds

// Wi-Fi Setup
void setupWiFi() {
    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

// MQTT Connection with TLS
void connectMQTT() {
    espClient.setInsecure();
    client.setBufferSize(1024); // Increase buffer size
    
    while (!client.connected()) {
        Serial.println("Connecting to MQTT (Secure)...");

        if (client.connect(mqttClientID, mqttUser, mqttPassword)) {
            Serial.println("Connected to MQTT broker");
            return;
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Retrying in 5s...");
            delay(5000);
        }
    }
}

// Initialize Sensors
void initializeSensors() {
    Serial.println("Initializing SPS30 & DHT...");
    dht.begin();
    sensirion_i2c_init();

    while (sps30_probe() != 0) {
        Serial.println("SPS30 sensor probing failed. Retrying...");
        delay(1000);
    }
    Serial.println("SPS30 sensor detected successfully.");
    sps30_set_fan_auto_cleaning_interval_days(4);
    sps30_start_measurement();
}

// Publish Sensor Data
void publishSensorData() {
    struct sps30_measurement sps_data;
    if (sps30_read_measurement(&sps_data) < 0) {
        Serial.println("Error reading SPS30 measurement");
        return;
    }

    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    if (isnan(temp) || isnan(humidity)) {
        Serial.println("Failed to read from DHT11 sensor");
        return;
    }

    StaticJsonDocument<512> jsonDoc;
    jsonDoc["PM1P0"] = sps_data.mc_1p0;
    jsonDoc["PM2P5"] = sps_data.mc_2p5;
    jsonDoc["PM4P0"] = sps_data.mc_4p0;
    jsonDoc["PM10P0"] = sps_data.mc_10p0;
    jsonDoc["NC_0N5"] = sps_data.nc_0p5;
    jsonDoc["NC_1N0"] = sps_data.nc_1p0;
    jsonDoc["NC_2N5"] = sps_data.nc_2p5;
    jsonDoc["NC_4N0"] = sps_data.nc_4p0;
    jsonDoc["NC_10N0"] = sps_data.nc_10p0;
    jsonDoc["TypicalParticleSize"] = sps_data.typical_particle_size;
    jsonDoc["temp_out"] = temp;
    jsonDoc["humidity_out"] = humidity;

    char jsonBuffer[512];
    size_t jsonLength = serializeJson(jsonDoc, jsonBuffer);
    //Serial.print("Payload Size: ");
    //Serial.println(jsonLength);

    if (jsonLength > 0) {
        if (client.publish(mqttTopic, jsonBuffer, true)) {
            Serial.println("Data published successfully:");
            Serial.println(jsonBuffer);
        } else {
            Serial.print("Error: Failed to publish data. MQTT State: ");
            Serial.println(client.state());
        }
    } else {
        Serial.println("Error: Failed to serialize JSON.");
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    setupWiFi();

    espClient.setInsecure();  
    client.setServer(mqttServer, mqttPort);
    client.setKeepAlive(120);

    initializeSensors();  // Initialize SPS30 & DHT11
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) setupWiFi();
    if (!client.connected()) connectMQTT();
    client.loop();

    unsigned long currentTime = millis();
    if (currentTime - lastPublishTime >= publishInterval) {
        publishSensorData();
        lastPublishTime = currentTime;
    }
}
