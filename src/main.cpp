#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <wifi_credentials.h>

// MQTT Broker settings
const char *mqtt_server = "192.168.0.63";
const int mqtt_port = 1883;

// MQTT topics
const char *topic_base = "sensors/data";
const char *topic_discovery = "sensors/discovery";

// DHT22 sensor settings
#define DHTPIN 2      // DHT22 data pin
#define DHTTYPE DHT22 // DHT22 sensor type
DHT dht(DHTPIN, DHTTYPE);

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Timing variables
unsigned long lastMeasurement = 0;
const unsigned long measurementInterval = 60000; // 1 minute in milliseconds

// Retry settings
const int maxRetries = 5;
const unsigned long retryDelay = 5000; // 5 seconds

// JSON document size
const size_t JSON_DOC_SIZE = 256;

// Device identification
String deviceId;
String deviceTopic;

// Function declarations
void connectWiFi();
void connectMQTT();
bool publishSensorData();
bool publishDiscovery();
String getDeviceId();

void setup()
{
    Serial.begin(115200);

    // Generate unique device ID from MAC address
    deviceId = getDeviceId();
    deviceTopic = String(topic_base) + "/" + deviceId;

    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("MQTT Topic: ");
    Serial.println(deviceTopic);

    dht.begin();
    connectWiFi();
    client.setServer(mqtt_server, mqtt_port);
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi connection lost. Reconnecting...");
        connectWiFi();
    }

    if (!client.connected())
    {
        connectMQTT();
    }
    client.loop();

    unsigned long currentMillis = millis();
    if (currentMillis - lastMeasurement >= measurementInterval)
    {
        if (publishSensorData())
        {
            lastMeasurement = currentMillis;
        }
    }
}

void connectWiFi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED && retryCount < maxRetries)
    {
        delay(retryDelay);
        Serial.print(".");
        retryCount++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("Failed to connect to WiFi. Restarting...");
        ESP.restart();
    }
}

void connectMQTT()
{
    int retryCount = 0;
    while (!client.connected() && retryCount < maxRetries)
    {
        Serial.print("Attempting MQTT connection...");

        if (client.connect(deviceId.c_str()))
        {
            Serial.println("connected");
            // Publish discovery information when connection is established
            publishDiscovery();
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(retryDelay);
            retryCount++;
        }
    }

    if (!client.connected() && retryCount >= maxRetries)
    {
        Serial.println("Failed to connect to MQTT broker. Restarting...");
        ESP.restart();
    }
}

bool publishSensorData()
{
    float temperature = dht.readTemperature(true); // true = Fahrenheit
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity))
    {
        Serial.println("Failed to read from DHT sensor!");
        return false;
    }

    JsonDocument doc;
    char buffer[JSON_DOC_SIZE];

    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["device_id"] = deviceId;
    // doc["timestamp"] = millis();

    serializeJson(doc, buffer);

    if (!client.publish(deviceTopic.c_str(), buffer, true)) // retained message
    {
        Serial.println("Failed to publish sensor data");
        return false;
    }

    Serial.println("Published sensor data:");
    Serial.print("Device: ");
    Serial.print(deviceId);
    Serial.print(", Temperature: ");
    Serial.print(temperature);
    Serial.print("°F, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");

    return true;
}

bool publishDiscovery()
{
    JsonDocument doc;
    char buffer[JSON_DOC_SIZE];

    doc["device_id"] = deviceId;
    doc["topic"] = deviceTopic;
    doc["ip"] = WiFi.localIP().toString();

    serializeJson(doc, buffer);

    return client.publish(topic_discovery, buffer, true); // retained message
}

String getDeviceId()
{
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char deviceId[13];
    snprintf(deviceId, sizeof(deviceId), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(deviceId);
}