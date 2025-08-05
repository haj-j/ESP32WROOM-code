#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <MAX30100_PulseOximeter.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// Initialize devices
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
PulseOximeter pox;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Verify I2C address with I2C scanner

// WiFi credentials
const char* ssid = "MULUNDA";
const char* password = "12345678";

// MQTT configuration
const char* mqtt_hostname = "LESANJELELA-PC";
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_HealthMonitor";

// MQTT Topics
const char* MQTT_COMPLETE_DATA_TOPIC = "esp32/data/complete";
const char* MQTT_SUBSCRIBE_PATIENT_TOPIC = "esp32/patient/data";
const char* MQTT_ECG_DATA_TOPIC = "esp32/ecg"; // New topic for ECG data

// AD8232 ECG Pins
const int ad8232OutputPin = 34;
const int ad8232LoMinus = 16;
const int ad8232LoPlus = 17;

// ECG Configuration
const int fs = 360;          // Sampling frequency (Hz)
const int samplesNeeded = 100; // The sample buffer is now 100
const int CHUNK_SIZE = 100;  // ECG samples per MQTT message

// Global Variables
float heartRate = 0;
float spo2 = 0;
float objectTemp = 0;
int ecgSamples[samplesNeeded];
int sampleCount = 0;
unsigned long lastSampleTime = 0;
const int sampleInterval = 1000000 / fs; // microseconds between samples
bool ecgConnected = false;
unsigned long lastStatusUpdate = 0;
unsigned long lastEcgPublishTime = 0;
const long ecgPublishInterval = 100; // Publish ECG every 100ms
IPAddress mqtt_server;
bool brokerDiscovered = false;
bool newPatientDataReceived = false;

// Patient Data
String patientID = "";
int patientAge = 0;
float patientWeight = 0;
float patientHeight = 0;
String patientGender = "";

// WiFi and MQTT Clients
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void showError(const char* message) {
    lcd.clear();
    lcd.print("ERROR:");
    lcd.setCursor(0, 1);
    lcd.print(message);
    Serial.printf("CRITICAL ERROR: %s\n", message);
    Serial.println("Rebooting in 10 seconds...");
    delay(10000);
    ESP.restart();
}

void onBeatDetected() {
    Serial.println("Beat detected!");
}

void readECG() {
    if (micros() - lastSampleTime >= sampleInterval) {
        lastSampleTime = micros();
        int currentSample = 0;

        if ((digitalRead(ad8232LoMinus) == 1 || (digitalRead(ad8232LoPlus) == 1))) {
            ecgConnected = false;
            currentSample = 0;
        } else {
            ecgConnected = true;
            currentSample = analogRead(ad8232OutputPin);
        }

        if (sampleCount < samplesNeeded) {
            ecgSamples[sampleCount] = currentSample;
            sampleCount++;
        } else {
            memmove(&ecgSamples[0], &ecgSamples[1], (samplesNeeded - 1) * sizeof(int));
            ecgSamples[samplesNeeded - 1] = currentSample;
        }
    }
}

// --- MODIFIED FUNCTION ---
// This function now sends the raw ECG value as a simple string, not a JSON object.
void sendECGData() {
    if (millis() - lastEcgPublishTime >= ecgPublishInterval) {
        lastEcgPublishTime = millis();
        
        // Get the latest ECG sample
        int currentEcgValue = 0;
        if (sampleCount > 0) {
            currentEcgValue = ecgSamples[sampleCount - 1];
        }
        
        // Convert the integer ECG value to a String
        String payload = String(currentEcgValue);
        
        // Publish the simple numerical string as the payload
        mqttClient.publish(MQTT_ECG_DATA_TOPIC, payload.c_str());
    }
}
// --- END MODIFIED FUNCTION ---

void readAuxSensorData() {
    Wire.setClock(50000);
    objectTemp = mlx.readObjectTempC();
    Wire.setClock(100000);

    heartRate = pox.getHeartRate();
    spo2 = pox.getSpO2();

    if (isnan(objectTemp)) objectTemp = 0;
    if (isnan(heartRate)) heartRate = 0;
    if (isnan(spo2)) spo2 = 0;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.printf("Message arrived on topic: %s\n", topic);

    if (strcmp(topic, MQTT_SUBSCRIBE_PATIENT_TOPIC) == 0) {
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, message);
        
        if (!error) {
            patientID = doc["patient_id"].as<String>();
            patientAge = doc["age"] | 0;
            patientWeight = doc["weight"] | 0.0f;
            patientHeight = doc["height"] | 0.0f;
            patientGender = doc["gender"].as<String>();
            newPatientDataReceived = true;
            
            Serial.printf("Patient Data: ID=%s\n", patientID.c_str());
            lcd.clear();
            lcd.print("Patient Data OK");
            delay(1500);
        }
    }
}

bool sendDataChunk(int startIndex) {
    StaticJsonDocument<1024> doc;
    
    doc["patientID"] = patientID;
    doc["chunk_index"] = startIndex / CHUNK_SIZE;
    doc["total_chunks"] = ceil((float)samplesNeeded / CHUNK_SIZE);
    
    JsonArray ecgChunk = doc.createNestedArray("ecg");
    int endIndex = min(startIndex + CHUNK_SIZE, samplesNeeded);
    
    for (int i = startIndex; i < endIndex; i++) {
        ecgChunk.add(ecgSamples[i]);
    }

    String output;
    serializeJson(doc, output);
    
    if (!mqttClient.publish(MQTT_COMPLETE_DATA_TOPIC, output.c_str())) {
        Serial.println("Failed to send chunk");
        return false;
    }
    
    return true;
}

void sendCompleteData() {
    if (sampleCount < samplesNeeded) {
        Serial.println("Waiting for ECG samples...");
        return;
    }

    // First send metadata
    StaticJsonDocument<512> metaDoc;
    metaDoc["patientID"] = patientID;
    metaDoc["age"] = patientAge;
    metaDoc["weight"] = patientWeight;
    metaDoc["height"] = patientHeight;
    metaDoc["gender"] = patientGender;
    metaDoc["heartRate"] = heartRate;
    metaDoc["spo2"] = spo2;
    metaDoc["temperature"] = objectTemp;
    metaDoc["total_samples"] = samplesNeeded;
    metaDoc["chunk_size"] = CHUNK_SIZE;
    
    String metaOutput;
    serializeJson(metaDoc, metaOutput);
    mqttClient.publish(MQTT_COMPLETE_DATA_TOPIC, metaOutput.c_str());

    // Then send ECG data in chunks
    for (int i = 0; i < samplesNeeded; i += CHUNK_SIZE) {
        if (!sendDataChunk(i)) {
            showError("Data Send Failed");
            return;
        }
        delay(50); // Small delay between chunks
    }

    // Send completion marker
    StaticJsonDocument<128> completeDoc;
    completeDoc["complete"] = true;
    completeDoc["patientID"] = patientID;
    
    String completeOutput;
    serializeJson(completeDoc, completeOutput);
    mqttClient.publish(MQTT_COMPLETE_DATA_TOPIC, completeOutput.c_str());

    newPatientDataReceived = false;
    Serial.println("All data sent successfully");
}

void mqttReconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        lcd.clear();
        lcd.print("MQTT Connecting");
        
        if (mqttClient.connect(mqtt_client_id)) {
            Serial.println("connected");
            lcd.clear();
            lcd.print("MQTT Connected");
            delay(1000);
            mqttClient.subscribe(MQTT_SUBSCRIBE_PATIENT_TOPIC);
        } else {
            Serial.printf("failed, rc=%d\n", mqttClient.state());
            lcd.setCursor(0, 1);
            lcd.printf("Failed: %d", mqttClient.state());
            delay(5000);
        }
    }
}

void discoverBroker() {
    Serial.println("Discovering MQTT broker...");
    lcd.clear();
    lcd.print("Finding Broker...");

    int attempts = 0;
    while (!brokerDiscovered && attempts < 10) {
        mqtt_server = MDNS.queryHost(mqtt_hostname);
        
        if (mqtt_server[0] != 0) {
            brokerDiscovered = true;
            mqttClient.setServer(mqtt_server, mqtt_port);
            Serial.printf("Broker found at: %s\n", mqtt_server.toString().c_str());
            lcd.clear();
            lcd.print("Broker Found!");
            lcd.setCursor(0, 1);
            lcd.print(mqtt_server.toString());
            delay(2000);
        } else {
            Serial.println("Host not found, retrying...");
            lcd.setCursor(0, 1);
            lcd.print("Retry " + String(attempts));
            delay(3000);
            attempts++;
        }
    }
    
    if (!brokerDiscovered) {
        showError("Broker Not Found");
    }
}

void displayDataOnLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("HR:");
    lcd.print(heartRate, 0);
    lcd.print(" SpO2:");
    lcd.print(spo2, 0);
    lcd.print("%");
    
    lcd.setCursor(0, 1);
    lcd.print("ECG:");
    lcd.print(ecgConnected ? "OK" : "NC");
    lcd.print(" T:");
    lcd.print(objectTemp, 1);
    lcd.print((char)223);
    lcd.print("C");
}

void initializeSensors() {
    pinMode(ad8232LoPlus, INPUT);
    pinMode(ad8232LoMinus, INPUT);
    analogReadResolution(12);

    Wire.begin();
    Wire.setClock(100000);

    if (!mlx.begin()) showError("MLX90614 Error");
    if (!pox.begin()) showError("MAX30100 Error");

    pox.setOnBeatDetectedCallback(onBeatDetected);
}

void initializeSystem() {
    Serial.begin(115200);
    lcd.init();
    lcd.backlight();
    lcd.print("System Booting...");
    delay(1000);

    initializeSensors();

    lcd.clear();
    lcd.print("Connecting WiFi...");
    Serial.println("\nConnecting to WiFi...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        lcd.print(".");
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        lcd.clear();
        lcd.print("WiFi Connected");
        lcd.setCursor(0, 1);
        lcd.print(WiFi.localIP());
        Serial.println("\nWiFi Connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        delay(2000);
    } else {
        showError("WiFi Failed");
    }

    if (!MDNS.begin("esp32-health-monitor")) {
        showError("mDNS Error");
    }

    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(4096);
}

void setup() {
    initializeSystem();
    discoverBroker();
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
        delay(1000);
        return;
    }

    if (!mqttClient.connected()) {
        mqttReconnect();
    }
    mqttClient.loop();

    pox.update();
    readECG();
    sendECGData(); // Send ECG data to MQTT topic

    if (millis() - lastStatusUpdate > 1000) {
        readAuxSensorData();
        displayDataOnLCD();
        lastStatusUpdate = millis();
        
        Serial.printf("Heap: %d, Samples: %d/%d\n", 
                    ESP.getFreeHeap(), sampleCount, samplesNeeded);
    }

    if (newPatientDataReceived && sampleCount >= samplesNeeded) {
        sendCompleteData();
    }
    
    delay(1);
}
