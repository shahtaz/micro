#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

//df pklaeyr setup done
static const uint8_t PIN_MP3_TX = 2; // Nano TX → DFPlayer RX
static const uint8_t PIN_MP3_RX = 3; // Nano RX → DFPlayer TX
SoftwareSerial mp3Serial(PIN_MP3_RX, PIN_MP3_TX);
DFRobotDFPlayerMini player;

//adx345 setup
Adafruit_ADXL345_Unified accelerometer = Adafruit_ADXL345_Unified(12345);

//ultrasonic sensor
const int trigPin = 8;
const int echoPin = 7;

//stop switch
const int stopButtonPin = 4;   // D4 → GND

// adxl345
bool freefallDetected = false;
bool impactDetected = false;
unsigned long freefallTime = 0;
unsigned long impactTime = 0;
bool fallSoundPlayed = false;

// fall detections stuff
const float freefallThreshold = 5.0;
const float impactThreshold = 20.0;
const float stillnessThreshold = 12.0;



// ultrasonic threshold
float currentDistance = 0;
float lastDetectedDistance = -1;
unsigned long lastSonarReading = 0;
bool objectDetected = false;
unsigned long objectCooldownStart = 0;
const unsigned long objectCooldownDuration = 5000; // 5 seconds
bool objectCooldownActive = false;

// ultrasonic_const
const float minRange = 10.0;
const float maxRange = 400.0;
const float objectThreshold = 30.0;  // Distance limit for detection
const unsigned long sonarInterval = 150;

// functionss
float calculateMagnitude(sensors_event_t &event);
void detectFall();
void triggerFallAlert(sensors_event_t &event);
float readSonarDistance();
void checkObjectDetection();
void playSound(int trackNumber);
void displayStatus();
void resetFallDetection();
void playFallSoundUntilButton();

// --------------------- Setup ----------------------
void setup() {
    Serial.begin(115200);
    delay(2000);
    Wire.begin();

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(stopButtonPin, INPUT_PULLUP);

    Serial.println("Initializing system...");

    // DFPlayer Mini
    mp3Serial.begin(9600);
    if (player.begin(mp3Serial)) {
        Serial.println("DFPlayer Mini ready");
        player.volume(25);
    } else {
        Serial.println("DFPlayer Mini not detected!");
    }

    // Accelerometer
    if (!accelerometer.begin()) {
        Serial.println("No ADXL345 detected!");
        while (1);
    }
    accelerometer.setRange(ADXL345_RANGE_16_G);
    accelerometer.setDataRate(ADXL345_DATARATE_100_HZ);

    Serial.println("System ready - monitoring for falls and objects...");
    delay(1000);
}

// --------------------- Magnitude Calculation ----------------------
float calculateMagnitude(sensors_event_t &event) {
    return sqrt(event.acceleration.x * event.acceleration.x +
                event.acceleration.y * event.acceleration.y +
                event.acceleration.z * event.acceleration.z);
}

// --------------------- Sonar Distance ----------------------
float readSonarDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return -1;
    float distance = (duration * 0.0343) / 2;
    if (distance >= minRange && distance <= maxRange) return distance;
    return -1;
}

// --------------------- Object Detection ----------------------
void checkObjectDetection() {
    unsigned long currentTime = millis();

    // End cooldown after 5 seconds
    if (objectCooldownActive && (currentTime - objectCooldownStart >= objectCooldownDuration)) {
        objectCooldownActive = false;
        Serial.println("Object detection cooldown ended.");
    }

    // Skip detection if still in cooldown
    if (objectCooldownActive) return;

    if (currentDistance > 0 && currentDistance <= objectThreshold) {
        objectDetected = true;
        Serial.println("Object detected within 60 cm!");
        playSound(2); // 0002.mp3
        objectCooldownActive = true;
        objectCooldownStart = currentTime;
    } else {
        objectDetected = false;
    }

    lastDetectedDistance = currentDistance;
}

// --------------------- Fall Detection ----------------------
void detectFall() {
    sensors_event_t event;
    accelerometer.getEvent(&event);
    float magnitude = calculateMagnitude(event);
    unsigned long currentTime = millis();

    if (!freefallDetected && magnitude < freefallThreshold) {
        freefallDetected = true;
        freefallTime = currentTime;
        Serial.println("Freefall detected");
        return;
    }

    if (freefallDetected && !impactDetected) {
        if (magnitude > impactThreshold && (currentTime - freefallTime < 1000)) {
            impactDetected = true;
            impactTime = currentTime;
            Serial.println("Impact detected");
        }
        if (currentTime - freefallTime > 1500) resetFallDetection();
    }

    if (freefallDetected && impactDetected && !fallSoundPlayed) {
        if (magnitude < stillnessThreshold && (currentTime - impactTime > 500)) {
            Serial.println("Fall confirmed!");
            triggerFallAlert(event);
            playFallSoundUntilButton();  // Repeat sound until button press
            fallSoundPlayed = true;
            resetFallDetection();
        }
    }
}


void playFallSoundUntilButton() {
    bool stopTriggered = false;
    Serial.println("Playing fall sound repeatedly until button pressed...");

    while (!stopTriggered) {
        player.playMp3Folder(3); // Play 0003.mp3 repeatedly
        unsigned long trackStart = millis();
        while (millis() - trackStart < 4000) { // Adjust duration to your audio length
            bool buttonState = digitalRead(stopButtonPin);
            if (buttonState == LOW) {
                stopTriggered = true;
                Serial.println("Stop button pressed — stopping fall alert.");
                player.stop();
                break;
            }
            delay(10);
        }
        delay(100);
    }
}

void triggerFallAlert(sensors_event_t &event) {
    Serial.print("Fall detected - Coordinates: X=");
    Serial.print(event.acceleration.x, 2);
    Serial.print(", Y=");
    Serial.print(event.acceleration.y, 2);
    Serial.print(", Z=");
    Serial.print(event.acceleration.z, 2);
    Serial.println(" m/s²");
}


void resetFallDetection() {
    freefallDetected = false;
    impactDetected = false;
    fallSoundPlayed = false;
}


void displayStatus() {
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay > 500) {
        sensors_event_t event;
        accelerometer.getEvent(&event);
        float magnitude = calculateMagnitude(event);

        Serial.print("Accel: ");
        Serial.print(magnitude, 2);
        Serial.print(" m/s² | Distance: ");

        if (objectDetected) Serial.print(currentDistance);
        else if (lastDetectedDistance > 0) Serial.print(lastDetectedDistance);
        else Serial.print("Out of range");

        Serial.println(" cm");
        lastDisplay = millis();
    }
}


void playSound(int trackNumber) {
    if (player.available()) {
        player.playMp3Folder(trackNumber);
        Serial.print("Playing track ");
        Serial.println(trackNumber);
    }
}


void loop() {
    detectFall();

    unsigned long currentTime = millis();
    if (currentTime - lastSonarReading >= sonarInterval) {
        float distance = readSonarDistance();
        if (distance > 0) currentDistance = distance;
        checkObjectDetection();
        lastSonarReading = currentTime;
    }

    displayStatus();
    delay(10);
}

