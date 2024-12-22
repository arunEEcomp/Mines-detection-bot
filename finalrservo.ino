#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
}
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h> // Include Servo library

// GPS setup
TinyGPSPlus gps;
#define RXPin D2
#define TXPin D1
#define GPSBaud 9600
SoftwareSerial gpsSerial(RXPin, TXPin);

// Motor control pins
const int motorA1 = D4; 
const int motorA2 = D5; 
const int motorB1 = D6; 
const int motorB2 = D7; 

// Servo setup
Servo servoMotor;
const int servoPin = D8; // Servo connected to pin D8

// Structure for ESP-NOW communication
typedef struct struct_message {
    char message[32];
    int motorCommand;  // Motor commands: 0=Stop, 1=Forward, 2=Backward, 3=Left, 4=Right, 5=Send GPS, 6=Activate Servo
    double latitude;   // GPS Latitude
    double longitude;  // GPS Longitude
    float altitude;    // GPS Altitude
} struct_message;

struct_message myData;

// Function to stop the motor
void stopMotor() {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
}

// Function to move forward
void moveForward() {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
}

// Function to move backward
void moveBackward() {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
}

// Function to turn left
void turnLeft() {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
}

// Function to turn right
void turnRight() {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
}

// Function to read GPS data
void readGPSData() {
    while (gpsSerial.available() > 0) {
        char data = gpsSerial.read();
        gps.encode(data);
    }

    if (gps.location.isValid()) {
        myData.latitude = gps.location.lat();
        myData.longitude = gps.location.lng();
        myData.altitude = gps.altitude.meters();
    } else {
        myData.latitude = 0.0;
        myData.longitude = 0.0;
        myData.altitude = 0.0;
    }
}

// Function to handle servo motor
void activateServo() {
    servoMotor.attach(servoPin); // Ensure the servo is attached
    servoMotor.write(90);        // Rotate servo to 90 degrees
    delay(1000);                 // Hold position for 1 second
    servoMotor.write(0);         // Return servo to 0 degrees
    delay(500);                  // Allow time to settle
    servoMotor.detach();         // Detach servo after operation
}

// Function to stop the servo motor
void stopServo() {
    servoMotor.detach();         // Detach the servo to stop it immediately
}

// Callback function when data is received
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Message received: ");
    Serial.println(myData.message);

    // Handle received command
    switch (myData.motorCommand) {
        case 0:
            Serial.println("Command: Stop");
            stopMotor();
            stopServo();      // Ensure servo is stopped
            break;
        case 1:
            Serial.println("Command: Forward");
            moveForward();
            break;
        case 2:
            Serial.println("Command: Backward");
            moveBackward();
            break;
        case 3:
            Serial.println("Command: Left");
            turnLeft();
            break;
        case 4:
            Serial.println("Command: Right");
            turnRight();
            break;
        case 5:
            Serial.println("Command: Send GPS");
            readGPSData(); // Fetch the latest GPS data
            strcpy(myData.message, "GPS Data"); // Update the message
            esp_now_send(mac_addr, (uint8_t *)&myData, sizeof(myData)); // Send GPS data back
            break;
        case 6:
            Serial.println("Command: Activate Servo");
            activateServo(); // Activate servo motor
            break;
        default:
            Serial.println("Unknown Command");
            stopMotor();
            stopServo();      // Ensure servo is stopped
            break;
    }
}

void setup() {
    // Initialize Serial Monitor and GPS
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud);
    Serial.println("Receiver with GPS and Servo Starting...");

    // Set motor control pins as outputs
    pinMode(motorA1, OUTPUT);
    pinMode(motorA2, OUTPUT);
    pinMode(motorB1, OUTPUT);
    pinMode(motorB2, OUTPUT);

    // Attach servo to pin (initially detached to keep it stopped)
    servoMotor.attach(servoPin);
    servoMotor.write(0); // Set servo to initial position (0 degrees)
    servoMotor.detach();

    // Stop the motor initially
    stopMotor();

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register the receive callback
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    // Continuously parse GPS data
    readGPSData();
}
