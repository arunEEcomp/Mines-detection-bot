#include <ESP8266WiFi.h>
extern "C" {
  #include <espnow.h>
}

// MAC Address of the receiver NodeMCU
uint8_t broadcastAddress[] = {0x8C, 0xAA, 0xB5, 0x16, 0x05, 0x92}; // Replace with actual receiver MAC address

// Button pins for motor commands, GPS request, and servo control
const int buttonPins[4] = {D3, D4, D5, D6}; // GPIO pins for motor control buttons
const int gpsRequestButton = D7;            // GPIO pin for GPS request button
const int servoControlButton = D1;          // GPIO pin for servo control button
int lastButtonStates[4] = {HIGH, HIGH, HIGH, HIGH}; // Initial states of the motor command buttons
int lastGpsButtonState = HIGH;              // Initial state of the GPS request button
int lastServoButtonState = HIGH;            // Initial state of the servo control button

// Structure for ESP-NOW communication
typedef struct struct_message {
    char message[32];
    int motorCommand;  // Motor commands: 0=Stop, 1=Forward, 2=Reverse, 3=Left, 4=Right, 5=Send GPS, 6=Servo Button Pressed, 7=Servo Button Released
    double latitude;   // GPS Latitude
    double longitude;  // GPS Longitude
    float altitude;    // GPS Altitude
} struct_message;

struct_message myData;

// Callback function for send status
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(sendStatus == 0 ? "Delivery Success" : "Delivery Fail");
}

// Callback function for receiving data
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len) {
    memcpy(&myData, incomingData, sizeof(myData));

    // Display GPS data if received
    if (myData.motorCommand == 5) { // MotorCommand=5 indicates GPS data
        Serial.println("GPS Data Received:");
        Serial.print("Latitude: ");
        Serial.println(myData.latitude, 6);
        Serial.print("Longitude: ");
        Serial.println(myData.longitude, 6);
        Serial.print("Altitude: ");
        Serial.print(myData.altitude);
        Serial.println(" meters");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();

    // Initialize button pins as input with pull-up
    for (int i = 0; i < 4; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);
    }
    pinMode(gpsRequestButton, INPUT_PULLUP); // GPS request button
    pinMode(servoControlButton, INPUT_PULLUP); // Servo control button

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register send and receive callbacks
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Add peer
    esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
    // Handle motor command buttons
    for (int i = 0; i < 4; i++) {
        int buttonState = digitalRead(buttonPins[i]);

        // Check if button state has changed
        if (buttonState != lastButtonStates[i]) {
            lastButtonStates[i] = buttonState;

            // Prepare the message for motor control
            if (buttonState == LOW) { // Button pressed
                myData.motorCommand = i + 1; // Map buttons to motor commands (1 to 4)
                sprintf(myData.message, "Motor Command: %d", myData.motorCommand);
            } else { // Button released
                myData.motorCommand = 0; // Stop motor
                sprintf(myData.message, "Motor Command: Stop");
            }

            // Send the message for motor control
            esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

            // Print status to Serial Monitor
            Serial.println(myData.message);
        }
    }

    // Handle GPS request button
    int gpsButtonState = digitalRead(gpsRequestButton);
    if (gpsButtonState != lastGpsButtonState) {
        lastGpsButtonState = gpsButtonState;

        if (gpsButtonState == LOW) { // Button pressed
            myData.motorCommand = 5; // GPS request command
            strcpy(myData.message, "Request GPS");
            esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
            Serial.println("GPS request sent!");
        }
    }

    // Handle servo control button
    int servoButtonState = digitalRead(servoControlButton);
    if (servoButtonState != lastServoButtonState) {
        lastServoButtonState = servoButtonState;

        if (servoButtonState == LOW) { // Button pressed
            myData.motorCommand = 6; // Servo button pressed command
            strcpy(myData.message, "Servo Button Pressed");
            esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
            Serial.println("Servo button pressed!");
        } else { // Button released
            myData.motorCommand = 7; // Servo button released command
            strcpy(myData.message, "Servo Button Released");
            esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
            Serial.println("Servo button released!");
        }
    }

    delay(100); // Debounce delay
}
