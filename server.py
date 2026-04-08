#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);

// Pin Definitions
const int LED_RX = 8;       // Yellow LED - RX indicator
const int LED_VALID = 9;    // Green LED - Valid message
const int LED_INVALID = 7;  // Red LED - Invalid message
const int LED_STATUS = 6;   // Blue LED - Status
const int BUTTON = 5;       // Diagnostics button

// Statistics
unsigned long receivedCount = 0;
unsigned long validCount = 0;
unsigned long invalidCount = 0;
unsigned long lastMessageTime = 0;
float messageRate = 0;

// CAN Frames
struct can_frame canMsg;
struct can_frame ackMsg;

// Message buffer for analysis
struct MessageLog {
    uint16_t canId;
    int value;
    unsigned long timestamp;
    bool valid;
};
MessageLog msgBuffer[100];
int bufferIndex = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    // Initialize pins
    pinMode(LED_RX, OUTPUT);
    pinMode(LED_VALID, OUTPUT);
    pinMode(LED_INVALID, OUTPUT);
    pinMode(LED_STATUS, OUTPUT);
    pinMode(BUTTON, INPUT_PULLUP);
    
    // Initialize MCP2515
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    
    // Startup sequence
    startupSequence();
    
    Serial.println("ECU2:READY");
    Serial.println("ECU2:TYPE:RECEIVER");
    Serial.println("ECU2:VERSION:2.0");
}

void loop() {
    // Check for CAN messages
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        processMessage();
    }
    
    // Check diagnostic button
    if (digitalRead(BUTTON) == LOW) {
        delay(50);
        if (digitalRead(BUTTON) == LOW) {
            printDiagnostics();
            while (digitalRead(BUTTON) == LOW); // Wait for release
        }
    }
    
    // Check for serial commands
    if (Serial.available()) {
        handleSerialCommand();
    }
    
    // Update message rate
    updateMessageRate();
    
    // Update status LED
    updateStatusLED();
}

void startupSequence() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_RX, HIGH);
        delay(100);
        digitalWrite(LED_VALID, HIGH);
        delay(100);
        digitalWrite(LED_STATUS, HIGH);
        delay(100);
        digitalWrite(LED_RX, LOW);
        digitalWrite(LED_VALID, LOW);
        digitalWrite(LED_STATUS, LOW);
        delay(100);
    }
    digitalWrite(LED_STATUS, HIGH);
}

void handleSerialCommand() {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "STATUS") {
        sendStatus();
    } else if (cmd == "DIAG") {
        printDiagnostics();
    } else if (cmd == "RESET") {
        resetAll();
    } else if (cmd == "BUFFER") {
        printBuffer();
    }
}

void processMessage() {
    receivedCount++;
    lastMessageTime = millis();
    digitalWrite(LED_RX, HIGH);
    
    // Extract data
    int value = (canMsg.data[0] << 8) | canMsg.data[1];
    int msgNum = canMsg.data[2];
    int phase = canMsg.data[3];
    int timestamp = (canMsg.data[4] << 8) | canMsg.data[5];
    uint8_t checksum = canMsg.data[6];
    
    // Validate message
    bool isValid = validateMessage(canMsg.can_id, value, checksum);
    
    if (isValid) {
        validCount++;
        digitalWrite(LED_VALID, HIGH);
        digitalWrite(LED_INVALID, LOW);
    } else {
        invalidCount++;
        digitalWrite(LED_VALID, LOW);
        digitalWrite(LED_INVALID, HIGH);
    }
    
    // Store in buffer
    storeMessage(canMsg.can_id, value, isValid);
    
    // Send acknowledgment
    sendAcknowledgment(canMsg.can_id, isValid);
    
    // Log to serial
    Serial.print("RX:");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(":");
    Serial.print(value);
    Serial.print(":");
    Serial.print(msgNum);
    Serial.print(":");
    Serial.print(phase);
    Serial.print(":");
    Serial.println(isValid ? "VALID" : "INVALID");
    
    delay(10);
    digitalWrite(LED_RX, LOW);
}

bool validateMessage(uint16_t canId, int value, uint8_t receivedChecksum) {
    // Range validation
    bool rangeValid = false;
    
    switch (canId) {
        case 0x100: // Engine RPM
            rangeValid = (value >= 0 && value <= 8000);
            break;
        case 0x101: // Speed
            rangeValid = (value >= 0 && value <= 250);
            break;
        case 0x102: // Temperature
            rangeValid = (value >= -40 && value <= 150);
            break;
        case 0x103: // Fuel
            rangeValid = (value >= 0 && value <= 100);
            break;
        case 0x104: // Throttle
            rangeValid = (value >= 0 && value <= 100);
            break;
        case 0x105: // Brake Pressure
            rangeValid = (value >= 0 && value <= 300);
            break;
        case 0x106: // Steering
            rangeValid = (value >= -720 && value <= 720);
            break;
        case 0x107: // Gear
            rangeValid = (value >= -1 && value <= 8);
            break;
        default:
            rangeValid = true;
    }
    
    // Checksum validation
    uint8_t calculatedChecksum = 0;
    for (int i = 0; i < 6; i++) {
        calculatedChecksum ^= canMsg.data[i];
    }
    bool checksumValid = (calculatedChecksum == receivedChecksum);
    
    return rangeValid && checksumValid;
}

void storeMessage(uint16_t canId, int value, bool valid) {
    msgBuffer[bufferIndex].canId = canId;
    msgBuffer[bufferIndex].value = value;
    msgBuffer[bufferIndex].timestamp = millis();
    msgBuffer[bufferIndex].valid = valid;
    
    bufferIndex = (bufferIndex + 1) % 100;
}

void sendAcknowledgment(uint16_t canId, bool valid) {
    ackMsg.can_id = 0x200;
    ackMsg.can_dlc = 4;
    ackMsg.data[0] = (canId >> 8) & 0xFF;
    ackMsg.data[1] = canId & 0xFF;
    ackMsg.data[2] = valid ? 0x01 : 0x00;
    ackMsg.data[3] = receivedCount & 0xFF;
    
    mcp2515.sendMessage(&ackMsg);
}

void printDiagnostics() {
    Serial.println("DIAG:START");
    Serial.print("RECEIVED:");
    Serial.println(receivedCount);
    Serial.print("VALID:");
    Serial.println(validCount);
    Serial.print("INVALID:");
    Serial.println(invalidCount);
    Serial.print("RATE:");
    Serial.print(messageRate, 1);
    Serial.println(" msg/s");
    
    float successRate = receivedCount > 0 ? 
        (float)validCount / receivedCount * 100 : 0;
    Serial.print("SUCCESS_RATE:");
    Serial.print(successRate, 1);
    Serial.println("%");
    
    Serial.println("DIAG:END");
    
    // Visual indication
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_STATUS, LOW);
        delay(100);
        digitalWrite(LED_STATUS, HIGH);
        delay(100);
    }
}

void printBuffer() {
    Serial.println("BUFFER:START");
    for (int i = 0; i < min(bufferIndex, 100); i++) {
        Serial.print(i);
        Serial.print(":");
        Serial.print(msgBuffer[i].canId, HEX);
        Serial.print(":");
        Serial.print(msgBuffer[i].value);
        Serial.print(":");
        Serial.println(msgBuffer[i].valid ? "V" : "I");
    }
    Serial.println("BUFFER:END");
}

void sendStatus() {
    Serial.print("STATUS:");
    Serial.print(receivedCount);
    Serial.print(":");
    Serial.print(validCount);
    Serial.print(":");
    Serial.print(invalidCount);
    Serial.print(":");
    Serial.println(messageRate, 1);
}

void resetAll() {
    receivedCount = 0;
    validCount = 0;
    invalidCount = 0;
    bufferIndex = 0;
    messageRate = 0;
    
    digitalWrite(LED_RX, LOW);
    digitalWrite(LED_VALID, LOW);
    digitalWrite(LED_INVALID, LOW);
    digitalWrite(LED_STATUS, HIGH);
    
    Serial.println("RESET:OK");
}

void updateMessageRate() {
    static unsigned long lastRateCalc = 0;
    static unsigned long lastCount = 0;
    
    if (millis() - lastRateCalc >= 1000) {
        messageRate = receivedCount - lastCount;
        lastCount = receivedCount;
        lastRateCalc = millis();
    }
}

void updateStatusLED() {
    static unsigned long lastBlink = 0;
    
    // Blink if no messages recently
    if (millis() - lastMessageTime > 5000) {
        if (millis() - lastBlink > 1000) {
            digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
            lastBlink = millis();
        }
    } else {
        digitalWrite(LED_STATUS, HIGH);
    }
}