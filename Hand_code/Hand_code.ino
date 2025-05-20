//HAND:
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define LOW_POWER_TIMEOUT 60000 // 1 minute in milliseconds
#define LOW_POWER_INTERVAL 1000 // Check every 1 second in low power mode
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MPU6050 mpu;
RF24 radio(8, 9); // CE = 8, CSN = 9

const uint64_t pipeOut = 0xF9E8F0F0E1LL;
const uint64_t pipeIn = 0xF9E8F0F0E2LL;
int lastXValue = 0;
int lastYValue = 0;

const int GESTURE_THRESHOLD = 3; // Minimum change in degrees to count as movement
const unsigned long GESTURE_IDLE_TIMEOUT = 1800000;  // 1 min to enter low power
const unsigned long GESTURE_WAKE_DURATION = 50000;  // Must move for at least 2s to wake

unsigned long gestureStillStart = 0;
unsigned long gestureMoveStart = 0;

struct PacketData {
byte xAxisValue;
byte yAxisValue;
byte speedCommand; // 0-254 representing speed
byte commandFlags; // Bitmask for commands (bit 0: stop, bit 1: brake)
} data;

struct FeedbackData {
bool obstacleDetected;
byte batteryLevel;
bool isEmergencyStopped;
float currentSpeed; // in mph
unsigned long uptime; // in seconds
byte systemStatus; // Bitmask for system status
} carStatus;

uint8_t fifoBuffer[64];
uint16_t packetSize;
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool dmpReady = false;
unsigned long lastDisplayUpdate = 0;
unsigned long lastActivityTime = 0;
bool lowPowerMode = false;

// Speed settings (calibrated for 8mph max)
const byte MIN_SPEED = 100; // Corresponds to ~3mph
const byte MAX_SPEED = 254; // Corresponds to ~8mph
const byte BRAKE_SPEED = 50; // Reduced speed for braking

void enterLowPowerMode();
void exitLowPowerMode();
void updateDisplay();

void setup() {
Serial.begin(115200);
Wire.begin();
if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
Serial.println("OLED failed");
while (1);
}
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,0);
display.println("Starting...");
display.display();

mpu.initialize();
if (mpu.dmpInitialize() == 0) {
mpu.setDMPEnabled(true);
dmpReady = true;
packetSize = mpu.dmpGetFIFOPacketSize();
Serial.println("✅ MPU6050 Ready.");
} else {
Serial.println("❌ MPU6050 Failed.");
}

if (!radio.begin()) {
Serial.println("❌ NRF24L01 not responding.");
while (1);
}

radio.setPALevel(RF24_PA_LOW);
radio.setDataRate(RF24_250KBPS);
radio.setChannel(108);
radio.enableAckPayload();
radio.setRetries(5,5);
radio.openWritingPipe(pipeOut);
radio.openReadingPipe(1, pipeIn);


Serial.println("✅ RF Transmitter Ready.");

// Initialize data structure
data.speedCommand = MIN_SPEED;
data.commandFlags = 0;

display.clearDisplay();
display.setCursor(0,0);
display.println("Ready!");
display.display();
lastActivityTime = millis();
}

void loop() {
if (!dmpReady) return;
// Function prototypes

// Check if we should enter low power mode
if (!lowPowerMode && (millis() - lastActivityTime > LOW_POWER_TIMEOUT)) {
enterLowPowerMode();
}
// In low power mode, we check less frequently
if (lowPowerMode) {
delay(LOW_POWER_INTERVAL);
// Check if we should exit low power mode (if there's any activity)
if (radio.available() || mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
exitLowPowerMode();
}
return; // Skip the rest of the loop in low power mode
}

if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

int xValue = constrain(ypr[2] * 180 / M_PI, -90, 90); // Roll
int yValue = constrain(ypr[1] * 180 / M_PI, -90, 90); // Pitch

// Gesture change detection
  if (abs(xValue - lastXValue) > GESTURE_THRESHOLD || abs(yValue - lastYValue) > GESTURE_THRESHOLD) {
    if (!lowPowerMode) {
      lastActivityTime = millis(); // Reset idle timer
    } else {
      if (gestureMoveStart == 0) gestureMoveStart = millis();
      if (millis() - gestureMoveStart > GESTURE_WAKE_DURATION) {
        exitLowPowerMode();
      }
    }
  } else {
    if (!lowPowerMode && millis() - lastActivityTime > GESTURE_IDLE_TIMEOUT) {
      enterLowPowerMode();
    }
    gestureMoveStart = 0; // Reset if no movement
  }

  lastXValue = xValue;
  lastYValue = yValue;

data.xAxisValue = map(xValue, -90, 90, 0, 254);
data.yAxisValue = map(yValue, -90, 90, 254, 0);
// Detect stop gesture (hand flat)
if (abs(ypr[1]) < 0.2 && abs(ypr[2]) < 0.2) {
data.commandFlags |= 0b00000001; // Set stop flag
} else {
data.commandFlags &= ~0b00000001; // Clear stop flag
}
// Detect brake gesture (hand tilted back slightly)
if (ypr[1] < -0.3) {
data.commandFlags |= 0b00000010; // Set brake flag
} else {
data.commandFlags &= ~0b00000010; // Clear brake flag
}

bool success = radio.write(&data, sizeof(data));

if (success) {
Serial.print("✅ Sent | ");
} else {
Serial.print("❌ Failed | ");
}

Serial.print("X: "); Serial.print(xValue);
Serial.print(" | Y: "); Serial.println(yValue);
// Set speed based on forward tilt (only when moving forward)
if (yValue > 10) { // Forward tilt
data.speedCommand = map(yValue, 10, 90, MIN_SPEED, MAX_SPEED);
} else {
data.speedCommand = MIN_SPEED;
}
/// Send control data
radio.stopListening();
bool sent = radio.write(&data, sizeof(data));
radio.startListening();

// Receive feedback
if (radio.available()) {
radio.read(&carStatus, sizeof(carStatus));
//Exit low power mode if we receive data
if (lowPowerMode) {
exitLowPowerMode();
}
}

// Update display every 100ms to avoid flickering
if (millis() - lastDisplayUpdate > 100) {
updateDisplay();
lastDisplayUpdate = millis();
}

}
}


void updateDisplay() {
display.clearDisplay();
display.setCursor(0,0);
// First line: Status and battery
display.print("S:");
display.print(carStatus.currentSpeed, 1); // Print speed with 1 decimal place
display.print("mph B:");
display.print(carStatus.batteryLevel); // Print battery level
display.print("%");
// Second line: Status indicators
display.setCursor(0, 8);
if (carStatus.batteryLevel < 10) {
display.print("LOW BAT! ");
}
if (carStatus.obstacleDetected) {
display.print("OBSTACLE ");
}
if (data.commandFlags & 0b00000001) {
display.print("STOP ");
} else if (data.commandFlags & 0b00000010) {
display.print("BRK ");
}
if (carStatus.isEmergencyStopped) {
display.print("EMERG STOP ");
}
// Third line: System status
display.setCursor(0, 16);
display.print("Uptime: ");
unsigned long hours = carStatus.uptime / 3600;
unsigned long minutes = (carStatus.uptime % 3600) / 60;
unsigned long seconds = carStatus.uptime % 60;
display.print(hours);
display.print(":");
if (minutes < 10) display.print("0");
display.print(minutes);
display.print(":");
if (seconds < 10) display.print("0");
display.print(seconds);
// Fourth line: Error status
display.setCursor(0, 24);
if (carStatus.systemStatus & 0b00000001) {
display.print("MOTOR ERR ");
}
if (carStatus.systemStatus & 0b00000010) {
display.print("SENSOR ERR ");
}
if (carStatus.systemStatus & 0b00000100) {
display.print("RADIO ERR ");
}
if (carStatus.systemStatus & 0b00001000) {
display.print("LOW BATTERY ");
}
display.display();
} // <-- This was the missing closing brace

void enterLowPowerMode() {
Serial.println("Entering low power mode");
lowPowerMode = true;
// Update display with low power message
display.clearDisplay();
display.setCursor(0, 0);
display.println("Low Power Mode");
display.println("%");
display.println("Move to wake");
display.display();
radio.powerDown();
mpu.setSleepEnabled(true);
}
void exitLowPowerMode() {
Serial.println("Exiting low power mode");
lowPowerMode = false;
gestureMoveStart = 0;
lastActivityTime = millis();

// No need to turn display on since it was never off
radio.powerUp();
mpu.setSleepEnabled(false);
// Clear the low power message by updating display normally
updateDisplay();
}

