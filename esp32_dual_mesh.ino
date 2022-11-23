// LoRa requirements
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <EEPROM.h>
#include <mutex>
// Display requirement
#include "SSD1306Wire.h"

// ******************************************
// INITS
// ******************************************

#define LED_BUILTIN 25
#define SS 18
#define RST 14
#define DI0 26

// Integer stored in EEPROM for NodeID?
#define EEPROM_SIZE 1

// Battery Voltage Inits
#define BAT_READ 37
#define BAT_DIVIDER 21
// Battery voltage multipliers
float XS = 0.0029679;
uint16_t MUL = 1000;
uint16_t MMUL = 100;
unsigned long lastBatRead = 0;
String battery_status = "";
float actualVoltage = 0;
bool battery_status_updated;
std::mutex battery_lock;
TaskHandle_t batteryTask;

// OLED init
SSD1306Wire display(0x3c, 4, 15);

// LoRa Driver/Manager init
RH_RF95 driver(SS, DI0);
RHMesh manager(driver, 2);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
int16_t lastRssi = 0;
uint8_t neighbour = 255;
// TODO: Delete
uint8_t data[] = "ACK";
uint8_t nodeNum;

// ******************************************
// Setup Methods
// ******************************************
void setupDisplay() {

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH);
    // Display init
  display.init();
  display.setFont(ArialMT_Plain_16);
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.display();
  SPI.begin(5, 19, 27, 18);  
}

void setupBatteryReading() {
  adcAttachPin(BAT_READ);
  pinMode(BAT_DIVIDER, OUTPUT);
}

// Read eeprom?
// Current Button press for nodeID
void nodeIDSetup() {
  EEPROM.begin(EEPROM_SIZE);
  uint8_t stored_node = EEPROM.read(0);
  pinMode(0, INPUT_PULLUP);
  unsigned long startTs = millis();
  unsigned long lastPress = millis();
  int lastState = digitalRead(0);
  int currentState = digitalRead(0);
  nodeNum = (stored_node != 0) ? stored_node : 10;
  // uint8_t nodeNum = 10;
  // Sets up that there node so we don't have conflict issues.
  while(millis() - startTs < 5000) {
    display.clear();
    display.drawString(5, 5, "Node ID Setup");
    int reading = digitalRead(0);
    if (reading != lastState) {
      lastPress = millis();
    }
    if (millis() - lastPress > 50) {
      if (reading != currentState) {
        currentState = reading;
        if (reading == LOW) {
          // TODO: change away from 0 here.
          nodeNum = (nodeNum < 20) ? nodeNum + 1 : 1;
        }
      }
    }
    String out = String("Node Num:");
    out.concat(nodeNum);
    display.drawString(5, 25, out);
    display.display();
    lastState = reading;
  }

  // Store current Node id for future ease of use
  EEPROM.write(0, nodeNum);
  EEPROM.commit();
  manager.setThisAddress(nodeNum);
  if (!manager.init())
    Serial.println("init failed");
  driver.setFrequency(915.0);  // US band
  driver.setTxPower(14, false);
  manager.setTimeout(5000);
  manager.setRetries(2);
}

void setup() {
  setupBatteryReading();
  setupDisplay();
  Serial.begin(38400);
  nodeIDSetup();
  xTaskCreatePinnedToCore(batteryMonitoring, "BatteryMon", 10000, NULL, 1,&batteryTask,0);
}

// ******************************************
// Battery Voltage Monitoring Loop
// ******************************************
String checkBatteryVoltage() {
  lastBatRead = millis();
  uint16_t c1 = readBatteryVoltage();
  String out = String("V:");
  actualVoltage = float(c1)/1000;
  out.concat(actualVoltage);
  return out;
}
uint16_t readBatteryVoltage() {
  // Need to pull down 21 in order to get a more accurate reading of the voltage.
  digitalWrite(BAT_DIVIDER, LOW);
  // Buffer read to clear out previous value. 
  analogRead(BAT_READ);
  analogRead(BAT_READ);
  analogRead(BAT_READ);
  uint16_t c1  =  analogRead(37)*XS*MUL;
  digitalWrite(BAT_DIVIDER, HIGH);
  return c1;
}

// Potentially could do more here i guess.
void batteryMonitoring( void * pvParameters ) {
  for(;;) {
    // Check battery voltage
    battery_lock.lock();
    battery_status = checkBatteryVoltage();
    battery_status_updated = true;
    battery_lock.unlock();
    delay(7000);
  }
}

// ******************************************
// Main loop
// ******************************************

//TODO: Maybe do a 'push button to verify connection to Node 0?'
char drawBuffer[64];
String old_battery_status;
bool should_keep_alive;
void loop() {
  
  // There is justification for not clearing this less frequently to save power?
  display.clear();
  // TODO: Update Keep-Alive process??
  battery_lock.lock();
  if (battery_status_updated) {
    old_battery_status = battery_status;
    battery_status_updated = false;
    // Send keep-alive
    should_keep_alive = true;
  }
  battery_lock.unlock();
  display.drawString(75, 40, old_battery_status);
  display.drawStringf(5, 19, drawBuffer, "RX RSSI: %i", lastRssi);
  display.drawStringf(5,5, drawBuffer, "N: %i", (int)nodeNum);
  display.drawStringf(5, 31, drawBuffer, "RX: %i", neighbour);

  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    uint8_t hops;
    if (manager.recvfromAck(buf, &len, &from))
    {
      // TODO: remove debugging Serial messages
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      lastRssi = driver.lastRssi();   
      neighbour = from;
      // manager.printRoutingTable();  
    }
  }

  if (should_keep_alive) {
    printf("\n\nkeepalive");
    char charBuff[12];
    sprintf(charBuff, "%f", actualVoltage);
    Serial.printf("SendToWait: %i", manager.sendtoWait((uint8_t *)charBuff, sizeof((uint8_t *)charBuff), 1));
    should_keep_alive = false;
  }
  display.display();
}