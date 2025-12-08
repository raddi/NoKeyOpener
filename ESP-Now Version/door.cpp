#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ----------------------------
// Pins Door-ESP
// ----------------------------

const int PIN_RING_IN         = 32; // Eingang von der Klingel (Relais nach GND -> aktiv LOW)
const int PIN_OPEN_DOOR_RELAY = 25; // Relais für Türöffner

const int RING_ACTIVE_LEVEL   = LOW; 
const int RELAY_ACTIVE_LEVEL  = LOW; // LOW = Relais an (Relaisboard aktiv LOW)

// Türöffner-Dauer
const unsigned long DOOR_OPEN_DURATION = 5000UL;

// ----------------------------
// ESP-NOW
// ----------------------------

// MAC des Gateway-ESP32 (anpassen!)
uint8_t gatewayMac[] = { 0x24, 0x6F, 0x28, 0x11, 0x22, 0x33 };

// Datenstruktur Door -> Gateway
typedef struct {
  bool ringing;       // aktueller Klingelzustand
  bool doorActive;    // ob das Türrelais gerade läuft (optional, für Anzeige)
} DoorToGatewayMsg;

// Datenstruktur Gateway -> Door
typedef struct {
  bool triggerDoor;   // wenn true -> Tür 5 Sekunden öffnen
} GatewayToDoorMsg;

esp_now_peer_info_t peerInfo;

// ----------------------------
// Zustände
// ----------------------------

bool lastRingState     = false;
bool currentRingState  = false;

bool doorRelayActive   = false;
unsigned long doorRelayStartTime = 0;

// ----------------------------
// Hilfsfunktionen
// ----------------------------

void startDoorRelayPulse() {
  doorRelayActive = true;
  doorRelayStartTime = millis();
  digitalWrite(PIN_OPEN_DOOR_RELAY, RELAY_ACTIVE_LEVEL);

  Serial.println("Door-ESP: Türöffner-Relais EIN (ESP-NOW Befehl)");
  
  // Status an Gateway senden (optional)
  DoorToGatewayMsg msg;
  msg.ringing = currentRingState;
  msg.doorActive = true;
  esp_now_send(gatewayMac, (uint8_t*)&msg, sizeof(msg));
}

void stopDoorRelayPulse() {
  doorRelayActive = false;
  digitalWrite(PIN_OPEN_DOOR_RELAY, !RELAY_ACTIVE_LEVEL);

  Serial.println("Door-ESP: Türöffner-Relais AUS (Zeit abgelaufen)");

  // Status an Gateway senden (optional)
  DoorToGatewayMsg msg;
  msg.ringing = currentRingState;
  msg.doorActive = false;
  esp_now_send(gatewayMac, (uint8_t*)&msg, sizeof(msg));
}

void sendRingingState(bool ringing) {
  DoorToGatewayMsg msg;
  msg.ringing = ringing;
  msg.doorActive = doorRelayActive;
  esp_now_send(gatewayMac, (uint8_t*)&msg, sizeof(msg));

  Serial.print("Door-ESP: Sende Ringing = ");
  Serial.println(ringing ? "true" : "false");
}

// ----------------------------
// ESP-NOW Callbacks
// ----------------------------

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Door-ESP: ESP-NOW send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolg" : "Fehler");
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println("Door-ESP: Daten von Gateway empfangen");

  if (len == sizeof(GatewayToDoorMsg)) {
    GatewayToDoorMsg msg;
    memcpy(&msg, incomingData, sizeof(msg));

    if (msg.triggerDoor) {
      Serial.println("Door-ESP: TriggerDoor erhalten -> Tür öffnen");
      startDoorRelayPulse();
    }
  } else {
    Serial.println("Door-ESP: Unbekanntes Datenformat");
  }
}

// ----------------------------
// Setup & Loop
// ----------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_RING_IN, INPUT_PULLUP);
  pinMode(PIN_OPEN_DOOR_RELAY, OUTPUT);

  // Relais aus
  digitalWrite(PIN_OPEN_DOOR_RELAY, !RELAY_ACTIVE_LEVEL);

  // WiFi nur im STA Modus (für ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("Door-ESP: ESP-NOW Init fehlgeschlagen!");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Peer (Gateway) hinzufügen
  memcpy(peerInfo.peer_addr, gatewayMac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Door-ESP: Konnte Peer nicht hinzufügen!");
    return;
  }

  // Startzustand von RingIn lesen
  int pinLevel = digitalRead(PIN_RING_IN);
  currentRingState = (pinLevel == RING_ACTIVE_LEVEL);
  lastRingState = currentRingState;

  // Anfangsstatus senden
  sendRingingState(currentRingState);
}

void loop() {
  // Klingelzustand einlesen
  int pinLevel = digitalRead(PIN_RING_IN);
  currentRingState = (pinLevel == RING_ACTIVE_LEVEL);

  // Wenn sich der Zustand geändert hat -> an Gateway senden
  if (currentRingState != lastRingState) {
    sendRingingState(currentRingState);
    lastRingState = currentRingState;
  }

  // Türöffner-Timer
  if (doorRelayActive) {
    unsigned long now = millis();
    if (now - doorRelayStartTime >= DOOR_OPEN_DURATION) {
      stopDoorRelayPulse();
    }
  }

  delay(20);
}