#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>

// ----------------------------
// Pins Door-ESP32-C3
// ----------------------------

const int PIN_RING_IN          = 2;  // RTC-fähiger Pin für Wake (Klingel-Relais nach GND -> aktiv LOW)
const int PIN_OPEN_DOOR_RELAY  = 3;  // Relais für Türöffner
const int PIN_SPEAKER_RELAY    = 4;  // Relais für Lautsprecher-Stummschaltung

const int RING_ACTIVE_LEVEL    = LOW; 
const int RELAY_ACTIVE_LEVEL   = LOW; // LOW = Relais an (Relaisboard aktiv LOW)

// Türöffner-Dauer
const unsigned long DOOR_OPEN_DURATION = 5000UL;
// Wie lange nach letzter Aktivität (kein Klingeln, kein Türöffner, kein SpeakerMute) bis Sleep
const unsigned long IDLE_BEFORE_SLEEP  = 3000UL;

// ----------------------------
// ESP-NOW
// ----------------------------

// MAC des Gateway-ESP32 (ANPASSEN!)
uint8_t gatewayMac[] = { 0x24, 0x6F, 0x28, 0x11, 0x22, 0x33 };

// Datenstruktur Door -> Gateway
typedef struct {
  bool ringing;       // aktueller Klingelzustand
  bool doorActive;    // ob das Türrelais gerade läuft
} DoorToGatewayMsg;

// Datenstruktur Gateway -> Door
typedef struct {
  bool triggerDoor;   // wenn true -> Tür für 5 Sekunden öffnen
  bool speakerMuted;  // aktueller Mute-Status (von MQTT)
} GatewayToDoorMsg;

esp_now_peer_info_t peerInfo;

// ----------------------------
// Zustände
// ----------------------------

bool lastRingState       = false;
bool currentRingState    = false;

bool doorRelayActive     = false;
unsigned long doorRelayStartTime = 0;

bool speakerMuted        = false;   // kommt vom Gateway über ESP-NOW

volatile bool triggerDoorRequested = false;

unsigned long lastActivityTime     = 0; // für "Ruhe"-Erkennung

// ----------------------------
// Hilfsfunktionen
// ----------------------------

void markActivity() {
  lastActivityTime = millis();
}

void updateSpeakerRelay() {
  digitalWrite(PIN_SPEAKER_RELAY, speakerMuted ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
  Serial.print("Door-ESP: SpeakerMute Relais: ");
  Serial.println(speakerMuted ? "AN (mute)" : "AUS (normal)");
}

void sendDoorStatus(bool ringing) {
  DoorToGatewayMsg msg;
  msg.ringing    = ringing;
  msg.doorActive = doorRelayActive;
  esp_err_t result = esp_now_send(gatewayMac, (uint8_t*)&msg, sizeof(msg));

  Serial.print("Door-ESP: Sende Status -> ringing=");
  Serial.print(ringing ? "true" : "false");
  Serial.print(", doorActive=");
  Serial.println(doorRelayActive ? "true" : "false");
  Serial.print("Door-ESP: esp_now_send result: ");
  Serial.println(result == ESP_OK ? "OK" : "FEHLER");

  markActivity();
}

void startDoorRelayPulse() {
  if (doorRelayActive) return; // doppelt vermeiden

  doorRelayActive = true;
  doorRelayStartTime = millis();
  digitalWrite(PIN_OPEN_DOOR_RELAY, RELAY_ACTIVE_LEVEL);

  Serial.println("Door-ESP: Türöffner-Relais EIN");

  sendDoorStatus(currentRingState);
}

void stopDoorRelayPulse() {
  if (!doorRelayActive) return;

  doorRelayActive = false;
  digitalWrite(PIN_OPEN_DOOR_RELAY, !RELAY_ACTIVE_LEVEL);

  Serial.println("Door-ESP: Türöffner-Relais AUS");

  sendDoorStatus(currentRingState);
}

void goToDeepSleep() {
  Serial.println("Door-ESP: Gehe in Deep-Sleep, warte auf Klingel...");

  // Wakeup auf RingIn, aktiv LOW
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_RING_IN, RING_ACTIVE_LEVEL);

  // Pins in sicheren Zustand
  pinMode(PIN_OPEN_DOOR_RELAY, INPUT);
  pinMode(PIN_SPEAKER_RELAY,  INPUT);
  pinMode(PIN_RING_IN,        INPUT_PULLUP);

  delay(100);
  esp_deep_sleep_start();
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
      Serial.println("Door-ESP: triggerDoor erhalten -> Tür öffnen");
      triggerDoorRequested = true;
      markActivity();
    }

    // SpeakerMute-Status übernehmen
    speakerMuted = msg.speakerMuted;
    updateSpeakerRelay();
    markActivity();

  } else {
    Serial.println("Door-ESP: Unbekanntes Datenformat");
  }
}

// ----------------------------
// Setup & Loop
// ----------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_RING_IN,          INPUT_PULLUP);
  pinMode(PIN_OPEN_DOOR_RELAY,  OUTPUT);
  pinMode(PIN_SPEAKER_RELAY,    OUTPUT);

  digitalWrite(PIN_OPEN_DOOR_RELAY, !RELAY_ACTIVE_LEVEL);
  digitalWrite(PIN_SPEAKER_RELAY,   !RELAY_ACTIVE_LEVEL);

  // WiFi für ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Door-ESP: ESP-NOW Init fehlgeschlagen!");
    goToDeepSleep();
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, gatewayMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Door-ESP: Konnte Peer nicht hinzufügen!");
    goToDeepSleep();
  }

  int pinLevel = digitalRead(PIN_RING_IN);
  currentRingState = (pinLevel == RING_ACTIVE_LEVEL);
  lastRingState    = currentRingState;

  Serial.print("Door-ESP: Initial RingState = ");
  Serial.println(currentRingState ? "true" : "false");

  sendDoorStatus(currentRingState);
  markActivity();
}

void loop() {
  // Klingelzustand lesen
  int pinLevel = digitalRead(PIN_RING_IN);
  currentRingState = (pinLevel == RING_ACTIVE_LEVEL);

  // Klingel-Zustand geändert -> Status senden
  if (currentRingState != lastRingState) {
    Serial.print("Door-ESP: RingState geändert -> ");
    Serial.println(currentRingState ? "true" : "false");
    sendDoorStatus(currentRingState);
    lastRingState = currentRingState;
  }

  // Trigger zum Türöffnen?
  if (triggerDoorRequested) {
    triggerDoorRequested = false;
    startDoorRelayPulse();
  }

  // Türöffner-Timer
  if (doorRelayActive) {
    unsigned long now = millis();
    if (now - doorRelayStartTime >= DOOR_OPEN_DURATION) {
      stopDoorRelayPulse();
    }
  }

  // Deep-Sleep NUR wenn:
  // - nicht geklingelt wird
  // - Türöffner aus
  // - SpeakerMute AUS (sonst müsste Relais ja angezogen bleiben)
  if (!currentRingState && !doorRelayActive && !speakerMuted) {
    if (millis() - lastActivityTime > IDLE_BEFORE_SLEEP) {
      goToDeepSleep();
    }
  }

  delay(20);
}
