#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>

// ----------------------------
// WLAN & MQTT
// ----------------------------

const char* WIFI_SSID     = "SSID";
const char* WIFI_PASSWORD = "WLAN_PASSWORT";

const char* MQTT_BROKER   = "192.168.1.100";
const uint16_t MQTT_PORT  = 1883;

// Topics (alle true/false)
const char* TOPIC_SPEAKER_MUTE  = "wohnung/sprechanlage/SpeakerMute";
const char* TOPIC_RING_TO_OPEN  = "wohnung/sprechanlage/RingToOpen";
const char* TOPIC_RINGING       = "wohnung/sprechanlage/Ringing";
const char* TOPIC_OPENING_DOOR  = "wohnung/sprechanlage/OpeningDoor";

const int PIN_SPEAKER_RELAY    = 26;    // optional: Relais im Gateway
const int RELAY_ACTIVE_LEVEL   = LOW;   // LOW = Relais an

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ----------------------------
// ESP-NOW
// ----------------------------

// MAC des Door-ESP (ANPASSEN!)
uint8_t doorMac[] = { 0xC8, 0x2E, 0x18, 0xAA, 0xBB, 0xCC };

// Structs (müssen identisch zu Door-ESP sein)
typedef struct {
  bool ringing;
  bool doorActive;
} DoorToGatewayMsg;

typedef struct {
  bool triggerDoor;
  bool speakerMuted;
} GatewayToDoorMsg;

esp_now_peer_info_t peerInfo;

// ----------------------------
// Zustände
// ----------------------------

bool ringToOpenEnabled         = false;
bool speakerMuted              = false;

bool lastRingStateFromDoor     = false;
bool currentRingStateFromDoor  = false;

bool lastDoorActiveFromDoor    = false;
bool currentDoorActiveFromDoor = false;

// ----------------------------
// Helper: MQTT
// ----------------------------

void publishBool(const char* topic, bool value) {
  const char* payload = value ? "true" : "false";
  mqttClient.publish(topic, payload, true);
  Serial.print("Gateway: MQTT publish ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(payload);
}

void connectToWiFi() {
  Serial.print("Gateway: Verbinde mit WLAN: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Gateway: WLAN verbunden");
  Serial.print("IP-Adresse: ");
  Serial.println(WiFi.localIP());
}

void updateSpeakerRelay() {
  digitalWrite(PIN_SPEAKER_RELAY, speakerMuted ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
  Serial.print("Gateway: SpeakerRelais (lokal): ");
  Serial.println(speakerMuted ? "MUTE (an)" : "NORMAL (aus)");
}

// sendet aktuellen `speakerMuted` und optional TriggerDoor
void sendStateToDoor(bool triggerDoor) {
  GatewayToDoorMsg msg;
  msg.triggerDoor  = triggerDoor;
  msg.speakerMuted = speakerMuted;

  esp_err_t result = esp_now_send(doorMac, (uint8_t*)&msg, sizeof(msg));
  Serial.print("Gateway: ESP-NOW -> Door: triggerDoor=");
  Serial.print(triggerDoor ? "true" : "false");
  Serial.print(", speakerMuted=");
  Serial.print(speakerMuted ? "true" : "false");
  Serial.print(" => result=");
  Serial.println(result == ESP_OK ? "OK" : "FEHLER");
}

void sendOpenDoorCommand() {
  // Tür öffnen + SpeakerMute-Status mitsenden
  sendStateToDoor(true);
}

// MQTT Callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  message.trim();
  message.toLowerCase();

  Serial.print("Gateway: MQTT Nachricht empfangen. Topic: ");
  Serial.print(topic);
  Serial.print(" Payload: ");
  Serial.println(message);

  bool value = (message == "true");

  if (String(topic) == TOPIC_SPEAKER_MUTE) {
    speakerMuted = value;
    updateSpeakerRelay();
    // neuen SpeakerMute-Status an Door schicken (ohne TriggerDoor)
    sendStateToDoor(false);
  } 
  else if (String(topic) == TOPIC_RING_TO_OPEN) {
    ringToOpenEnabled = value;
    Serial.print("Gateway: RingToOpen: ");
    Serial.println(ringToOpenEnabled ? "AKTIVIERT" : "DEAKTIVIERT");
  }
}

void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Gateway: Verbinde mit MQTT-Broker: ");
    Serial.print(MQTT_BROKER);
    Serial.print(":");
    Serial.println(MQTT_PORT);

    String clientId = "ESP32-Gateway-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Gateway: MQTT verbunden");

      mqttClient.subscribe(TOPIC_SPEAKER_MUTE);
      mqttClient.subscribe(TOPIC_RING_TO_OPEN);

      Serial.println("Gateway: Abonniert:");
      Serial.println(TOPIC_SPEAKER_MUTE);
      Serial.println(TOPIC_RING_TO_OPEN);

      // Anfangszustände publizieren
      publishBool(TOPIC_RINGING, currentRingStateFromDoor);
      publishBool(TOPIC_OPENING_DOOR, currentDoorActiveFromDoor);
      updateSpeakerRelay();
    } else {
      Serial.print("Gateway: MQTT-Verbindung fehlgeschlagen, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Neuer Versuch in 5 Sekunden...");
      delay(5000);
    }
  }
}

// ----------------------------
// ESP-NOW Callbacks
// ----------------------------

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.println("Gateway: ESP-NOW Daten von Door empfangen");

  if (len == sizeof(DoorToGatewayMsg)) {
    DoorToGatewayMsg msg;
    memcpy(&msg, incomingData, sizeof(msg));

    currentRingStateFromDoor  = msg.ringing;
    currentDoorActiveFromDoor = msg.doorActive;

    Serial.print("Gateway: Door -> ringing=");
    Serial.print(currentRingStateFromDoor ? "true" : "false");
    Serial.print(", doorActive=");
    Serial.println(currentDoorActiveFromDoor ? "true" : "false");

    // Ringing Änderungen -> MQTT + ggf. Türöffnung
    if (currentRingStateFromDoor != lastRingStateFromDoor) {
      publishBool(TOPIC_RINGING, currentRingStateFromDoor);

      if (currentRingStateFromDoor == true) {
        if (ringToOpenEnabled) {
          Serial.println("Gateway: RingToOpen aktiv -> Türöffnung an Door senden");
          sendOpenDoorCommand();
        } else {
          Serial.println("Gateway: Es klingelt, aber RingToOpen ist deaktiviert.");
        }
      }

      lastRingStateFromDoor = currentRingStateFromDoor;
    }

    // DoorActive Änderungen -> OpeningDoor Topic
    if (currentDoorActiveFromDoor != lastDoorActiveFromDoor) {
      publishBool(TOPIC_OPENING_DOOR, currentDoorActiveFromDoor);
      lastDoorActiveFromDoor = currentDoorActiveFromDoor;
    }

  } else {
    Serial.println("Gateway: Unbekanntes Datenformat von Door");
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Gateway: ESP-NOW send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolg" : "Fehler");
}

// ----------------------------
// Setup & Loop
// ----------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_SPEAKER_RELAY, OUTPUT);
  digitalWrite(PIN_SPEAKER_RELAY, !RELAY_ACTIVE_LEVEL);

  connectToWiFi();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Gateway: ESP-NOW Init fehlgeschlagen!");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, doorMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Gateway: Konnte Door-Peer nicht hinzufügen!");
    return;
  }

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  connectToMQTT();

  // Anfangszustände
  publishBool(TOPIC_RINGING, currentRingStateFromDoor);
  publishBool(TOPIC_OPENING_DOOR, currentDoorActiveFromDoor);

  // Fail-Safe: Nach Boot Mute standardmäßig AUS
  speakerMuted = false;
  updateSpeakerRelay();
  sendStateToDoor(false);   // triggerDoor = false, speakerMuted = false

  Serial.println("Gateway: Fail-Safe -> SpeakerMute standardmäßig AUS nach Boot.");
}

void loop() {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();

  delay(20);
}
