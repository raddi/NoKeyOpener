#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// ----------------------------
// Konfiguration
// ----------------------------

// WLAN
const char* WIFI_SSID     = "Todesstern";
const char* WIFI_PASSWORD = "60394892918901270329";

// MQTT
const char* MQTT_BROKER   = "192.168.108.130";   // <- Deine Broker-IP oder Hostname
const uint16_t MQTT_PORT  = 1883;

// Topics (alle true/false)
const char* TOPIC_SPEAKER_MUTE = "wohnung/sprechanlage/SpeakerMute";  // erwartet "true"/"false"
const char* TOPIC_RING_TO_OPEN = "wohnung/sprechanlage/RingToOpen";   // erwartet "true"/"false"
const char* TOPIC_RINGING      = "wohnung/sprechanlage/Ringing";      // sendet "true"/"false"

// Pins (anpassen, falls nötig)
const int PIN_RING_IN          = 14;  // Eingang von der Klingel (potentialfreies Relais)
const int PIN_OPEN_DOOR_RELAY  = 12;  // Relais für Türöffner
const int PIN_SPEAKER_RELAY    = 13;  // Relais für Lautsprecher stumm/aktiv

// Pegel-Definitionen
// -> RingIn aktiv LOW (Relaiskontakt nach GND)
// -> Relais aktiv LOW (Standard-Relaisboard)
const int RING_ACTIVE_LEVEL   = LOW; 
const int RELAY_ACTIVE_LEVEL  = LOW; // LOW = Relais an

// Türöffner-Dauer (ms)
const unsigned long DOOR_OPEN_DURATION = 5000UL; // 5 Sekunden

// ----------------------------
// Globale Variablen
// ----------------------------

WiFiClient espClient;
PubSubClient mqttClient(espClient);

bool ringToOpenEnabled = false;  // Wert aus RingToOpen-Topic
bool speakerMuted      = false;  // Wert aus SpeakerMute-Topic

bool lastRingState     = false;  // letzter Klingelzustand (für Flankenerkennung)
bool currentRingState  = false;  // aktueller Klingelzustand

bool doorRelayActive   = false;  // ob Türöffner gerade läuft
unsigned long doorRelayStartTime = 0;

// ----------------------------
// Hilfsfunktionen
// ----------------------------

void connectToWiFi() {
  Serial.print("Verbinde mit WLAN: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WLAN verbunden");
  Serial.print("IP-Adresse: ");
  Serial.println(WiFi.localIP());
}

void publishRingingState(bool ringing) {
  const char* payload = ringing ? "true" : "false";
  Serial.print("MQTT publish Ringing = ");
  Serial.println(payload);
  mqttClient.publish(TOPIC_RINGING, payload, true); // retained
}

void updateSpeakerRelay() {
  // speakerMuted = true  -> Relais AN (Lautsprecher stumm)
  // speakerMuted = false -> Relais AUS (normal)
  digitalWrite(PIN_SPEAKER_RELAY, speakerMuted ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
  Serial.print("SpeakerRelais: ");
  Serial.println(speakerMuted ? "MUTE (Relais an)" : "NORMAL (Relais aus)");
}

void startDoorRelayPulse() {
  doorRelayActive = true;
  doorRelayStartTime = millis();
  digitalWrite(PIN_OPEN_DOOR_RELAY, RELAY_ACTIVE_LEVEL);
  Serial.println("Türöffner-Relais EIN (RingToOpen aktiv + Klingel betätigt)");
}

void stopDoorRelayPulse() {
  doorRelayActive = false;
  digitalWrite(PIN_OPEN_DOOR_RELAY, !RELAY_ACTIVE_LEVEL);
  Serial.println("Türöffner-Relais AUS (Zeit abgelaufen)");
}

// ----------------------------
// MQTT Callback
// ----------------------------

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Payload in String umwandeln
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  message.trim();
  message.toLowerCase();

  Serial.print("MQTT Nachricht empfangen. Topic: ");
  Serial.print(topic);
  Serial.print(" Payload: ");
  Serial.println(message);

  // Boolean aus "true"/"false" ableiten
  // alles, was nicht exakt "true" ist, wird als false gewertet
  bool value = (message == "true");

  // SpeakerMute
  if (String(topic) == TOPIC_SPEAKER_MUTE) {
    speakerMuted = value;
    updateSpeakerRelay();
  }

  // RingToOpen
  else if (String(topic) == TOPIC_RING_TO_OPEN) {
    ringToOpenEnabled = value;
    Serial.print("RingToOpen: ");
    Serial.println(ringToOpenEnabled ? "AKTIVIERT" : "DEAKTIVIERT");
  }
}

// ----------------------------
// MQTT-Verbindung aufbauen
// ----------------------------

void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Verbinde mit MQTT-Broker: ");
    Serial.print(MQTT_BROKER);
    Serial.print(":");
    Serial.println(MQTT_PORT);

    String clientId = "ESP32-DoorOpener-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("MQTT verbunden");

      mqttClient.subscribe(TOPIC_SPEAKER_MUTE);
      mqttClient.subscribe(TOPIC_RING_TO_OPEN);

      Serial.println("Abonniert:");
      Serial.println(TOPIC_SPEAKER_MUTE);
      Serial.println(TOPIC_RING_TO_OPEN);

      // aktuellen Ring-Status & Relais-Status beim Connect publizieren
      publishRingingState(currentRingState);
      updateSpeakerRelay();
    } else {
      Serial.print("MQTT-Verbindung fehlgeschlagen, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Neuer Versuch in 5 Sekunden...");
      delay(5000);
    }
  }
}

// ----------------------------
// Setup & Loop
// ----------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Pins
  pinMode(PIN_RING_IN, INPUT_PULLUP); // Klingel über Relais nach GND -> aktiv LOW
  pinMode(PIN_OPEN_DOOR_RELAY, OUTPUT);
  pinMode(PIN_SPEAKER_RELAY, OUTPUT);

  // Relais initial AUS
  digitalWrite(PIN_OPEN_DOOR_RELAY, !RELAY_ACTIVE_LEVEL);
  digitalWrite(PIN_SPEAKER_RELAY, !RELAY_ACTIVE_LEVEL);

  connectToWiFi();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  connectToMQTT();

  // Startzustand Ringing ermitteln
  int pinLevel = digitalRead(PIN_RING_IN);
  currentRingState = (pinLevel == RING_ACTIVE_LEVEL);
  lastRingState = currentRingState;
  publishRingingState(currentRingState);
}

void loop() {
  // MQTT-Verbindung halten
  if (!mqttClient.connected()) {
    connectToMQTT();
  }
  mqttClient.loop();

  // Klingelzustand einlesen
  int pinLevel = digitalRead(PIN_RING_IN);
  currentRingState = (pinLevel == RING_ACTIVE_LEVEL);

  // 1) Ringing-Topic bei Änderungen aktualisieren
  if (currentRingState != lastRingState) {
    publishRingingState(currentRingState);

    // 2) Türöffner nur auf steigende "Klingel-Flanke"
    //    (Übergang von NICHT klingeln -> klingeln)
    if (currentRingState == true) {
      if (ringToOpenEnabled) {
        startDoorRelayPulse();
      } else {
        Serial.println("Es klingelt, aber RingToOpen ist deaktiviert.");
      }
    }

    lastRingState = currentRingState;
  }

  // 3) Türöffner-Zeitsteuerung
  if (doorRelayActive) {
    unsigned long now = millis();
    if (now - doorRelayStartTime >= DOOR_OPEN_DURATION) {
      stopDoorRelayPulse();
    }
  }

  delay(20); // kleine Entprell-/Entlastungspause
}