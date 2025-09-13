# IoT_AMF_2025-02

## Envio para o MQTT

```bash  
#include <WiFi.h>
#include <PubSubClient.h>

// ======== CONFIG Wi-Fi ========
const char* ssid = "NOME_DO_WIFI";
const char* password = "SENHA_DO_WIFI";

// ======== CONFIG MQTT ========
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_topic = "grupo1/teste";

WiFiClient espClient;
PubSubClient client(espClient);

// ======== Conectar Wi-Fi ========
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// ======== Reconectar ao broker ========
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      // conectado
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // Mensagem fixa para teste
  const char* msg = "Ola do ESP32!";
  client.publish(mqtt_topic, msg);

  Serial.print("Publicado: ");
  Serial.println(msg);

  delay(2000);  // publica a cada 2s
}

```

## Com mais variáveis

```bash
#include <WiFi.h>
#include <PubSubClient.h>

// ======== CONFIG Wi-Fi ========
const char* ssid = "NOME_DO_WIFI";
const char* password = "SENHA_DO_WIFI";

// ======== CONFIG MQTT ========
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_topic = "grupo1/teste";

WiFiClient espClient;
PubSubClient client(espClient);

// ======== Conectar Wi-Fi ========
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// ======== Reconectar ao broker ========
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      // conectado
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // Simulação de valores (poderiam ser sensores reais)
  int temp = random(20, 30);   // temperatura simulada
  int umid = random(40, 80);   // umidade simulada
  int luz  = random(200, 800); // luminosidade simulada

  // Buffer para montar a mensagem
  char msg[100];
  sprintf(msg, "TEMP:%dC; UMID:%d%%; LUZ:%d", temp, umid, luz);

  client.publish(mqtt_topic, msg);

  Serial.print("Publicado: ");
  Serial.println(msg);

  delay(2000);  // publica a cada 2s
}


```

## Recebe Dados também
```bash
#include <WiFi.h>
#include <PubSubClient.h>

// ======== CONFIG Wi-Fi ========
const char* ssid = "NOME_DO_WIFI";
const char* password = "SENHA_DO_WIFI";

// ======== CONFIG MQTT ========
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* pub_topic = "grupo1/dados";  // publica dados
const char* sub_topic = "grupo1/cmd";    // recebe comandos

WiFiClient espClient;
PubSubClient client(espClient);

#define LED_PIN 2  // LED no GPIO2 (pode trocar conforme placa)

// ======== Conectar Wi-Fi ========
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// ======== Callback: executado quando chega mensagem no sub_topic ========
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("Mensagem recebida em [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  // Controle do LED
  if (msg == "ON") {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED ligado!");
  } else if (msg == "OFF") {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED desligado!");
  }
}

// ======== Reconectar ao broker ========
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      client.subscribe(sub_topic); // assina o tópico de comandos
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // Publicação de dados simulados
  int temp = random(20, 30);
  int umid = random(40, 80);

  char msg[50];
  sprintf(msg, "TEMP:%dC; UMID:%d%%", temp, umid);

  client.publish(pub_topic, msg);

  Serial.print("Publicado: ");
  Serial.println(msg);

  delay(3000);
}



```

<!--
tudo daqui pra baixo não aparece no GitHub

## Enviar por JSON
 

```
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======== CONFIG Wi-Fi ========
const char* ssid = "NOME_DO_WIFI";
const char* password = "SENHA_DO_WIFI";

// ======== CONFIG MQTT ========
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* pub_topic = "grupo1/dados";  // publica dados
const char* sub_topic = "grupo1/cmd";    // recebe comandos

WiFiClient espClient;
PubSubClient client(espClient);

#define LED_PIN 2  // LED no GPIO2

// ======== Conectar Wi-Fi ========
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// ======== Callback: executado quando chega mensagem no sub_topic ========
void callback(char* topic, byte* payload, unsigned int length) {
  // Converte payload em string
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("Mensagem recebida: ");
  Serial.println(msg);

  // Tentar interpretar como JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (!error) {
    // Exemplo: {"led":"ON"}
    const char* ledCmd = doc["led"];
    if (ledCmd) {
      if (strcmp(ledCmd, "ON") == 0) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED ligado!");
      } else if (strcmp(ledCmd, "OFF") == 0) {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED desligado!");
      }
    }
  } else {
    Serial.println("Erro ao interpretar JSON recebido!");
  }
}

// ======== Reconectar ao broker ========
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      client.subscribe(sub_topic); // assina comandos
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // Simula sensores
  int temp = random(20, 30);
  int umid = random(40, 80);
  int luz  = random(200, 800);

  // Montar JSON
  StaticJsonDocument<200> doc;
  doc["grupo"] = "grupo1";
  doc["temp"]  = temp;
  doc["umid"]  = umid;
  doc["luz"]   = luz;
  doc["uptime"] = millis();

  char buffer[200];
  serializeJson(doc, buffer);

  // Publicar JSON
  client.publish(pub_topic, buffer);
  Serial.print("Publicado JSON: ");
  Serial.println(buffer);

  delay(3000);
}

```

## Envia e Recebe - Com intervalo de tempo para o recebimento

```bash
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ======== CONFIG Wi-Fi ========
const char* ssid = "NOME_DO_WIFI";
const char* password = "SENHA_DO_WIFI";

// ======== CONFIG MQTT ========
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;

const char* pub_topic = "grupo1/dados";   // publica dados
const char* sub_topic = "grupo1/cmd";     // recebe comandos

WiFiClient espClient;
PubSubClient client(espClient);

// ======== CONFIG DE FUNÇÃO ========
// Descomente para ativar os modos
#define ENABLE_PUBLISH
#define ENABLE_SUBSCRIBE

// ======== VARS ========
unsigned long lastMsg = 0;
const long interval = 3000;  // publica a cada 3s
unsigned long subStart = 0;
bool subActive = false;

// ======== Conectar Wi-Fi ========
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
}

// ======== Callback: quando chega mensagem no tópico sub_topic ========
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("Mensagem recebida em [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  // Tenta interpretar como JSON booleano: {"flag":true}
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (!error) {
    bool flag = doc["flag"];
    Serial.print("Booleano recebido: ");
    Serial.println(flag ? "true" : "false");
  } else {
    Serial.println("Erro ao interpretar JSON!");
  }
}

// ======== Reconectar ========
void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conectar ao broker...");
    if (client.connect("ESP32Client")) {
#ifdef ENABLE_SUBSCRIBE
      client.subscribe(sub_topic);
      Serial.println(" - inscrito no tópico de comandos");
      subStart = millis();
      subActive = true;
#endif
    } else {
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

#ifdef ENABLE_PUBLISH
  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;

    // Publica dados simulados
    StaticJsonDocument<200> doc;
    doc["grupo"] = "grupo1";
    doc["valor"] = random(0, 100);

    char buffer[200];
    serializeJson(doc, buffer);

    client.publish(pub_topic, buffer);
    Serial.print("Publicado: ");
    Serial.println(buffer);
  }
#endif

#ifdef ENABLE_SUBSCRIBE
  if (subActive && (millis() - subStart > 60000)) { // 1 min
    Serial.println("⏱ Tempo de 1 min para receber comandos acabou!");
    subActive = false;
    // Se quiser, pode cancelar a inscrição no tópico:
    // client.unsubscribe(sub_topic);
  }
#endif
}


```

-->
