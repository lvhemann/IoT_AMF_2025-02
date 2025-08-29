## Aula 29/08

# Para o Cloud
```bash
export default {
  async fetch(request, env) {
    try {
      const url = new URL(request.url);

      if (url.pathname === "/insert" && request.method === "POST") {
        let body;
        try {
          body = await request.json();
        } catch (e) {
          return new Response("Erro no JSON recebido: " + e.message, { status: 400 });
        }

        if (!env.SENSOR_DB) {
          return new Response("ERRO: Binding SENSOR_KV não encontrado!", { status: 500 });
        }

        const nome = body.sensor || "temp";
        const valor = body.valor || "0";

        await env.SENSOR_DB.put(`sensor:${nome}`, JSON.stringify({
          valor: valor,
          timestamp: Date.now()
        }));

        return new Response(`OK: ${nome}=${valor}`);
      }

      if (url.pathname === "/get") {
        const nome = url.searchParams.get("sensor");
        if (!nome) return new Response("Informe ?sensor=temp", { status: 400 });

        const data = await env.SENSOR_DB.get(`sensor:${nome}`, { type: "json" });
        if (!data) return new Response("Nenhum valor encontrado", { status: 404 });

        return new Response(JSON.stringify(data, null, 2), {
          headers: { "Content-Type": "application/json" }
        });
      }

      return new Response("Use POST /insert ou GET /get?sensor=nome");
    } catch (e) {
      return new Response("Erro inesperado: " + e.message, { status: 500 });
    }
  }
}

```

# Inserção
```bash
Invoke-RestMethod -Uri "https://teste-kv.lvhemann.workers.dev/insert" `
  -Method POST `
  -Body (@{ sensor="temp"; valor=27.8 } | ConvertTo-Json) `
  -ContentType "application/json"

```

# Leitura

```bash

Invoke-RestMethod -Uri "https://teste-kv.lvhemann.workers.dev/get?sensor=temp"

```

# Para o ESP32
```bash
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "SEU_WIFI";
const char* password = "SENHA_WIFI";

// URL do Worker, já com o nome do sensor
String url = "https://teste-kv.lvhemann.workers.dev/get?sensor=temp";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(url);

    int httpCode = http.GET();

    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("Resposta JSON: " + payload);

      // 👉 se quiser pegar só o valor (sem usar ArduinoJson), dá pra procurar o número
      int pos = payload.indexOf("valor");
      if (pos > 0) {
        int start = payload.indexOf(":", pos) + 1;
        int end = payload.indexOf(",", pos);
        String valor = payload.substring(start, end);
        valor.trim();
        Serial.println("Valor do sensor: " + valor);
      }

    } else {
      Serial.printf("Erro HTTP: %d\n", httpCode);
    }

    http.end();
  }

  delay(10000); // consulta a cada 10 segundos
}


```


# Para um valor determinado 
```bash
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "SEU_WIFI";
const char* password = "SENHA_WIFI";

// URL do seu Worker que lê do KV
String url = "https://teste-kv.lvhemann.workers.dev/get?sensor=temp";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(url);

    int httpCode = http.GET();

    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("JSON recebido: " + payload);

      // Parse do JSON
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        float valor = doc["valor"];
        long timestamp = doc["timestamp"];

        if (valor > 30) {
          Serial.println("⚠️ Valor acima de 30 detectado!");
          Serial.println("Valor: " + String(valor) + " | Timestamp: " + String(timestamp));
          // 👉 aqui você pode acionar um relé, buzzer, LED etc.
        } else {
          Serial.println("Valor abaixo do limite: " + String(valor));
        }
      } else {
        Serial.println("Erro ao processar JSON");
      }
    } else {
      Serial.printf("Erro HTTP: %d\n", httpCode);
    }

    http.end();
  }

  delay(10000); // lê a cada 10 segundos
}


```
