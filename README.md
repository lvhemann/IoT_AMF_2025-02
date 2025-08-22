# IoT_AMF_2025-02

## Código Wifi ESP32
```bash
#include <WiFi.h>
#include <HTTPClient.h>

// Substitua abaixo pelos seus dados em seu ambiente (não compartilhe!)
const char* WIFI_SSID = "AMF";
const char* WIFI_PASS = " ";  // Troque aqui com sua senha real
const char* WORKER_URL = "https://workersdev/"; // aqui vai o URL do Works

void setup() {
  Serial.begin(115200);
  Serial.print("Conectando ao Wi-Fi ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado!");

  // Teste um GET simples
  HTTPClient http;
  if (http.begin(WORKER_URL)) {
    int code = http.GET();
    Serial.printf("GET HTTP %d\n", code);
    if (code == HTTP_CODE_OK) {
      String body = http.getString();
      Serial.println("Resposta do Worker:");
      Serial.println(body);
    } else {
      Serial.printf("Erro %d ao chamar o Worker\n", code);
    }
    http.end();
  } else {
    Serial.println("Falha ao iniciar http.begin()");
  }
}

void loop() {
  // deixar vazio após o teste
}

```

## Código para o Cloudflare Workers

```bash
export default {
	async fetch(request, env, ctx) {
	  if (request.method === "POST") {
		const data = await request.json();
		return new Response(JSON.stringify({
		  status: "ok",
		  recebido: data
		}), {
		  headers: { "Content-Type": "application/json" }
		});
	  }
  
	  if (request.method === "GET") {
		return new Response(JSON.stringify({
		  status: "ok",
		  mensagem: "API do Worker funcionando!"
		}), {
		  headers: { "Content-Type": "application/json" }
		});
	  }
  
	  return new Response("Método não permitido", { status: 405 });
	}
  }
  

```
## Implementação Soma
# ESP32
```bash
#include <WiFi.h>
#include <HTTPClient.h>

const char* WIFI_SSID = "AMF";
const char* WIFI_PASS = "SUA_SENHA_AQUI";  // coloque a senha real
const char* WORKER_URL = "https://workers-playground-cool-poetry-36d6.lvhemann.workers.dev/";

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Conectando ao Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado!");
  
  sendCalcGET(10, 5, "sum");
  sendCalcGET(10, 5, "mul");
  sendCalcGET(10, 5, "div");
}

void loop() {}

void sendCalcGET(float a, float b, const char* op) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String url = String(WORKER_URL) + "?a=" + a + "&b=" + b + "&op=" + op;
    Serial.println("Chamando URL: " + url);

    http.begin(url);
    int httpCode = http.GET();

    if (httpCode > 0) {
      Serial.printf("HTTP %d\n", httpCode);
      String payload = http.getString();
      Serial.println("Resposta: " + payload);
    } else {
      Serial.printf("Erro HTTP: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  }
}

```
# Worker
```bash
export default {
  async fetch(request) {
    const url = new URL(request.url);
    const a = parseFloat(url.searchParams.get("a") || "0");
    const b = parseFloat(url.searchParams.get("b") || "0");
    const op = url.searchParams.get("op") || "sum";

    let result;
    switch (op) {
      case "sum": result = a + b; break;
      case "sub": result = a - b; break;
      case "mul": result = a * b; break;
      case "div": result = b !== 0 ? a / b : null; break;
      default: result = null;
    }

    return new Response(
      JSON.stringify({ a, b, op, result }),
      { headers: { "Content-Type": "application/json" } }
    );
  }
};

```
