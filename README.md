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
## Com parte visual
# Código ESP32
```bash
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "";          // 👉 seu Wi-Fi
const char* password = "";       // 👉 senha

// URL da API no seu Worker
const char* serverName = "https://workers/api";

// Valores de exemplo
int A = 12;
int B = 8;
String operacao = "soma";  // pode ser: soma, sub, mul, div

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Conectar ao Wi-Fi
  Serial.println("Conectando ao Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado!");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Prepara a URL
    String url = String(serverName) + "?A=" + A + "&B=" + B + "&operacao=" + operacao;

    Serial.println("➡️ Enviando requisição: " + url);

    http.begin(url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      Serial.print("✅ HTTP ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println("📥 Resposta do Worker:");
      Serial.println(payload);
    } else {
      Serial.print("❌ Erro HTTP: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("⚠️ Wi-Fi desconectado");
  }

  delay(10000); // envia a cada 10 segundos
}

```

# Código Index
```bash
export default {
	async fetch(request, env, ctx) {
	  const url = new URL(request.url);
  
	  // 🚀 API para o ESP32
	  if (url.pathname === "/api") {
		// Aqui você poderia salvar em KV ou D1, se quiser persistir os dados
		return new Response(
		  JSON.stringify({
			status: "ok",
			mensagem: "Recebi os dados do ESP32!",
			A: 10,
			B: 20,
			operacao: "soma",
			resultado: 30
		  }),
		  { headers: { "Content-Type": "application/json" } }
		);
	  }
  
	  // 🚀 Frontend HTML
	  if (url.pathname === "/" || url.pathname === "/index.html") {
		const html = `
		<!DOCTYPE html>
		<html lang="pt-BR">
		<head>
		  <meta charset="UTF-8" />
		  <title>ESP32 Operações</title>
		  <style>
			body { font-family: Arial, sans-serif; padding: 20px; background: #f5f5f5; }
			h1 { text-align: center; }
			table { width: 100%; border-collapse: collapse; margin-top: 20px; background: white; }
			th, td { border: 1px solid #ccc; padding: 10px; text-align: center; }
			th { background: #007BFF; color: white; }
		  </style>
		</head>
		<body>
		  <h1>📊 Dados do ESP32</h1>
		  <table>
			<thead>
			  <tr><th>A</th><th>B</th><th>Operação</th><th>Resultado</th></tr>
			</thead>
			<tbody id="tabela"></tbody>
		  </table>
  
		  <script>
			async function atualizar() {
			  const res = await fetch('/api');
			  const data = await res.json();
			  document.getElementById("tabela").innerHTML = 
				'<tr><td>' + data.A + '</td><td>' + data.B + '</td><td>' + data.operacao + '</td><td>' + data.resultado + '</td></tr>';
			}
			atualizar();
			setInterval(atualizar, 5000);
		  </script>
		</body>
		</html>
		`;
		return new Response(html, { headers: { "Content-Type": "text/html;charset=UTF-8" } });
	  }
  
	  return new Response("Rota não encontrada!", { status: 404 });
	}
  };
  
```

```bash

let ultimoPayload = {
  A: null,
  B: null,
  operacao: "nenhuma",
  resultado: null
};

export default {
  async fetch(request, env, ctx) {
    const url = new URL(request.url);

    // 🚀 API
    if (url.pathname === "/api") {
      if (request.method === "POST") {
        // ESP32 manda os dados (A, B e operacao)
        const body = await request.json();
        let resultado = null;
        const { A, B, operacao } = body;

        switch (operacao) {
          case "sum":
            resultado = A + B;
            break;
          case "sub":
            resultado = A - B;
            break;
          case "mul":
            resultado = A * B;
            break;
          case "div":
            if (B === 0) {
              return new Response(
                JSON.stringify({ ok: false, error: "Divisão por zero!" }),
                { headers: { "Content-Type": "application/json" }, status: 400 }
              );
            }
            resultado = A / B;
            break;
          default:
            return new Response(
              JSON.stringify({ ok: false, error: "op deve ser sum|sub|mul|div." }),
              { headers: { "Content-Type": "application/json" }, status: 400 }
            );
        }

        // Salva os últimos dados recebidos + resultado
        ultimoPayload = { A, B, operacao, resultado };

        return new Response(JSON.stringify(ultimoPayload), {
          headers: { "Content-Type": "application/json" }
        });
      }

      if (request.method === "GET") {
        // Front pede os dados processados
        return new Response(JSON.stringify(ultimoPayload), {
          headers: { "Content-Type": "application/json" }
        });
      }
    }

    // 🚀 Frontend HTML
    if (url.pathname === "/" || url.pathname === "/index.html") {
      const html = `
      <!DOCTYPE html>
      <html lang="pt-BR">
      <head>
        <meta charset="UTF-8" />
        <title>ESP32 Operações</title>
        <style>
          body { font-family: Arial, sans-serif; padding: 20px; background: #f5f5f5; }
          h1 { text-align: center; }
          table { width: 100%; border-collapse: collapse; margin-top: 20px; background: white; }
          th, td { border: 1px solid #ccc; padding: 10px; text-align: center; }
          th { background: #007BFF; color: white; }
        </style>
      </head>
      <body>
        <h1>📊 Última Operação do ESP32</h1>
        <table>
          <thead>
            <tr><th>A</th><th>B</th><th>Operação</th><th>Resultado</th></tr>
          </thead>
          <tbody id="tabela"></tbody>
        </table>

        <script>
          async function atualizar() {
            const res = await fetch('/api');
            const data = await res.json();
            document.getElementById("tabela").innerHTML = 
              '<tr>' +
                '<td>' + (data.A ?? '-') + '</td>' +
                '<td>' + (data.B ?? '-') + '</td>' +
                '<td>' + (data.operacao ?? '-') + '</td>' +
                '<td>' + (data.resultado ?? '-') + '</td>' +
              '</tr>';
          }
          atualizar();
          setInterval(atualizar, 5000);
        </script>
      </body>
      </html>
      `;
      return new Response(html, { headers: { "Content-Type": "text/html;charset=UTF-8" } });
    }

    return new Response("Rota não encontrada!", { status: 404 });
  }
};

```
