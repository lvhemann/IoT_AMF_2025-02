# IoT_AMF_2025-02

## Criação de Banco D1

```bash  
 export default {
  async fetch(request, env) {
    const url = new URL(request.url);

    try {
      if (!env.teste01) {
        return new Response("ERRO: Binding DB_SENSORES não encontrado!", { status: 500 });
      }

      if (url.pathname === "/init") {
        await env.teste01.exec(`
          CREATE TABLE IF NOT EXISTS sensores (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            nome TEXT,
            valor REAL,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
          )
        `);
        return new Response("Tabela criada com sucesso!");
      }

      if (url.pathname === "/list") {
        let { results } = await env.teste01
          .prepare("SELECT * FROM sensores ORDER BY timestamp DESC LIMIT 5")
          .all();
        return new Response(JSON.stringify(results, null, 2), {
          headers: { "Content-Type": "application/json" }
        });
      }

      return new Response("Use /init ou /list");
    } catch (e) {
      return new Response("Erro ao acessar D1: " + e.message, { status: 500 });
    }
  }
}

```


## Código com Insert e com SELECT * FROM
```bash
export default {
  async fetch(request, env) {
    const url = new URL(request.url);

    try {
      if (!env.DB_SENSORES) {
        return new Response("ERRO: Binding DB_SENSORES não encontrado!", { status: 500 });
      }

      // criar tabela
      if (url.pathname === "/init") {
        await env.DB_SENSORES.exec(
          "CREATE TABLE IF NOT EXISTS sensores (" +
          "id INTEGER PRIMARY KEY AUTOINCREMENT, " +
          "nome TEXT, " +
          "valor REAL, " +
          "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP" +
          ")"
        );
        return new Response("Tabela criada com sucesso!");
      }

      // inserir via POST JSON
      if (url.pathname === "/insert" && request.method === "POST") {
        const body = await request.json();
        const nome = body.sensor || "temp";
        const valor = parseFloat(body.valor || 0);

        await env.DB_SENSORES
          .prepare("INSERT INTO sensores (nome, valor) VALUES (?, ?)")
          .bind(nome, valor)
          .run();

        return new Response(`OK: inserido ${nome}=${valor}`);
      }

      // listar últimos 10
      if (url.pathname === "/list") {
        const { results } = await env.DB_SENSORES
          .prepare("SELECT * FROM sensores ORDER BY timestamp DESC LIMIT 10")
          .all();

        return new Response(JSON.stringify(results, null, 2), {
          headers: { "Content-Type": "application/json" }
        });
      }

      // obter por ID
      if (url.pathname === "/get") {
        const id = url.searchParams.get("id");
        if (!id) return new Response("Informe ?id=", { status: 400 });

        const row = await env.DB_SENSORES
          .prepare("SELECT * FROM sensores WHERE id = ?")
          .bind(id)
          .first();

        if (!row) return new Response("Nenhum dado encontrado", { status: 404 });

        return new Response(JSON.stringify(row, null, 2), {
          headers: { "Content-Type": "application/json" }
        });
      }

      // pesquisa filtrada
      if (url.pathname === "/search") {
        const nome = url.searchParams.get("sensor");
        const min = parseFloat(url.searchParams.get("min") || "0");

        if (!nome) return new Response("Informe ?sensor=nome", { status: 400 });

        const { results } = await env.DB_SENSORES
          .prepare("SELECT * FROM sensores WHERE nome = ? AND valor >= ? ORDER BY timestamp DESC LIMIT 10")
          .bind(nome, min)
          .all();

        return new Response(JSON.stringify(results, null, 2), {
          headers: { "Content-Type": "application/json" }
        });
      }

      return new Response("Use /init, /insert, /list, /get?id=, /search?sensor=nome&min=30");
    } catch (e) {
      return new Response("Erro ao acessar D1: " + e.message, { status: 500 });
    }
  }
}
```

## Código com criação de outras colunas

```bash
export default {
  async fetch(request, env) {
    const url = new URL(request.url);

    try {
      if (!env.DB_SENSORES) {
        return new Response("ERRO: Binding DB_SENSORES não encontrado!", { status: 500 });
      }

      // 🔹 criar tabela inicial
      if (url.pathname === "/init") {
        await env.DB_SENSORES.exec(
          "CREATE TABLE IF NOT EXISTS sensores (" +
          "id INTEGER PRIMARY KEY AUTOINCREMENT, " +
          "nome TEXT, " +
          "valor REAL, " +
          "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP" +
          ")"
        );
        return new Response("Tabela criada com sucesso!");
      }

      // 🔹 inserir dado via POST JSON
      if (url.pathname === "/insert" && request.method === "POST") {
        const body = await request.json();
        const nome = body.sensor || "temp";
        const valor = parseFloat(body.valor || 0);

        await env.DB_SENSORES
          .prepare("INSERT INTO sensores (nome, valor) VALUES (?, ?)")
          .bind(nome, valor)
          .run();

        return new Response(`OK: inserido ${nome}=${valor}`);
      }

      // 🔹 listar últimos 10 registros
      if (url.pathname === "/list") {
        const { results } = await env.DB_SENSORES
          .prepare("SELECT * FROM sensores ORDER BY timestamp DESC LIMIT 10")
          .all();

        return new Response(JSON.stringify(results, null, 2), {
          headers: { "Content-Type": "application/json" }
        });
      }

      // 🔹 obter registro por ID
      if (url.pathname === "/get") {
        const id = url.searchParams.get("id");
        if (!id) return new Response("Informe ?id=", { status: 400 });

        const row = await env.DB_SENSORES
          .prepare("SELECT * FROM sensores WHERE id = ?")
          .bind(id)
          .first();

        if (!row) return new Response(`Nenhum dado encontrado para id=${id}`, { status: 404 });

        return new Response(JSON.stringify(row, null, 2), {
          headers: { "Content-Type": "application/json" }
        });
      }

      // 🔹 pesquisa filtrada (por sensor e valor mínimo)
      if (url.pathname === "/search") {
        const nome = url.searchParams.get("sensor");
        const min = parseFloat(url.searchParams.get("min") || "0");

        if (!nome) return new Response("Informe ?sensor=nome", { status: 400 });

        const { results } = await env.DB_SENSORES
          .prepare("SELECT * FROM sensores WHERE nome = ? AND valor >= ? ORDER BY timestamp DESC LIMIT 10")
          .bind(nome, min)
          .all();

        return new Response(JSON.stringify(results, null, 2), {
          headers: { "Content-Type": "application/json" }
        });
      }

      // 🔹 rota para adicionar coluna (migração controlada)
      if (url.pathname === "/migrate" && request.method === "POST") {
        const body = await request.json();
        const coluna = body.coluna;
        const tipo = body.tipo;
        const secret = body.secret;

        // senha de proteção (altere para algo só seu)
        const MASTER_KEY = "minha_senha_super_secreta";

        if (secret !== MASTER_KEY) {
          return new Response("Acesso negado", { status: 403 });
        }

        if (!coluna || !tipo) {
          return new Response("Informe {coluna, tipo}", { status: 400 });
        }

        await env.DB_SENSORES.exec(`ALTER TABLE sensores ADD COLUMN ${coluna} ${tipo}`);

        return new Response(`Coluna '${coluna}' adicionada como ${tipo}`);
      }

      // rota padrão
      return new Response("Use /init, /insert, /list, /get?id=, /search?sensor=nome&min=, ou POST /migrate");
    } catch (e) {
      return new Response("Erro ao acessar D1: " + e.message, { status: 500 });
    }
  }
}


```


## Código para o Esp32 - inserção

```bash
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "SEU_WIFI";
const char* password = "SENHA_WIFI";
const char* serverName = "https://work-d1.lvhemann.workers.dev/insert";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado ao WiFi!");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"sensor\":\"temp\",\"valor\":31.2}";
    int httpCode = http.POST(payload);

    Serial.print("HTTP Response code: ");
    Serial.println(httpCode);

    if (httpCode > 0) {
      String response = http.getString();
      Serial.println("Resposta: " + response);
    }
    http.end();
  }
  delay(10000); // envia a cada 10s
}


```

## Código para buscar
```bash
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "SEU_WIFI";
const char* password = "SENHA_WIFI";
// pesquisa: valores do sensor temp acima de 30
String url = "https://work-d1.lvhemann.workers.dev/search?sensor=temp&min=30";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado ao WiFi!");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("Resultados: " + payload);
    } else {
      Serial.printf("Erro HTTP: %d\n", httpCode);
    }
    http.end();
  }
  delay(15000); // consulta a cada 15s
}

```


## Para enviar dados
```bash
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "SEU_WIFI";
const char* password = "SENHA_WIFI";

// URL do seu Worker D1
const char* serverName = "https://work-d1.lvhemann.workers.dev/insert";

// função auxiliar para enviar dado
void enviarDado(String sensor, float valor) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");

    // monta JSON dinamicamente
    String payload = "{\"sensor\":\"" + sensor + "\",\"valor\":" + String(valor, 2) + "}";

    int httpCode = http.POST(payload);
    Serial.print("HTTP Response code: ");
    Serial.println(httpCode);

    if (httpCode > 0) {
      String response = http.getString();
      Serial.println("Resposta: " + response);
    }
    http.end();
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Conectando ao WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");
}

void loop() {
  // 🔹 exemplo 1: enviar temperatura
  enviarDado("temp", 28.7);

  delay(5000);

  // 🔹 exemplo 2: enviar umidade
  enviarDado("umid", 62.3);

  delay(5000);

  // 🔹 exemplo 3: enviar luminosidade
  enviarDado("lux", 124.9);

  delay(10000); // espera antes do próximo ciclo
}


```

## Para buscar

```bash
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "SEU_WIFI";
const char* password = "SENHA_WIFI";

// URL base do seu Worker D1
const char* baseUrl = "https://work-d1.lvhemann.workers.dev";

// ---------- Funções de busca ----------

// Lista últimos registros
void buscarTodos() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(baseUrl) + "/list";
    http.begin(url);

    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("📋 Últimos registros:");
      Serial.println(payload);
    } else {
      Serial.printf("Erro HTTP em /list: %d\n", httpCode);
    }
    http.end();
  }
}

// Busca por ID específico
void buscarPorId(int id) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(baseUrl) + "/get?id=" + String(id);
    http.begin(url);

    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("🔍 Registro por ID:");
      Serial.println(payload);
    } else {
      Serial.printf("Erro HTTP em /get: %d\n", httpCode);
    }
    http.end();
  }
}

// Busca filtrada (ex: valores do sensor "temp" acima de 30)
void buscarFiltrado(String sensor, float minValor) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(baseUrl) + "/search?sensor=" + sensor + "&min=" + String(minValor, 2);
    http.begin(url);

    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      Serial.println("⚡ Resultados filtrados:");
      Serial.println(payload);
    } else {
      Serial.printf("Erro HTTP em /search: %d\n", httpCode);
    }
    http.end();
  }
}

// ---------- Setup e Loop ----------
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Conectando ao WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");
}

void loop() {
  buscarTodos();
  delay(5000);

  buscarPorId(1);
  delay(5000);

  buscarFiltrado("temp", 30);
  delay(10000);
}



```

## Código com insert de outras colunas
```bash
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "SEU_WIFI";
const char* password = "SENHA_WIFI";

// URL do Worker com rota /migrate
const char* serverName = "https://work-d1.lvhemann.workers.dev/migrate";

void adicionarColuna(String coluna, String tipo, String secret) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");

    // monta JSON de migração
    String payload = "{\"coluna\":\"" + coluna + "\",\"tipo\":\"" + tipo + "\",\"secret\":\"" + secret + "\"}";

    int httpCode = http.POST(payload);
    Serial.print("HTTP Response code: ");
    Serial.println(httpCode);

    if (httpCode > 0) {
      String response = http.getString();
      Serial.println("Resposta: " + response);
    }
    http.end();
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Conectando ao WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado!");

  // exemplo: adiciona coluna "status" do tipo TEXT
  adicionarColuna("status", "TEXT", "minha_senha_super_secreta");
}

void loop() {
  // nada aqui, só executa uma vez
}



```
