# IoT_AMF_2025-02

## Monitor de Memória e Stack

```bash  
  #include <Arduino.h>

void printMemoryInfo() {
  // Heap (memória livre global)
  size_t freeHeap = ESP.getFreeHeap();          // bytes livres no heap
  size_t minHeap  = ESP.getMinFreeHeap();       // menor valor de heap já disponível
  size_t maxAlloc = ESP.getMaxAllocHeap();      // maior bloco único alocável

  // Stack (da task atual)
  size_t freeStack = uxTaskGetStackHighWaterMark(NULL); // em palavras de 4 bytes
  freeStack *= sizeof(StackType_t);                     // converte para bytes

  Serial.println("=== Memória ESP32 ===");
  Serial.printf("Heap livre atual: %u bytes\n", freeHeap);
  Serial.printf("Heap mínimo já visto: %u bytes\n", minHeap);
  Serial.printf("Maior bloco contíguo disponível: %u bytes\n", maxAlloc);
  Serial.printf("Stack livre da task atual: %u bytes\n", freeStack);
  Serial.println("======================\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  printMemoryInfo();
  delay(2000); // imprime a cada 2 segundos
}

```

## Blink com Monitor de Memória e Stack
```bash
#ifndef APP_BLINK_H
#define APP_BLINK_H

#include <Arduino.h>

// Função simples para piscar um pino
inline void blink(uint8_t pin, uint32_t timeMs) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(timeMs);
  digitalWrite(pin, LOW);
  delay(timeMs);
}

#endif // APP_BLINK_H
```
# CPP
```bash
#include <Arduino.h>
#include "app_blink.h"

// --- Monitor de memória ---
void printMemoryInfo() {
  size_t freeHeap = ESP.getFreeHeap();
  size_t minHeap  = ESP.getMinFreeHeap();
  size_t maxAlloc = ESP.getMaxAllocHeap();

  size_t freeStack = uxTaskGetStackHighWaterMark(NULL);
  freeStack *= sizeof(StackType_t);

  Serial.println("=== Memória ESP32 ===");
  Serial.printf("Heap livre atual: %u bytes\n", (unsigned)freeHeap);
  Serial.printf("Heap mínimo já visto: %u bytes\n", (unsigned)minHeap);
  Serial.printf("Maior bloco contíguo disponível: %u bytes\n", (unsigned)maxAlloc);
  Serial.printf("Stack livre da task atual: %u bytes\n", (unsigned)freeStack);
  Serial.println("======================\n");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[Init] Blink + Monitor de Memória");
}

void loop() {
  blink(LED_BUILTIN, 500); // pisca com 500 ms HIGH e 500 ms LOW
  printMemoryInfo();       // imprime após cada ciclo
}


```


## Estouro de Memória
```bash
#include <Arduino.h>

/*
  ESP32 — Demonstração de Stack Overflow progressivo

  Ideia:
    - Uma task é criada com stack relativamente pequena.
    - A cada "passo", chamamos uma função recursiva com profundidade crescente.
    - Cada chamada usa um buffer local (FRAME_SIZE) para consumir stack.
    - Assim, a stack vai sendo "comida" com o tempo até estourar (<= 2 min).

  Ajustes:
    - STACK_WORDS: tamanho da stack da task em PALAVRAS (1 palavra = 4 bytes).
    - FRAME_SIZE:  bytes consumidos por frame de recursão (stack por chamada).
    - STEP_DELAY_MS: atraso entre passos (quanto menor, mais rápido estoura).
*/

static const uint16_t STACK_WORDS   = 768;   // 768 * 4 = ~3 KB para a task
static const uint16_t FRAME_SIZE    = 256;   // bytes por nível de recursão
static const uint16_t STEP_DELAY_MS = 150;   // atraso entre passos (<= 2 min total)

// ========== Monitor de heap/stack da task atual ==========
static void printMemoryInfo(const char* tag = "") {
  size_t freeHeap  = ESP.getFreeHeap();
  size_t minHeap   = ESP.getMinFreeHeap();
  size_t maxAlloc  = ESP.getMaxAllocHeap();
  size_t freeStack = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

  Serial.println("=== Memória ESP32 ===");
  if (tag && *tag) Serial.printf("[%s]\n", tag);
  Serial.printf("Heap livre atual: %u bytes\n", (unsigned)freeHeap);
  Serial.printf("Heap mínimo já visto: %u bytes\n", (unsigned)minHeap);
  Serial.printf("Maior bloco contíguo disponível: %u bytes\n", (unsigned)maxAlloc);
  Serial.printf("Stack livre (high-water): %u bytes\n", (unsigned)freeStack);
  Serial.println("======================\n");
}

// ========== Função recursiva que consome stack ==========
__attribute__((noinline)) static void eatStack(uint32_t depth) {
  // usa FRAME_SIZE bytes na stack desta chamada
  volatile uint8_t trash[FRAME_SIZE];
  for (uint16_t i = 0; i < FRAME_SIZE; ++i) trash[i] = (uint8_t)i;

  if (depth) eatStack(depth - 1);

  // impede otimizações/tail-call
  asm volatile("" ::: "memory");
}

// ========== Task que vai aumentar o consumo ao longo do tempo ==========
static void SlowOverflowTask(void* pv) {
  (void)pv;
  Serial.println("[SlowOverflowTask] Iniciada.");
  printMemoryInfo("inicio");

  uint32_t depth = 1; // começa leve e vai aumentando

  // Loop: a cada passo, aumenta 1 nível de recursão
  // e espera um pouquinho (STEP_DELAY_MS).
  for (;;) {
    Serial.printf("[SlowOverflowTask] depth=%u\n", (unsigned)depth);
    // Antes de atacar, mostra quanto de stack ainda sobrou
    size_t freeStack = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);
    Serial.printf("  stack livre ~%u bytes\n", (unsigned)freeStack);

    // Tenta consumir stack proporcional à profundidade
    // Cada nível usa ~FRAME_SIZE bytes.
    eatStack(depth);

    // Se não estourou, incrementa e tenta de novo
    depth++;

    // Ritmo do teste (ajuste para caber no seu alvo de tempo)
    delay(STEP_DELAY_MS);
  }
}

// ========== Hook do FreeRTOS em caso de stack overflow ==========
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  (void)xTask;
  Serial.println("\n[HOOK] *** STACK OVERFLOW DETECTADO ***");
  if (pcTaskName) Serial.printf("[HOOK] Task: %s\n", pcTaskName);
  Serial.flush();
  delay(200);
  // Reinicia o chip para recuperar
  esp_restart();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(300);
  Serial.println("\n[Init] Demo: stack overflow progressivo (<= 2 min)");

  // Cria a task com uma stack deliberadamente pequena
  BaseType_t ok = xTaskCreatePinnedToCore(
    SlowOverflowTask,          // função
    "SlowBoom",                // nome
    STACK_WORDS,               // stack em PALAVRAS (4 bytes cada)
    nullptr,                   // parâmetros
    tskIDLE_PRIORITY + 1,      // prioridade
    nullptr,                   // handle
    APP_CPU_NUM                // core (geralmente 1 para Arduino)
  );

  if (ok != pdPASS) {
    Serial.println("[Init] ERRO: não foi possível criar a task.");
  }
}

void loop() {
  // opcional: mostrar memória da loopTask periodicamente
  static uint32_t last = 0;
  if (millis() - last > 2000) {
    last = millis();
    printMemoryInfo("loop");
  }
  delay(1);
}



```

## Função Deep Sleep
```bash
#include <Arduino.h>

// Variável que sobrevive ao deep sleep (fica na RTC Memory)
RTC_DATA_ATTR int bootCount = 0;

// Função para imprimir motivo do último wakeup
void printWakeupReason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();

  switch (reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Acordou por sinal externo usando RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Acordou por sinal externo usando RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Acordou por temporizador"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Acordou por touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Acordou por ULP"); break;
    default: Serial.printf("Motivo de wakeup não identificado: %d\n", reason); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // espera Serial abrir

  bootCount++;
  Serial.printf("Boot número: %d\n", bootCount);
  printWakeupReason();

  // Configura o tempo de deep sleep (em microssegundos)
  const uint64_t sleepTimeSec = 10;
  Serial.printf("Entrando em deep sleep por %llu segundos...\n", sleepTimeSec);

  esp_sleep_enable_timer_wakeup(sleepTimeSec * 1000000ULL);

  // Dá tempo de ver a mensagem no Serial antes de dormir
  delay(2000);

  Serial.println("Indo dormir agora...");
  Serial.flush();
  esp_deep_sleep_start();
}

void loop() {
  // Não será executado — ESP32 reinicia ao acordar do deep sleep
}

```








