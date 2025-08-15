# IoT_AMF_2025-02

'''
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

'''
