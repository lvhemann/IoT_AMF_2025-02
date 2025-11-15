# IoT_AMF_2025-02

## BMA400 - EdgeImpulse

```bash  
 #include <Arduino.h>
#include <BMA400.h>
#include <Wire.h>

BMA400 bma400;

// === Configuração da taxa de amostragem para o Edge Impulse ===
const float SAMPLE_RATE_HZ = 25.0;                 // taxa de amostragem em Hz
const unsigned long SAMPLE_INTERVAL_MS = 1000.0 / SAMPLE_RATE_HZ;
unsigned long lastSampleTime = 0;

uint32_t bma400GetSteps()
{
    uint32_t current_steps = bma400.GetTotalSteps();
    if (current_steps > 0)
    {
        bool success = false;
        uint32_t startTime = millis();
        do
        {
            if (bma400.ResetStepCounter())
                success = true;
            else
                delay(500);
        } while (!success && ((millis() - startTime) < 3000));
    }
    return current_steps;
}

void setup()
{
  pinMode(GPIO_NUM_27, OUTPUT);
  digitalWrite(GPIO_NUM_27, LOW);

  Serial.begin(115200);
  while (!Serial) {
    ; // espera a Serial (em algumas placas USB)
  }

  Wire.begin();

  if (bma400.Initialize(Wire)) // Using default (Wire) interface & automatically resolving the address
  {
    bma400.Setup(
        BMA400::power_mode_t::NORMAL_LOW_NOISE,
        BMA400::output_data_rate_t::Filter2_100Hz, // ODR 100Hz, você está lendo a 25Hz, ok
        BMA400::acceleation_range_t::RANGE_2G);

    bma400.DisableInterrupts(); // disables all interrupts if previously set
    bma400.ResetStepCounter();

    bma400.ConfigureStepDetectorCounter(
        true,                             // Enable Single tap / step
        BMA400::interrupt_pin_t::INT_NONE // sem interrupção em pino
    );

    // Cabeçalho opcional (só uma vez). Ajuda a identificar no Data Forwarder
    Serial.println("accX,accY,accZ,steps");
  }
  else
  {
    Serial.println("BMA400 initialization failed!");
    while (1) {
      delay(1000);
    }
  }
}

void loop()
{
  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) {
    return; // ainda não é hora da próxima amostra
  }
  lastSampleTime = now;

  // --- Leitura do acelerômetro ---
  float acceleration[3] = {0};
  bma400.ReadAcceleration(acceleration);
  float accX = acceleration[0];
  float accY = acceleration[1];
  float accZ = acceleration[2];

  // --- Leitura e acumulo dos passos ---
  static uint32_t totalSteps = 0;
  totalSteps += bma400GetSteps();

  // === SAÍDA NO FORMATO PARA O EDGE IMPULSE DATA FORWARDER ===
  // Somente números, separados por vírgula
  Serial.print(accX, 6);
  Serial.print(',');
  Serial.print(accY, 6);
  Serial.print(',');
  Serial.print(accZ, 6);
  Serial.print(',');
  Serial.println(totalSteps);
}
```

## Acelerometro MPU 6050

```bash

// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// === Configuração da taxa de amostragem para o Edge Impulse ===
const float SAMPLE_RATE_HZ = 25.0;                 // taxa de amostragem em Hz
const unsigned long SAMPLE_INTERVAL_MS = 1000.0 / SAMPLE_RATE_HZ;
unsigned long lastSampleTime = 0;


void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop()
{
  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) {
    return; // ainda não é hora da próxima amostra
  }
  lastSampleTime = now;

  // --- Leitura do acelerômetro ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accX = a.acceleration.x;
  float accY = a.acceleration.y;
  float accZ = a.acceleration.z;

  // === SAÍDA NO FORMATO PARA O EDGE IMPULSE DATA FORWARDER ===
  // Somente números, separados por vírgula
  Serial.print(accX, 6);
  Serial.print(',');
  Serial.print(accY, 6);
  Serial.print(',');
  Serial.print(accZ, 6);
}
```





