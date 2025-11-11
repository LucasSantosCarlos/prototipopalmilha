// ESP32 + 6x FSR402 em ADC1
// Escala fixa: 0..4095 => 0..100%
// Envia por BLE JSON: {"t":ms,"pct":[..],"ema":[..]}

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ---- Pinos ADC1 (sem conflito com Wi-Fi) ----
const int PINS[] = {36, 39, 34, 35, 33, 32};  // VP, VN, 34, 35, 33, 32
const int NSENS   = sizeof(PINS)/sizeof(PINS[0]);

// ---- Amostragem / média ----
const unsigned SAMPLE_MS = 150;  // ~6–7 Hz por varredura
const int N_MEDIA        = 8;

// ---- Filtro/normalização ----
const float ALPHA    = 0.18f;  // EMA 0..1 (maior = responde mais rápido)
const int   DEADBAND = 6;      // zona morta pós-offset

// ---- Estado por sensor ----
int   offsetV[NSENS];   // baseline sem pressão
float emaV[NSENS];      // valor filtrado (ADC após offset)

// ---- BLE ----
#define BLE_DEVICE_NAME   "PalmilhaESP32"
#define SERVICE_UUID      "a62b7a2a-6f05-4b39-9a0c-3a0a4b6c1c10"
#define CHAR_JSON_UUID    "b5e8c0f4-0a2a-47c7-9f83-0bcf3c5b1a11"

BLEServer*         pServer    = nullptr;
BLECharacteristic* pJsonChar  = nullptr;
bool deviceConnected          = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* p) override { deviceConnected = true; }
  void onDisconnect(BLEServer* p) override {
    deviceConnected = false;
    p->getAdvertising()->start();
  }
};

int leituraMedia(int pin, int n) {
  long s = 0;
  for (int i = 0; i < n; i++) { s += analogRead(pin); delay(3); }
  return (int)(s / n);
}

void calibraBaseline(unsigned dur_ms = 1000) {
  const int n = dur_ms / 20;
  for (int k = 0; k < NSENS; k++) {
    long soma = 0;
    for (int i = 0; i < n; i++) { soma += analogRead(PINS[k]); delay(20); }
    offsetV[k] = (int)(soma / n);
    emaV[k]    = 0.0f;
  }
}

void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(185); // ajuda a caber JSON maior

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pJsonChar = pService->createCharacteristic(
    CHAR_JSON_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pJsonChar->addDescriptor(new BLE2902());
  pJsonChar->setValue("{}\n");

  pService->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
}

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12); // 0..4095
  for (int k = 0; k < NSENS; k++) {
    analogSetPinAttenuation(PINS[k], ADC_11db); // ~0..3.3V
  }

  Serial.println("Calibrando baseline por 1s (NAO pressione os sensores)...");
  calibraBaseline(1000);
  Serial.print("Offsets: ");
  for (int k = 0; k < NSENS; k++) { Serial.print(offsetV[k]); Serial.print(k==NSENS-1?'\n':','); }

  // Cabeçalho CSV: t_ms, pct0..pct5, ema0..ema5
  Serial.print("t_ms");
  for (int k = 0; k < NSENS; k++) Serial.printf(",pct%d", k);
  for (int k = 0; k < NSENS; k++) Serial.printf(",ema%d", k);
  Serial.println();

  setupBLE();
}

void loop() {
  unsigned long t0 = millis();

  // Vetores de saída
  int   pctV[NSENS];   // 0..100
  int   emaInt[NSENS]; // ema em inteiro para envio

  for (int k = 0; k < NSENS; k++) {
    // 1) leitura crua com média
    int adc = leituraMedia(PINS[k], N_MEDIA);

    // 2) remove baseline + zona morta
    int corr = adc - offsetV[k];
    if (corr < DEADBAND) corr = 0;

    // 3) filtro EMA (em "unidades ADC após offset")
    emaV[k] = emaV[k] + ALPHA * (corr - emaV[k]);

    // 4) normalização fixa (0..4095 => 0..1)
    float nrm = emaV[k] / 4095.0f;
    if (nrm < 0.0f) nrm = 0.0f;
    if (nrm > 1.0f) nrm = 1.0f;

    // 5) porcentagem 0..100 (arredondada)
    int pct = (int)lroundf(nrm * 100.0f);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;

    pctV[k]   = pct;
    emaInt[k] = (int)lroundf(emaV[k]); // útil para debug
  }

  // ---------- Saída Serial CSV ----------
  Serial.print(t0);
  for (int k = 0; k < NSENS; k++) { Serial.print(','); Serial.print(pctV[k]); }
  for (int k = 0; k < NSENS; k++) { Serial.print(','); Serial.print(emaInt[k]); }
  Serial.println();

  // ---------- Envio BLE: JSON ----------
  // {"t":12345,"pct":[..],"ema":[..]}\n
  static char jsonBuf[220];
  int n = 0;
  n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n, "{\"t\":%lu,\"pct\":[", t0);
  for (int k = 0; k < NSENS; k++) {
    n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n, (k?",%d":"%d"), pctV[k]);
  }
  n += snprintf(jsonBuf + n, sizeof(jsonBuf) - n, "]}%c", '\n');

  if (deviceConnected) {
    pJsonChar->setValue((uint8_t*)jsonBuf, n);
    pJsonChar->notify();
  } else {
    pJsonChar->setValue((uint8_t*)jsonBuf, n);
  }

  // Ritmo de atualização
  unsigned long dt = millis() - t0;
  if (dt < SAMPLE_MS) delay(SAMPLE_MS - dt);
}
