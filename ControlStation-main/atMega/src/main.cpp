#include <Arduino.h>
#include <string.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// ===== Пины и настройки радио, которые можно безопасно менять =====
constexpr uint8_t HC12_SET_PIN = 4;      // Пин SET модуля HC-12. На вашей плате это PD4.
constexpr uint8_t STATUS_LED_PIN = LED_BUILTIN;  // Светодиод для индикации активности.
constexpr uint8_t HC12_CHANNEL = 5;      // Радиоканал. У мастера и слейва должен быть одинаковым.
constexpr long HC12_UART_BAUD = 9600L;   // Скорость UART между МК и HC-12. Должна совпадать с обеих сторон.

// ===== Временные настройки =====
constexpr uint16_t AT_ENTER_DELAY_MS = 50;       // Пауза после перевода SET в LOW перед AT-командами.
constexpr uint16_t UART_RETURN_DELAY_MS = 80;    // Пауза после возврата SET в HIGH перед обычной работой.
constexpr uint16_t AT_RESPONSE_TIMEOUT_MS = 300; // Сколько ждать ответ на AT-команду.

// Скорости, которые скетч перебирает при поиске модуля в AT-режиме.
// Обычно этот список менять не нужно.
const long SUPPORTED_BAUDS[] = {
    9600L, 19200L, 4800L, 2400L, 1200L, 38400L, 57600L, 115200L};
const char *REPEATED_RESPONSE_BASE_IDS[] = {
    "6", "10", "11", "12", "16", "17", "20", "21", "22", "23", "24"};
constexpr unsigned long REPEATED_RESPONSE_DELAY_MS = 2000UL;
constexpr uint8_t MAX_PENDING_SECOND_RESPONSES = 4;

char rxLine[48];      // Буфер для одного принятого текстового пакета, например "REQ,20_1".
size_t rxLineLen = 0; // Текущая длина данных, накопленных в rxLine.

bool radioReady = false; // true, если HC-12 успешно настроен через AT-команды.
char pendingSecondResponseIds[MAX_PENDING_SECOND_RESPONSES][40];
unsigned long pendingSecondResponseAtMs[MAX_PENDING_SECOND_RESPONSES];
uint8_t pendingSecondResponseCount = 0;

uint8_t normalizeChannel(uint8_t channel) {
  if (channel < 1) {
    return 1;
  }
  if (channel > 127) {
    return 127;
  }
  return channel;
}

void flushSerialRx() {
  while (Serial.available()) {
    Serial.read();
  }
}

void enterATMode() {
  digitalWrite(HC12_SET_PIN, LOW);
  delay(AT_ENTER_DELAY_MS);
}

void exitATMode() {
  digitalWrite(HC12_SET_PIN, HIGH);
  delay(UART_RETURN_DELAY_MS);
}

bool readATResponse(char *out, size_t outSize, uint16_t timeoutMs) {
  size_t idx = 0;
  unsigned long startMs = millis();
  unsigned long lastByteMs = startMs;

  while (millis() - startMs < timeoutMs) {
    while (Serial.available()) {
      const char c = static_cast<char>(Serial.read());

      if (c == '\r' || c == '\n') {
        continue;
      }

      if (idx < outSize - 1) {
        out[idx++] = c;
      }

      lastByteMs = millis();
    }

    if (idx > 0 && millis() - lastByteMs > 40) {
      break;
    }
  }

  out[idx] = '\0';
  return idx > 0;
}

bool sendATCommand(const char *command, char *response, size_t responseSize) {
  flushSerialRx();
  Serial.print(command);
  return readATResponse(response, responseSize, AT_RESPONSE_TIMEOUT_MS);
}

long detectCurrentBaudInATMode() {
  char response[24];

  for (size_t i = 0; i < sizeof(SUPPORTED_BAUDS) / sizeof(SUPPORTED_BAUDS[0]); ++i) {
    const long baud = SUPPORTED_BAUDS[i];

    Serial.end();
    Serial.begin(baud);
    delay(20);

    if (sendATCommand("AT", response, sizeof(response)) &&
        strncmp(response, "OK", 2) == 0) {
      return baud;
    }
  }

  return 0;
}

bool applyRadioSettings(uint8_t channel, long uartBaud) {
  char command[20];
  char response[32];

  enterATMode();

  const long currentBaud = detectCurrentBaudInATMode();
  if (currentBaud == 0) {
    exitATMode();
    return false;
  }

  Serial.end();
  Serial.begin(currentBaud);
  delay(20);

  snprintf(command, sizeof(command), "AT+C%03u", normalizeChannel(channel));
  if (!sendATCommand(command, response, sizeof(response)) ||
      strncmp(response, "OK", 2) != 0) {
    exitATMode();
    return false;
  }

  if (!sendATCommand("AT+FU3", response, sizeof(response)) ||
      strncmp(response, "OK", 2) != 0) {
    exitATMode();
    return false;
  }

  snprintf(command, sizeof(command), "AT+B%ld", uartBaud);
  if (!sendATCommand(command, response, sizeof(response)) ||
      strncmp(response, "OK", 2) != 0) {
    exitATMode();
    return false;
  }

  exitATMode();

  Serial.end();
  Serial.begin(uartBaud);
  delay(20);
  flushSerialRx();

  return true;
}

bool readRadioLine() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (rxLineLen == 0) {
        continue;
      }

      rxLine[rxLineLen] = '\0';
      rxLineLen = 0;
      return true;
    }

    if (rxLineLen < sizeof(rxLine) - 1) {
      rxLine[rxLineLen++] = c;
    } else {
      rxLineLen = 0;
    }
  }

  return false;
}

bool isValidRequestIdChar(char value) {
  return (value >= '0' && value <= '9') || value == '_';
}

bool extractRequestId(const char *packet, char *requestId, size_t requestIdSize) {
  static const char prefix[] = "REQ,";

  if (strncmp(packet, prefix, sizeof(prefix) - 1) != 0) {
    return false;
  }

  const char *payload = packet + (sizeof(prefix) - 1);
  const size_t payloadLength = strlen(payload);

  if (payloadLength == 0 || payloadLength >= requestIdSize) {
    return false;
  }

  if (strcmp(payload, "-1") == 0) {
    strncpy(requestId, payload, requestIdSize - 1);
    requestId[requestIdSize - 1] = '\0';
    return true;
  }

  if (payload[0] == '_' || payload[payloadLength - 1] == '_') {
    return false;
  }

  bool previousWasUnderscore = false;

  for (size_t i = 0; i < payloadLength; ++i) {
    const char currentChar = payload[i];

    if (!isValidRequestIdChar(currentChar)) {
      return false;
    }

    if (currentChar == '_') {
      if (previousWasUnderscore) {
        return false;
      }

      previousWasUnderscore = true;
    } else {
      previousWasUnderscore = false;
    }

    requestId[i] = currentChar;
  }

  requestId[payloadLength] = '\0';
  return true;
}

void extractBaseRequestId(const char *requestId, char *baseRequestId, size_t baseRequestIdSize) {
  size_t index = 0;

  while (requestId[index] != '\0' &&
         requestId[index] != '_' &&
         index < baseRequestIdSize - 1) {
    baseRequestId[index] = requestId[index];
    ++index;
  }

  baseRequestId[index] = '\0';
}

bool requestIdNeedsRepeatedResponse(const char *requestId) {
  char baseRequestId[16];
  extractBaseRequestId(requestId, baseRequestId, sizeof(baseRequestId));

  for (size_t i = 0; i < sizeof(REPEATED_RESPONSE_BASE_IDS) / sizeof(REPEATED_RESPONSE_BASE_IDS[0]); ++i) {
    if (strcmp(baseRequestId, REPEATED_RESPONSE_BASE_IDS[i]) == 0) {
      return true;
    }
  }

  return false;
}

void sendSlaveResponse(const char *requestId, int state) {
  char response[64];

  if (state < 0) {
    snprintf(response, sizeof(response), "RESP,%s", requestId);
  } else {
    snprintf(response, sizeof(response), "RESP,%s,%d", requestId, state);
  }

  Serial.println(response);
  digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
}

bool enqueueSecondResponse(const char *requestId) {
  if (pendingSecondResponseCount >= MAX_PENDING_SECOND_RESPONSES) {
    return false;
  }

  strncpy(
      pendingSecondResponseIds[pendingSecondResponseCount],
      requestId,
      sizeof(pendingSecondResponseIds[pendingSecondResponseCount]) - 1);
  pendingSecondResponseIds[pendingSecondResponseCount]
                          [sizeof(pendingSecondResponseIds[pendingSecondResponseCount]) - 1] = '\0';
  pendingSecondResponseAtMs[pendingSecondResponseCount] = millis() + REPEATED_RESPONSE_DELAY_MS;
  ++pendingSecondResponseCount;
  return true;
}

void removePendingSecondResponse(uint8_t index) {
  if (index >= pendingSecondResponseCount) {
    return;
  }

  for (uint8_t currentIndex = index; currentIndex + 1 < pendingSecondResponseCount; ++currentIndex) {
    strncpy(
        pendingSecondResponseIds[currentIndex],
        pendingSecondResponseIds[currentIndex + 1],
        sizeof(pendingSecondResponseIds[currentIndex]) - 1);
    pendingSecondResponseIds[currentIndex][sizeof(pendingSecondResponseIds[currentIndex]) - 1] = '\0';
    pendingSecondResponseAtMs[currentIndex] = pendingSecondResponseAtMs[currentIndex + 1];
  }

  if (pendingSecondResponseCount > 0) {
    --pendingSecondResponseCount;
    pendingSecondResponseIds[pendingSecondResponseCount][0] = '\0';
    pendingSecondResponseAtMs[pendingSecondResponseCount] = 0;
  }
}

bool isTimeReached(unsigned long nowMs, unsigned long targetMs) {
  return static_cast<long>(nowMs - targetMs) >= 0;
}

void sendPendingSecondResponsesIfReady() {
  const unsigned long nowMs = millis();
  uint8_t index = 0;

  while (index < pendingSecondResponseCount) {
    if (!isTimeReached(nowMs, pendingSecondResponseAtMs[index])) {
      ++index;
      continue;
    }

    sendSlaveResponse(pendingSecondResponseIds[index], 1);
    removePendingSecondResponse(index);
  }
}

void runSlave() {
  char requestId[40];

  if (!readRadioLine()) {
    return;
  }

  if (extractRequestId(rxLine, requestId, sizeof(requestId))) {
    if (requestIdNeedsRepeatedResponse(requestId)) {
      sendSlaveResponse(requestId, 0);
      if (!enqueueSecondResponse(requestId)) {
        sendSlaveResponse(requestId, 1);
      }
      return;
    }

    sendSlaveResponse(requestId, -1);
  }
}

void blinkErrorPattern() {
  static unsigned long lastToggleMs = 0;
  static bool ledState = false;

  if (millis() - lastToggleMs >= 200) {
    lastToggleMs = millis();
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);
  }
}

void setup() {
  pinMode(HC12_SET_PIN, OUTPUT);
  digitalWrite(HC12_SET_PIN, HIGH);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(9600);
  delay(50);

  radioReady = applyRadioSettings(HC12_CHANNEL, HC12_UART_BAUD);
}

void loop() {
  if (!radioReady) {
    blinkErrorPattern();
    return;
  }

  sendPendingSecondResponsesIfReady();
  runSlave();
}
