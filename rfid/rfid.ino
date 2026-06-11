#include <MFRC522.h>
#include <SPI.h>

// ESP8266 pin definitions for RC522
// RST_PIN: D3 (GPIO0) - reset line for RC522 module
// SS_PIN:  D8 (GPIO15) - SPI slave select / chip select for RC522
// SPI hardware pins (ESP8266 default): SCK=D5, MOSI=D7, MISO=D6
// Serial1 TX pin: GPIO2 (TX1) is used for sending payloads D4
static const uint8_t RST_PIN = 0;  // D3
static const uint8_t SS_PIN = 15;  // D8
static const uint32_t SERIAL_BAUD = 115200;

static const uint8_t AUTO_BTN_PIN = 4;

MFRC522 mfrc522(SS_PIN, RST_PIN);

void setupReader() {
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(100);
  digitalWrite(RST_PIN, HIGH);
  delay(100);

  SPI.begin();
  SPI.setFrequency(1000000);
  mfrc522.PCD_Init();
}

String formatUid(const MFRC522::Uid& uid) {
  String result;
  result.reserve(uid.size * 2 + 1);

  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) {
      result += '0';
    }
    result += String(uid.uidByte[i], HEX);
  }

  result.toUpperCase();
  return result;
}

uint8_t calculateUidChecksum(const String& uidString) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < uidString.length(); i++) {
    checksum += uidString[i];
  }
  return checksum;
}

/**
 * @brief Create a payload string for the given UID.
 * @param uidString The UID string.
 * @return The formatted payload string.
 * Payload format: "UID|<UID_STRING>|<CHECKSUM>"
 */
String createPayload(const String& uidString) {
  uint8_t checksum = calculateUidChecksum(uidString);
  String checksumString = String(checksum, HEX);
  checksumString.toUpperCase();
  if (checksumString.length() == 1) {
    checksumString = "0" + checksumString;
  }

  String payload;
  payload.reserve(uidString.length() + checksumString.length() + 1);
  payload = F("UID|");
  payload += uidString;
  payload += F("|");
  payload += checksumString;
  return payload;
}

static uint32_t packetId = 0;

/**
 * @brief Send a UID payload with retry logic.
 * @param payload The payload string to send.
 * @param retries The number of retry attempts.
 * @param intervalMs The interval between retries in milliseconds.
 * Payload format: "UID|<UID_STRING>|<CHECKSUM>|<COUNTER>|"
 */
void sendUidWithRetries(const String& payload, uint8_t retries = 1,
                        uint16_t intervalMs = 500, bool isAuto = false) {
  Serial1.println();
  packetId++;

  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    Serial1.println(payload + "|" + String(packetId) + "|");
    if (attempt + 1 < retries) {
      delay(intervalMs);
    }
  }
  Serial1.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial1.begin(SERIAL_BAUD);

  setupReader();

  Serial.println(F("--- Kiểm tra kết nối RC522 ---"));
  mfrc522.PCD_DumpVersionToSerial();
  Serial.println();
  Serial.println(F("Đưa thẻ RFID vào gần đầu đọc..."));
}

void loop() {
  Serial1.println();

  if (!mfrc522.PICC_IsNewCardPresent()) return;

  if (!mfrc522.PICC_ReadCardSerial()) return;

  String uidString = formatUid(mfrc522.uid);
  String payload = createPayload(uidString);

  Serial.print(F("Đã đọc thẻ có mã: "));
  Serial.println(uidString);
  sendUidWithRetries(payload, 2, 500, false);

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  delay(1000);
}
