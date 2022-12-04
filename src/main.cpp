#include "Config.h"
#include "Secret.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

esp_now_peer_info_t peerInfo = {};

esp_now_peer_info_t responsePeerInfo = {};


#ifndef ESPNOW_PEER_MAC
// Broadcast Address
const uint8_t peerMAC[] PROGMEM = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#else
const uint8_t peerMAC[] PROGMEM = ESPNOW_PEER_MAC;
#endif

Print* debugStream = &Serial;

#if ESPNOW_ENCRYPTED
// ESP-NOW PMK and LMK keys (16 bytes each)
static const uint8_t PMK_KEY[] = ESPNOW_PMK_KEY;
static const uint8_t LMK_KEY[] = ESPNOW_LMK_KEY;
#endif

#if USE_AES_ENCRYPT
#include "mbedtls/aes.h"
mbedtls_aes_context aes;
const uint8_t aesKey[] PROGMEM = AES_KEY;
const uint8_t aesIV[] PROGMEM = AES_IV;

typedef struct __attribute__ ((packed)) AesHeader {
  uint16_t magic;
  uint16_t sequence;
  uint8_t len;  // ESP-NOW messages can only be 250 bytes.
  uint8_t retry;  
  uint32_t uptime;
} AesHeader;

typedef struct __attribute__ ((packed)) AesMessage {
  AesHeader header;
  uint8_t data[250-sizeof(AesHeader)];
} AesMessage;

typedef struct __attribute__ ((packed)) AesResponse {
  uint16_t magic;
  uint16_t sequence;
  //uint8_t mac[6];
} AesMResponse;

static AesMResponse _msgResponse;
static int32_t _msgSendTs = 0;
static uint8_t _msgState = 0;

#endif


bool espNowSendPacket(const uint8_t* mac, uint8_t* data, size_t size) {
  _msgSendTs = millis();
  _msgState = 0;

  // Add peer
  memcpy(responsePeerInfo.peer_addr, mac, sizeof(peerMAC));
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;
  esp_err_t eres = esp_now_add_peer(&responsePeerInfo);
  if (eres != ESP_OK) {
    char estr[256];
    esp_err_to_name_r(eres, estr, sizeof(estr));
    Serial.printf("Failed to add peer. Error: %X %s\n", eres, estr);
  }

  eres = esp_now_send(mac, data, size);
  if (eres != ESP_OK) {
    char estr[256];
    esp_err_to_name_r(eres, estr, sizeof(estr));
    Serial.printf("Failed to send packet. Error: %X %s\n", eres, estr);
    return true;
  }
  return false;
}

// Callback when data done sending
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  INFO(DEBUG_printf(FST("Sent to %02X:%02X:%02X:%02X:%02X:%02X - status: %d\n"),
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], status));
  esp_err_t eres = esp_now_del_peer(responsePeerInfo.peer_addr);
  if (eres != ESP_OK) {
    char estr[256];
    esp_err_to_name_r(eres, estr, sizeof(estr));
    Serial.printf("Failed to remove peer. Error: %X %s\n", eres, estr);
  }
}


// callback function that will be executed when data is received
void onDataRecv(const uint8_t * mac, const uint8_t *rdata, int len) {

  INFO(DEBUG_printf(FST("%02X:%02X:%02X:%02X:%02X:%02X: received %d bytes\n"),
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len));
  // DEBUG_println((const char*) data);
const uint8_t* data = rdata;
#if USE_AES_ENCRYPT
  uint8_t clear[256];
  AesMessage* msg = (AesMessage*) data;
  size_t blockLen = (16 - (msg->header.len & 0xF) & 0xF) + msg->header.len;
  INFO(DEBUG_printf(FST("Header magic:%04X seq:%d retry:%d len:%d blockLen:%d.\n"), msg->header.magic, msg->header.sequence, msg->header.retry, msg->header.len, blockLen));
  if (msg->header.magic == AES_MAGIC_WAIT_ACK || msg->header.magic == AES_MAGIC_NO_ACK) {
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc( &aes, (const unsigned char*) aesKey, sizeof(aesKey) * 8 );
    uint8_t iv[sizeof(aesIV)];
    memcpy(iv, aesIV, sizeof(aesIV));
    esp_aes_crypt_cbc(&aes, ESP_AES_DECRYPT, blockLen, iv, msg->data, clear);
    mbedtls_aes_free(&aes);
    INFO(DEBUG_printf(FST("Decrypted %d (%d) bytes including 4 byte salt.\n"), blockLen, msg->header.len));
    data = clear; 
    data += 4; // Skip the salt
    len = msg->header.len - 4; // Skip the salt

    if (msg->header.magic == AES_MAGIC_WAIT_ACK) {
      _msgResponse.magic = AES_MAGIC_ACK;
      _msgResponse.sequence = msg->header.sequence;
      INFO(DEBUG_printf(FST("Sending ACK for: %d\n"), _msgResponse.sequence));
      espNowSendPacket(mac, (uint8_t*) &_msgResponse, sizeof(AesResponse));
    }
  } else {
    msg = nullptr;
  }
#endif

  // check for JSON
  if (data[0] <= 0x40 || data[0] >= 0x7F || data[len-1] != 0) {
    // Not a \0 terminated string. Treat it as binary.
    Serial.write(mac, 6);
    Serial.write(data, len);
    return;
  } else if (data[0] == '{' &&  data[len-2] == '}') {
    // JSON
    Serial.printf(FST("{\"MAC\":\"%02X%02X%02X%02X%02X%02X\","), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    #if USE_AES_ENCRYPT
    if(msg) {
      Serial.printf(FST("\"Packet\":%d,\"Retry\":%d,\"Uptime\":%d,"), msg->header.sequence, msg->header.retry, msg->header.uptime);
    }
    #endif
    Serial.println((const char*)data+1);
  } else {
    // Assume CSV
    Serial.printf(FST("%02X%02X%02X%02X%02X%02X,"), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println((const char*)data);
  }
}
 

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(10);
  INFO(DEBUG_println("\n\n== ESP-NOW to Serial Gateway =="));

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  INFO(DEBUG_println(WiFi.macAddress()));
  

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    DEBUG_println(FST("Error initializing ESP-NOW"));
    return;
  }
  // esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, ESPNOW_CHANNEL);

#if ESPNOW_ENCRYPTED
  // Set the PMK key
  esp_now_set_pmk(PMK_KEY);
  
  // Register self as peer
  memcpy(peerInfo.peer_addr, peerMAC, sizeof(peerMAC));
  peerInfo.channel = ESPNOW_CHANNEL;
  // Setting the master device LMK key
  memcpy(peerInfo.lmk, LMK_KEY, 16);
  
  // Set encryption to true
  peerInfo.encrypt = true;
  
  // Add self as peer       
  esp_err_t eres = esp_now_add_peer(&peerInfo);
  if (eres != ESP_OK){
    char buffer[256];
    esp_err_to_name_r(eres, buffer, sizeof(buffer));
    Serial.printf("Failed to add peer. Error: %X %s\n", eres, buffer);
  } else {
    INFO(DEBUG_printf(FST("Added self as encrypted peer %02X:%02X:%02X:%02X:%02X:%02X\n"),
      peerMAC[0], peerMAC[1], peerMAC[2], peerMAC[3], peerMAC[4], peerMAC[5]));
  }
#endif

  /*
  // Add peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = ESPNOW_ENCRYPTED;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    DEBUG_println(FST("Failed to add peer"));
    return;
  }
  */

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

}

void loop() {
  // Nothing to do here.
}
