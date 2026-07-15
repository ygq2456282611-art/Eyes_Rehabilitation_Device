#include <Arduino.h>
#include <WiFi.h>
#include <esp_mac.h>

void setup()
{
    uint8_t mac[6] = {};

    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.println();
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        Serial.printf("STA MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        Serial.println("STA MAC read failed");
    }
}

void loop()
{
    delay(1000);
}
