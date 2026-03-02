#include <WiFi.h>
#include <esp_now.h>

// Replace with the receiver's MAC Address
//08:A6:F7:24:CA:E4
uint8_t receiverMAC[] = {0x08, 0xA6, 0xF7, 0x24, 0xCA, 0xE4};  


typedef struct struct_message {
    char text[32];
    int value;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo={};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    strcpy(myData.text, "Bismillahir Rahmanir Raheem");
    myData.value = random(0, 100);

    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }

    delay(2000); // Send data every 2 seconds
}
