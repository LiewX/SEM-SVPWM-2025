#include <Arduino.h>
#include <ArduinoWebsockets.h>
#include "Websocket.h"
#include "ProgramRunConfigs.h"

WebsocketsServer server; // Create a WebSocket server
WebsocketsClient client; // Store the connected client
bool clientConnected = false;        // Track client connection status

// WebSocket Server Setup
void websocket_setup() {
    #if (ENABLE_T3_WEBSOCKET_HANDLER && ENABLE_T4_SEND_TO_WIFI)
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    #if FORCE_WIFI_CONNECTION
        // Wait for the ESP32 to connect to Wi-Fi
        while (WiFi.status() != WL_CONNECTED) {
            static int retryCount = 0;
            Serial.printf("Connecting to WiFi (%s) ...\n", WIFI_SSID);
            retryCount++;
            if (retryCount >= 10) {
                Serial.println("Can't connect to WiFi. Restarting.");
                ESP.restart(); // Restart ESP32 if can't connect to WiFi after 10 tries
            }
            delay(500);
        }
        Serial.printf("Connected to WiFi!\nESP32 IP Address: ");
        Serial.print(WiFi.localIP());
        Serial.printf(":81\n");

        // Start the WebSocket server
        server.listen(81); // Listen on port 81
        Serial.println("WebSocket server started!");
    #endif
    #endif
}
