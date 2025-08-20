#pragma once
#include <Arduino.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

extern WebsocketsServer server; // Create a WebSocket server
extern WebsocketsClient client; // Store the connected client
extern bool clientConnected;    // Track client connection status

void websocket_setup();