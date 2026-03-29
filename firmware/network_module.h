#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// mouse coordinates received from python
struct MouseState {
  int x = 0;
  int y = 0;
  bool valid = false;
};

// controller state received
struct RemoteControllerState {
  float lx = 0.0f;
  float ly = 0.0f;
  float rx = 0.0f;
  float ry = 0.0f;
  float throttle = 0.0f;
  float brake = 0.0f;

  int cross = 0;
  int circle = 0;
  int square = 0;
  int triangle = 0;

  int dpadX = 0;
  int dpadY = 0;

  bool valid = false;
};

class Net {
public:
  // Begin with SSID / pass and torch GPIO pin. WebSocket on port 81 to avoid HTTP conflicts.
  void begin(const char* ssid, const char* pass, uint8_t torchPin);
  void loop();

  // Text messages (e.g., controller, mouse)
  void send(String s);
  void sendTo(uint8_t clientNum, String s);

  // Binary messages (e.g., JPEG frames)
  void sendBinary(const uint8_t* data, size_t len);
  void sendToBinary(uint8_t clientNum, const uint8_t* data, size_t len);

  bool hasClients() const;

  // Accessors for the last received states 
  MouseState getMouseState() const { return s_mouseState; }
  RemoteControllerState getRemoteControllerState() const { return s_remoteCtlState; }
  uint32_t getLastControllerRxMs() const { return s_lastControllerRxMs; }


private:
  static void onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len);
  static void printCombinedStateToSerial();

  static WebSocketsServer* s_ws;
  static uint8_t s_torchPin;
  static volatile int s_lastClient;
  static volatile uint32_t s_lastControllerRxMs;

  static MouseState s_mouseState;
  static RemoteControllerState s_remoteCtlState;
};
