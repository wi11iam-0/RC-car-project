#include "network_module.h"
#include "esp_wifi.h"   // for esp_wifi_set_ps

WebSocketsServer* Net::s_ws = nullptr;
uint8_t Net::s_torchPin = 255;
volatile int Net::s_lastClient = -1;
volatile uint32_t Net::s_lastControllerRxMs = 0;

MouseState Net::s_mouseState;
RemoteControllerState Net::s_remoteCtlState;

void Net::begin(const char* ssid, const char* pass, uint8_t torchPin) {
  s_torchPin = torchPin;
  pinMode(s_torchPin, OUTPUT);
  digitalWrite(s_torchPin, LOW);

  Serial.println("Connecting WiFi...");
  WiFi.mode(WIFI_STA);

  // with classic bluetooth running, wifi settings can be a bit touchy
  // this setup gave better camera streaming results
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\nConnected. IP: %s\n", WiFi.localIP().toString().c_str());

  // websocket runs on 81 so port 80 stays free if needed later
  s_ws = new WebSocketsServer(81);
  s_ws->begin();

  // leave this here in case lower latency is needed later
  // s_ws->setNoDelay(true);

  s_ws->onEvent(Net::onEvent);
}

void Net::loop() {
  if (s_ws) s_ws->loop();
}

void Net::send(String s) {
  if (s_ws) s_ws->broadcastTXT(s);
}

void Net::sendTo(uint8_t clientNum, String s) {
  if (s_ws) s_ws->sendTXT(clientNum, s);
}

void Net::sendBinary(const uint8_t* data, size_t len) {
  if (s_ws) s_ws->broadcastBIN(data, len);
}

void Net::sendToBinary(uint8_t clientNum, const uint8_t* data, size_t len) {
  if (s_ws) s_ws->sendBIN(clientNum, data, len);
}

bool Net::hasClients() const {
  return s_lastClient >= 0;
}

void Net::printCombinedStateToSerial() {
  // turned off because it was flooding the serial monitor
  // only motor output is really useful there right now
  return;

  // old debug print left here in case it is needed again later
  Serial.print("[STATE] Mouse: ");
  if (s_mouseState.valid) {
    Serial.printf("X=%d Y=%d  ", s_mouseState.x, s_mouseState.y);
  } else {
    Serial.print("X=NA Y=NA  ");
  }

  Serial.print("Controller: ");
  if (s_remoteCtlState.valid) {
    Serial.printf(
      "LX=%.2f LY=%.2f RX=%.2f RY=%.2f Th=%.2f Br=%.2f  X=%d O=%d []=%d /=%d  DPAD=(%d,%d)\n",
      s_remoteCtlState.lx,
      s_remoteCtlState.ly,
      s_remoteCtlState.rx,
      s_remoteCtlState.ry,
      s_remoteCtlState.throttle,
      s_remoteCtlState.brake,
      s_remoteCtlState.cross,
      s_remoteCtlState.circle,
      s_remoteCtlState.square,
      s_remoteCtlState.triangle,
      s_remoteCtlState.dpadX,
      s_remoteCtlState.dpadY
    );
  } else {
    Serial.println("LX=NA LY=NA RX=NA RY=NA Th=NA Br=NA X=NA O=NA []=NA /=NA DPAD=(NA,NA)");
  }
}

void Net::onEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = s_ws->remoteIP(num);
      Serial.printf("[%u] WS connected from %s\n", num, ip.toString().c_str());
      s_lastClient = num;
      s_ws->sendTXT(num, "ESP32: hello");
      break;
    }

    case WStype_DISCONNECTED:
      Serial.printf("[%u] WS disconnected\n", num);
      if ((int)num == s_lastClient) s_lastClient = -1;
      digitalWrite(s_torchPin, LOW);
      break;

    case WStype_TEXT: {
      String msg((char*)payload, len);

      // raw rx printing was useful once, but it gets noisy fast
      // Serial.printf("[%u] RX: %s\n", num, msg.c_str());

      // 1) current mouse format: "M:x,y"
      if (msg.startsWith("M:")) {
        int commaIdx = msg.indexOf(',', 2);
        if (commaIdx > 2) {
          String sx = msg.substring(2, commaIdx);
          String sy = msg.substring(commaIdx + 1);

          s_mouseState.x = sx.toInt();
          s_mouseState.y = sy.toInt();
          s_mouseState.valid = true;

          // torch still follows mouse x position
          if (s_mouseState.x >= 250) digitalWrite(s_torchPin, HIGH);
          else                       digitalWrite(s_torchPin, LOW);
        }
        printCombinedStateToSerial();
        break;
      }

      // 2) current controller format:
      //    "C:lx,ly,rx,ry,throttle,brake,cross,
      //       circle,square,triangle,dpadX,dpadY"
      if (msg.startsWith("C:")) {
        String rest = msg.substring(2);

        float valsF[6] = {0};
        int   valsI[6] = {0};
        int fieldIdx = 0;

        // parse the comma-separated fields by hand
        int start = 0;
        while (fieldIdx < 12) {
          int sep = rest.indexOf(',', start);
          String token;
          if (sep == -1) {
            token = rest.substring(start);
          } else {
            token = rest.substring(start, sep);
          }

          token.trim();

          if (fieldIdx < 6) {
            valsF[fieldIdx] = token.toFloat();
          } else {
            valsI[fieldIdx - 6] = token.toInt();
          }

          fieldIdx++;
          if (sep == -1) break;
          start = sep + 1;
        }

        if (fieldIdx >= 6) {
          s_remoteCtlState.lx       = valsF[0];
          s_remoteCtlState.ly       = valsF[1];
          s_remoteCtlState.rx       = valsF[2];
          s_remoteCtlState.ry       = valsF[3];
          s_remoteCtlState.throttle = valsF[4];
          s_remoteCtlState.brake    = valsF[5];
        }

        if (fieldIdx >= 12) {
          s_remoteCtlState.cross    = valsI[0];
          s_remoteCtlState.circle   = valsI[1];
          s_remoteCtlState.square   = valsI[2];
          s_remoteCtlState.triangle = valsI[3];
          s_remoteCtlState.dpadX    = valsI[4];
          s_remoteCtlState.dpadY    = valsI[5];
          s_remoteCtlState.valid    = true;
          s_lastControllerRxMs = millis();
        } else {
          // if the message is cut short, just mark it invalid
          s_remoteCtlState.valid = false;
        }

        printCombinedStateToSerial();
        break;
      }

      // 3) older mouse format: "X<val>Y<val>"
      {
        int xIdx = msg.indexOf('X');
        int yIdx = msg.indexOf('Y');

        if (xIdx >= 0 && yIdx > xIdx) {
          String sx = msg.substring(xIdx + 1, yIdx);
          String sy = msg.substring(yIdx + 1);

          s_mouseState.x = sx.toInt();
          s_mouseState.y = sy.toInt();
          s_mouseState.valid = true;

          if (s_mouseState.x >= 250) digitalWrite(s_torchPin, HIGH);
          else                       digitalWrite(s_torchPin, LOW);

          printCombinedStateToSerial();
        }
      }
      break;
    }

    default:
      break;
  }
}