#include <Arduino.h>
#include "esp_camera.h"
#include "math.h"

#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"
#include "network_module.h"

#include "motor_driver.h"
#include "drive_control.h"

// ================== config ==================


static const char* WIFI_SSID     = "ssid";
static const char* WIFI_PASSWORD = "pass";

// flash LED pin
// for the AI thinker board this is usually GPIO 4
static const uint8_t TORCH_PIN = 4;

// ================== motor pins + pwm channels ==================
// change these if your H-bridge is wired differently
// each motor uses 2 pwm pins:
// A = forward, B = reverse
static const uint8_t MOTOR_L_PIN_A = 14;
static const uint8_t MOTOR_L_PIN_B = 15;
static const uint8_t MOTOR_R_PIN_A = 12;
static const uint8_t MOTOR_R_PIN_B = 13;

// each MotorDriver needs its own LEDC channels
// the camera already uses channel 0 / timer 0, so avoid that
static const uint8_t MOTOR_L_CH_A = 1;
static const uint8_t MOTOR_L_CH_B = 2;
static const uint8_t MOTOR_R_CH_A = 3;
static const uint8_t MOTOR_R_CH_B = 4;

// ================== globals ==================
Net Network;
MotorDriver MotorL;
MotorDriver MotorR;
DriveControl Drive;

// ================== camera init ==================
static bool cameraInit() {
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;

  c.pin_d0       = Y2_GPIO_NUM;
  c.pin_d1       = Y3_GPIO_NUM;
  c.pin_d2       = Y4_GPIO_NUM;
  c.pin_d3       = Y5_GPIO_NUM;
  c.pin_d4       = Y6_GPIO_NUM;
  c.pin_d5       = Y7_GPIO_NUM;
  c.pin_d6       = Y8_GPIO_NUM;
  c.pin_d7       = Y9_GPIO_NUM;
  c.pin_xclk     = XCLK_GPIO_NUM;
  c.pin_pclk     = PCLK_GPIO_NUM;
  c.pin_vsync    = VSYNC_GPIO_NUM;
  c.pin_href     = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn     = PWDN_GPIO_NUM;
  c.pin_reset    = RESET_GPIO_NUM;

  // some boards behave better if PWDN is set explicitly
#if PWDN_GPIO_NUM >= 0
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, LOW);
#endif

  // keeping this a bit conservative helps when Wi-Fi and other stuff are active
  c.xclk_freq_hz = 20000000;

  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_QVGA;     // 320x240 is a decent middle ground
  c.jpeg_quality = 15;                 // bigger number = more compression
  c.fb_count     = 2;                  // use double buffering

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.print("Camera init failed, err=0x");
    Serial.println((uint32_t)err, HEX);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("Camera sensor get failed");
    return false;
  }

  // basic sensor setup
  // most of this is just left at fairly safe values
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, 0);
  s->set_aec_value(s, 300);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)0);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);

  return true;
}

// ================== setup ==================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println("=== ESP32-CAM: camera over WS (binary), mouse+controller over WS RX ===");

  // start Wi-Fi and websocket stuff
  Network.begin(WIFI_SSID, WIFI_PASSWORD, TORCH_PIN);

  // set up both motor drivers
  MotorL.begin(MOTOR_L_PIN_A, MOTOR_L_PIN_B, MOTOR_L_CH_A, MOTOR_L_CH_B);
  MotorR.begin(MOTOR_R_PIN_A, MOTOR_R_PIN_B, MOTOR_R_CH_A, MOTOR_R_CH_B);

  // drive config
  DriveControl::Config cfg;
  cfg.initialDirection = DriveControl::Direction::Forward;
  cfg.failsafeTimeoutMs = 500;           
  cfg.throttleEpsForDirChange = 0.03f;
  cfg.throttleDeadband = 0.02f;
  cfg.lxDeadband = 0.20f;                // raise this if the stick centre drifts
  Drive.begin(cfg);

  // bring up the camera
  if (!cameraInit()) {
    Serial.println("Camera init failed - rebooting in 5s");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Camera initialised.");
}

// ================== loop ==================
void loop() {
  // handle websocket traffic first
  Network.loop();

  // latest controller state
  RemoteControllerState cs = Network.getRemoteControllerState();
  uint32_t lastRx = Network.getLastControllerRxMs();

  // convert controller input into left/right motor commands
  DriveControl::DriveCmd cmd = Drive.update(cs, lastRx, millis());

  // print motor changes only when something actually changes
  auto pwmFromCmd = [](float x) -> int {
    float ax = fabsf(x);
    if (ax > 1.0f) ax = 1.0f;
    return (int)(ax * 255.0f);
  };

  auto dirFromCmd = [](float x) -> int {
    return (x > 0.001f) ? 1 : ((x < -0.001f) ? -1 : 0);
  };

  static int lastPwmL = -1, lastPwmR = -1;
  static int lastDirL = 999, lastDirR = 999;

  int pwmL = pwmFromCmd(cmd.left);
  int pwmR = pwmFromCmd(cmd.right);
  int dirL = dirFromCmd(cmd.left);
  int dirR = dirFromCmd(cmd.right);

  if (pwmL != lastPwmL || pwmR != lastPwmR || dirL != lastDirL || dirR != lastDirR) {
    const char* dirStrL = (dirL == 1) ? "FORWARD" : ((dirL == -1) ? "REVERSE" : "STOP");
    const char* dirStrR = (dirR == 1) ? "FORWARD" : ((dirR == -1) ? "REVERSE" : "STOP");
    Serial.printf("MotorL: speed=%d dir=%s | MotorR: speed=%d dir=%s\n", pwmL, dirStrL, pwmR, dirStrR);

    lastPwmL = pwmL; lastPwmR = pwmR;
    lastDirL = dirL; lastDirR = dirR;
  }

  // send commands to the motors
  MotorL.setCommand(cmd.left);
  MotorR.setCommand(cmd.right);

  static unsigned long lastCamMs = 0;
  const unsigned long now = millis();

  // send camera frames over websocket as binary JPEG packets
  if (now - lastCamMs >= 16) {
    lastCamMs = now;

    if (Network.hasClients()) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("esp_camera_fb_get() failed");
        return;
      }

      Network.sendBinary(fb->buf, fb->len);

      esp_camera_fb_return(fb);
    }
  }

  delay(1);
}