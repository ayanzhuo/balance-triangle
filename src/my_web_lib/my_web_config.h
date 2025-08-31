#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <LittleFS.h>
#include "my_web.h"
#include "my_led.h"
// 变量暴露
// 文件系统
#define FSYS LittleFS
// 以下是参数定义

// web刷新限制
#define REFRESH_RATE_DEF 10
#define REFRESH_RATE_MAX 60
#define REFRESH_RATE_MIN 1
// 异步服务器对象
extern AsyncWebServer server;
extern AsyncWebSocket ws;

// UI 配置打包
JsonDocument cb_ui_config();
uint32_t cb_telemetry();
void cb_pid_set(JsonObject param);
JsonDocument cb_pid_get();
void cb_testmode(bool enabled, int mode, float value);
void cb_joystick(float x, float y, float a);
void cb_led_set(JsonObject param);
void cb_led_get(AsyncWebSocketClient *c);

// fs函数
static String contentType(const String &path);

// webtool函数
void wsSendTo(AsyncWebSocketClient *c, const JsonDocument &doc);
void my_wsheart();
bool wsCanBroadcast();
void wsBroadcast(const JsonDocument &doc);
bool handleFileRead(AsyncWebServerRequest *req, String path);

// tool函数
float my_math_deadband(float value, float deadband);
float my_math_limit(float value, float min, float max);
void rtrim_inplace(char *s);