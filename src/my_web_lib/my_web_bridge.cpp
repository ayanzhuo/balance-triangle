#include "my_web_config.h"
#include "my_control.h"

bool robot_run = false;
bool charts_send = true;
bool fallcheck_enable = true;

bool testmode_enabled = false;
int testmode_motor_mode = 0;
float testmode_value = 0;

float joyY = 0;
float joyX = 0;
float joyA = 0;

int telem_hz = 10;

bool send_fall = false;
float send_msg[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float pid_cache[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const char *keys[13] = {"key01", "key02", "key03", "key04", "key05", "key06", "key07", "key08", "key09", "key10", "key11", "key12", "key13"};
// ui标签
ChartConfig chart_cfg[3] = {
    {"", {"", "", ""}},
    {"", {"", "", ""}},
    {"", {"", "", ""}},
};

SliderGroup slider_cfg[4] = {
    {"", {"", "", ""}},
    {"", {"", "", ""}},
    {"", {"", "", ""}},
    {"", {"", "", ""}},
};

void my_web_ui_init(SliderGroup slider_group[4], ChartConfig chart_config[3])
{
    for (int i = 0; i < 3; i++)
        chart_cfg[i] = chart_config[i];
    for (int i = 0; i < 4; i++)
        slider_cfg[i] = slider_group[i];
}
// ========== UI 配置 JSON 打包（仅首连时发送一次） ==========
JsonDocument cb_ui_config()
{
    StaticJsonDocument<512> doc; // 容量偏小，容易截断；改为动态并给足余量
    // DynamicJsonDocument doc(1536);

    // 把 type 放最前，前端能更快分发；哪怕极端截断也更易诊断
    doc["type"] = "ui_config";

    // charts
    JsonArray charts = doc.createNestedArray("charts");
    for (const auto &c : chart_cfg)
    {
        JsonObject o = charts.createNestedObject();
        o["title"] = c.title;
        JsonArray l = o.createNestedArray("legends");
        for (int i = 0; i < 3; ++i)
            l.add(c.legend[i]);
    }

    // sliders
    JsonArray sliders = doc.createNestedArray("sliders");
    for (const auto &sg : slider_cfg)
    {
        JsonObject g = sliders.createNestedObject();
        g["group"] = sg.group;
        JsonArray n = g.createNestedArray("names");
        for (int i = 0; i < 3; ++i)
            n.add(sg.names[i]);
    }

    doc.shrinkToFit(); // 发送前收紧空间，减轻带宽
    return doc;
}

// 12+1 路遥测数据（自定义映射：电流/角速度/位置/温度...）
uint32_t cb_telemetry()
{
    StaticJsonDocument<320> doc;
    uint32_t hz = telem_hz ? telem_hz : 1;
    uint32_t dt_ms = 1000 / hz;

    doc["type"] = "telemetry";
    doc["fallen"] = send_fall;
    doc["pitch"] = send_msg[0];
    doc["roll"] = send_msg[1];
    doc["yaw"] = send_msg[2];

    JsonArray arr = doc.createNestedArray("d");
    if (charts_send)
        for (int i = 3; i < 12; i++)
            arr.add(send_msg[i]);

    wsBroadcast(doc);
    return dt_ms; // 返回下次采样间隔（ms）
}

// PID 设置（顺序：角度P/I/D，速度P/I/D，位置P/I/D，航向P/I/D，zero）
void cb_pid_set(JsonObject param)
{
    if (!param.isNull())
        for (int i = 0; i < 13; i++)
            pid_cache[i] = param[keys[i]] | pid_cache[i]; // 没有就用原始数据

    swing_up_voltage = pid_cache[0];
    // 其他 PID 参数在你接入后逐个赋值
    target_angle = pid_cache[12];
}

// PID 读取
JsonDocument cb_pid_get()
{
    StaticJsonDocument<320> out;
    JsonObject pr = out.createNestedObject("param");
    out["type"] = "pid";

    pid_cache[0] = swing_up_voltage;
    pid_cache[12] = target_angle;

    for (int i = 0; i < 13; i++)
        pr[keys[i]] = pid_cache[i];

    return out;
}

// 测试模式
void cb_testmode(bool enabled, int mode, float value)
{
    testmode_enabled = enabled, testmode_value = value, testmode_motor_mode = mode;
}

// 摇杆
void cb_joystick(float x, float y, float a)
{
    joyX = my_math_deadband(my_math_limit(x, -1.0f, 1.0f), 0.02f);
    joyY = my_math_deadband(my_math_limit(y, -1.0f, 1.0f), 0.02f);
    joyA = my_math_deadband(my_math_limit(a, -1.0f, 1.0f), 0.02f);
}

// led set
void cb_led_set(JsonObject param)
{
    JsonObject cfg_in = param["cfg"].as<JsonObject>();
    if (cfg_in.isNull())
        return;
    led_config.mode = cfg_in["mode"] | (int)led_config.mode;
    led_config.dir = cfg_in["dir"] | led_config.dir;
    led_config.interval = cfg_in["interval"] | led_config.interval;
    led_config.brightness = cfg_in["brightness"] | led_config.brightness;
    led_config.tailDecay = cfg_in["tailDecay"] | led_config.tailDecay;
    led_config.theaterStep = cfg_in["theaterStep"] | led_config.theaterStep;
    led_config.breathSpeed = cfg_in["breathSpeed"] | led_config.breathSpeed;
    led_config.rainbowSpeed = cfg_in["rainbowSpeed"] | led_config.rainbowSpeed;
    if (cfg_in.containsKey("color"))
    {
        const char *s = cfg_in["color"]; // "#rrggbb"
        uint32_t r = 0, g = 0, b = 0;
        if (s && strlen(s) == 7 && s[0] == '#')
        {
            auto hex = [&](char ch) -> uint8_t
            {
                if (ch >= '0' && ch <= '9')
                    return ch - '0';
                ch |= 32;
                return (ch >= 'a' && ch <= 'f') ? (ch - 'a' + 10) : 0;
            };
            r = (hex(s[1]) << 4) | hex(s[2]);
            g = (hex(s[3]) << 4) | hex(s[4]);
            b = (hex(s[5]) << 4) | hex(s[6]);
            led_config.color = (r << 16) | (g << 8) | b;
        }
    }
    leds_apply();
}
// led get
void cb_led_get(AsyncWebSocketClient *c)
{
    StaticJsonDocument<256> out;
    JsonObject s = out.createNestedObject("state");
    out["type"] = "led_state";
    char cc[8];
    snprintf(cc, sizeof(cc), "#%02X%02X%02X", (led_config.color >> 16) & 0xFF, (led_config.color >> 8) & 0xFF, led_config.color & 0xFF);
    s["color"] = cc;
    s["mode"] = (int)led_config.mode;
    s["dir"] = led_config.dir;
    s["interval"] = led_config.interval;
    s["brightness"] = led_config.brightness;
    s["tailDecay"] = led_config.tailDecay;
    s["theaterStep"] = led_config.theaterStep;
    s["breathSpeed"] = led_config.breathSpeed;
    s["rainbowSpeed"] = led_config.rainbowSpeed;
    s["power"] = led_config.powerOn;
    wsSendTo(c, out);
}