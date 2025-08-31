#include "my_web_config.h"
// ======================= 工具：发送 JSON 到某个客户端/全部 =======================
void wsSendTo(AsyncWebSocketClient *c, const JsonDocument &doc)
{
    String s;
    serializeJson(doc, s);
    if (c)
        c->text(s);
}

// ======================= 工具函数：WS 保活与广播 =======================
void my_wsheart()
{
    ws.cleanupClients(); // 清理断开的
    size_t n = ws.count();
    for (size_t i = 0; i < n; i++)
    {
        AsyncWebSocketClient *c = ws.client(i);
        if (!c)
            continue;
        if (c->canSend())
            c->ping(); // 发送 ping 帧
    }
    vTaskDelay(pdMS_TO_TICKS(15000)); // 每 15 秒 ping 一次
}

bool wsCanBroadcast()
{
    bool ok = true;
    size_t n = ws.count();
    for (size_t i = 0; i < n; i++)
    {
        AsyncWebSocketClient *c = ws.client(i);
        if (!c)
            continue;
        if (c->queueIsFull() || !c->canSend())
        {
            ok = false;
            break;
        }
    }
    return ok;
}

void wsBroadcast(const JsonDocument &doc)
{
    // 清理断开的客户端，避免幽灵连接占坑
    ws.cleanupClients();
    if (!wsCanBroadcast())
        return; // 丢掉这一帧，避免把队列越堆越高

    String s;
    s.reserve(320); // 预留缓冲，减少堆分配
    serializeJson(doc, s);
    ws.textAll(s);
}

float my_math_deadband(float value, float deadband)
{
    float new_value = value;
    if (fabsf(value) < deadband)
        float new_value = 0.0f;
    return new_value;
}

float my_math_limit(float value, float min, float max)
{
    float new_value = value;
    if (value < min)
        new_value = min;
    else if (value > max)
        new_value = max;
    return new_value;
}

// ====== 字符串工具函数 ======

void rtrim_inplace(char *s)
{
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n'))
        --e;
    *e = '\0';
}