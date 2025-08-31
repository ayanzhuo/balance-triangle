#include "my_web_config.h"
// ======================= MIME & 静态文件 =======================
static String contentType(const String &path)
{
    if (path.endsWith(".htm") || path.endsWith(".html"))
        return "text/html; charset=utf-8";
    if (path.endsWith(".css"))
        return "text/css; charset=utf-8";
    if (path.endsWith(".js"))
        return "application/javascript; charset=utf-8";
    if (path.endsWith(".json"))
        return "application/json; charset=utf-8";
    if (path.endsWith(".png"))
        return "image/png";
    if (path.endsWith(".jpg") || path.endsWith(".jpeg"))
        return "image/jpeg";
    if (path.endsWith(".gif"))
        return "image/gif";
    if (path.endsWith(".svg"))
        return "image/svg+xml";
    if (path.endsWith(".ico"))
        return "image/x-icon";
    if (path.endsWith(".txt"))
        return "text/plain; charset=utf-8";
    return "application/octet-stream";
}

bool handleFileRead(AsyncWebServerRequest *req, String path)
{
    if (path.endsWith("/"))
        path += "home.html"; // 默认页
    const String gz = path + ".gz";
    const String type = contentType(path);

    if (FSYS.exists(gz))
    {
        auto *res = req->beginResponse(FSYS, gz, type);
        res->addHeader("Content-Encoding", "gzip");
        res->addHeader("Cache-Control", "public, max-age=604800");
        req->send(res);
        return true;
    }
    if (FSYS.exists(path))
    {
        auto *res = req->beginResponse(FSYS, path, type);
        res->addHeader("Cache-Control", "public, max-age=604800");
        req->send(res);
        return true;
    }
    return false;
}
