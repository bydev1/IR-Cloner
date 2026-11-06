/**
 * @file web_server.cpp
 * @brief Wi-Fi AP + LittleFS + REST API for Universal IR Transceiver (Demo)
 * @note Requires: Arduino-ESP32, ArduinoJson v7, LittleFS
 */

#include "web_server.h"
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <vector>

// true = ESP32 สั่ง arm_rx ไปแล้ว และยังไม่เห็นว่า FPGA จับเฟรมเสร็จ
// (FPGA จะเมิน start_tx / write_tx_buf แบบเงียบๆ ระหว่างรอจับสัญญาณ)
static bool s_rxArmed = false;

// =============================================================================
// โครงสร้างใหม่: อุปกรณ์ -> ปุ่ม
//   /profiles/<devId>/_dev.json = {"name":"ทีวีบ้าน","buttons":[{"id":"b1","name":"เปิด/ปิด"}]}
//   /profiles/<devId>/<btnId>.json = {"length":N,"data":[...]}   (รูปแบบเดียวกับโปรไฟล์เดิม)
// รหัส devId/btnId สร้างอัตโนมัติ (d1, d2 / b1, b2) ส่วนชื่อที่แสดงเก็บใน _dev.json
// (ตั้งเป็นภาษาไทยได้) โปรไฟล์เดิมแบบ /profiles/<name>.json ยังใช้งานได้เหมือนเดิม
// =============================================================================
static String devDir(const String &dev)  { return String(PROFILE_DIR) + "/" + dev; }
static String devMeta(const String &dev) { return devDir(dev) + "/_dev.json"; }
static String btnPath(const String &dev, const String &btn) { return devDir(dev) + "/" + btn + ".json"; }

static bool idOk(const String &id) {   // a-z A-Z 0-9 _ - ยาว 1..PROFILE_NAME_MAX
    size_t n = id.length();
    if (n == 0 || n > PROFILE_NAME_MAX) return false;
    for (size_t i = 0; i < n; i++) {
        char c = id[i];
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

static void outJson(WebServer &srv, int code, const String &body) {
    srv.sendHeader("Cache-Control", "no-store");
    srv.send(code, "application/json", body);
}

static void outErr(WebServer &srv, int code, const char *msg) {
    outJson(srv, code, String("{\"success\":false,\"message\":\"") + msg + "\"}");
}

static bool loadMeta(const String &dev, JsonDocument &doc) {
    File f = LittleFS.open(devMeta(dev), "r");
    if (!f) return false;
    DeserializationError e = deserializeJson(doc, f);
    f.close();
    return e == DeserializationError::Ok && doc["buttons"].is<JsonArray>();
}

static bool saveMeta(const String &dev, JsonDocument &doc) {
    File f = LittleFS.open(devMeta(dev), "w");
    if (!f) return false;
    size_t n = serializeJson(doc, f);
    f.close();
    return n > 0;
}

// GET /api/devices -> {"devices":[{"id":"d1","name":"ทีวีบ้าน","buttons":[{"id":"b1","name":"เปิด/ปิด"}]}]}
static void apiDevices(WebServer &srv) {
    JsonDocument out;
    JsonArray devs = out["devices"].to<JsonArray>();

    File root = LittleFS.open(PROFILE_DIR);
    if (root && root.isDirectory()) {
        File f = root.openNextFile();
        while (f) {
            bool isDir = f.isDirectory();
            String id = String(f.name());
            f.close();
            int slash = id.lastIndexOf('/');
            if (slash >= 0) id = id.substring(slash + 1);

            if (isDir && idOk(id)) {
                JsonDocument meta;
                if (loadMeta(id, meta)) {
                    JsonObject d = devs.add<JsonObject>();
                    d["id"]   = id;
                    d["name"] = String(meta["name"] | "");
                    JsonArray ob = d["buttons"].to<JsonArray>();
                    for (JsonVariant b : meta["buttons"].as<JsonArray>()) {
                        JsonObject nb = ob.add<JsonObject>();
                        nb["id"]   = String(b["id"] | "");
                        nb["name"] = String(b["name"] | "");
                    }
                }
            }
            f = root.openNextFile();
        }
    }
    if (root) root.close();

    String body;
    serializeJson(out, body);
    outJson(srv, 200, body);
}

// POST /api/devices/create  Body: {"name":"ทีวีบ้าน"} -> {"success":true,"id":"d1"}
static void apiDeviceCreate(WebServer &srv) {
    JsonDocument req;
    if (deserializeJson(req, srv.arg("plain")) != DeserializationError::Ok) { outErr(srv, 400, "Invalid JSON"); return; }

    String name = req["name"] | "";
    name.trim();
    if (name.length() == 0 || name.length() > 60) { outErr(srv, 400, "Device name must be 1-60 bytes"); return; }

    String id;
    for (int n = 1; n < 100; n++) {
        String t = "d" + String(n);
        if (!LittleFS.exists(devDir(t))) { id = t; break; }
    }
    if (id.length() == 0) { outErr(srv, 507, "Too many devices"); return; }
    if (!LittleFS.mkdir(devDir(id))) { outErr(srv, 500, "mkdir failed"); return; }

    JsonDocument meta;
    meta["name"] = name;
    meta["buttons"].to<JsonArray>();
    if (!saveMeta(id, meta)) {
        LittleFS.rmdir(devDir(id));
        outErr(srv, 500, "Write failed (storage full?)");
        return;
    }
    outJson(srv, 200, "{\"success\":true,\"id\":\"" + id + "\"}");
}

// POST /api/devices/delete  Body: {"id":"d1"}  ลบอุปกรณ์พร้อมปุ่มทั้งหมด
static void apiDeviceDelete(WebServer &srv) {
    JsonDocument req;
    if (deserializeJson(req, srv.arg("plain")) != DeserializationError::Ok) { outErr(srv, 400, "Invalid JSON"); return; }

    String id = req["id"] | "";
    if (!idOk(id)) { outErr(srv, 400, "Invalid device id"); return; }

    String dir = devDir(id);
    File d = LittleFS.open(dir);
    if (!d || !d.isDirectory()) {
        if (d) d.close();
        outErr(srv, 404, "Device not found");
        return;
    }

    std::vector<String> files;          // เก็บรายชื่อก่อน แล้วค่อยลบ
    File f = d.openNextFile();
    while (f) {
        String n = String(f.name());
        f.close();
        int slash = n.lastIndexOf('/');
        if (slash >= 0) n = n.substring(slash + 1);
        files.push_back(n);
        f = d.openNextFile();
    }
    d.close();

    for (size_t i = 0; i < files.size(); i++) LittleFS.remove(dir + "/" + files[i]);
    LittleFS.rmdir(dir);
    outJson(srv, 200, "{\"success\":true}");
}

// POST /api/buttons/save  Body: {"device":"d1","name":"เปิด/ปิด","length":N,"data":[...]}
// ถ้าในอุปกรณ์นั้นมีปุ่มชื่อเดียวกันอยู่แล้ว จะเขียนทับปุ่มเดิม
static void apiButtonSave(WebServer &srv) {
    JsonDocument req;   // v7: จัดสรรบน Heap
    if (deserializeJson(req, srv.arg("plain")) != DeserializationError::Ok) { outErr(srv, 400, "Invalid JSON"); return; }

    String dev   = req["device"] | "";
    String bname = req["name"] | "";
    bname.trim();
    if (!idOk(dev)) { outErr(srv, 400, "Invalid device id"); return; }
    if (bname.length() == 0 || bname.length() > 60) { outErr(srv, 400, "Button name must be 1-60 bytes"); return; }

    JsonDocument meta;
    if (!loadMeta(dev, meta)) { outErr(srv, 404, "Device not found"); return; }

    JsonArray arr = req["data"].as<JsonArray>();
    if (arr.isNull()) { outErr(srv, 400, "Missing data array"); return; }
    size_t count = arr.size();
    if (count == 0 || count > MAX_IR_WORDS) { outErr(srv, 400, "data size must be 1-1024"); return; }
    if ((size_t)(req["length"] | 0) != count) { outErr(srv, 400, "length does not match data size"); return; }

    // ตรวจค่าพัลส์ 1..65535 แล้วสร้างข้อความที่จะเขียนลงไฟล์
    String out;
    out.reserve(40 + count * 6);
    out = "{\"length\":";
    out += (unsigned)count;
    out += ",\"data\":[";
    bool first = true;
    for (JsonVariant v : arr) {
        long x = v.as<long>();
        if (!v.is<long>() || x < 1 || x > 65535) { outErr(srv, 400, "Pulse value must be 1-65535"); return; }
        if (!first) out += ',';
        out += (unsigned)x;
        first = false;
    }
    out += "]}";

    // หาปุ่มชื่อเดิม (เขียนทับ) หรือสร้างรหัสใหม่
    JsonArray btns = meta["buttons"].as<JsonArray>();
    String bid;
    for (JsonVariant b : btns) {
        if (String(b["name"] | "") == bname) { bid = String(b["id"] | ""); break; }
    }
    bool isNew = (bid.length() == 0);
    if (isNew) {
        for (int n = 1; n < 100; n++) {
            String t = "b" + String(n);
            bool used = false;
            for (JsonVariant b : btns) {
                if (String(b["id"] | "") == t) { used = true; break; }
            }
            if (!used) { bid = t; break; }
        }
        if (bid.length() == 0) { outErr(srv, 507, "Too many buttons"); return; }
    }

    File f = LittleFS.open(btnPath(dev, bid), "w");
    if (!f) { outErr(srv, 500, "Cannot open file"); return; }
    size_t written = f.print(out);
    f.close();
    if (written != out.length()) {
        LittleFS.remove(btnPath(dev, bid));
        outErr(srv, 500, "Write failed (storage full?)");
        return;
    }

    if (isNew) {
        JsonObject nb = btns.add<JsonObject>();
        nb["id"]   = bid;
        nb["name"] = bname;
        if (!saveMeta(dev, meta)) {
            LittleFS.remove(btnPath(dev, bid));
            outErr(srv, 500, "Write failed (storage full?)");
            return;
        }
    }
    outJson(srv, 200, "{\"success\":true,\"id\":\"" + bid + "\"}");
}

// POST /api/buttons/delete  Body: {"device":"d1","button":"b1"}
static void apiButtonDelete(WebServer &srv) {
    JsonDocument req;
    if (deserializeJson(req, srv.arg("plain")) != DeserializationError::Ok) { outErr(srv, 400, "Invalid JSON"); return; }

    String dev = req["device"] | "";
    String btn = req["button"] | "";
    if (!idOk(dev) || !idOk(btn)) { outErr(srv, 400, "Invalid device or button"); return; }

    JsonDocument meta;
    if (!loadMeta(dev, meta)) { outErr(srv, 404, "Device not found"); return; }

    JsonArray btns = meta["buttons"].as<JsonArray>();
    bool found = false;
    for (size_t i = 0; i < btns.size(); i++) {
        if (String(btns[i]["id"] | "") == btn) { btns.remove(i); found = true; break; }
    }
    if (!found) { outErr(srv, 404, "Button not found"); return; }

    LittleFS.remove(btnPath(dev, btn));
    if (!saveMeta(dev, meta)) { outErr(srv, 500, "Write failed"); return; }
    outJson(srv, 200, "{\"success\":true}");
}

// =============================================================================
// Constructor / begin / handle
// =============================================================================
IrWebServer::IrWebServer(SpiMaster &spi)
    : _spi(spi), _server(HTTP_PORT), _spiMutex(nullptr) {
    memset(_pulseBuf, 0, sizeof(_pulseBuf));
}

bool IrWebServer::begin() {
    _spiMutex = xSemaphoreCreateMutex();
    if (_spiMutex == nullptr) {
        Serial.println("[WEB] Mutex create failed");
        return false;
    }

    // ---- LittleFS ----
    if (!LittleFS.begin(true)) {   // true = format ถ้ายังไม่เคย format
        Serial.println("[WEB] LittleFS mount failed");
        return false;
    }
    if (!LittleFS.exists(PROFILE_DIR)) {
        LittleFS.mkdir(PROFILE_DIR);
    }

    // ---- Wi-Fi AP ----
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS)) {
        Serial.println("[WEB] softAP failed");
        return false;
    }
    Serial.printf("[WEB] AP '%s' ready, IP: %s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

    // ---- Routes ----
    _server.on("/",                      HTTP_GET,  [this]() { handleIndex(); });
    _server.on("/api/status",            HTTP_GET,  [this]() { handleStatus(); });
    _server.on("/api/capture/arm",       HTTP_POST, [this]() { handleCaptureArm(); });
    _server.on("/api/capture/poll",      HTTP_GET,  [this]() { handleCapturePoll(); });
    _server.on("/api/profiles",          HTTP_GET,  [this]() { handleProfileList(); });
    _server.on("/api/profiles/save",     HTTP_POST, [this]() { handleProfileSave(); });
    _server.on("/api/profiles/delete",   HTTP_POST, [this]() { handleProfileDelete(); });
    _server.on("/api/send",              HTTP_POST, [this]() { handleSend(); });
    // โครงสร้างอุปกรณ์ -> ปุ่ม
    _server.on("/api/devices",           HTTP_GET,  [this]() { apiDevices(_server); });
    _server.on("/api/devices/create",    HTTP_POST, [this]() { apiDeviceCreate(_server); });
    _server.on("/api/devices/delete",    HTTP_POST, [this]() { apiDeviceDelete(_server); });
    _server.on("/api/buttons/save",      HTTP_POST, [this]() { apiButtonSave(_server); });
    _server.on("/api/buttons/delete",    HTTP_POST, [this]() { apiButtonDelete(_server); });
    _server.onNotFound([this]() { handleNotFound(); });

    _server.begin();
    Serial.println("[WEB] HTTP server started");
    return true;
}

void IrWebServer::handle() {
    _server.handleClient();
}

// =============================================================================
// Helpers
// =============================================================================
void IrWebServer::sendJson(int code, const String &body) {
    _server.sendHeader("Cache-Control", "no-store");
    _server.send(code, "application/json", body);
}

void IrWebServer::sendError(int code, const char *msg) {
    String body = "{\"success\":false,\"message\":\"";
    body += msg;
    body += "\"}";
    sendJson(code, body);
}

bool IrWebServer::validName(const String &name) const {
    size_t n = name.length();
    if (n == 0 || n > PROFILE_NAME_MAX) return false;
    for (size_t i = 0; i < n; i++) {
        char c = name[i];
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

String IrWebServer::profilePath(const String &name) const {
    return String(PROFILE_DIR) + "/" + name + ".json";
}

bool IrWebServer::spiLock(uint32_t timeoutMs) {
    return xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void IrWebServer::spiUnlock() {
    xSemaphoreGive(_spiMutex);
}

// =============================================================================
// GET /  -> index.html จาก LittleFS
// =============================================================================
void IrWebServer::handleIndex() {
    File f = LittleFS.open(INDEX_FILE, "r");
    if (!f) {
        _server.send(404, "text/plain", "index.html not found. Upload it to LittleFS.");
        return;
    }
    _server.streamFile(f, "text/html");
    f.close();
}

// =============================================================================
// GET /api/status
// =============================================================================
void IrWebServer::handleStatus() {
    bool    connected = false;
    uint8_t status    = 0;

    if (!spiLock()) { sendError(503, "SPI busy"); return; }
    connected = _spi.ping();
    if (connected) {
        if (_spi.get_status(&status) != ESP_OK) status = 0;
    }
    spiUnlock();

    String body = "{\"connected\":";
    body += connected ? "true" : "false";
    body += ",\"rx_done\":";
    body += (status & STATUS_RX_DONE) ? "true" : "false";
    body += ",\"tx_busy\":";
    body += (status & STATUS_TX_BUSY) ? "true" : "false";
    body += ",\"rx_overflow\":";
    body += (status & STATUS_RX_OVERFLOW) ? "true" : "false";
    body += "}";
    sendJson(200, body);
}

// =============================================================================
// POST /api/capture/arm
// =============================================================================
void IrWebServer::handleCaptureArm() {
    if (!spiLock()) { sendError(503, "SPI busy"); return; }
    esp_err_t err = _spi.arm_rx();
    spiUnlock();

    if (err != ESP_OK) { sendError(500, "arm_rx failed"); return; }
    s_rxArmed = true;
    sendJson(200, "{\"success\":true,\"message\":\"RX Armed\"}");
}

// =============================================================================
// GET /api/capture/poll
// =============================================================================
void IrWebServer::handleCapturePoll() {
    uint8_t   status = 0;
    uint16_t  len    = 0;
    esp_err_t err;

    if (!spiLock()) { sendError(503, "SPI busy"); return; }

    err = _spi.get_status(&status);
    if (err != ESP_OK) {
        spiUnlock();
        sendError(500, "get_status failed");
        return;
    }

    if (!(status & STATUS_RX_DONE)) {
        spiUnlock();
        sendJson(200, "{\"status\":\"waiting\"}");
        return;
    }

    // rx_done == 1 -> FPGA จับเฟรมเสร็จแล้ว ไม่ได้รออยู่อีก
    s_rxArmed = false;

    // อ่านข้อมูลออกมา
    err = _spi.get_rx_len(&len);
    if (err == ESP_OK) {
        if (len > MAX_IR_WORDS) len = MAX_IR_WORDS;
        if (len > 0) err = _spi.read_rx_buf(_pulseBuf, len);
    }
    spiUnlock();

    if (err != ESP_OK) { sendError(500, "read_rx_buf failed"); return; }

    String body;
    body.reserve(80 + (size_t)len * 6);
    body = "{\"status\":\"captured\",\"overflow\":";
    body += (status & STATUS_RX_OVERFLOW) ? "true" : "false";
    body += ",\"length\":";
    body += len;
    body += ",\"data\":[";
    for (uint16_t i = 0; i < len; i++) {
        if (i) body += ',';
        body += _pulseBuf[i];
    }
    body += "]}";
    sendJson(200, body);
}

// =============================================================================
// GET /api/profiles
// =============================================================================
void IrWebServer::handleProfileList() {
    String body = "{\"profiles\":[";
    bool first = true;

    File dir = LittleFS.open(PROFILE_DIR);
    if (dir && dir.isDirectory()) {
        File f = dir.openNextFile();
        while (f) {
            String fn = String(f.name());
            f.close();
            int slash = fn.lastIndexOf('/');
            if (slash >= 0) fn = fn.substring(slash + 1);
            if (fn.endsWith(".json")) {
                fn = fn.substring(0, fn.length() - 5);
                if (validName(fn)) {
                    if (!first) body += ',';
                    body += '"';
                    body += fn;
                    body += '"';
                    first = false;
                }
            }
            f = dir.openNextFile();
        }
    }
    if (dir) dir.close();

    body += "]}";
    sendJson(200, body);
}

// =============================================================================
// POST /api/profiles/save   Body: {"name":"..","length":N,"data":[..]}
// =============================================================================
void IrWebServer::handleProfileSave() {
    String body = _server.arg("plain");
    if (body.length() == 0) { sendError(400, "Empty body"); return; }

    JsonDocument doc;   // v7: จัดสรรบน Heap อัตโนมัติ
    if (deserializeJson(doc, body) != DeserializationError::Ok) {
        sendError(400, "Invalid JSON");
        return;
    }

    String name = doc["name"] | "";
    if (!validName(name)) { sendError(400, "Invalid name (a-z A-Z 0-9 _ - max 32)"); return; }

    JsonArray arr = doc["data"].as<JsonArray>();
    if (arr.isNull()) { sendError(400, "Missing data array"); return; }

    size_t count = arr.size();
    if (count == 0 || count > MAX_IR_WORDS) { sendError(400, "data size must be 1-1024"); return; }
    if ((size_t)(doc["length"] | 0) != count) { sendError(400, "length does not match data size"); return; }

    // ตรวจค่าพัลส์ทุกตัว 1..65535 แล้วสร้าง JSON ที่จะเขียนลงไฟล์
    String out;
    out.reserve(80 + count * 6);
    out = "{\"name\":\"";
    out += name;
    out += "\",\"length\":";
    out += (unsigned)count;
    out += ",\"data\":[";
    bool first = true;
    for (JsonVariant v : arr) {
        long x = v.as<long>();
        if (!v.is<long>() || x < 1 || x > 65535) {
            sendError(400, "Pulse value must be 1-65535");
            return;
        }
        if (!first) out += ',';
        out += (unsigned)x;
        first = false;
    }
    out += "]}";

    File f = LittleFS.open(profilePath(name), "w");
    if (!f) { sendError(500, "Cannot open file"); return; }
    size_t written = f.print(out);
    f.close();

    if (written != out.length()) {
        LittleFS.remove(profilePath(name));
        sendError(500, "Write failed (storage full?)");
        return;
    }
    sendJson(200, "{\"success\":true}");
}

// =============================================================================
// POST /api/profiles/delete   Body: {"name":".."}
// =============================================================================
void IrWebServer::handleProfileDelete() {
    JsonDocument doc;
    if (deserializeJson(doc, _server.arg("plain")) != DeserializationError::Ok) {
        sendError(400, "Invalid JSON");
        return;
    }

    String name = doc["name"] | "";
    if (!validName(name)) { sendError(400, "Invalid name"); return; }

    String path = profilePath(name);
    if (!LittleFS.exists(path)) { sendError(404, "Profile not found"); return; }
    if (!LittleFS.remove(path)) { sendError(500, "Delete failed"); return; }

    sendJson(200, "{\"success\":true}");
}

// =============================================================================
// POST /api/send   Body: {"name":".."}
// =============================================================================
void IrWebServer::handleSend() {
    JsonDocument req;
    if (deserializeJson(req, _server.arg("plain")) != DeserializationError::Ok) {
        sendError(400, "Invalid JSON");
        return;
    }

    // รองรับ 2 แบบ: {device, button} (โครงสร้างใหม่) หรือ {name} (โปรไฟล์เดิม)
    String path;
    if (req["device"].is<const char *>()) {
        String dev = req["device"] | "";
        String btn = req["button"] | "";
        if (!idOk(dev) || !idOk(btn)) { sendError(400, "Invalid device or button"); return; }
        path = btnPath(dev, btn);
    } else {
        String name = req["name"] | "";
        if (!validName(name)) { sendError(400, "Invalid name"); return; }
        path = profilePath(name);
    }

    File f = LittleFS.open(path, "r");
    if (!f) { sendError(404, "Profile not found"); return; }

    // อ่านโปรไฟล์จากไฟล์ (ใช้ Heap) แล้วคัดลอกลง static buffer
    uint16_t count = 0;
    {
        JsonDocument prof;
        DeserializationError derr = deserializeJson(prof, f);
        f.close();
        if (derr != DeserializationError::Ok) { sendError(500, "Profile file corrupted"); return; }

        JsonArray arr = prof["data"].as<JsonArray>();
        if (arr.isNull() || arr.size() == 0 || arr.size() > MAX_IR_WORDS) {
            sendError(500, "Profile data invalid");
            return;
        }
        for (JsonVariant v : arr) {
            long x = v.as<long>();
            if (x < 1 || x > 65535) { sendError(500, "Profile pulse out of range"); return; }
            _pulseBuf[count++] = (uint16_t)x;
        }
    }

    if (!spiLock()) { sendError(503, "SPI busy"); return; }

    // ต้องเช็คว่า FPGA ไม่ได้ยิงอยู่ก่อนเริ่มรอบใหม่
    uint8_t status = 0;
    esp_err_t err = _spi.get_status(&status);
    if (err != ESP_OK) {
        spiUnlock();
        sendError(500, "get_status failed");
        return;
    }
    if (status & STATUS_TX_BUSY) {
        spiUnlock();
        sendError(409, "Busy: FPGA is transmitting");
        return;
    }

    // FPGA ที่ยังรอจับสัญญาณอยู่จะเมินทั้ง write_tx_buf และ start_tx แบบเงียบๆ
    // ถ้า rx_done = 1 แปลว่าจับเฟรมไปแล้ว (ออกจากสถานะรอ) ส่งต่อได้
    if (s_rxArmed) {
        if (status & STATUS_RX_DONE) {
            s_rxArmed = false;
        } else {
            spiUnlock();
            sendError(412, "FPGA is waiting for an IR signal. Press the remote once (or reset the FPGA), then send again");
            return;
        }
    }

    // ลำดับตามสเปก: write_tx_buf -> set_tx_len -> start_tx
    err = _spi.write_tx_buf(_pulseBuf, count);
    if (err == ESP_OK) err = _spi.set_tx_len(count);
    if (err == ESP_OK) err = _spi.start_tx();
    spiUnlock();

    if (err != ESP_OK) { sendError(500, "SPI transfer failed"); return; }
    sendJson(200, "{\"success\":true,\"message\":\"TX Started\"}");
}

// =============================================================================
// 404
// =============================================================================
void IrWebServer::handleNotFound() {
    sendError(404, "Not found");
}
