/**********************************************************************
  Filename    : app_httpd.cpp
  Description : HTTP server for Fridge CAM.
                Endpoints:
                  /          — Web UI with door status, recording indicator,
                               last capture, live stream, video list
                  /capture   — Take a live JPEG snapshot
                  /stream    — MJPEG live stream
                  /last      — Last door-open capture (JPEG)
                  /status    — JSON with door state, battery, recording, uptime
                  /videos    — JSON list of recorded AVI files on SD card
                  /video     — Download a specific AVI file (?name=filename.avi)
**********************************************************************/
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "SD_MMC.h"
#include "board_config.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// Shared state from main sketch
extern volatile bool     doorOpen;
extern volatile uint8_t  batteryPercent;
extern volatile uint32_t lastEventTime;
extern bool              recording;
extern bool              sdCardOK;
extern camera_fb_t      *lastCapture;
extern SemaphoreHandle_t captureMux;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// ========================
// HTML page (inline)
// ========================
static const char index_html[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Fridge CAM</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: -apple-system, sans-serif; background: #1a1a2e; color: #eee; padding: 16px; }
    h1 { text-align: center; margin-bottom: 12px; font-size: 1.5em; }
    .status-bar { display: flex; justify-content: center; gap: 16px; margin-bottom: 16px; font-size: 1.1em; flex-wrap: wrap; }
    .status-bar .badge { padding: 6px 16px; border-radius: 20px; font-weight: bold; }
    .open { background: #e74c3c; }
    .closed { background: #27ae60; }
    .rec { background: #e74c3c; animation: blink 1s infinite; }
    @keyframes blink { 50% { opacity: 0.5; } }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; max-width: 900px; margin: 0 auto; }
    .card { background: #16213e; border-radius: 10px; padding: 12px; }
    .card h2 { font-size: 1em; margin-bottom: 8px; color: #a8b2d1; }
    img, .stream { width: 100%; border-radius: 6px; background: #0f0f23; min-height: 180px; }
    .btn { display: inline-block; margin-top: 8px; padding: 8px 16px; background: #0f3460; color: #fff;
           border: none; border-radius: 6px; cursor: pointer; font-size: 0.9em; }
    .btn:hover { background: #1a5276; }
    .full-width { grid-column: 1 / -1; }
    table { width: 100%; border-collapse: collapse; margin-top: 8px; }
    th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid #1a1a2e; }
    th { color: #a8b2d1; font-size: 0.85em; }
    a { color: #5dade2; text-decoration: none; }
    a:hover { text-decoration: underline; }
    @media (max-width: 600px) { .grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <h1>&#x1F4F7; Fridge CAM</h1>
  <div class="status-bar">
    <span id="door" class="badge closed">Door: Closed</span>
    <span id="recbadge" class="badge" style="background:#2c3e50;display:none">&#x1F534; Recording</span>
    <span id="battery" class="badge" style="background:#2c3e50">Battery: --%</span>
  </div>
  <div class="grid">
    <div class="card">
      <h2>Live Stream</h2>
      <img class="stream" id="stream" src="">
      <button class="btn" onclick="toggleStream()">Start Stream</button>
    </div>
    <div class="card">
      <h2>Last Door-Open Capture</h2>
      <img id="lastimg" src="/last" onerror="this.alt='No capture yet'">
      <button class="btn" onclick="refreshLast()">Refresh</button>
      <button class="btn" onclick="manualCapture()">Capture Now</button>
    </div>
    <div class="card full-width">
      <h2>Recorded Videos</h2>
      <button class="btn" onclick="loadVideos()">Refresh List</button>
      <table>
        <thead><tr><th>Filename</th><th>Size</th><th></th></tr></thead>
        <tbody id="vlist"><tr><td colspan="3">Click Refresh to load</td></tr></tbody>
      </table>
    </div>
  </div>
  <script>
    let streaming = false;
    const streamPort = parseInt(location.port || 8080) + 1;
    function toggleStream() {
      const img = document.getElementById('stream');
      if (streaming) { img.src = ''; streaming = false; }
      else { img.src = 'http://' + location.hostname + ':' + streamPort + '/stream'; streaming = true; }
    }
    function refreshLast() {
      document.getElementById('lastimg').src = '/last?t=' + Date.now();
    }
    function manualCapture() {
      document.getElementById('lastimg').src = '/capture?t=' + Date.now();
    }
    function pollStatus() {
      fetch('/status').then(r => r.json()).then(d => {
        const el = document.getElementById('door');
        if (d.door_open) { el.textContent = 'Door: OPEN'; el.className = 'badge open'; }
        else { el.textContent = 'Door: Closed'; el.className = 'badge closed'; }
        document.getElementById('battery').textContent = 'Battery: ' + d.battery + '%';
        const rb = document.getElementById('recbadge');
        if (d.recording) { rb.style.display = ''; rb.className = 'badge rec'; }
        else { rb.style.display = 'none'; }
      }).catch(() => {});
    }
    function loadVideos() {
      fetch('/videos').then(r => r.json()).then(files => {
        const tb = document.getElementById('vlist');
        if (!files.length) { tb.innerHTML = '<tr><td colspan="3">No recordings</td></tr>'; return; }
        tb.innerHTML = files.map(f =>
          '<tr><td>' + f.name + '</td><td>' + formatSize(f.size) + '</td>' +
          '<td><a href="/video?name=' + encodeURIComponent(f.name) + '">Download</a></td></tr>'
        ).join('');
      }).catch(() => {});
    }
    function formatSize(b) {
      if (b < 1024) return b + ' B';
      if (b < 1048576) return (b/1024).toFixed(1) + ' KB';
      return (b/1048576).toFixed(1) + ' MB';
    }
    setInterval(pollStatus, 2000);
    pollStatus();
    loadVideos();
  </script>
</body>
</html>
)rawliteral";

// ========================
// Handlers
// ========================

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, index_html, sizeof(index_html) - 1);
}

static esp_err_t status_handler(httpd_req_t *req) {
  char json[160];
  snprintf(json, sizeof(json),
    "{\"door_open\":%s,\"battery\":%u,\"recording\":%s,\"last_event_ms\":%lu,\"uptime_ms\":%lu}",
    doorOpen ? "true" : "false",
    batteryPercent,
    recording ? "true" : "false",
    (unsigned long)lastEventTime,
    (unsigned long)millis());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  esp_err_t res;
  if (fb->format == PIXFORMAT_JPEG) {
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
    if (ok) {
      res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
      free(jpg_buf);
    } else {
      res = ESP_FAIL;
      httpd_resp_send_500(req);
    }
  }
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t last_capture_handler(httpd_req_t *req) {
  xSemaphoreTake(captureMux, portMAX_DELAY);
  if (!lastCapture) {
    xSemaphoreGive(captureMux);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=door_open.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char *)lastCapture->buf, lastCapture->len);
  xSemaphoreGive(captureMux);
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[128];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART,
                             _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }
  }
  return res;
}

// ========================
// Video list endpoint — returns JSON array of .avi files on SD
// ========================
static esp_err_t videos_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  if (!sdCardOK) {
    return httpd_resp_send(req, "[]", 2);
  }

  File root = SD_MMC.open("/sdcard");
  if (!root || !root.isDirectory()) {
    root = SD_MMC.open("/");
    if (!root || !root.isDirectory()) {
      return httpd_resp_send(req, "[]", 2);
    }
  }

  httpd_resp_send_chunk(req, "[", 1);
  bool first = true;
  File entry = root.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      const char *name = entry.name();
      // Only list .avi files
      size_t nlen = strlen(name);
      if (nlen > 4 && strcasecmp(name + nlen - 4, ".avi") == 0) {
        char item[128];
        // Strip leading slash for display
        const char *displayName = name;
        if (displayName[0] == '/') displayName++;
        int len = snprintf(item, sizeof(item),
          "%s{\"name\":\"%s\",\"size\":%lu}",
          first ? "" : ",",
          displayName,
          (unsigned long)entry.size());
        httpd_resp_send_chunk(req, item, len);
        first = false;
      }
    }
    entry = root.openNextFile();
  }
  httpd_resp_send_chunk(req, "]", 1);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// ========================
// Video download endpoint — serves a specific AVI file
// ========================
static esp_err_t video_handler(httpd_req_t *req) {
  char query[128] = {0};
  char name[64] = {0};

  if (httpd_req_get_url_query_len(req) == 0) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  if (httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  // Sanitize: reject path traversal
  if (strstr(name, "..") || strstr(name, "//")) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  char filepath[96];
  snprintf(filepath, sizeof(filepath), "/%s", name);

  File file = SD_MMC.open(filepath, FILE_READ);
  if (!file) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "video/x-msvideo");
  char disposition[128];
  snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", name);
  httpd_resp_set_hdr(req, "Content-Disposition", disposition);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  // Stream file in chunks
  uint8_t *buf = (uint8_t *)malloc(4096);
  if (!buf) {
    file.close();
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  size_t bytesRead;
  while ((bytesRead = file.read(buf, 4096)) > 0) {
    if (httpd_resp_send_chunk(req, (const char *)buf, bytesRead) != ESP_OK) {
      break;
    }
  }

  free(buf);
  file.close();
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// ========================
// Server init
// ========================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 8080;
  config.max_uri_handlers = 10;

  httpd_uri_t index_uri   = { .uri = "/",        .method = HTTP_GET, .handler = index_handler,        .user_ctx = NULL };
  httpd_uri_t status_uri  = { .uri = "/status",   .method = HTTP_GET, .handler = status_handler,       .user_ctx = NULL };
  httpd_uri_t capture_uri = { .uri = "/capture",  .method = HTTP_GET, .handler = capture_handler,      .user_ctx = NULL };
  httpd_uri_t last_uri    = { .uri = "/last",     .method = HTTP_GET, .handler = last_capture_handler, .user_ctx = NULL };
  httpd_uri_t videos_uri  = { .uri = "/videos",   .method = HTTP_GET, .handler = videos_handler,       .user_ctx = NULL };
  httpd_uri_t video_uri   = { .uri = "/video",    .method = HTTP_GET, .handler = video_handler,        .user_ctx = NULL };
  httpd_uri_t stream_uri  = { .uri = "/stream",   .method = HTTP_GET, .handler = stream_handler,       .user_ctx = NULL };

  log_i("Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &last_uri);
    httpd_register_uri_handler(camera_httpd, &videos_uri);
    httpd_register_uri_handler(camera_httpd, &video_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  log_i("Starting stream server on port: '%d'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
