#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {

constexpr char TAG[] = "micromouse";

constexpr gpio_num_t PWMA_PIN = GPIO_NUM_4;
constexpr gpio_num_t AIN1_PIN = GPIO_NUM_6;
constexpr gpio_num_t AIN2_PIN = GPIO_NUM_5;

constexpr gpio_num_t PWMB_PIN = GPIO_NUM_15;
constexpr gpio_num_t BIN1_PIN = GPIO_NUM_16;
constexpr gpio_num_t BIN2_PIN = GPIO_NUM_17;

constexpr gpio_num_t IR_LEFT_PIN = GPIO_NUM_10;
constexpr gpio_num_t IR_FRONT_PIN = GPIO_NUM_11;
constexpr gpio_num_t IR_RIGHT_PIN = GPIO_NUM_12;

constexpr ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
constexpr ledc_channel_t LEFT_PWM_CHANNEL = LEDC_CHANNEL_0;
constexpr ledc_channel_t RIGHT_PWM_CHANNEL = LEDC_CHANNEL_1;
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_8_BIT;

constexpr char WIFI_SSID[] = "Micromouse_Tuner";
constexpr char WIFI_PASSWORD[] = "";

volatile float Kp = 4.5f;
volatile float Ki = 0.05f;
volatile float Kd = 1.2f;
volatile int BASE_SPEED = 80;
volatile int TURN_SPEED = 60;
volatile int FORWARD_STEP_MS = 140;
volatile int TURN_DURATION_MS = 340;
volatile int LEFT_TRIM = 0;
volatile int RIGHT_TRIM = -10;

enum RobotState { EXPLORE, STOPPED };
volatile RobotState currentState = STOPPED;

httpd_handle_t httpServer = nullptr;
bool gpioAvailable = false;
bool motorPwmAvailable = false;
bool wifiAvailable = false;
bool httpServerAvailable = false;
int irLeftRaw = 1;
int irFrontRaw = 1;
int irRightRaw = 1;
bool lastLeftWall = false;
bool lastFrontWall = false;
bool lastRightWall = false;
int lastCommandedLeftSpeed = 0;
int lastCommandedRightSpeed = 0;
std::vector<int> websocketClients;
portMUX_TYPE websocketClientsMux = portMUX_INITIALIZER_UNLOCKED;

void stopMotors();
void broadcastStatusToWebClients();

void logInitResult(const char *name, esp_err_t result) {
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "%s initialized", name);
    } else {
        ESP_LOGE(TAG, "%s init failed: %s", name, esp_err_to_name(result));
    }
}

constexpr char HTML_PAGE[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Micromouse PID Tuner</title>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <style>
    body { font-family: Arial; text-align: center; background: #222; color: white; margin: 0; }
    .container { max-width: 760px; margin: auto; padding: 20px; }
    .card { background: #333; padding: 20px; border-radius: 10px; margin-bottom: 20px; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
    .field { text-align: left; }
    label { display: block; margin-bottom: 6px; font-size: 14px; color: #ddd; }
    input[type=number] { width: 100%; padding: 10px; border-radius: 6px; border: none; box-sizing: border-box; }
    button, input[type=submit] { padding: 10px 16px; background: #007bff; color: white; border: none; border-radius: 6px; cursor: pointer; margin: 6px; }
    button.stop { background: #b00020; }
    .telemetry { text-align: left; line-height: 1.7; font-family: Consolas, monospace; font-size: 14px; }
    .status { display: inline-block; min-width: 140px; padding: 8px 12px; border-radius: 999px; background: #444; }
    .hint { color: #bbb; font-size: 13px; }
  </style>
</head>
<body>
  <div class='container'>
    <h2>TechnoXian Control Panel</h2>
    <div class='card'>
      <div style='margin-bottom:12px;'>
        <button onclick='sendCommand("/start")'>START</button>
        <button class='stop' onclick='sendCommand("/stop")'>STOP</button>
      </div>
      <div class='hint'>SoftAP stays on continuously at <b>192.168.4.1</b>.</div>
    </div>
    <div class='card telemetry'>
      <div><span class='status' id='state'>STATE=...</span></div>
      <div id='ir_line'>IR[periodic] L/F/R=...</div>
      <div id='motor_line'>motor=.../... pwm=.../...</div>
      <div id='tune_line'>speed=... turn=... trim=.../...</div>
      <div id='pid_line'>pid=.../.../... step=... turnms=...</div>
    </div>
    <div class='card'>
      <form action='/update' method='GET'>
        <div class='grid'>
          <div class='field'><label>Base Speed</label><input type='number' name='speed' value='{speed}'></div>
          <div class='field'><label>Turn Speed</label><input type='number' name='turn' value='{turn}'></div>
          <div class='field'><label>Left Trim</label><input type='number' name='ltrim' value='{ltrim}'></div>
          <div class='field'><label>Right Trim</label><input type='number' name='rtrim' value='{rtrim}'></div>
          <div class='field'><label>Kp</label><input type='number' step='0.01' name='kp' value='{kp}'></div>
          <div class='field'><label>Ki</label><input type='number' step='0.01' name='ki' value='{ki}'></div>
          <div class='field'><label>Kd</label><input type='number' step='0.01' name='kd' value='{kd}'></div>
          <div class='field'><label>Forward Step (ms)</label><input type='number' name='fstep' value='{fstep}'></div>
          <div class='field'><label>Turn Duration (ms)</label><input type='number' name='tdur' value='{tdur}'></div>
        </div>
        <input type='submit' value='Save & Update'>
      </form>
    </div>
    <p>Serial commands: <b>start</b>, <b>stop</b></p>
  </div>
  <script>
    let ws = null;

    function applyStatus(data) {
      document.getElementById('state').textContent = 'STATE=' + data.state;
      document.getElementById('ir_line').textContent = 'IR[periodic] L/F/R=' + data.ir_left + '/' + data.ir_front + '/' + data.ir_right;
      document.getElementById('motor_line').textContent = 'motor=' + data.motor_left_dir + '/' + data.motor_right_dir + ' pwm=' + data.motor_left_pwm + '/' + data.motor_right_pwm;
      document.getElementById('tune_line').textContent = 'speed=' + data.base_speed + ' turn=' + data.turn_speed + ' trim=' + data.left_trim + '/' + data.right_trim;
      document.getElementById('pid_line').textContent = 'pid=' + data.kp.toFixed(2) + '/' + data.ki.toFixed(2) + '/' + data.kd.toFixed(2) + ' step=' + data.forward_step_ms + ' turnms=' + data.turn_duration_ms;
    }

    async function sendCommand(path) {
      await fetch(path);
    }

    async function refreshStatusFallback() {
      try {
        const res = await fetch('/status');
        applyStatus(await res.json());
      } catch (err) {
        console.log(err);
      }
    }

    function connectWebSocket() {
      ws = new WebSocket('ws://' + window.location.host + '/ws');
      ws.onmessage = (event) => {
        applyStatus(JSON.parse(event.data));
      };
      ws.onopen = () => {
        console.log('WebSocket connected');
      };
      ws.onclose = () => {
        console.log('WebSocket disconnected, retrying...');
        setTimeout(connectWebSocket, 1000);
      };
      ws.onerror = () => {
        ws.close();
      };
    }

    refreshStatusFallback();
    connectWebSocket();
  </script>
</body>
</html>
)rawliteral";

float normalizeAngle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

const char *robotStateToString(RobotState state) {
    switch (state) {
        case EXPLORE:
            return "EXPLORE";
        case STOPPED:
        default:
            return "STOPPED";
    }
}

const char *motorDirectionString(int speed) {
    if (speed > 0) {
        return "F";
    }
    if (speed < 0) {
        return "R";
    }
    return "S";
}

void sampleIrSensors() {
    if (!gpioAvailable) {
        return;
    }

    irLeftRaw = gpio_get_level(IR_LEFT_PIN);
    irFrontRaw = gpio_get_level(IR_FRONT_PIN);
    irRightRaw = gpio_get_level(IR_RIGHT_PIN);

    lastLeftWall = (irLeftRaw == 0);
    lastFrontWall = (irFrontRaw == 0);
    lastRightWall = (irRightRaw == 0);
}

void logIrSnapshot(const char *reason) {
    sampleIrSensors();
    ESP_LOGI(TAG, "IR[%s] L/F/R=%d/%d/%d", reason, irLeftRaw, irFrontRaw, irRightRaw);
}

void startRobot() {
    currentState = EXPLORE;
    ESP_LOGI(TAG, "Robot started from serial command");
    broadcastStatusToWebClients();
}

void stopRobot() {
    currentState = STOPPED;
    stopMotors();
    ESP_LOGI(TAG, "Robot stopped");
    broadcastStatusToWebClients();
}

std::string buildStatusJson() {
    char buffer[512];
    std::snprintf(
        buffer,
        sizeof(buffer),
        "{\"state\":\"%s\",\"ir_left\":%d,\"ir_front\":%d,\"ir_right\":%d,"
        "\"motor_left_dir\":\"%s\",\"motor_right_dir\":\"%s\","
        "\"motor_left_pwm\":%d,\"motor_right_pwm\":%d,"
        "\"base_speed\":%d,\"turn_speed\":%d,\"left_trim\":%d,\"right_trim\":%d,"
        "\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"forward_step_ms\":%d,\"turn_duration_ms\":%d}",
        robotStateToString(currentState),
        irLeftRaw,
        irFrontRaw,
        irRightRaw,
        motorDirectionString(lastCommandedLeftSpeed),
        motorDirectionString(lastCommandedRightSpeed),
        std::abs(lastCommandedLeftSpeed),
        std::abs(lastCommandedRightSpeed),
        BASE_SPEED,
        TURN_SPEED,
        LEFT_TRIM,
        RIGHT_TRIM,
        static_cast<double>(Kp),
        static_cast<double>(Ki),
        static_cast<double>(Kd),
        FORWARD_STEP_MS,
        TURN_DURATION_MS);
    return std::string(buffer);
}

void addWebsocketClient(int socketFd) {
    taskENTER_CRITICAL(&websocketClientsMux);
    if (std::find(websocketClients.begin(), websocketClients.end(), socketFd) == websocketClients.end()) {
        websocketClients.push_back(socketFd);
    }
    taskEXIT_CRITICAL(&websocketClientsMux);
}

void removeWebsocketClient(int socketFd) {
    taskENTER_CRITICAL(&websocketClientsMux);
    websocketClients.erase(std::remove(websocketClients.begin(), websocketClients.end(), socketFd), websocketClients.end());
    taskEXIT_CRITICAL(&websocketClientsMux);
}

void broadcastStatusToWebClients() {
    if (!httpServerAvailable || httpServer == nullptr) {
        return;
    }

    sampleIrSensors();
    const std::string payload = buildStatusJson();
    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = reinterpret_cast<uint8_t *>(const_cast<char *>(payload.c_str()));
    frame.len = payload.size();

    std::vector<int> clientsCopy;
    taskENTER_CRITICAL(&websocketClientsMux);
    clientsCopy = websocketClients;
    taskEXIT_CRITICAL(&websocketClientsMux);

    for (const int clientFd : clientsCopy) {
        if (httpd_ws_send_frame_async(httpServer, clientFd, &frame) != ESP_OK) {
            removeWebsocketClient(clientFd);
        }
    }
}

std::string renderHtml() {
    std::string html(HTML_PAGE);

    auto replace_all = [&html](const char *token, const std::string &value) {
        size_t pos = 0;
        while ((pos = html.find(token, pos)) != std::string::npos) {
            html.replace(pos, std::strlen(token), value);
            pos += value.size();
        }
    };

    replace_all("{speed}", std::to_string(static_cast<int>(BASE_SPEED)));
    replace_all("{turn}", std::to_string(static_cast<int>(TURN_SPEED)));
    replace_all("{ltrim}", std::to_string(static_cast<int>(LEFT_TRIM)));
    replace_all("{rtrim}", std::to_string(static_cast<int>(RIGHT_TRIM)));
    replace_all("{kp}", std::to_string(static_cast<double>(Kp)));
    replace_all("{ki}", std::to_string(static_cast<double>(Ki)));
    replace_all("{kd}", std::to_string(static_cast<double>(Kd)));
    replace_all("{fstep}", std::to_string(static_cast<int>(FORWARD_STEP_MS)));
    replace_all("{tdur}", std::to_string(static_cast<int>(TURN_DURATION_MS)));

    return html;
}


esp_err_t initMotorPwm() {
    ledc_timer_config_t timerConfig = {};
    timerConfig.speed_mode = LEDC_MODE;
    timerConfig.timer_num = LEDC_TIMER;
    timerConfig.duty_resolution = PWM_RESOLUTION;
    timerConfig.freq_hz = PWM_FREQ_HZ;
    timerConfig.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timerConfig), TAG, "Failed to configure LEDC timer");

    ledc_channel_config_t leftConfig = {};
    leftConfig.gpio_num = PWMA_PIN;
    leftConfig.speed_mode = LEDC_MODE;
    leftConfig.channel = LEFT_PWM_CHANNEL;
    leftConfig.intr_type = LEDC_INTR_DISABLE;
    leftConfig.timer_sel = LEDC_TIMER;
    leftConfig.duty = 0;
    leftConfig.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&leftConfig), TAG, "Failed to configure left PWM");

    ledc_channel_config_t rightConfig = {};
    rightConfig.gpio_num = PWMB_PIN;
    rightConfig.speed_mode = LEDC_MODE;
    rightConfig.channel = RIGHT_PWM_CHANNEL;
    rightConfig.intr_type = LEDC_INTR_DISABLE;
    rightConfig.timer_sel = LEDC_TIMER;
    rightConfig.duty = 0;
    rightConfig.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&rightConfig), TAG, "Failed to configure right PWM");

    return ESP_OK;
}

esp_err_t initSerialCommands() {
    const int rxBufferSize = 256;
    const esp_err_t result = uart_driver_install(UART_NUM_0, rxBufferSize, 0, 0, nullptr, 0);
    if (result == ESP_OK || result == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return result;
}

esp_err_t initGpio() {
    gpio_config_t outputConfig = {};
    outputConfig.pin_bit_mask = (1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) |
                                (1ULL << BIN1_PIN) | (1ULL << BIN2_PIN);
    outputConfig.mode = GPIO_MODE_OUTPUT;
    outputConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    outputConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputConfig.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&outputConfig), TAG, "Failed to configure motor direction pins");

    gpio_config_t inputConfig = {};
    inputConfig.pin_bit_mask = (1ULL << IR_LEFT_PIN) | (1ULL << IR_FRONT_PIN) | (1ULL << IR_RIGHT_PIN);
    inputConfig.mode = GPIO_MODE_INPUT;
    inputConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    inputConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    inputConfig.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&inputConfig), TAG, "Failed to configure IR pins");

    return ESP_OK;
}

void moveMotors(int leftSpeed, int rightSpeed) {
    // Apply trim only when both motors are driving in the same direction.
    if ((leftSpeed > 0 && rightSpeed > 0) || (leftSpeed < 0 && rightSpeed < 0)) {
        leftSpeed += LEFT_TRIM;
        rightSpeed += RIGHT_TRIM;
    }

    lastCommandedLeftSpeed = leftSpeed;
    lastCommandedRightSpeed = rightSpeed;

    if (leftSpeed >= 0) {
        gpio_set_level(AIN1_PIN, 0);
        gpio_set_level(AIN2_PIN, 1);
    } else {
        gpio_set_level(AIN1_PIN, 1);
        gpio_set_level(AIN2_PIN, 0);
        leftSpeed = -leftSpeed;
    }

    if (rightSpeed >= 0) {
        gpio_set_level(BIN1_PIN, 1);
        gpio_set_level(BIN2_PIN, 0);
    } else {
        gpio_set_level(BIN1_PIN, 0);
        gpio_set_level(BIN2_PIN, 1);
        rightSpeed = -rightSpeed;
    }

    leftSpeed = std::clamp(leftSpeed, 0, 255);
    rightSpeed = std::clamp(rightSpeed, 0, 255);

    ledc_set_duty(LEDC_MODE, LEFT_PWM_CHANNEL, leftSpeed);
    ledc_update_duty(LEDC_MODE, LEFT_PWM_CHANNEL);
    ledc_set_duty(LEDC_MODE, RIGHT_PWM_CHANNEL, rightSpeed);
    ledc_update_duty(LEDC_MODE, RIGHT_PWM_CHANNEL);
}

void stopMotors() {
    moveMotors(0, 0);
}

void driveForwardStep() {
    moveMotors(BASE_SPEED, BASE_SPEED);
    vTaskDelay(pdMS_TO_TICKS(FORWARD_STEP_MS));
    stopMotors();
}

void turnLeftTimed() {
    moveMotors(-TURN_SPEED, TURN_SPEED);
    vTaskDelay(pdMS_TO_TICKS(TURN_DURATION_MS));
    stopMotors();
}

void turnRightTimed() {
    moveMotors(TURN_SPEED, -TURN_SPEED);
    vTaskDelay(pdMS_TO_TICKS(TURN_DURATION_MS));
    stopMotors();
}

void uTurnTimed() {
    moveMotors(TURN_SPEED, -TURN_SPEED);
    vTaskDelay(pdMS_TO_TICKS(TURN_DURATION_MS * 2));
    stopMotors();
}

void navigateMaze() {
    sampleIrSensors();
    const bool leftWall = lastLeftWall;
    const bool frontWall = lastFrontWall;
    const bool rightWall = lastRightWall;

    if (!frontWall) {
        driveForwardStep();
    } else if (!rightWall) {
        turnRightTimed();
        driveForwardStep();
    } else if (!leftWall) {
        turnLeftTimed();
        driveForwardStep();
    } else {
        uTurnTimed();
        driveForwardStep();
    }
}

esp_err_t rootHandler(httpd_req_t *req) {
    const std::string html = renderHtml();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html.c_str(), html.size());
}

void parseAndApplyQuery(const char *query) {
    char value[32] = {};
    if (httpd_query_key_value(query, "speed", value, sizeof(value)) == ESP_OK) {
        BASE_SPEED = std::atoi(value);
    }
    if (httpd_query_key_value(query, "turn", value, sizeof(value)) == ESP_OK) {
        TURN_SPEED = std::atoi(value);
    }
    if (httpd_query_key_value(query, "ltrim", value, sizeof(value)) == ESP_OK) {
        LEFT_TRIM = std::atoi(value);
    }
    if (httpd_query_key_value(query, "rtrim", value, sizeof(value)) == ESP_OK) {
        RIGHT_TRIM = std::atoi(value);
    }
    if (httpd_query_key_value(query, "kp", value, sizeof(value)) == ESP_OK) {
        Kp = std::strtof(value, nullptr);
    }
    if (httpd_query_key_value(query, "ki", value, sizeof(value)) == ESP_OK) {
        Ki = std::strtof(value, nullptr);
    }
    if (httpd_query_key_value(query, "kd", value, sizeof(value)) == ESP_OK) {
        Kd = std::strtof(value, nullptr);
    }
    if (httpd_query_key_value(query, "fstep", value, sizeof(value)) == ESP_OK) {
        FORWARD_STEP_MS = std::atoi(value);
    }
    if (httpd_query_key_value(query, "tdur", value, sizeof(value)) == ESP_OK) {
        TURN_DURATION_MS = std::atoi(value);
    }
    broadcastStatusToWebClients();
}

esp_err_t updateHandler(httpd_req_t *req) {
    char query[256] = {};
    if (httpd_req_get_url_query_len(req) > 0) {
        const esp_err_t result = httpd_req_get_url_query_str(req, query, sizeof(query));
        if (result == ESP_OK) {
            parseAndApplyQuery(query);
        }
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t stopHandler(httpd_req_t *req) {
    stopRobot();
    return httpd_resp_sendstr(req, "Robot Stopped.");
}

esp_err_t startHandler(httpd_req_t *req) {
    startRobot();
    return httpd_resp_sendstr(req, "Robot Started.");
}

esp_err_t statusHandler(httpd_req_t *req) {
    sampleIrSensors();
    const std::string statusJson = buildStatusJson();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, statusJson.c_str(), statusJson.size());
}

esp_err_t websocketHandler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        const int socketFd = httpd_req_to_sockfd(req);
        addWebsocketClient(socketFd);

        const std::string payload = buildStatusJson();
        httpd_ws_frame_t frame = {};
        frame.type = HTTPD_WS_TYPE_TEXT;
        frame.payload = reinterpret_cast<uint8_t *>(const_cast<char *>(payload.c_str()));
        frame.len = payload.size();
        return httpd_ws_send_frame(req, &frame);
    }

    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    ESP_RETURN_ON_ERROR(httpd_ws_recv_frame(req, &frame, 0), TAG, "Failed to get WebSocket frame length");

    std::vector<uint8_t> payload(frame.len + 1, 0);
    frame.payload = payload.data();
    ESP_RETURN_ON_ERROR(httpd_ws_recv_frame(req, &frame, frame.len), TAG, "Failed to read WebSocket frame");

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        removeWebsocketClient(httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    if (frame.type == HTTPD_WS_TYPE_TEXT && frame.len > 0) {
        const char *message = reinterpret_cast<const char *>(frame.payload);
        if (std::strcmp(message, "start") == 0) {
            startRobot();
        } else if (std::strcmp(message, "stop") == 0) {
            stopRobot();
        }
    }

    return ESP_OK;
}

esp_err_t startHttpServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    ESP_RETURN_ON_ERROR(httpd_start(&httpServer, &config), TAG, "Failed to start HTTP server");

    httpd_uri_t rootUri = {};
    rootUri.uri = "/";
    rootUri.method = HTTP_GET;
    rootUri.handler = rootHandler;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(httpServer, &rootUri), TAG, "Failed to register root route");

    httpd_uri_t updateUri = {};
    updateUri.uri = "/update";
    updateUri.method = HTTP_GET;
    updateUri.handler = updateHandler;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(httpServer, &updateUri), TAG, "Failed to register update route");

    httpd_uri_t stopUri = {};
    stopUri.uri = "/stop";
    stopUri.method = HTTP_GET;
    stopUri.handler = stopHandler;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(httpServer, &stopUri), TAG, "Failed to register stop route");

    httpd_uri_t startUri = {};
    startUri.uri = "/start";
    startUri.method = HTTP_GET;
    startUri.handler = startHandler;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(httpServer, &startUri), TAG, "Failed to register start route");

    httpd_uri_t statusUri = {};
    statusUri.uri = "/status";
    statusUri.method = HTTP_GET;
    statusUri.handler = statusHandler;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(httpServer, &statusUri), TAG, "Failed to register status route");

    httpd_uri_t websocketUri = {};
    websocketUri.uri = "/ws";
    websocketUri.method = HTTP_GET;
    websocketUri.handler = websocketHandler;
    websocketUri.is_websocket = true;
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(httpServer, &websocketUri), TAG, "Failed to register websocket route");

    return ESP_OK;
}

esp_err_t startWifiAccessPoint() {
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Failed to init netif");
    const esp_err_t eventLoopResult = esp_event_loop_create_default();
    if (eventLoopResult != ESP_OK && eventLoopResult != ESP_ERR_INVALID_STATE) {
        return eventLoopResult;
    }
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifiInitConfig = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifiInitConfig), TAG, "Failed to init Wi-Fi");

    wifi_config_t wifiConfig = {};
    std::snprintf(reinterpret_cast<char *>(wifiConfig.ap.ssid), sizeof(wifiConfig.ap.ssid), "%s", WIFI_SSID);
    std::snprintf(reinterpret_cast<char *>(wifiConfig.ap.password), sizeof(wifiConfig.ap.password), "%s", WIFI_PASSWORD);
    wifiConfig.ap.ssid_len = std::strlen(WIFI_SSID);
    wifiConfig.ap.max_connection = 4;
    wifiConfig.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifiConfig.ap.channel = 1;
    wifiConfig.ap.beacon_interval = 100;

    if (std::strlen(WIFI_PASSWORD) == 0) {
        wifiConfig.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "Failed to set Wi-Fi AP mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifiConfig), TAG, "Failed to set AP config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Failed to start Wi-Fi");

    ESP_LOGI(TAG, "SoftAP started. SSID: %s Password: %s", WIFI_SSID, WIFI_PASSWORD);
    return ESP_OK;
}

void controlTask(void *) {
    currentState = STOPPED;

    while (true) {
        if (currentState != STOPPED) {
            navigateMaze();
        } else {
            stopMotors();
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void diagnosticsTask(void *) {
    while (true) {
        if (gpioAvailable) {
            logIrSnapshot("periodic");
        } else {
            ESP_LOGW(TAG, "IR[periodic] GPIO unavailable");
        }

        ESP_LOGI(TAG,
                 "STATE=%s speed=%d turn=%d trim=%d/%d motor=%s/%s pwm=%d/%d",
                 robotStateToString(currentState),
                 BASE_SPEED,
                 TURN_SPEED,
                 LEFT_TRIM,
                 RIGHT_TRIM,
                 motorDirectionString(lastCommandedLeftSpeed),
                 motorDirectionString(lastCommandedRightSpeed),
                 std::abs(lastCommandedLeftSpeed),
                 std::abs(lastCommandedRightSpeed));

        broadcastStatusToWebClients();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void serialCommandTask(void *) {
    constexpr size_t kBufferSize = 64;
    char buffer[kBufferSize] = {};
    size_t index = 0;

    ESP_LOGI(TAG, "Serial commands ready: start, stop");

    while (true) {
        uint8_t ch = 0;
        const int read = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100));
        if (read <= 0) {
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (index == 0) {
                continue;
            }

            buffer[index] = '\0';
            if (std::strcmp(buffer, "start") == 0) {
                startRobot();
            } else if (std::strcmp(buffer, "stop") == 0) {
                stopRobot();
            } else {
                ESP_LOGI(TAG, "Unknown serial command: %s", buffer);
            }
            index = 0;
            continue;
        }

        if (index < (kBufferSize - 1)) {
            buffer[index++] = static_cast<char>(ch);
        } else {
            index = 0;
        }
    }
}

}  // namespace

extern "C" void app_main(void) {
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS required erase, retrying initialization");
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    logInitResult("NVS", result);

    result = initGpio();
    gpioAvailable = (result == ESP_OK);
    logInitResult("GPIO", result);
    if (gpioAvailable) {
        logIrSnapshot("startup");
    }

    if (gpioAvailable) {
        result = initMotorPwm();
        motorPwmAvailable = (result == ESP_OK);
        logInitResult("Motor PWM", result);
    } else {
        ESP_LOGW(TAG, "Skipping motor PWM because GPIO init failed");
    }

    result = initSerialCommands();
    logInitResult("Serial Commands", result);

    result = startWifiAccessPoint();
    wifiAvailable = (result == ESP_OK);
    logInitResult("Wi-Fi SoftAP", result);

    if (wifiAvailable) {
        result = startHttpServer();
        httpServerAvailable = (result == ESP_OK);
        logInitResult("HTTP server", result);
    } else {
        ESP_LOGW(TAG, "Skipping HTTP server because Wi-Fi SoftAP did not start");
    }

    if (gpioAvailable && motorPwmAvailable) {
        const BaseType_t taskResult = xTaskCreate(controlTask, "control_task", 4096, nullptr, 5, nullptr);
        if (taskResult == pdPASS) {
            ESP_LOGI(TAG, "Control task started");
        } else {
            ESP_LOGE(TAG, "Control task creation failed");
        }
    } else {
        ESP_LOGW(TAG, "Skipping control task because motor control prerequisites are unavailable");
    }

    const BaseType_t diagnosticsTaskResult = xTaskCreate(diagnosticsTask, "diagnostics_task", 4096, nullptr, 4, nullptr);
    if (diagnosticsTaskResult == pdPASS) {
        ESP_LOGI(TAG, "Diagnostics task started");
    } else {
        ESP_LOGE(TAG, "Diagnostics task creation failed");
    }

    const BaseType_t serialTaskResult = xTaskCreate(serialCommandTask, "serial_command_task", 4096, nullptr, 4, nullptr);
    if (serialTaskResult == pdPASS) {
        ESP_LOGI(TAG, "Serial command task started");
    } else {
        ESP_LOGE(TAG, "Serial command task creation failed");
    }
}
