#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#include <algorithm>

constexpr int PWMA_PIN = 4;
constexpr int AIN1_PIN = 6;
constexpr int AIN2_PIN = 5;

constexpr int PWMB_PIN = 15;
constexpr int BIN1_PIN = 16;
constexpr int BIN2_PIN = 17;

constexpr int IR_LEFT_PIN = 10;
constexpr int IR_FRONT_PIN = 11;
constexpr int IR_RIGHT_PIN = 12;

constexpr int LEFT_PWM_CHANNEL = 0;
constexpr int RIGHT_PWM_CHANNEL = 1;
constexpr int PWM_FREQ_HZ = 20000;
constexpr int PWM_RESOLUTION_BITS = 8;

const char *WIFI_SSID = "Micromouse_Tuner";
const char *WIFI_PASSWORD = "";

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

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

int irLeftRaw = 1;
int irFrontRaw = 1;
int irRightRaw = 1;
bool lastLeftWall = false;
bool lastFrontWall = false;
bool lastRightWall = false;
int lastCommandedLeftSpeed = 0;
int lastCommandedRightSpeed = 0;

unsigned long lastDiagnosticsMs = 0;
unsigned long actionUntilMs = 0;

enum MotionPhase { PHASE_IDLE, PHASE_TURN_RIGHT, PHASE_TURN_LEFT, PHASE_UTURN, PHASE_FORWARD };
MotionPhase motionPhase = PHASE_IDLE;

const char HTML_PAGE[] PROGMEM = R"rawliteral(
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
      document.getElementById('pid_line').textContent = 'pid=' + Number(data.kp).toFixed(2) + '/' + Number(data.ki).toFixed(2) + '/' + Number(data.kd).toFixed(2) + ' step=' + data.forward_step_ms + ' turnms=' + data.turn_duration_ms;
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
      ws = new WebSocket('ws://' + window.location.hostname + ':81/');
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

const char *robotStateToString(RobotState state) {
  return state == EXPLORE ? "EXPLORE" : "STOPPED";
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
  irLeftRaw = digitalRead(IR_LEFT_PIN);
  irFrontRaw = digitalRead(IR_FRONT_PIN);
  irRightRaw = digitalRead(IR_RIGHT_PIN);

  lastLeftWall = (irLeftRaw == 0);
  lastFrontWall = (irFrontRaw == 0);
  lastRightWall = (irRightRaw == 0);
}

void logIrSnapshot(const char *reason) {
  sampleIrSensors();
  Serial.printf("IR[%s] L/F/R=%d/%d/%d\n", reason, irLeftRaw, irFrontRaw, irRightRaw);
}

String buildStatusJson() {
  char buffer[512];
  sampleIrSensors();
  snprintf(
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
      abs(lastCommandedLeftSpeed),
      abs(lastCommandedRightSpeed),
      BASE_SPEED,
      TURN_SPEED,
      LEFT_TRIM,
      RIGHT_TRIM,
      static_cast<double>(Kp),
      static_cast<double>(Ki),
      static_cast<double>(Kd),
      FORWARD_STEP_MS,
      TURN_DURATION_MS);
  return String(buffer);
}

void broadcastStatusToWebClients() {
  webSocket.broadcastTXT(buildStatusJson());
}

String renderHtml() {
  String html = String(HTML_PAGE);
  html.replace("{speed}", String(BASE_SPEED));
  html.replace("{turn}", String(TURN_SPEED));
  html.replace("{ltrim}", String(LEFT_TRIM));
  html.replace("{rtrim}", String(RIGHT_TRIM));
  html.replace("{kp}", String(Kp));
  html.replace("{ki}", String(Ki));
  html.replace("{kd}", String(Kd));
  html.replace("{fstep}", String(FORWARD_STEP_MS));
  html.replace("{tdur}", String(TURN_DURATION_MS));
  return html;
}

void moveMotors(int leftSpeed, int rightSpeed) {
  if ((leftSpeed > 0 && rightSpeed > 0) || (leftSpeed < 0 && rightSpeed < 0)) {
    leftSpeed += LEFT_TRIM;
    rightSpeed += RIGHT_TRIM;
  }

  lastCommandedLeftSpeed = leftSpeed;
  lastCommandedRightSpeed = rightSpeed;

  if (leftSpeed >= 0) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  } else {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    leftSpeed = -leftSpeed;
  }

  if (rightSpeed >= 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    rightSpeed = -rightSpeed;
  }

  leftSpeed = std::clamp(leftSpeed, 0, 255);
  rightSpeed = std::clamp(rightSpeed, 0, 255);

  ledcWrite(LEFT_PWM_CHANNEL, leftSpeed);
  ledcWrite(RIGHT_PWM_CHANNEL, rightSpeed);
}

void stopMotors() {
  moveMotors(0, 0);
}

void startRobot() {
  currentState = EXPLORE;
  motionPhase = PHASE_IDLE;
  actionUntilMs = 0;
  Serial.println("Robot started from serial command");
  broadcastStatusToWebClients();
}

void stopRobot() {
  currentState = STOPPED;
  motionPhase = PHASE_IDLE;
  actionUntilMs = 0;
  stopMotors();
  Serial.println("Robot stopped");
  broadcastStatusToWebClients();
}

void beginPhase(MotionPhase phase, unsigned long durationMs) {
  motionPhase = phase;
  actionUntilMs = millis() + durationMs;

  if (phase == PHASE_TURN_RIGHT) {
    moveMotors(TURN_SPEED, -TURN_SPEED);
  } else if (phase == PHASE_TURN_LEFT) {
    moveMotors(-TURN_SPEED, TURN_SPEED);
  } else if (phase == PHASE_UTURN) {
    moveMotors(TURN_SPEED, -TURN_SPEED);
  } else if (phase == PHASE_FORWARD) {
    moveMotors(BASE_SPEED, BASE_SPEED);
  } else {
    stopMotors();
  }
}

void navigateMaze() {
  sampleIrSensors();
  const bool leftWall = lastLeftWall;
  const bool frontWall = lastFrontWall;
  const bool rightWall = lastRightWall;

  if (!frontWall) {
    beginPhase(PHASE_FORWARD, FORWARD_STEP_MS);
  } else if (!rightWall) {
    beginPhase(PHASE_TURN_RIGHT, TURN_DURATION_MS);
  } else if (!leftWall) {
    beginPhase(PHASE_TURN_LEFT, TURN_DURATION_MS);
  } else {
    beginPhase(PHASE_UTURN, TURN_DURATION_MS * 2);
  }
}

void handleMotion() {
  if (currentState == STOPPED) {
    if (motionPhase != PHASE_IDLE) {
      motionPhase = PHASE_IDLE;
      stopMotors();
    }
    return;
  }

  if (motionPhase == PHASE_IDLE) {
    navigateMaze();
    return;
  }

  if (millis() < actionUntilMs) {
    return;
  }

  if (motionPhase == PHASE_TURN_RIGHT || motionPhase == PHASE_TURN_LEFT || motionPhase == PHASE_UTURN) {
    beginPhase(PHASE_FORWARD, FORWARD_STEP_MS);
  } else if (motionPhase == PHASE_FORWARD) {
    motionPhase = PHASE_IDLE;
    stopMotors();
  }
}

void parseAndApplyQuery() {
  if (server.hasArg("speed")) {
    BASE_SPEED = server.arg("speed").toInt();
  }
  if (server.hasArg("turn")) {
    TURN_SPEED = server.arg("turn").toInt();
  }
  if (server.hasArg("ltrim")) {
    LEFT_TRIM = server.arg("ltrim").toInt();
  }
  if (server.hasArg("rtrim")) {
    RIGHT_TRIM = server.arg("rtrim").toInt();
  }
  if (server.hasArg("kp")) {
    Kp = server.arg("kp").toFloat();
  }
  if (server.hasArg("ki")) {
    Ki = server.arg("ki").toFloat();
  }
  if (server.hasArg("kd")) {
    Kd = server.arg("kd").toFloat();
  }
  if (server.hasArg("fstep")) {
    FORWARD_STEP_MS = server.arg("fstep").toInt();
  }
  if (server.hasArg("tdur")) {
    TURN_DURATION_MS = server.arg("tdur").toInt();
  }
  broadcastStatusToWebClients();
}

void handleRoot() {
  server.send(200, "text/html", renderHtml());
}

void handleUpdate() {
  parseAndApplyQuery();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStop() {
  stopRobot();
  server.send(200, "text/plain", "Robot Stopped.");
}

void handleStart() {
  startRobot();
  server.send(200, "text/plain", "Robot Started.");
}

void handleStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buildStatusJson());
}

void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    webSocket.sendTXT(num, buildStatusJson());
    return;
  }

  if (type == WStype_TEXT && length > 0) {
    String message(reinterpret_cast<char *>(payload));
    message = message.substring(0, length);
    if (message == "start") {
      startRobot();
    } else if (message == "stop") {
      stopRobot();
    }
  }
}

void handleSerialCommands() {
  static char buffer[64];
  static size_t index = 0;

  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read());

    if (ch == '\r' || ch == '\n') {
      if (index == 0) {
        continue;
      }

      buffer[index] = '\0';
      if (strcmp(buffer, "start") == 0) {
        startRobot();
      } else if (strcmp(buffer, "stop") == 0) {
        stopRobot();
      } else {
        Serial.print("Unknown serial command: ");
        Serial.println(buffer);
      }
      index = 0;
      continue;
    }

    if (index < (sizeof(buffer) - 1)) {
      buffer[index++] = ch;
    } else {
      index = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);

  pinMode(IR_LEFT_PIN, INPUT_PULLUP);
  pinMode(IR_FRONT_PIN, INPUT_PULLUP);
  pinMode(IR_RIGHT_PIN, INPUT_PULLUP);

  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(PWMA_PIN, LEFT_PWM_CHANNEL);
  ledcAttachPin(PWMB_PIN, RIGHT_PWM_CHANNEL);
  stopMotors();

  sampleIrSensors();
  logIrSnapshot("startup");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("SoftAP started. SSID: ");
  Serial.print(WIFI_SSID);
  Serial.print(" Password: ");
  Serial.println(WIFI_PASSWORD);

  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/stop", handleStop);
  server.on("/start", handleStart);
  server.on("/status", handleStatus);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(handleWebSocketEvent);

  Serial.println("Serial commands ready: start, stop");
}

void loop() {
  server.handleClient();
  webSocket.loop();
  handleSerialCommands();
  handleMotion();

  if (millis() - lastDiagnosticsMs >= 1000) {
    lastDiagnosticsMs = millis();
    logIrSnapshot("periodic");
    Serial.printf("STATE=%s speed=%d turn=%d trim=%d/%d motor=%s/%s pwm=%d/%d\n",
                  robotStateToString(currentState),
                  BASE_SPEED,
                  TURN_SPEED,
                  LEFT_TRIM,
                  RIGHT_TRIM,
                  motorDirectionString(lastCommandedLeftSpeed),
                  motorDirectionString(lastCommandedRightSpeed),
                  abs(lastCommandedLeftSpeed),
                  abs(lastCommandedRightSpeed));
    broadcastStatusToWebClients();
  }
}
