#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#else
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#include <esp_ota_ops.h>
#include <esp_task_wdt.h>

#define TXD1 17
#define RXD1 18
#define GPIO_PIN_CH1 1
#define GPIO_PIN_CH2 2
#define GPIO_PIN_CH3 41
#define GPIO_PIN_CH4 42
#define GPIO_PIN_CH5 45
#define GPIO_PIN_CH6 46
#define GPIO_PIN_RGB 38
#define GPIO_PIN_BUZZER 21
#define GPIO_PIN_BOOT 0

static constexpr uint16_t MODBUS_PORT = 502;
static constexpr uint8_t MODBUS_UNIT_ID = 1;
static constexpr const char *FIRMWARE_VERSION = "1.0.6-static-ip";
static constexpr bool RELAY_ACTIVE_HIGH = true;
static constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
static constexpr uint32_t MODBUS_CLIENT_TIMEOUT_MS = 15000;
static constexpr uint32_t STATUS_LED_INTERVAL_MS = 10000;
static constexpr uint32_t OTA_HTTP_TIMEOUT_MS = 20000;
static constexpr uint16_t WIFI_MANAGER_CONNECT_TIMEOUT_SEC = 30;
static constexpr uint16_t WIFI_MANAGER_PORTAL_TIMEOUT_SEC = 300;
static constexpr uint16_t OTA_TRIGGER_COIL = 100;
static constexpr uint16_t BOOT_COUNTER_RESET_COIL = 101;
static constexpr uint16_t OTA_STATUS_INPUT_REGISTER = 100;
static constexpr uint16_t DIAG_REGISTER_FIRST = 100;
static constexpr uint16_t DIAG_REGISTER_LAST = 111;
static constexpr uint32_t BOOT_PORTAL_HOLD_MS = 5000;
static constexpr uint32_t BOOT_RESET_WIFI_HOLD_MS = 10000;
static constexpr uint32_t OTA_VALIDATE_AFTER_MS = 60000;

// Static IP configuration
static constexpr const char *STATIC_IP = "192.168.0.150";
static constexpr const char *GATEWAY_IP = "192.168.0.1";
static constexpr const char *SUBNET_MASK = "255.255.255.0";
static constexpr const char *DNS_IP = "192.168.0.1";

// WiFi credentials (configure these)
static constexpr const char *WIFI_SSID = "YOUR_SSID";
static constexpr const char *WIFI_PASSWORD = "YOUR_PASSWORD";

// GitHub Actions updates this fixed release asset. OTA is triggered manually by Modbus coil 100.
static constexpr const char *OTA_FIRMWARE_URL =
    "https://github.com/koziolacab-afk/811962/blob/main/firmware_manifest.json";

// Simple mode for home devices. For stricter security, pin GitHub's root CA instead of setInsecure().
static constexpr bool OTA_ALLOW_INSECURE_TLS = true;

static const uint8_t RELAY_PINS[] = {
    GPIO_PIN_CH1,
    GPIO_PIN_CH2,
    GPIO_PIN_CH3,
    GPIO_PIN_CH4,
    GPIO_PIN_CH5,
    GPIO_PIN_CH6,
};

static constexpr uint8_t RELAY_COUNT = sizeof(RELAY_PINS) / sizeof(RELAY_PINS[0]);
static bool relayStates[RELAY_COUNT] = {false, false, false, false, false, false};

WiFiServer modbusServer(MODBUS_PORT);
WiFiClient modbusClient;
Preferences preferences;
Adafruit_NeoPixel rgb(1, GPIO_PIN_RGB, NEO_GRB + NEO_KHZ800);

uint32_t lastWifiReconnectAttempt = 0;
uint32_t lastModbusActivity = 0;
uint32_t lastStatusLedUpdate = 0;
bool statusBlink = false;
bool watchdogStarted = false;
uint32_t bootButtonPressedAt = 0;
bool bootButtonWasPressed = false;
bool otaValidationDone = false;

enum OtaStatus : uint16_t {
    OTA_STATUS_IDLE = 0,
    OTA_STATUS_CHECKING = 1,
    OTA_STATUS_DOWNLOADING = 2,
    OTA_STATUS_UPDATED_REBOOTING = 3,
    OTA_STATUS_NO_UPDATE = 4,
    OTA_STATUS_ERROR = 5,
};

OtaStatus otaStatus = OTA_STATUS_IDLE;
bool otaRequested = false;
uint32_t bootCount = 0;
esp_reset_reason_t lastResetReason = ESP_RST_UNKNOWN;

static void setRgb(uint8_t r, uint8_t g, uint8_t b) {
    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
}

static void beep(uint16_t durationMs) {
    digitalWrite(GPIO_PIN_BUZZER, HIGH);
    delay(durationMs);
    digitalWrite(GPIO_PIN_BUZZER, LOW);
}

static void applyRelay(uint8_t index, bool state) {
    if (index >= RELAY_COUNT) {
        return;
    }

    relayStates[index] = state;
    const uint8_t level = RELAY_ACTIVE_HIGH ? HIGH : LOW;
    digitalWrite(RELAY_PINS[index], state ? level : !level);
}

static void allRelaysOff() {
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        applyRelay(i, false);
    }
}

static void setupRelays() {
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        pinMode(RELAY_PINS[i], OUTPUT);
    }

    allRelaysOff();
}

static void setupWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t config = {
        .timeout_ms = 8000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&config);
#else
    esp_task_wdt_init(8, true);
#endif
    esp_task_wdt_add(nullptr);
}

static void startWatchdog() {
    if (watchdogStarted) {
        return;
    }

    setupWatchdog();
    watchdogStarted = true;
}

static void stopWatchdog() {
    if (!watchdogStarted) {
        return;
    }

    esp_task_wdt_delete(nullptr);
    watchdogStarted = false;
}

static void setupDiagnostics() {
    lastResetReason = esp_reset_reason();

    preferences.begin("diag", false);
    bootCount = preferences.getUInt("boot_count", 0) + 1;
    preferences.putUInt("boot_count", bootCount);
    preferences.end();
}

static void resetBootCounter() {
    preferences.begin("diag", false);
    preferences.putUInt("boot_count", 0);
    preferences.end();

    bootCount = 0;
    Serial.println("Diagnostics: boot counter reset");
}

static void handleWifi() {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    const uint32_t now = millis();
    if (now - lastWifiReconnectAttempt < WIFI_RECONNECT_INTERVAL_MS) {
        return;
    }

    lastWifiReconnectAttempt = now;
    WiFi.reconnect();
}

static void handleBootButton() {
    const bool pressed = digitalRead(GPIO_PIN_BOOT) == LOW;
    const uint32_t now = millis();

    if (pressed && !bootButtonWasPressed) {
        bootButtonWasPressed = true;
        bootButtonPressedAt = now;
    }

    if (!pressed && bootButtonWasPressed) {
        const uint32_t heldMs = now - bootButtonPressedAt;
        bootButtonWasPressed = false;
        bootButtonPressedAt = 0;

        if (heldMs >= BOOT_RESET_WIFI_HOLD_MS) {
            Serial.println("BOOT: reset triggered (static IP version - restarting)");
            delay(500);
            ESP.restart();
        }
    }

    if (pressed && bootButtonPressedAt != 0) {
        const uint32_t heldMs = now - bootButtonPressedAt;
        if (heldMs >= BOOT_RESET_WIFI_HOLD_MS) {
            setRgb(80, 0, 0);
        }
    }
}

static void updateStatusLed() {
    const uint32_t now = millis();
    if (now - lastStatusLedUpdate < STATUS_LED_INTERVAL_MS) {
        return;
    }

    lastStatusLedUpdate = now;
    statusBlink = !statusBlink;

    if (WiFi.status() != WL_CONNECTED) {
        setRgb(statusBlink ? 80 : 0, 0, 0);
        return;
    }

    if (modbusClient && modbusClient.connected()) {
        setRgb(0, statusBlink ? 60 : 10, 0);
        return;
    }

    setRgb(0, 0, statusBlink ? 60 : 10);
}

static bool isPlaceholderUrl(const char *url) {
    return strstr(url, "OWNER/REPO/BRANCH") != nullptr;
}

static bool parseVersion(const String &version, int parts[3]) {
    int start = 0;

    for (int i = 0; i < 3; i++) {
        const int dot = version.indexOf('.', start);
        const int end = (dot == -1) ? version.length() : dot;

        if (end <= start) {
            return false;
        }

        parts[i] = version.substring(start, end).toInt();
        start = end + 1;

        if (i < 2 && dot == -1) {
            return false;
        }
    }

    return true;
}

static uint16_t clampPercent(int value) {
    if (value < 0) {
        return 0;
    }
    if (value > 100) {
        return 100;
    }
    return static_cast<uint16_t>(value);
}

static uint16_t getWifiSignalPercent() {
    if (WiFi.status() != WL_CONNECTED) {
        return 0;
    }

    const int rssi = WiFi.RSSI();
    return clampPercent(2 * (rssi + 100));
}

static uint16_t getHeapUsedPercent() {
    const uint32_t heapSize = ESP.getHeapSize();
    if (heapSize == 0) {
        return 0;
    }

    const uint32_t freeHeap = ESP.getFreeHeap();
    const uint32_t usedHeap = heapSize - freeHeap;
    return static_cast<uint16_t>((usedHeap * 100UL) / heapSize);
}

static uint16_t getResetReasonCode() {
    return static_cast<uint16_t>(lastResetReason);
}

static uint16_t getInputRegisterValue(uint16_t address) {
    int versionParts[3] = {0, 0, 0};
    parseVersion(FIRMWARE_VERSION, versionParts);

    switch (address) {
        case OTA_STATUS_INPUT_REGISTER:
            return static_cast<uint16_t>(otaStatus);
        case OTA_STATUS_INPUT_REGISTER + 1:
            return static_cast<uint16_t>(versionParts[0]);
        case OTA_STATUS_INPUT_REGISTER + 2:
            return static_cast<uint16_t>(versionParts[1]);
        case OTA_STATUS_INPUT_REGISTER + 3:
            return static_cast<uint16_t>(versionParts[2]);
        case OTA_STATUS_INPUT_REGISTER + 4:
            return getWifiSignalPercent();
        case OTA_STATUS_INPUT_REGISTER + 5:
            return getHeapUsedPercent();
        case OTA_STATUS_INPUT_REGISTER + 6:
            return static_cast<uint16_t>(ESP.getFreeHeap() / 1024UL);
        case OTA_STATUS_INPUT_REGISTER + 7:
            return static_cast<uint16_t>(ESP.getHeapSize() / 1024UL);
        case OTA_STATUS_INPUT_REGISTER + 8:
            return static_cast<uint16_t>(bootCount & 0xFFFF);
        case OTA_STATUS_INPUT_REGISTER + 9:
            return static_cast<uint16_t>((bootCount >> 16) & 0xFFFF);
        case OTA_STATUS_INPUT_REGISTER + 10:
            return getResetReasonCode();
        case OTA_STATUS_INPUT_REGISTER + 11:
            return WiFi.status() == WL_CONNECTED ? 1 : 0;
        default:
            return 0;
    }
}

static bool isValidInputRegister(uint16_t address) {
    return address >= DIAG_REGISTER_FIRST && address <= DIAG_REGISTER_LAST;
}

static void configureHttpsClient(WiFiClientSecure &client) {
    if (OTA_ALLOW_INSECURE_TLS) {
        client.setInsecure();
    }
}

static bool performFirmwareUpdate(const char *firmwareUrl, const char *md5) {
    WiFiClientSecure secureClient;
    configureHttpsClient(secureClient);

    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(OTA_HTTP_TIMEOUT_MS);

    if (!http.begin(secureClient, firmwareUrl)) {
        Serial.println("OTA: failed to begin firmware request");
        return false;
    }

    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("OTA: firmware HTTP status %d\n", code);
        http.end();
        return false;
    }

    const int contentLength = http.getSize();
    if (contentLength <= 0) {
        Serial.println("OTA: invalid firmware size");
        http.end();
        return false;
    }

    if (!Update.begin(contentLength)) {
        Serial.printf("OTA: Update.begin failed: %s\n", Update.errorString());
        http.end();
        return false;
    }

    if (md5 != nullptr && strlen(md5) == 32) {
        Update.setMD5(md5);
    }

    setRgb(60, 0, 60);
    WiFiClient *stream = http.getStreamPtr();
    uint8_t buffer[1024];
    size_t written = 0;
    uint32_t lastDataAt = millis();

    while (written < static_cast<size_t>(contentLength)) {
        esp_task_wdt_reset();

        const int available = stream->available();
        if (available > 0) {
            const size_t toRead = min(sizeof(buffer), static_cast<size_t>(available));
            const int readBytes = stream->readBytes(buffer, toRead);

            if (readBytes > 0) {
                const size_t updateWritten = Update.write(buffer, readBytes);
                written += updateWritten;
                lastDataAt = millis();

                if (updateWritten != static_cast<size_t>(readBytes)) {
                    Serial.println("OTA: flash write failed");
                    Update.abort();
                    http.end();
                    return false;
                }
            }
        } else {
            if (millis() - lastDataAt > OTA_HTTP_TIMEOUT_MS) {
                Serial.println("OTA: firmware download timeout");
                Update.abort();
                http.end();
                return false;
            }
            delay(1);
        }
    }

    if (written != static_cast<size_t>(contentLength)) {
        Serial.printf("OTA: written %u of %d bytes\n", static_cast<unsigned>(written), contentLength);
        Update.abort();
        http.end();
        return false;
    }

    if (!Update.end()) {
        Serial.printf("OTA: Update.end failed: %s\n", Update.errorString());
        http.end();
        return false;
    }

    if (!Update.isFinished()) {
        Serial.println("OTA: update not finished");
        http.end();
        return false;
    }

    http.end();
    return true;
}

static void checkForOtaUpdate() {
    if (WiFi.status() != WL_CONNECTED) {
        otaStatus = OTA_STATUS_ERROR;
        return;
    }

    otaStatus = OTA_STATUS_CHECKING;
    Serial.print("OTA: downloading fixed GitHub release asset: ");
    Serial.println(OTA_FIRMWARE_URL);
    otaStatus = OTA_STATUS_DOWNLOADING;
    allRelaysOff();

    if (performFirmwareUpdate(OTA_FIRMWARE_URL, "")) {
        Serial.println("OTA: update successful, restarting");
        otaStatus = OTA_STATUS_UPDATED_REBOOTING;
        beep(120);
        delay(300);
        ESP.restart();
    }

    Serial.println("OTA: update failed");
    otaStatus = OTA_STATUS_ERROR;
}

static void handleOtaRequest() {
    if (!otaRequested) {
        return;
    }

    otaRequested = false;
    checkForOtaUpdate();
}

static void validateRunningFirmwareAfterStableBoot() {
    if (otaValidationDone || millis() < OTA_VALIDATE_AFTER_MS) {
        return;
    }

    otaValidationDone = true;

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == nullptr) {
        Serial.println("OTA rollback: running partition unavailable");
        return;
    }

    esp_ota_img_states_t otaState;
    const esp_err_t stateResult = esp_ota_get_state_partition(running, &otaState);
    if (stateResult != ESP_OK) {
        Serial.printf("OTA rollback: state unavailable: %d\n", stateResult);
        return;
    }

    if (otaState == ESP_OTA_IMG_PENDING_VERIFY) {
        const esp_err_t validResult = esp_ota_mark_app_valid_cancel_rollback();
        if (validResult == ESP_OK) {
            Serial.println("OTA rollback: firmware marked valid");
        } else {
            Serial.printf("OTA rollback: mark valid failed: %d\n", validResult);
        }
    } else {
        Serial.printf("OTA rollback: current state %d, no validation needed\n", static_cast<int>(otaState));
    }
}

static void writeU16(uint8_t *buffer, size_t offset, uint16_t value) {
    buffer[offset] = value >> 8;
    buffer[offset + 1] = value & 0xFF;
}

static uint16_t readU16(const uint8_t *buffer, size_t offset) {
    return (static_cast<uint16_t>(buffer[offset]) << 8) | buffer[offset + 1];
}

static void sendModbusResponse(WiFiClient &client, const uint8_t *request, const uint8_t *pdu, uint16_t pduLength) {
    uint8_t response[260];
    memcpy(response, request, 4);
    writeU16(response, 4, pduLength + 1);
    response[6] = request[6];
    memcpy(response + 7, pdu, pduLength);
    client.write(response, pduLength + 7);
}

static void sendModbusException(WiFiClient &client, const uint8_t *request, uint8_t functionCode, uint8_t exceptionCode) {
    uint8_t pdu[] = {
        static_cast<uint8_t>(functionCode | 0x80),
        exceptionCode,
    };
    sendModbusResponse(client, request, pdu, sizeof(pdu));
}

static bool isValidCoil(uint16_t address) {
    return address < RELAY_COUNT || address == OTA_TRIGGER_COIL || address == BOOT_COUNTER_RESET_COIL;
}

static bool getCoilValue(uint16_t address) {
    if (address < RELAY_COUNT) {
        return relayStates[address];
    }

    return false;
}

static bool writeCoilValue(uint16_t address, bool state) {
    if (address < RELAY_COUNT) {
        applyRelay(address, state);
        return true;
    }

    if (address == OTA_TRIGGER_COIL) {
        if (state) {
            otaRequested = true;
            otaStatus = OTA_STATUS_CHECKING;
            Serial.println("OTA: requested by Modbus coil 100");
        }
        return true;
    }

    if (address == BOOT_COUNTER_RESET_COIL) {
        if (state) {
            resetBootCounter();
        }
        return true;
    }

    return false;
}

static void handleReadCoils(WiFiClient &client, const uint8_t *request, uint16_t pduLength) {
    if (pduLength < 5) {
        sendModbusException(client, request, 0x01, 0x03);
        return;
    }

    const uint16_t start = readU16(request, 8);
    const uint16_t quantity = readU16(request, 10);

    if (quantity < 1 || quantity > 64) {
        sendModbusException(client, request, 0x01, 0x03);
        return;
    }

    for (uint16_t i = 0; i < quantity; i++) {
        if (!isValidCoil(start + i)) {
            sendModbusException(client, request, 0x01, 0x02);
            return;
        }
    }

    if (start >= RELAY_COUNT && quantity > 1) {
        sendModbusException(client, request, 0x01, 0x02);
        return;
    }

    uint8_t pdu[16] = {0};
    const uint8_t byteCount = (quantity + 7) / 8;
    pdu[0] = 0x01;
    pdu[1] = byteCount;

    for (uint16_t i = 0; i < quantity; i++) {
        if (getCoilValue(start + i)) {
            pdu[2 + (i / 8)] |= 1 << (i % 8);
        }
    }

    sendModbusResponse(client, request, pdu, byteCount + 2);
}

static void handleWriteSingleCoil(WiFiClient &client, const uint8_t *request, uint16_t pduLength) {
    if (pduLength < 5) {
        sendModbusException(client, request, 0x05, 0x03);
        return;
    }

    const uint16_t address = readU16(request, 8);
    const uint16_t value = readU16(request, 10);

    if (!isValidCoil(address)) {
        sendModbusException(client, request, 0x05, 0x02);
        return;
    }

    if (value == 0xFF00) {
        writeCoilValue(address, true);
    } else if (value == 0x0000) {
        writeCoilValue(address, false);
    } else {
        sendModbusException(client, request, 0x05, 0x03);
        return;
    }

    client.write(request, 12);
}

static void handleWriteMultipleCoils(WiFiClient &client, const uint8_t *request, uint16_t pduLength) {
    if (pduLength < 6) {
        sendModbusException(client, request, 0x0F, 0x03);
        return;
    }

    const uint16_t start = readU16(request, 8);
    const uint16_t quantity = readU16(request, 10);
    const uint8_t byteCount = request[12];

    if (quantity < 1 || quantity > 64) {
        sendModbusException(client, request, 0x0F, 0x03);
        return;
    }

    for (uint16_t i = 0; i < quantity; i++) {
        if (!isValidCoil(start + i)) {
            sendModbusException(client, request, 0x0F, 0x02);
            return;
        }
    }

    if (start >= RELAY_COUNT && quantity > 1) {
        sendModbusException(client, request, 0x0F, 0x02);
        return;
    }

    if (byteCount != (quantity + 7) / 8 || pduLength < static_cast<uint16_t>(6 + byteCount)) {
        sendModbusException(client, request, 0x0F, 0x03);
        return;
    }

    for (uint16_t i = 0; i < quantity; i++) {
        const bool state = (request[13 + (i / 8)] & (1 << (i % 8))) != 0;
        writeCoilValue(start + i, state);
    }

    uint8_t pdu[5] = {0x0F, 0, 0, 0, 0};
    writeU16(pdu, 1, start);
    writeU16(pdu, 3, quantity);
    sendModbusResponse(client, request, pdu, sizeof(pdu));
}

static void handleReadInputRegisters(WiFiClient &client, const uint8_t *request, uint16_t pduLength) {
    if (pduLength < 5) {
        sendModbusException(client, request, 0x04, 0x03);
        return;
    }

    const uint16_t start = readU16(request, 8);
    const uint16_t quantity = readU16(request, 10);

    if (quantity < 1 || quantity > 16) {
        sendModbusException(client, request, 0x04, 0x03);
        return;
    }

    for (uint16_t i = 0; i < quantity; i++) {
        if (!isValidInputRegister(start + i)) {
            sendModbusException(client, request, 0x04, 0x02);
            return;
        }
    }

    uint8_t pdu[40] = {0};
    pdu[0] = 0x04;
    pdu[1] = quantity * 2;

    for (uint16_t i = 0; i < quantity; i++) {
        writeU16(pdu, 2 + i * 2, getInputRegisterValue(start + i));
    }

    sendModbusResponse(client, request, pdu, 2 + quantity * 2);
}

static void handleModbusFrame(WiFiClient &client, const uint8_t *request, uint16_t frameLength) {
    if (frameLength < 8) {
        return;
    }

    const uint16_t protocolId = readU16(request, 2);
    const uint16_t length = readU16(request, 4);

    if (protocolId != 0 || length < 2 || frameLength < static_cast<uint16_t>(length + 6)) {
        return;
    }

    const uint8_t unitId = request[6];
    const uint8_t functionCode = request[7];
    const uint16_t pduLength = length - 1;

    if (unitId != MODBUS_UNIT_ID && unitId != 0) {
        sendModbusException(client, request, functionCode, 0x0B);
        return;
    }

    lastModbusActivity = millis();

    switch (functionCode) {
        case 0x01:
            handleReadCoils(client, request, pduLength);
            break;
        case 0x04:
            handleReadInputRegisters(client, request, pduLength);
            break;
        case 0x05:
            handleWriteSingleCoil(client, request, pduLength);
            break;
        case 0x0F:
            handleWriteMultipleCoils(client, request, pduLength);
            break;
        default:
            sendModbusException(client, request, functionCode, 0x01);
            break;
    }
}

static void handleModbusServer() {
    if (WiFi.status() != WL_CONNECTED) {
        if (modbusClient) {
            modbusClient.stop();
        }
        return;
    }

    if (!modbusClient || !modbusClient.connected()) {
        WiFiClient newClient = modbusServer.available();
        if (newClient) {
            modbusClient.stop();
            modbusClient = newClient;
            modbusClient.setNoDelay(true);
            modbusClient.setTimeout(100);
            lastModbusActivity = millis();
        }
    }

    if (!modbusClient || !modbusClient.connected()) {
        return;
    }

    if (millis() - lastModbusActivity > MODBUS_CLIENT_TIMEOUT_MS) {
        modbusClient.stop();
        return;
    }

    while (modbusClient.available() >= 7) {
        uint8_t frame[260];
        const size_t headerRead = modbusClient.readBytes(frame, 7);
        if (headerRead != 7) {
            return;
        }

        const uint16_t length = readU16(frame, 4);

        if (length < 2 || length > 253) {
            modbusClient.stop();
            return;
        }

        const uint16_t frameLength = length + 6;
        const uint16_t remaining = frameLength - 7;
        const size_t remainingRead = modbusClient.readBytes(frame + 7, remaining);
        if (remainingRead != remaining) {
            return;
        }

        handleModbusFrame(modbusClient, frame, frameLength);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);

    pinMode(GPIO_PIN_BUZZER, OUTPUT);
    digitalWrite(GPIO_PIN_BUZZER, LOW);
    pinMode(GPIO_PIN_BOOT, INPUT_PULLUP);

    rgb.begin();
    setRgb(80, 40, 0);

    setupDiagnostics();
    setupRelays();

    WiFi.mode(WIFI_STA);
    
    // Configure static IP
    IPAddress staticIP;
    IPAddress gateway;
    IPAddress subnet;
    IPAddress dns;
    
    staticIP.fromString(STATIC_IP);
    gateway.fromString(GATEWAY_IP);
    subnet.fromString(SUBNET_MASK);
    dns.fromString(DNS_IP);
    
    WiFi.config(staticIP, gateway, subnet, dns);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait for WiFi connection with timeout
    uint32_t connectStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - connectStart) < 15000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFailed to connect to WiFi, restarting...");
        delay(1000);
        ESP.restart();
    }

    startWatchdog();

    modbusServer.begin();
    beep(80);
    Serial.println();
    Serial.print("WiFi connected, Static IP: ");
    Serial.println(WiFi.localIP());
    Serial.println("Modbus TCP server started on port 502");
    Serial.print("Firmware version: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.print("Boot count: ");
    Serial.println(bootCount);
    Serial.print("Last reset reason: ");
    Serial.println(static_cast<uint16_t>(lastResetReason));
}

void loop() {
    esp_task_wdt_reset();
    handleBootButton();
    handleWifi();
    handleModbusServer();
    handleOtaRequest();
    validateRunningFirmwareAfterStableBoot();
    updateStatusLed();
    delay(2);
}
