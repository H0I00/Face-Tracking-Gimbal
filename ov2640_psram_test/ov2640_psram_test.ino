/*
 * ESP32-S3-N16R8 + OV2640 人脸跟踪云台固件（UDP视频 + TCP控制版）
 *
 * 拓扑：
 *   视频上行：ESP32 -> PC，UDP 单播到 PC_IP:8889
 *   控制双向：ESP32 <-> PC，TCP 连接到 PC_IP:8888
 *   UDP 无反压：丢帧不卡顿，永远不会拖慢摄像头采集
 *
 * 协议（与上位机 protocol.py 一致）：
 *   上行视频帧（UDP 分片）：
 *     [frame_id u16 小端][total u8][idx u8][JPEG 片段]
 *     每包 ≤ 1400 字节，收齐分片拼出完整 JPEG
 *   下行控制（TCP 文本行，\n 结尾）：
 *     Y{yaw}P{pitch}\n   设定绝对角度（受校准范围钳制）
 *     R\n                 舵机复位到行程中点
 *     C\n                 查询校准范围，回送 JSON 行
 *     B0\n                蜂鸣器关
 *     B1\n                蜂鸣器开（持续）
 *     B2\n                蜂鸣器短鸣 120ms
 *
 * 依赖库：esp32-camera（Espressif）、LittleFS（arduino-esp32 自带）
 *   Arduino IDE → 工具 → 管理库 → 搜索 "esp32-camera" 安装
 *
 * Arduino IDE 工具菜单设置：
 *   Board            : ESP32S3 Dev Module
 *   Flash Size       : 16MB (128Mb)
 *   Partition Scheme : 选带 LittleFS 的方案（否则校准 JSON 无法落盘）
 *   PSRAM            : OPI PSRAM (8MB)        <- 必须选 OPI
 *   CPU Freq         : 240MHz (WiFi/BT)
 *   USB CDC On Boot  : Disabled               <- 释放 GPIO19/20 给摇杆 LEFT/RIGHT
 *
 * ★使用前先改下面 WIFI_SSID / WIFI_PASS / PC_IP 三项★
 *
 * === 优化清单 ===
 * [1] 摄像头: fb_count 2→3, jpeg_quality 10→12, sharpness 1→2
 * [2] 传输: portENTER_CRITICAL→FreeRTOS互斥锁, TCP NoDelay, 栈 12K→16K
 * [3] 舵机: EMA平滑插值 + 死区抑制 + 独立50Hz更新任务
 * [4] 网络: WiFi TX最大功率, TCP连接后setNoDelay(true)
 * [5] 调度: loop delay 2→1ms, 舵机任务优先级4(高于视频3)
 */

#include "esp_camera.h"
#include "Arduino.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <LittleFS.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <lwip/sockets.h>   // SO_KEEPALIVE, TCP_KEEPIDLE/INTVL/CNT, IPPROTO_TCP

// ==================== 用户配置（按实际修改） ====================
#define WIFI_SSID        "YOUR_WIFI_SSID"     // Wi-Fi 名字
#define WIFI_PASS        "YOUR_WIFI_PASSWORD" // Wi-Fi 密码
#define PC_IP            "192.168.1.100"      // PC 的局域网 IP（ipconfig 查看 WLAN 的 IPv4）
#define TCP_PORT         8888             // PC TCP 控制端口
#define UDP_VIDEO_PORT   8889             // PC UDP 视频端口
#define STREAM_FPS       10                // VGA 10fps
// ================================================================

// ---------- 发现广播协议（已废弃，改为硬编码 PC IP） ----------

// ---------- 云台舵机（SG90，依据项目 pinmap.txt） ----------
#define SERVO_YAW_PIN     41
#define SERVO_PITCH_PIN   42
#define SERVO_FREQ        50
#define SERVO_RES_BITS    14
#define SERVO_MIN_US      500
#define SERVO_MAX_US      2500
#define SERVO_ANGLE_MIN   0.0f
#define SERVO_ANGLE_MAX   180.0f
#define YAW_CENTER        90.0f
#define PITCH_CENTER      45.0f
#define YAW_MIN_DEF       0.0f
#define YAW_MAX_DEF       180.0f
#define PITCH_MIN_DEF     0.0f
#define PITCH_MAX_DEF     90.0f

// ---------- 舵机平滑控制参数（自适应 EMA） ----------
// α 随误差大小动态变化：误差大→α大→快速冲刺；误差小→α小→精细微调
#define SERVO_ALPHA_MIN   0.08f   // 误差≈0 时的 EMA 系数（精细微调）
#define SERVO_ALPHA_MAX   0.50f   // 误差≥阈值时的 EMA 系数（快速冲刺）
#define SERVO_ADAPT_THRESH 8.0f   // 误差阈值(度)：超过此值用 ALPHA_MAX
#define SERVO_MAX_STEP    8.0f    // 单次更新最大步进(度)，50Hz下=400°/s速度上限
#define SERVO_DEADZONE    0.05f   // 死区：极小，避免到位前突然停转
#define SERVO_UPDATE_HZ   50      // 舵机更新频率 (Hz)

// ---------- 摇杆按键 ----------
#define JS_PIN_UP     1
#define JS_PIN_DOWN   2
#define JS_PIN_LEFT   19
#define JS_PIN_RIGHT  20
#define JS_PIN_MID    21
#define JS_PIN_SET    15
#define JS_PIN_RST    14

// ---------- 有源蜂鸣器 ----------
#define BUZZER_PIN     47

// ---------- 校准参数 ----------
#define CALIB_STEP_DEG   1.0f
#define CALIB_HOLD_MS    300
#define CALIB_REPEAT_MS  80
#define CALIB_FILE       "/calib.json"

// ---------- OV2640 引脚定义 ----------
#define PWDN_GPIO_NUM     39
#define RESET_GPIO_NUM    38
#define XCLK_GPIO_NUM     (-1)
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5
#define Y2_GPIO_NUM       11
#define Y3_GPIO_NUM        9
#define Y4_GPIO_NUM        8
#define Y5_GPIO_NUM       10
#define Y6_GPIO_NUM       12
#define Y7_GPIO_NUM       18
#define Y8_GPIO_NUM       17
#define Y9_GPIO_NUM       16
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13
#define FLASH_GPIO_NUM    40

// ---------- 采集参数 ----------
#define CAM_FRAME_SIZE    FRAMESIZE_VGA   // 640×480
#define CAM_JPEG_QUALITY  12              // 较高质量
#define CAM_FB_COUNT      3               // 三缓冲：采集/发送可并行，减少等待卡顿
#define CAM_XCLK_FREQ     20000000

// ---------- TCP ----------
#define TCP_MAX_FRAME_LEN  200000    // 单帧 JPEG 上限
#define TCP_RX_BUF_SZ     256       // 控制指令行缓冲

// ---------- UDP 视频 ----------
#define UDP_MAX_PAYLOAD  1400       // 单包最大载荷（避免 IP 分片）
#define UDP_HEADER_SIZE  4          // frame_id(u16) + total(u8) + idx(u8)

static camera_config_t cam_cfg;
static bool cam_ready = false;

static WiFiClient tcpClient;                // 连接到 PC（仅控制指令）
static WiFiUDP udpVideo;                    // UDP 视频发送
static TaskHandle_t streamTaskHandle = nullptr;
static TaskHandle_t servoTaskHandle = nullptr;  // [优化] 独立舵机更新任务

// [优化] 用 FreeRTOS 互斥锁替代 portENTER_CRITICAL，避免在发送大帧时关中断
static SemaphoreHandle_t tcp_mutex = nullptr;

// [修复] UDP 视频发送：帧序号（循环 0..65535）
static uint16_t udp_frame_id = 0;

// 舵机当前角度（由控制指令/摇杆直接设置的"目标角度"）
static volatile float target_yaw = YAW_CENTER;
static volatile float target_pitch = PITCH_CENTER;

// [优化] 舵机平滑后的"实际输出角度"（EMA 滤波后，由独立任务写入 PWM）
static float smooth_yaw = YAW_CENTER;
static float smooth_pitch = PITCH_CENTER;

// cur_yaw / cur_pitch 仍保留，用于校准和摇杆手动模式（直接写 PWM）
static float cur_yaw = YAW_CENTER;
static float cur_pitch = PITCH_CENTER;

// 校准行程范围
static float calib_yaw_min   = YAW_MIN_DEF;
static float calib_yaw_max   = YAW_MAX_DEF;
static float calib_pitch_min = PITCH_MIN_DEF;
static float calib_pitch_max = PITCH_MAX_DEF;

static const char DQ = 34;

// ---------- 摄像头初始化 ----------
static bool init_camera(void)
{
    memset(&cam_cfg, 0, sizeof(cam_cfg));

    cam_cfg.ledc_channel = LEDC_CHANNEL_0;
    cam_cfg.ledc_timer   = LEDC_TIMER_0;

    cam_cfg.pin_d0       = Y2_GPIO_NUM;
    cam_cfg.pin_d1       = Y3_GPIO_NUM;
    cam_cfg.pin_d2       = Y4_GPIO_NUM;
    cam_cfg.pin_d3       = Y5_GPIO_NUM;
    cam_cfg.pin_d4       = Y6_GPIO_NUM;
    cam_cfg.pin_d5       = Y7_GPIO_NUM;
    cam_cfg.pin_d6       = Y8_GPIO_NUM;
    cam_cfg.pin_d7       = Y9_GPIO_NUM;
    cam_cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cam_cfg.pin_href     = HREF_GPIO_NUM;
    cam_cfg.pin_pclk     = PCLK_GPIO_NUM;
    cam_cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cam_cfg.pin_reset    = RESET_GPIO_NUM;
    cam_cfg.pin_xclk     = XCLK_GPIO_NUM;
    cam_cfg.pin_sccb_sda = SIOD_GPIO_NUM;
    cam_cfg.pin_sccb_scl = SIOC_GPIO_NUM;

    cam_cfg.xclk_freq_hz = CAM_XCLK_FREQ;
    cam_cfg.pixel_format = PIXFORMAT_JPEG;
    cam_cfg.frame_size   = CAM_FRAME_SIZE;
    cam_cfg.jpeg_quality = CAM_JPEG_QUALITY;
    cam_cfg.fb_count     = CAM_FB_COUNT;
    cam_cfg.grab_mode    = CAMERA_GRAB_LATEST;
    cam_cfg.fb_location  = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&cam_cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] 初始化失败: 0x%x (%s)\n", err, esp_err_to_name(err));
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != nullptr) {
        s->set_framesize(s, CAM_FRAME_SIZE);
        s->set_quality(s, CAM_JPEG_QUALITY);
        s->set_ae_level(s,  0);
        s->set_aec_value(s, 300);
        s->set_exposure_ctrl(s, 1);
        s->set_awb_gain(s, 1);
        s->set_gain_ctrl(s, 1);
        s->set_sharpness(s, 2);     // [优化] 1→2: 增强边缘有利于人脸检测
        s->set_saturation(s, 0);
        s->set_brightness(s, 0);
        s->set_contrast(s, 1);
    }
    return true;
}

// ---------- Wi-Fi 连接（STA） ----------
static void connect_wifi(void)
{
    Serial.printf("[WIFI] 正在连接 \"%s\" ...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 30000) {
        delay(500);
        Serial.println("正在连接...");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("连接成功！");
        Serial.print("[WIFI] 本机 IP: ");
        Serial.println(WiFi.localIP());
        WiFi.setSleep(false);
        WiFi.setAutoReconnect(true);
        // [优化] 设置最大发射功率，增强信号稳定性
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
    } else {
        Serial.println("[WIFI] 连接失败！将在 loop 中持续重试。");
    }
}

// ---------- 云台舵机 ----------
static void write_servo(uint8_t pin, float angle_deg)
{
    if (angle_deg < SERVO_ANGLE_MIN) angle_deg = SERVO_ANGLE_MIN;
    if (angle_deg > SERVO_ANGLE_MAX) angle_deg = SERVO_ANGLE_MAX;

    if (pin == SERVO_YAW_PIN)
        angle_deg = SERVO_ANGLE_MAX - angle_deg;

    float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US)
               * (angle_deg - SERVO_ANGLE_MIN)
               / (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN);
    uint32_t duty = (uint32_t)(us * (1UL << SERVO_RES_BITS) / 20000.0f);
    ledcWrite(pin, duty);
}

static void init_servo(void)
{
    ledcAttach(SERVO_YAW_PIN,   SERVO_FREQ, SERVO_RES_BITS);
    ledcAttach(SERVO_PITCH_PIN, SERVO_FREQ, SERVO_RES_BITS);

    cur_yaw = YAW_CENTER;
    cur_pitch = PITCH_CENTER;
    target_yaw = YAW_CENTER;
    target_pitch = PITCH_CENTER;
    smooth_yaw = YAW_CENTER;
    smooth_pitch = PITCH_CENTER;
    write_servo(SERVO_YAW_PIN, cur_yaw);
    write_servo(SERVO_PITCH_PIN, cur_pitch);

    Serial.printf("[SERVO] 初始化完成 | Yaw->GPIO%d @ %.1f°  Pitch->GPIO%d @ %.1f°\n",
                  SERVO_YAW_PIN, cur_yaw, SERVO_PITCH_PIN, cur_pitch);
}

// [优化] 设置目标角度（由控制指令调用），实际 PWM 由独立任务平滑更新
static void set_target_angles(float yaw, float pitch)
{
    if (yaw   < calib_yaw_min)   yaw   = calib_yaw_min;
    if (yaw   > calib_yaw_max)   yaw   = calib_yaw_max;
    if (pitch < calib_pitch_min) pitch = calib_pitch_min;
    if (pitch > calib_pitch_max) pitch = calib_pitch_max;
    target_yaw = yaw;
    target_pitch = pitch;
    cur_yaw = yaw;
    cur_pitch = pitch;
}

// 保留：直接写 PWM（校准模式和摇杆手动模式使用）
static void move_servos_raw(float yaw, float pitch)
{
    if (yaw   < SERVO_ANGLE_MIN) yaw   = SERVO_ANGLE_MIN;
    if (yaw   > SERVO_ANGLE_MAX) yaw   = SERVO_ANGLE_MAX;
    if (pitch < SERVO_ANGLE_MIN) pitch = SERVO_ANGLE_MIN;
    if (pitch > SERVO_ANGLE_MAX) pitch = SERVO_ANGLE_MAX;
    cur_yaw = yaw;
    cur_pitch = pitch;
    // 手动模式下同步目标和平滑值，避免切回自动时跳变
    target_yaw = yaw;
    target_pitch = pitch;
    smooth_yaw = yaw;
    smooth_pitch = pitch;
    write_servo(SERVO_YAW_PIN, cur_yaw);
    write_servo(SERVO_PITCH_PIN, cur_pitch);
}

// ---------- 舵机校准状态（前置声明，供 servoSmoothTask 使用） ----------
static bool calib_active = false;
static int  calib_stage  = 0;

// [优化] 独立舵机平滑更新任务（FreeRTOS，固定 50Hz）
// 使用 EMA + 步进钳位：小修正快速响应，大跳变速度受限
static void servoSmoothTask(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(1000 / SERVO_UPDATE_HZ);
    TickType_t last = xTaskGetTickCount();
    float prev_written_yaw = smooth_yaw;
    float prev_written_pitch = smooth_pitch;

    for (;;) {
        if (!calib_active) {
            float ty = target_yaw;
            float tp = target_pitch;

            // [自适应 EMA] α 随误差大小变化：大误差→快速冲刺，小误差→精细微调
            float err_y = fabsf(ty - smooth_yaw);
            float err_p = fabsf(tp - smooth_pitch);
            float ratio_y = err_y / SERVO_ADAPT_THRESH;
            if (ratio_y > 1.0f) ratio_y = 1.0f;
            float ratio_p = err_p / SERVO_ADAPT_THRESH;
            if (ratio_p > 1.0f) ratio_p = 1.0f;
            float ay = SERVO_ALPHA_MIN + (SERVO_ALPHA_MAX - SERVO_ALPHA_MIN) * ratio_y;
            float ap = SERVO_ALPHA_MIN + (SERVO_ALPHA_MAX - SERVO_ALPHA_MIN) * ratio_p;

            float raw_yaw   = smooth_yaw   + ay * (ty - smooth_yaw);
            float raw_pitch = smooth_pitch + ap * (tp - smooth_pitch);

            // 钳位单次步进：限制最大速度，防止通信卡顿后舵机猛转
            float step_yaw   = raw_yaw   - smooth_yaw;
            float step_pitch = raw_pitch - smooth_pitch;
            if (step_yaw > SERVO_MAX_STEP)        step_yaw = SERVO_MAX_STEP;
            else if (step_yaw < -SERVO_MAX_STEP)  step_yaw = -SERVO_MAX_STEP;
            if (step_pitch > SERVO_MAX_STEP)        step_pitch = SERVO_MAX_STEP;
            else if (step_pitch < -SERVO_MAX_STEP)  step_pitch = -SERVO_MAX_STEP;

            smooth_yaw   += step_yaw;
            smooth_pitch += step_pitch;

            // 死区判断
            float delta_y = fabsf(smooth_yaw - prev_written_yaw);
            float delta_p = fabsf(smooth_pitch - prev_written_pitch);

            if (delta_y > SERVO_DEADZONE || delta_p > SERVO_DEADZONE) {
                write_servo(SERVO_YAW_PIN, smooth_yaw);
                write_servo(SERVO_PITCH_PIN, smooth_pitch);
                prev_written_yaw = smooth_yaw;
                prev_written_pitch = smooth_pitch;
            }
        }

        vTaskDelayUntil(&last, period);
    }
}

// ---------- 有源蜂鸣器 ----------
static bool     buzzer_on = false;
static uint32_t buzzer_off_at = 0;

static void buzzer_set(bool on)
{
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
    buzzer_on = on;
}

static void buzzer_beep(uint32_t ms)
{
    buzzer_set(true);
    buzzer_off_at = millis() + ms;
}

static void buzzer_tick(void)
{
    if (buzzer_off_at && (int32_t)(millis() - buzzer_off_at) >= 0) {
        buzzer_set(false);
        buzzer_off_at = 0;
    }
}

static void init_buzzer(void)
{
    pinMode(BUZZER_PIN, OUTPUT);
    buzzer_set(false);
    Serial.printf("[BUZZER] GPIO%d 有源蜂鸣器就绪(高电平触发)\n", BUZZER_PIN);
}

// ---------- LittleFS 校准 JSON 读写 ----------
static float json_get_float(const char *txt, const char *key)
{
    char pat[40];
    snprintf(pat, sizeof(pat), "%c%s%c:", DQ, key, DQ);
    const char *p = strstr(txt, pat);
    if (!p) return NAN;
    p += strlen(pat);
    return strtof(p, nullptr);
}

static bool load_calibration(void)
{
    if (!LittleFS.exists(CALIB_FILE)) {
        Serial.printf("[CALIB] 未找到 %s，使用默认行程范围\n", CALIB_FILE);
        return false;
    }
    File f = LittleFS.open(CALIB_FILE, "r");
    if (!f) { Serial.println("[CALIB] 打开失败"); return false; }
    String s = f.readString();
    f.close();

    float ymn = json_get_float(s.c_str(), "yaw_min");
    float ymx = json_get_float(s.c_str(), "yaw_max");
    float pmn = json_get_float(s.c_str(), "pitch_min");
    float pmx = json_get_float(s.c_str(), "pitch_max");

    if (isnan(ymn) || isnan(ymx) || isnan(pmn) || isnan(pmx)) {
        Serial.println("[CALIB] JSON 解析失败，使用默认范围");
        return false;
    }
    calib_yaw_min   = ymn;  calib_yaw_max   = ymx;
    calib_pitch_min = pmn;  calib_pitch_max = pmx;
    Serial.printf("[CALIB] 已读取 | Yaw [%.2f, %.2f]  Pitch [%.2f, %.2f]\n",
                  calib_yaw_min, calib_yaw_max, calib_pitch_min, calib_pitch_max);
    return true;
}

static bool save_calibration(void)
{
    File f = LittleFS.open(CALIB_FILE, "w");
    if (!f) { Serial.println("[CALIB] 写入失败"); return false; }
    f.printf("{%cyaw_min%c:%.2f,%cyaw_max%c:%.2f,%cpitch_min%c:%.2f,%cpitch_max%c:%.2f}",
             DQ, calib_yaw_min,   DQ, calib_yaw_max,
             DQ, calib_pitch_min, DQ, calib_pitch_max);
    f.close();
    Serial.printf("[CALIB] 已保存 | Yaw [%.2f, %.2f]  Pitch [%.2f, %.2f]\n",
                  calib_yaw_min, calib_yaw_max, calib_pitch_min, calib_pitch_max);
    return true;
}

static void build_calib_json(char *out, size_t n)
{
    snprintf(out, n,
             "{%cyaw_min%c:%.2f,%cyaw_max%c:%.2f,%cpitch_min%c:%.2f,%cpitch_max%c:%.2f}",
             DQ, calib_yaw_min,   DQ, calib_yaw_max,
             DQ, calib_pitch_min, DQ, calib_pitch_max);
}

// ---------- 摇杆按键 ----------
struct JsKey {
    uint8_t  pin;
    bool     cur;
    bool     last;
    uint32_t last_debounce;
    uint32_t press_t0;
    uint32_t last_repeat;
};

#define JS_DEBOUNCE_MS 20

static JsKey jsKeys[] = {
    {JS_PIN_UP,    false, false, 0, 0, 0},
    {JS_PIN_DOWN,  false, false, 0, 0, 0},
    {JS_PIN_LEFT,  false, false, 0, 0, 0},
    {JS_PIN_RIGHT, false, false, 0, 0, 0},
    {JS_PIN_MID,   false, false, 0, 0, 0},
    {JS_PIN_SET,   false, false, 0, 0, 0},
    {JS_PIN_RST,   false, false, 0, 0, 0},
};
static const int JS_N = sizeof(jsKeys) / sizeof(jsKeys[0]);

static void init_joystick(void)
{
    for (int i = 0; i < JS_N; i++) {
        pinMode(jsKeys[i].pin, INPUT_PULLUP);
        jsKeys[i].cur  = (digitalRead(jsKeys[i].pin) == LOW);
        jsKeys[i].last = jsKeys[i].cur;
    }
    Serial.println("[JS] 摇杆按键就绪(上拉，按下=低电平)");
}

static void js_poll(int idx, bool &edge, bool &repeat)
{
    edge = false;
    repeat = false;
    JsKey &k = jsKeys[idx];
    bool raw = (digitalRead(k.pin) == LOW);
    if (raw != k.last) {
        k.last = raw;
        k.last_debounce = millis();
    }
    uint32_t dt = millis() - k.last_debounce;
    if (dt >= JS_DEBOUNCE_MS) {
        if (raw != k.cur) {
            k.cur = raw;
            if (k.cur) {
                k.press_t0   = millis();
                k.last_repeat = millis();
                edge = true;
            }
        }
        if (k.cur) {
            uint32_t held = millis() - k.press_t0;
            if (held >= CALIB_HOLD_MS) {
                if (millis() - k.last_repeat >= CALIB_REPEAT_MS) {
                    k.last_repeat = millis();
                    repeat = true;
                }
            }
        }
    }
}

// ---------- 舵机校准状态机 ----------

static void enter_calib(void)
{
    calib_active = true;
    calib_stage  = 1;
    Serial.println("[CALIB] 进入校准模式: 移动到最小角点，按 MID 记录");
    Serial.printf("       当前 Yaw=%.1f Pitch=%.1f\n", cur_yaw, cur_pitch);
    buzzer_beep(120);
}

static void record_corner(void)
{
    if (calib_stage == 1) {
        calib_yaw_min   = cur_yaw;
        calib_pitch_min = cur_pitch;
        calib_stage = 2;
        Serial.printf("[CALIB] 最小角点 Yaw=%.2f Pitch=%.2f\n", calib_yaw_min, calib_pitch_min);
        Serial.println("       继续移动到最大角点，按 MID 记录");
        buzzer_beep(80); delay(140); buzzer_beep(80);
    } else if (calib_stage == 2) {
        if (cur_yaw   < calib_yaw_min)   calib_yaw_min   = cur_yaw;
        if (cur_yaw   > calib_yaw_max)   calib_yaw_max   = cur_yaw;
        if (cur_pitch < calib_pitch_min) calib_pitch_min = cur_pitch;
        if (cur_pitch > calib_pitch_max) calib_pitch_max = cur_pitch;
        if (calib_yaw_min > calib_yaw_max) {
            float t = calib_yaw_min; calib_yaw_min = calib_yaw_max; calib_yaw_max = t;
        }
        if (calib_pitch_min > calib_pitch_max) {
            float t = calib_pitch_min; calib_pitch_min = calib_pitch_max; calib_pitch_max = t;
        }
        calib_stage = 0;
        calib_active = false;
        save_calibration();
        Serial.println("[CALIB] 校准完成并已保存");
        buzzer_beep(300);
    }
}

static void exit_calib(void)
{
    calib_active = false;
    calib_stage  = 0;
    Serial.println("[CALIB] 放弃校准，退出");
    buzzer_beep(60);
}

static void calib_step(int idx, bool edge, bool repeat)
{
    bool go = edge || repeat;
    if (!go) return;
    JsKey &k = jsKeys[idx];
    if (k.pin == JS_PIN_UP)         cur_pitch -= CALIB_STEP_DEG;
    else if (k.pin == JS_PIN_DOWN)  cur_pitch += CALIB_STEP_DEG;
    else if (k.pin == JS_PIN_LEFT)  cur_yaw   -= CALIB_STEP_DEG;
    else if (k.pin == JS_PIN_RIGHT) cur_yaw   += CALIB_STEP_DEG;
    move_servos_raw(cur_yaw, cur_pitch);
    Serial.printf("[CALIB] Yaw=%.1f Pitch=%.1f\n", cur_yaw, cur_pitch);
}

static void joystick_loop(void)
{
    for (int i = 0; i < JS_N; i++) {
        bool edge, repeat;
        js_poll(i, edge, repeat);
        if (!edge && !repeat) continue;
        uint8_t pin = jsKeys[i].pin;

        if (pin == JS_PIN_RST) {
            if (edge) {
                // 复位：直接写 PWM 并同步所有状态
                move_servos_raw(YAW_CENTER, PITCH_CENTER);
                Serial.printf("[JS] RST 归零 -> Yaw %.1f Pitch %.1f\n", cur_yaw, cur_pitch);
                buzzer_beep(50);
            }
            continue;
        }
        if (pin == JS_PIN_SET) {
            if (edge) {
                if (!calib_active) enter_calib();
                else               exit_calib();
            }
            continue;
        }
        if (!calib_active) {
            if (edge || repeat) {
                bool moved = false;
                if      (pin == JS_PIN_UP)    { cur_pitch -= CALIB_STEP_DEG; moved = true; }
                else if (pin == JS_PIN_DOWN)  { cur_pitch += CALIB_STEP_DEG; moved = true; }
                else if (pin == JS_PIN_LEFT)  { cur_yaw   -= CALIB_STEP_DEG; moved = true; }
                else if (pin == JS_PIN_RIGHT) { cur_yaw   += CALIB_STEP_DEG; moved = true; }
                if (moved) move_servos_raw(cur_yaw, cur_pitch);
            }
            continue;
        }
        if (pin == JS_PIN_MID) {
            if (edge) record_corner();
        } else {
            calib_step(i, edge, repeat);
        }
    }
}

// ---------- TCP 连接管理 ----------
// 状态：0=等待连接  1=正在连接  2=已连接
static int  tcp_state = 0;
static uint32_t last_connect_attempt = 0;
#define RECONNECT_INTERVAL_MS 3000

static bool tcp_connected(void)
{
    return tcp_state == 2 && tcpClient.connected();
}

// 尝试 TCP 连接（直接连硬编码的 PC IP）
static void tcp_try_connect(void)
{
    if (tcp_state != 0) return;
    if (millis() - last_connect_attempt < RECONNECT_INTERVAL_MS) return;
    last_connect_attempt = millis();

    Serial.printf("[TCP] 正在连接 %s:%u ...\n", PC_IP, TCP_PORT);
    tcpClient.stop();
    bool ok = tcpClient.connect(PC_IP, TCP_PORT);
    if (ok) {
        tcp_state = 2;
        tcpClient.setNoDelay(true);
        tcpClient.setTimeout(2000);
        // TCP keepalive：快速检测死连接
        int enable = 1;
        tcpClient.setSocketOption(SOL_SOCKET, SO_KEEPALIVE, &enable, sizeof(enable));
        int keepIdle = 3;
        tcpClient.setOption(TCP_KEEPIDLE, &keepIdle);
        int keepIntvl = 1;
        tcpClient.setOption(TCP_KEEPINTVL, &keepIntvl);
        int keepCnt = 3;
        tcpClient.setOption(TCP_KEEPCNT, &keepCnt);

        Serial.println("[TCP] 控制连接成功！(NoDelay, Keepalive=3s/1s/3)");
        buzzer_beep(80);
    } else {
        Serial.println("[TCP] 连接失败，3s 后重试");
    }
}

// 检测断线
static void tcp_check_alive(void)
{
    if (tcp_state == 2 && !tcpClient.connected()) {
        Serial.println("[TCP] 连接已断开，重新连接中...");
        tcp_state = 0;
        last_connect_attempt = 0;   // 立即重试
        buzzer_beep(60);
    }
}

// ---------- 下行控制指令解析（TCP 文本行） ----------
static char tcp_rx_buf[TCP_RX_BUF_SZ];
static int  tcp_rx_len = 0;

static void handle_control_line(char *line)
{
    // 去掉末尾 \r\n
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) {
        line[--len] = 0;
    }
    if (len == 0) return;

    // 校准模式内忽略角度指令
    if (calib_active && line[0] == 'Y') return;

    // 复位
    if (strcmp(line, "R") == 0 || strcmp(line, "r") == 0) {
        // 复位时直接设置所有状态，跳过平滑过渡
        move_servos_raw(YAW_CENTER, PITCH_CENTER);
        Serial.printf("[CTRL] 复位 -> Yaw %.1f° Pitch %.1f°\n", cur_yaw, cur_pitch);
        return;
    }

    // 查询校准
    if (line[0] == 'C') {
        char json[128];
        build_calib_json(json, sizeof(json));
        // [优化] 使用互斥锁替代临界区
        if (xSemaphoreTake(tcp_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            tcpClient.write((const uint8_t *)json, strlen(json));
            tcpClient.write((const uint8_t *)"\n", 1);
            xSemaphoreGive(tcp_mutex);
        }
        Serial.printf("[CTRL] 回送校准 JSON: %s\n", json);
        return;
    }

    // 蜂鸣器
    if (line[0] == 'B') {
        if (line[1] == '0')      { buzzer_set(false); buzzer_off_at = 0; }
        else if (line[1] == '1') { buzzer_set(true);  buzzer_off_at = 0; }
        else if (line[1] == '2') { buzzer_beep(120);                       }
        return;
    }

    // 角度指令 Y<yaw>P<pitch>
    // [优化] 改为 set_target_angles()，由独立任务平滑过渡到目标
    char *py = strchr(line, 'Y');
    char *pp = strchr(line, 'P');
    if (py != nullptr && pp != nullptr && pp > py) {
        float yaw   = atof(py + 1);
        float pitch = atof(pp + 1);
        set_target_angles(yaw, pitch);
        static float last_log_y = -999.0f, last_log_p = -999.0f;
        if (fabsf(yaw - last_log_y) > 0.5f || fabsf(pitch - last_log_p) > 0.5f) {
            Serial.printf("[CTRL] Yaw %.1f° Pitch %.1f°\n", yaw, pitch);
            last_log_y = yaw;
            last_log_p = pitch;
        }
    }
}

static void handle_control(void)
{
    if (!tcp_connected()) return;
    if (xSemaphoreTake(tcp_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    int avail = tcpClient.available();
    if (avail <= 0) {
        xSemaphoreGive(tcp_mutex);
        return;
    }
    int rd = tcpClient.read((uint8_t *)tcp_rx_buf + tcp_rx_len,
                             TCP_RX_BUF_SZ - 1 - tcp_rx_len);
    xSemaphoreGive(tcp_mutex);

    if (rd <= 0) return;
    tcp_rx_len += rd;
    tcp_rx_buf[tcp_rx_len] = 0;

    // 按行切分
    // [修复] 角度指令只取最后一条（丢弃卡顿期间积压的旧指令），其余指令正常处理
    float pending_yaw = -1.0f, pending_pitch = -1.0f;
    bool  has_pending_angle = false;
    int start = 0;
    for (int i = 0; i < tcp_rx_len; i++) {
        if (tcp_rx_buf[i] == '\n') {
            tcp_rx_buf[i] = 0;
            char *line = tcp_rx_buf + start;
            start = i + 1;

            // 去 \r
            int ll = strlen(line);
            while (ll > 0 && (line[ll-1] == '\r' || line[ll-1] == '\n')) line[--ll] = 0;
            if (ll == 0) continue;

            char *py = strchr(line, 'Y');
            char *pp = strchr(line, 'P');
            if (py != nullptr && pp != nullptr && pp > py) {
                // 角度指令：记录最新值，本轮最后统一应用
                pending_yaw   = atof(py + 1);
                pending_pitch = atof(pp + 1);
                has_pending_angle = true;
            } else {
                // 非角度指令（R/C/B）：立即处理
                handle_control_line(line);
            }
        }
    }
    // 只应用最后一条角度指令
    if (has_pending_angle) {
        char tmp[48];
        snprintf(tmp, sizeof(tmp), "Y%.2fP%.2f", pending_yaw, pending_pitch);
        handle_control_line(tmp);
    }

    // 移走已处理的行
    if (start > 0) {
        memmove(tcp_rx_buf, tcp_rx_buf + start, tcp_rx_len - start);
        tcp_rx_len -= start;
        tcp_rx_buf[tcp_rx_len] = 0;
    }
    // 缓冲溢出保护
    if (tcp_rx_len >= TCP_RX_BUF_SZ - 1) {
        tcp_rx_len = 0;
        tcp_rx_buf[0] = 0;
    }
}

// [修复] 强制断开 TCP 连接（控制指令发送失败时调用）
static void tcp_force_disconnect(void)
{
    Serial.println("[TCP] 强制断开控制连接");
    tcp_state = 0;
    last_connect_attempt = millis();
    tcpClient.stop();
    buzzer_beep(60);
}

// ---------- UDP 视频帧上行（分片发送，无反压） ----------
static bool send_jpeg_frame_udp(const uint8_t *data, size_t len)
{
    if (len == 0 || len > TCP_MAX_FRAME_LEN) return false;

    uint16_t fid = udp_frame_id++;
    const size_t chunk_max = UDP_MAX_PAYLOAD - UDP_HEADER_SIZE;  // 每片最大载荷
    // [修复] 分片总数：ceiling(len / chunk_max)，去掉多余的 +1
    uint8_t total = (uint8_t)((len + chunk_max - 1) / chunk_max);
    if (total == 0) total = 1;

    uint8_t packet[UDP_MAX_PAYLOAD];
    size_t offset = 0;
    for (uint8_t i = 0; i < total; i++) {
        size_t chunkLen = len - offset;
        if (chunkLen > chunk_max)
            chunkLen = chunk_max;

        // 包头：frame_id(u16 小端) + total(u8) + idx(u8)
        packet[0] = fid & 0xFF;
        packet[1] = (fid >> 8) & 0xFF;
        packet[2] = total;
        packet[3] = i;
        memcpy(packet + UDP_HEADER_SIZE, data + offset, chunkLen);

        udpVideo.beginPacket(PC_IP, UDP_VIDEO_PORT);
        udpVideo.write(packet, UDP_HEADER_SIZE + chunkLen);
        udpVideo.endPacket();

        offset += chunkLen;

        // [优化] 分片间微延时：避免 burst 突发导致 Wi-Fi / 路由器缓冲溢出丢包
        // 每帧约 5-15 片，每片间延 50μs ≈ 0.5-0.75ms 总开销，对 10fps 无影响
        if (i + 1 < total) {
            delayMicroseconds(50);
        }
    }
    return true;
}

// ---------- 视频上行任务 ----------
// [优化] 静态发送缓冲区，避免每帧 malloc/free（VGA JPEG 通常 15~40KB）
static uint8_t *tx_jpeg_buf = nullptr;
#define TX_JPEG_BUF_SIZE  60000   // 足够 VGA Q12 最大帧

static void streamTask(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(1000 / STREAM_FPS);
    TickType_t last = xTaskGetTickCount();
    uint32_t stat_t0 = millis();
    uint32_t stat_cnt = 0;

    // 在 PSRAM 中分配发送缓冲区
    tx_jpeg_buf = (uint8_t *)ps_malloc(TX_JPEG_BUF_SIZE);
    if (tx_jpeg_buf == nullptr) {
        Serial.println("[UP] PSRAM 分配发送缓冲失败！");
        vTaskDelete(nullptr);
        return;
    }

    for (;;) {
        // UDP 视频发送：WiFi 连上就发，不依赖 TCP 控制连接
        if (cam_ready && WiFi.status() == WL_CONNECTED) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb != nullptr) {
                if (fb->format == PIXFORMAT_JPEG && fb->len > 0
                    && fb->len <= TX_JPEG_BUF_SIZE) {
                    // [优化] 先复制到发送缓冲，立即归还 fb 给摄像头
                    // 这样摄像头可以继续采集下一帧，不被 UDP 发送阻塞
                    size_t jpeg_len = fb->len;
                    memcpy(tx_jpeg_buf, fb->buf, jpeg_len);
                    esp_camera_fb_return(fb);

                    send_jpeg_frame_udp(tx_jpeg_buf, jpeg_len);
                    stat_cnt++;
                } else {
                    esp_camera_fb_return(fb);
                }
            }
        }

        uint32_t now2 = millis();
        if (now2 - stat_t0 >= 5000) {
            float fps = (float)stat_cnt * 1000.0f / (now2 - stat_t0);
            Serial.printf("[UP] UDP 上行 %u 帧 | %.1f fps | PSRAM free=%u KB | TCP %s\n",
                          (unsigned)stat_cnt, fps,
                          (unsigned)(ESP.getFreePsram() / 1024),
                          tcp_connected() ? "OK" : "DISC");
            stat_t0 = now2;
            stat_cnt = 0;
        }

        vTaskDelayUntil(&last, period);
    }
}

// ==================== setup / loop ====================
void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("==================================");
    Serial.println(" ESP32-S3 + OV2640 TCP 透传固件");
    Serial.println("       (优化版 v2.0)");
    Serial.println("==================================");

    if (psramInit()) {
        Serial.printf("[PSRAM] OK | 总量 %u MB | 可用 %u KB\n",
                      (unsigned)(ESP.getPsramSize() / (1024 * 1024)),
                      (unsigned)(ESP.getFreePsram() / 1024));
    } else {
        Serial.println("[PSRAM] 启动失败！请检查 工具→PSRAM→OPI PSRAM");
        return;
    }

    if (LittleFS.begin(true)) {
        Serial.println("[FS] LittleFS 挂载成功");
    } else {
        Serial.println("[FS] LittleFS 挂载失败!");
    }

    // [优化] 创建 TCP 互斥锁（仅用于控制指令）
    tcp_mutex = xSemaphoreCreateMutex();
    if (tcp_mutex == nullptr) {
        Serial.println("[SYS] 互斥锁创建失败！");
        return;
    }

    init_buzzer();

    if (FLASH_GPIO_NUM >= 0) {
        pinMode(FLASH_GPIO_NUM, OUTPUT);
        digitalWrite(FLASH_GPIO_NUM, LOW);
    }

    load_calibration();

    connect_wifi();
    if (WiFi.status() == WL_CONNECTED) buzzer_beep(80);

    // [UDP] 启动 UDP 视频发送
    udpVideo.begin(UDP_VIDEO_PORT);
    Serial.printf("[UDP] 视频发送已启动 -> %s:%u\n", PC_IP, UDP_VIDEO_PORT);

    init_servo();
    init_joystick();

    // [修复] 舵机平滑任务在摄像头初始化之前创建
    // 确保即使摄像头初始化失败，舵机仍能正常工作
    xTaskCreatePinnedToCore(servoSmoothTask, "servo_smooth", 4096,
                             nullptr, 4, &servoTaskHandle, 0);
    Serial.println("[SYS] 舵机平滑任务已启动 (50Hz, 优先级4, Core0)");

    Serial.println("[CAM]  正在初始化 OV2640 ...");
    cam_ready = init_camera();
    Serial.println(cam_ready ? "[CAM]  OV2640 初始化成功！"
                              : "[CAM]  OV2640 初始化失败，请检查接线/供电。");
    if (!cam_ready) {
        Serial.println("[SYS] 摄像头不可用，仅舵机/按键功能可用");
        return;
    }

    // [优化] 视频上行任务：UDP 发送，无需 TCP 连接，WiFi 连上即可发
    xTaskCreatePinnedToCore(streamTask, "cam_stream", 16384,
                             nullptr, 3, &streamTaskHandle, 1);
    Serial.println("[SYS] UDP 视频任务已启动 (栈16K, 优先级3, Core1)");

    Serial.println("=================================");
    Serial.printf("等待连接 PC %s:%d ...\n", PC_IP, TCP_PORT);
    Serial.println("操作提示:");
    Serial.println("  按 SET(GPIO15) 校准; MID(21) 记录角点; RST(14) 归零");
    Serial.println("优化特性:");
    Serial.printf("  JPEG=%d, fb=%d, 自适应EMA α[%.2f~%.2f] thresh=%.1f° step=%.1f° deadzone=%.2f°\n",
                  CAM_JPEG_QUALITY, CAM_FB_COUNT,
                  SERVO_ALPHA_MIN, SERVO_ALPHA_MAX, SERVO_ADAPT_THRESH,
                  SERVO_MAX_STEP, SERVO_DEADZONE);
}

void loop()
{
    // WiFi 状态机
    static bool wasConnected = true;
    if (WiFi.status() == WL_CONNECTED) {
        if (!wasConnected) {
            wasConnected = true;
            WiFi.setSleep(false);
            WiFi.setTxPower(WIFI_POWER_19_5dBm);  // [优化] 重连后也设最大功率
            Serial.print("[WIFI] 已重连 | IP: ");
            Serial.println(WiFi.localIP());
            buzzer_beep(80);
        }
    } else {
        wasConnected = false;
        static uint32_t lastTry = 0;
        if (millis() - lastTry > 5000) {
            lastTry = millis();
            Serial.println("[WIFI] 断线，重连中 ...");
            WiFi.reconnect();
        }
    }

    // TCP 状态机：连接 → 透传
    if (WiFi.status() == WL_CONNECTED) {
        tcp_try_connect();         // 状态 0: 发起连接
        tcp_check_alive();         // 状态 2: 检测断线
    }

    // 摇杆 + 蜂鸣器
    joystick_loop();
    buzzer_tick();

    // 下行控制（TCP 文本行）
    if (tcp_connected()) {
        handle_control();
    }

    delay(1);   // [优化] 2→1ms: 更快的控制指令响应
}
