// Esteira Industrial (ESP32 + FreeRTOS) — 4 tasks com touch (polling + debounce)
// - ENC_SENSE (periódica 5 ms) -> notifica SPD_CTRL
// - SPD_CTRL (hard RT) -> controle PI simulado, trata HMI (soft) se solicitado
// - SORT_ACT (hard RT, evento Touch B) -> aciona "desviador"
// - SAFETY_TASK (hard RT, evento Touch D) -> E-stop
// ESP-IDF 5.5.x (driver touch legacy) — sem ISR (apenas polling + UART)

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"   // legacy API
#include "driver/uart.h"

#define TAG "ESTEIRA"

// ====== Mapeamento dos touch pads (ESP32) ======
// T7 = GPIO27, T8 = GPIO33, T9 = GPIO32
#define TP_OBJ   TOUCH_PAD_NUM7   // Touch B -> detecção de objeto (desvio)  (GPIO27)
#define TP_HMI   TOUCH_PAD_NUM8   // Touch C -> HMI/telemetria                (GPIO33)
#define TP_ESTOP TOUCH_PAD_NUM9   // Touch D -> E-stop                        (GPIO32)

// ====== Periodicidade, prioridades, stack ======
#define ENC_T_MS        5
#define PRIO_ESTOP      5
#define PRIO_ENC        4
#define PRIO_CTRL       3
#define PRIO_SORT       3
#define PRIO_TOUCH      2
#define PRIO_STATS      1
#define STK_MAIN        3072
#define STK_AUX         2048

// ====== Handles/IPC ======
static TaskHandle_t hENC = NULL, hCTRL = NULL, hSORT = NULL, hSAFE = NULL;
static TaskHandle_t hCtrlNotify = NULL;    // Notificação ENC->CTRL

// Evento SORT
typedef struct { int64_t t_evt_us; } sort_evt_t;
static QueueHandle_t qSort = NULL;

// Semáforos
static SemaphoreHandle_t semEStop = NULL;  // dado no polling do D
static SemaphoreHandle_t semHMI   = NULL;  // dado no polling do C

// Estado simulado
typedef struct {
    float rpm;
    float pos_mm;
    float set_rpm;
} belt_state_t;

static belt_state_t g_belt = { .rpm = 0.f, .pos_mm = 0.f, .set_rpm = 120.0f };

// ====== Busy loop (simula C) ======
static inline void cpu_tight_loop_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) { __asm__ __volatile__("nop"); }
}

// ====== Conversão segura ms -> ticks (garante >=1 tick quando ms>0) ======
static inline TickType_t ticks_from_ms(uint32_t ms) {
    TickType_t t = pdMS_TO_TICKS(ms);
    if (ms > 0 && t == 0) return 1;
    return t;
}

/* ====== Prototipação de tarefas ====== */
static void task_enc_sense(void *arg);
static void task_spd_ctrl(void *arg);
static void task_sort_act(void *arg);
static void task_safety(void *arg);
static void task_touch_poll(void *arg);
static void task_stats(void *arg);
static void task_uart_cmd(void *arg);

/* ====== ENC_SENSE (periódica 5 ms): estima velocidade/posição ====== */
static void task_enc_sense(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    TickType_t T = ticks_from_ms(ENC_T_MS);

    for (;;) {
        // Dinâmica simulada: aproxima rpm do setpoint, integra posição
        float err = g_belt.set_rpm - g_belt.rpm;
        g_belt.rpm += 0.05f * err;             // aproximação lenta para simular inércia
        g_belt.pos_mm += (g_belt.rpm / 60.0f) * (ENC_T_MS / 1000.0f) * 100.0f; // 100 mm por rev (exemplo)

        // clamps na rpm simulada
        if (g_belt.rpm < 0.0f)     g_belt.rpm = 0.0f;
        if (g_belt.rpm > 3000.0f)  g_belt.rpm = 3000.0f;

        // Carga determinística ~0.7 ms
        cpu_tight_loop_us(700);

        // Notifica controle
        if (hCtrlNotify) xTaskNotifyGive(hCtrlNotify);

        vTaskDelayUntil(&next, T);
    }
}

/* ====== SPD_CTRL (encadeada): controle PI simulado + HMI (soft) ====== */
static void task_spd_ctrl(void *arg)
{
    // PI minimalista
    float kp = 0.4f, ki = 0.1f, integ = 0.f;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // acorda após ENC_SENSE

        // Controle (hard): alvo = g_belt.set_rpm, atuando em g_belt.rpm (simulado)
        float err = g_belt.set_rpm - g_belt.rpm;
        integ += err * (ENC_T_MS / 1000.0f);
        // anti-windup simples
        if (integ > 10000.0f) integ = 10000.0f;
        if (integ < -10000.0f) integ = -10000.0f;

        float u = kp * err + ki * integ;
        // "Aplicar" u (mock): ajusta levemente o setpoint
        g_belt.set_rpm += 0.1f * u;
        // clamps de segurança
        if (g_belt.set_rpm < 0.0f)     g_belt.set_rpm = 0.0f;
        if (g_belt.set_rpm > 3000.0f)  g_belt.set_rpm = 3000.0f;

        // Carga determinística ~1.2 ms
        cpu_tight_loop_us(1200);

        // Trecho não-crítico (soft): se HMI solicitada, imprime e retorna rápido
        if (xSemaphoreTake(semHMI, 0) == pdTRUE) {
            printf("HMI: rpm=%.1f set=%.1f pos=%.1fmm\n", g_belt.rpm, g_belt.set_rpm, g_belt.pos_mm);
            cpu_tight_loop_us(400); // ~0.4 ms (soft)
        }
    }
}

/* ====== SORT_ACT (evento Touch B): aciona "desviador" ====== */
static void task_sort_act(void *arg)
{
    sort_evt_t ev;
    for (;;) {
        if (xQueueReceive(qSort, &ev, portMAX_DELAY) == pdTRUE) {
            int64_t t0 = esp_timer_get_time();
            // Janela crítica (ex.: 0.7 ms)
            cpu_tight_loop_us(700);
            (void)t0; // use ESP_LOGD se quiser logar latência da atuação
            // ESP_LOGD(TAG, "SORT_ACT at t=%lld us", (long long)ev.t_evt_us);
        }
    }
}

/* ====== SAFETY_TASK (evento Touch D): E-stop ====== */
static void task_safety(void *arg)
{
    for (;;) {
        if (xSemaphoreTake(semEStop, portMAX_DELAY) == pdTRUE) {
            int64_t t0 = esp_timer_get_time();
            // Ação crítica: zera "PWM", trava atuadores, sinaliza alarme (~0.9 ms)
            g_belt.set_rpm = 0.f;
            cpu_tight_loop_us(900);
            int64_t dt = esp_timer_get_time() - t0;
            ESP_LOGW(TAG, "E-STOP lat=%lld us", (long long)dt);
        }
    }
}

/* ====== TOUCH polling (B, C, D) — com FSM, voltagem, filtro, baseline e debounce ====== */
static void task_touch_poll(void *arg)
{
    // Init completo do touch (legacy)
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_set_measurement_interval(20)); // ~2.5ms
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
    ESP_ERROR_CHECK(touch_pad_filter_start(10));

    // Configura pads (threshold=0 porque usaremos comparação "virtual" com baseline)
    ESP_ERROR_CHECK(touch_pad_config(TP_OBJ,   0));
    ESP_ERROR_CHECK(touch_pad_config(TP_HMI,   0));
    ESP_ERROR_CHECK(touch_pad_config(TP_ESTOP, 0));

    vTaskDelay(ticks_from_ms(80)); // estabiliza

    // Calibração: baseline e threshold = 70% do baseline
    uint16_t base_obj=0, base_hmi=0, base_stop=0;
    ESP_ERROR_CHECK(touch_pad_read_raw_data(TP_OBJ,   &base_obj));
    ESP_ERROR_CHECK(touch_pad_read_raw_data(TP_HMI,   &base_hmi));
    ESP_ERROR_CHECK(touch_pad_read_raw_data(TP_ESTOP, &base_stop));

    ESP_LOGI(TAG, "RAW baseline: OBJ=%u HMI=%u ESTOP=%u", base_obj, base_hmi, base_stop);

    // se algum baseline vier 0, experimente alterar voltagem/intervalo (ver dicas)
    uint16_t th_obj  = (uint16_t)(base_obj  * 0.70f);
    uint16_t th_hmi  = (uint16_t)(base_hmi  * 0.70f);
    uint16_t th_stop = (uint16_t)(base_stop * 0.70f);

    bool prev_obj=false, prev_hmi=false, prev_stop=false;
    TickType_t debounce = ticks_from_ms(30);  // 30 ms

    while (1) {
        uint16_t raw;

        // OBJ (Touch B)
        touch_pad_read_raw_data(TP_OBJ, &raw);
        bool obj = (raw != 0) && (raw < th_obj);
        if (obj && !prev_obj) {
            sort_evt_t e = { .t_evt_us = esp_timer_get_time() };
            (void)xQueueSend(qSort, &e, 0);
            ESP_LOGD(TAG, "OBJ: raw=%u (thr=%u)", raw, th_obj);
        }
        prev_obj = obj;

        // HMI (Touch C)
        touch_pad_read_raw_data(TP_HMI, &raw);
        bool hmi = (raw != 0) && (raw < th_hmi);
        if (hmi && !prev_hmi) { (void)xSemaphoreGive(semHMI); ESP_LOGD(TAG, "HMI: raw=%u (thr=%u)", raw, th_hmi); }
        prev_hmi = hmi;

        // ESTOP (Touch D)
        touch_pad_read_raw_data(TP_ESTOP, &raw);
        bool stop = (raw != 0) && (raw < th_stop);
        if (stop && !prev_stop) { (void)xSemaphoreGive(semEStop); ESP_LOGW(TAG, "E-STOP: raw=%u (thr=%u)", raw, th_stop); }
        prev_stop = stop;

        vTaskDelay(debounce);
    }
}

/* ====== UART CMD: converte teclas do terminal em eventos (B/C/D) e lê RAWs ====== */
static void task_uart_cmd(void *arg)
{
    const uart_port_t U = UART_NUM_0;
    uart_driver_install(U, 256, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "UART: b=OBJ  c=HMI  d=E-STOP  r=RAWs");

    uint8_t ch;
    while (1) {
        if (uart_read_bytes(U, &ch, 1, pdMS_TO_TICKS(10)) == 1) {
            switch (ch) {
                case 'b': case 'B': { sort_evt_t e = { esp_timer_get_time() }; xQueueSend(qSort,&e,0); ESP_LOGI(TAG,"[UART] OBJ"); } break;
                case 'c': case 'C': { xSemaphoreGive(semHMI); ESP_LOGI(TAG,"[UART] HMI"); } break;
                case 'd': case 'D': { xSemaphoreGive(semEStop); ESP_LOGW(TAG,"[UART] E-STOP"); } break;
                case 'r': case 'R': {
                    uint16_t ro=0, rc=0, rd=0;
                    touch_pad_read_raw_data(TP_OBJ,&ro);
                    touch_pad_read_raw_data(TP_HMI,&rc);
                    touch_pad_read_raw_data(TP_ESTOP,&rd);
                    ESP_LOGI(TAG, "RAWs -> OBJ=%u  HMI=%u  ESTOP=%u", ro, rc, rd);
                } break;
                default: break;
            }
        }
        vTaskDelay(ticks_from_ms(5));
    }
}

/* ====== STATS: log 1x/s (opcional) ====== */
static void task_stats(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    TickType_t period = ticks_from_ms(1000);
    for (;;) {
        vTaskDelayUntil(&last, period);
        ESP_LOGI(TAG, "STATS: rpm=%.1f set=%.1f pos=%.1fmm", g_belt.rpm, g_belt.set_rpm, g_belt.pos_mm);
    }
}

/* ====== app_main ====== */
void app_main(void)
{
    // IPC
    qSort    = xQueueCreate(8, sizeof(sort_evt_t));
    semEStop = xSemaphoreCreateBinary();
    semHMI   = xSemaphoreCreateBinary();

    // Tarefas principais (core 0)
    xTaskCreatePinnedToCore(task_safety,    "SAFETY",    STK_MAIN, NULL, PRIO_ESTOP, &hSAFE, 0);
    xTaskCreatePinnedToCore(task_enc_sense, "ENC_SENSE", STK_MAIN, NULL, PRIO_ENC,   &hENC,  0);
    xTaskCreatePinnedToCore(task_spd_ctrl,  "SPD_CTRL",  STK_MAIN, NULL, PRIO_CTRL,  &hCTRL, 0);
    xTaskCreatePinnedToCore(task_sort_act,  "SORT_ACT",  STK_MAIN, NULL, PRIO_SORT,  &hSORT, 0);

    // Encadeamento ENC -> CTRL
    hCtrlNotify = hCTRL;

    // Tarefas auxiliares
    xTaskCreatePinnedToCore(task_touch_poll, "TOUCH",    STK_AUX, NULL, PRIO_TOUCH, NULL, 0);
    xTaskCreatePinnedToCore(task_uart_cmd,   "UART_CMD", STK_AUX, NULL, PRIO_TOUCH, NULL, 0);
    xTaskCreatePinnedToCore(task_stats,      "STATS",    STK_AUX, NULL, PRIO_STATS, NULL, 0);

    ESP_LOGI(TAG, "Sistema iniciado (polling de touch + UART)");
}
