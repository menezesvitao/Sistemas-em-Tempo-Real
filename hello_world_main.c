//GABRIEL LAURENTINO TOURINHO - VICTOR MENZES FERREIRA

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
#include "driver/touch_pad.h"
#include "driver/uart.h"

#define TAG "ESTEIRA"

// Mapeamento dos touch pads
// T7 = GPIO27, T8 = GPIO33, T9 = GPIO32
#define TP_OBJ   TOUCH_PAD_NUM7   // Touch B -> detecção de objeto (GPIO27)
#define TP_HMI   TOUCH_PAD_NUM8   // Touch C -> HMI/telemetria   (GPIO33)
#define TP_ESTOP TOUCH_PAD_NUM9   // Touch D -> E-stop           (GPIO32)

// Periodicidade, prioridades, stack
/*
//  RM 
#define ENC_T_MS        5
#define PRIO_ENC        5   // maior taxa -> maior prioridade
#define PRIO_CTRL       4
#define PRIO_SORT       3
#define PRIO_ESTOP      2   // no RM puro, fica abaixo
#define PRIO_TOUCH      2
#define PRIO_STATS      1
#define STK_MAIN        3072
#define STK_AUX         2048

//  DM 
#define ENC_T_MS        5
#define PRIO_ESTOP      5   // deadline mais curto -> prioridade máxima
#define PRIO_ENC        4
#define PRIO_SORT       3
#define PRIO_CTRL       2
#define PRIO_TOUCH      2
#define PRIO_STATS      1
#define STK_MAIN        3072
#define STK_AUX         2048
*/
//  CUSTOM 
#define ENC_T_MS        5
#define PRIO_ESTOP      5   // segurança sempre em 1º lugar
#define PRIO_ENC        4   // base temporal para todo o sistema
#define PRIO_CTRL       3   // controle da velocidade
#define PRIO_SORT       3   // desvio de objetos
#define PRIO_TOUCH      2   // HMI / interface
#define PRIO_STATS      1   // estatísticas (não crítico)
#define STK_MAIN        3072
#define STK_AUX         2048


//  Deadlines (ta em microssegundos) 
#define D_ENC_US    5000
#define D_CTRL_US  10000
#define D_SORT_US  10000
#define D_SAFE_US   5000

//  Handles/IPC 
static TaskHandle_t hENC = NULL, hCTRL = NULL, hSORT = NULL, hSAFE = NULL;
static TaskHandle_t hCtrlNotify = NULL;    // ENC -> CTRL (notify)
typedef struct { int64_t t_evt_us; } sort_evt_t;
static QueueHandle_t qSort = NULL;
static SemaphoreHandle_t semEStop = NULL;  // disparado pelo Touch D
static SemaphoreHandle_t semHMI   = NULL;  // disparado pelo Touch C

//  Estado simulado da esteira 
typedef struct {
    float rpm;     // medição de rpm
    float pos_mm;  // posição "andada" em mm
    float set_rpm; // referência de rpm
} belt_state_t;

static belt_state_t g_belt = { .rpm = 0.f, .pos_mm = 0.f, .set_rpm = 120.0f };

//  Instrumentação de tempo/métricas 
typedef struct {
    volatile uint32_t releases, starts, finishes;
    volatile uint32_t hard_miss, soft_miss;
    volatile int64_t  last_release_us, last_start_us, last_end_us;
    volatile int64_t  worst_exec_us, worst_latency_us, worst_response_us;
} rt_stats_t;

static rt_stats_t st_enc  = {0};
static rt_stats_t st_ctrl = {0};
static rt_stats_t st_sort = {0};
static rt_stats_t st_safe = {0};
static rt_stats_t st_hmi = {0};

static volatile int64_t g_last_touchB_us = 0;     // release do Touch B
static volatile int64_t g_last_touchD_us = 0;     // release do Touch D
static volatile int64_t g_last_enc_release_us = 0;// release do ENC (para CTRL)

static inline void stats_on_release(rt_stats_t *s, int64_t t_rel) {
    s->releases++;
    s->last_release_us = t_rel;
}

static inline void stats_on_start(rt_stats_t *s, int64_t t_start) {
    s->starts++;
    s->last_start_us = t_start;
    int64_t lat = t_start - s->last_release_us;
    if (lat > s->worst_latency_us) s->worst_latency_us = lat;
}

static inline void stats_on_finish(rt_stats_t *s, int64_t t_end, int64_t D_us, bool hard) {
    s->finishes++;
    s->last_end_us = t_end;
    int64_t exec = t_end - s->last_start_us;
    if (exec > s->worst_exec_us) s->worst_exec_us = exec;
    int64_t resp = t_end - s->last_release_us;
    if (resp > s->worst_response_us) s->worst_response_us = resp;
    if (resp > D_us) {
        if (hard) s->hard_miss++; else s->soft_miss++;
    }
}

// ====== Busy loop determinístico (~simula WCET) ======
static inline void cpu_tight_loop_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) { __asm__ __volatile__("nop"); }
}

// ====== ms -> ticks (garante >=1 tick quando ms>0) ======
static inline TickType_t ticks_from_ms(uint32_t ms) {
    TickType_t t = pdMS_TO_TICKS(ms);
    if (ms > 0 && t == 0) return 1;
    return t;
}

// ========= Prototipação =========
static void task_enc_sense(void *arg);
static void task_spd_ctrl(void *arg);
static void task_sort_act(void *arg);
static void task_safety(void *arg);
static void task_touch_poll(void *arg);
static void task_stats(void *arg);
static void task_uart_cmd(void *arg);

// Idle hook para %CPU simples
#if configUSE_IDLE_HOOK
static volatile int64_t idle_us_acc = 0;
void vApplicationIdleHook(void) {
    static int64_t last = 0;
    int64_t now = esp_timer_get_time();
    if (last) idle_us_acc += (now - last);
    last = now;
}
#endif

// Runtime stats + Trace Facility
#if (configUSE_TRACE_FACILITY==1) && (configGENERATE_RUN_TIME_STATS==1)
static void print_runtime_stats(void) {
    static char buf[1024];
    vTaskGetRunTimeStats(buf);
    ESP_LOGI(TAG, "\nTask               Time(us)   %%CPU\n%s", buf);
}
#endif

/*  ENC_SENSE estima velocidade/posição  */
static void task_enc_sense(void *arg)
{
    TickType_t next = xTaskGetTickCount();
    TickType_t T = ticks_from_ms(ENC_T_MS);

    for (;;) {
        int64_t t_rel = esp_timer_get_time();
        stats_on_release(&st_enc, t_rel);
        g_last_enc_release_us = t_rel;

        stats_on_start(&st_enc, esp_timer_get_time());

        // Dinâmica simulada: aproxima rpm do setpoint, integra posição
        float err = g_belt.set_rpm - g_belt.rpm;
        g_belt.rpm += 0.05f * err; // aproximação lenta para simular inércia
        g_belt.pos_mm += (g_belt.rpm / 60.0f) * (ENC_T_MS / 1000.0f) * 100.0f; // 100 mm por rev

        // Limites físicos da simulação
        if (g_belt.rpm < 0.0f)     g_belt.rpm = 0.0f;
        if (g_belt.rpm > 5000.0f)  g_belt.rpm = 5000.0f;

        // Carga determinística ~0.7 ms
        cpu_tight_loop_us(700);

        stats_on_finish(&st_enc, esp_timer_get_time(), D_ENC_US, /*hard=*/true);

        // Notifica controle (encadeamento)
        if (hCtrlNotify) xTaskNotifyGive(hCtrlNotify);

        vTaskDelayUntil(&next, T);
    }
}

/*  SPD_CTRL controle PI simulado + HMI (soft)  */
static void task_spd_ctrl(void *arg)
{
    float kp = 0.4f, ki = 0.1f, integ = 0.f;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // acorda após ENC
        stats_on_release(&st_ctrl, g_last_enc_release_us);
        stats_on_start(&st_ctrl, esp_timer_get_time());

        // Controle (hard): PI
        float err = g_belt.set_rpm - g_belt.rpm;
        integ += err * (ENC_T_MS / 1000.0f);
        // anti-windup simples
        if (integ > 10000.0f) integ = 10000.0f;
        if (integ < -10000.0f) integ = -10000.0f;

        float u = kp * err + ki * integ;
        g_belt.set_rpm += 0.1f * u;

        // clamps de segurança da referência
        if (g_belt.set_rpm < 0.0f)     g_belt.set_rpm = 0.0f;
        if (g_belt.set_rpm > 5000.0f)  g_belt.set_rpm = 5000.0f;

        // Carga determinística ~1.2 ms
        cpu_tight_loop_us(1200);

        stats_on_finish(&st_ctrl, esp_timer_get_time(), D_CTRL_US, /*hard=*/true);

        // Trecho não-crítico (soft): HMI
        if (xSemaphoreTake(semHMI, 0) == pdTRUE) {
            // Marca início da HMI
            stats_on_start(&st_hmi, esp_timer_get_time());

            printf("HMI: rpm=%.1f set=%.1f pos=%.1fmm\n", g_belt.rpm, g_belt.set_rpm, g_belt.pos_mm);

            cpu_tight_loop_us(400); // ~0.4 ms (soft)

            stats_on_finish(&st_hmi, esp_timer_get_time(), 50000, /*hard=*/false);
        }
    }
}

/*  SORT_ACT (evento Touch B): aciona "desviador"  */
static void task_sort_act(void *arg)
{
    sort_evt_t ev;
    for (;;) {
        if (xQueueReceive(qSort, &ev, portMAX_DELAY) == pdTRUE) {
            stats_on_release(&st_sort, ev.t_evt_us);
            stats_on_start(&st_sort, esp_timer_get_time());

            // Janela crítica (ex.: 0.7 ms) — aqui acionaria o atuador/suporte
            cpu_tight_loop_us(700);

            stats_on_finish(&st_sort, esp_timer_get_time(), D_SORT_US, /*hard=*/true);
        }
    }
}

/*  SAFETY_TASK (evento Touch D): E-stop  */
static void task_safety(void *arg)
{
    for (;;) {
        if (xSemaphoreTake(semEStop, portMAX_DELAY) == pdTRUE) {
            stats_on_release(&st_safe, g_last_touchD_us);
            stats_on_start(&st_safe, esp_timer_get_time());

            // Ação crítica: zera "PWM", trava atuadores, sinaliza alarme (~0.9 ms)
            g_belt.set_rpm = 0.f;
            cpu_tight_loop_us(900);

            stats_on_finish(&st_safe, esp_timer_get_time(), D_SAFE_US, /*hard=*/true);
            ESP_LOGW(TAG, "E-STOP executado");
        }
    }
}

/*  TOUCH polling (B, C, D) — voltagem, filtro, baseline e debounce  */
static void task_touch_poll(void *arg)
{
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_set_measurement_interval(20)); // ~2.5ms
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
    ESP_ERROR_CHECK(touch_pad_filter_start(10));

    // Configura pads (threshold=0; comparamos com baseline no software)
    ESP_ERROR_CHECK(touch_pad_config(TP_OBJ,   0));
    ESP_ERROR_CHECK(touch_pad_config(TP_HMI,   0));
    ESP_ERROR_CHECK(touch_pad_config(TP_ESTOP, 0));

    vTaskDelay(ticks_from_ms(80)); // estabiliza

    uint16_t base_obj=0, base_hmi=0, base_stop=0;
    ESP_ERROR_CHECK(touch_pad_read_raw_data(TP_OBJ,   &base_obj));
    ESP_ERROR_CHECK(touch_pad_read_raw_data(TP_HMI,   &base_hmi));
    ESP_ERROR_CHECK(touch_pad_read_raw_data(TP_ESTOP, &base_stop));
    if (base_obj  < 50) base_obj  = 2000;
    if (base_hmi  < 50) base_hmi  = 2000;
    if (base_stop < 50) base_stop = 2000;

    ESP_LOGI(TAG, "RAW baseline: OBJ=%u (T7/GPIO27)  HMI=%u (T8/GPIO33)  ESTOP=%u (T9/GPIO32)",
             base_obj, base_hmi, base_stop);

    uint16_t th_obj  = (uint16_t)(base_obj  * 0.70f);
    uint16_t th_hmi  = (uint16_t)(base_hmi  * 0.70f);
    uint16_t th_stop = (uint16_t)(base_stop * 0.70f);

    bool prev_obj=false, prev_hmi=false, prev_stop=false;
    TickType_t debounce = ticks_from_ms(10);

    while (1) {
        uint16_t raw;

        // OBJ (Touch B)
        touch_pad_read_raw_data(TP_OBJ, &raw);
        bool obj = (raw < th_obj);
        if (obj && !prev_obj) {
            g_last_touchB_us = esp_timer_get_time();         // release do evento
            sort_evt_t e = { .t_evt_us = g_last_touchB_us }; // passa release para a task
            (void)xQueueSend(qSort, &e, 0);
            ESP_LOGI(TAG, "OBJ: raw=%u (thr=%u)", raw, th_obj);
        }
        prev_obj = obj;

        // HMI (Touch C)
        touch_pad_read_raw_data(TP_HMI, &raw);
        bool hmi = (raw < th_hmi);
        if (hmi && !prev_hmi) { 
            stats_on_release(&st_hmi, esp_timer_get_time());
            (void)xSemaphoreGive(semHMI); 
            ESP_LOGI(TAG, "HMI: raw=%u (thr=%u)", raw, th_hmi); 
        }
        prev_hmi = hmi;

        // ESTOP (Touch D)
        touch_pad_read_raw_data(TP_ESTOP, &raw);
        bool stop = (raw < th_stop);
        if (stop && !prev_stop) {
            g_last_touchD_us = esp_timer_get_time();         // release do evento
            (void)xSemaphoreGive(semEStop);
            ESP_LOGW(TAG, "E-STOP: raw=%u (thr=%u)", raw, th_stop);
        }
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
                case 'b': case 'B': {
                    g_last_touchB_us = esp_timer_get_time();
                    sort_evt_t e = { g_last_touchB_us };
                    xQueueSend(qSort,&e,0);
                    ESP_LOGI(TAG,"[UART] OBJ");
                } break;
                case 'c': case 'C': { xSemaphoreGive(semHMI); ESP_LOGI(TAG,"[UART] HMI"); } break;
                case 'd': case 'D': {
                    g_last_touchD_us = esp_timer_get_time();
                    xSemaphoreGive(semEStop);
                    ESP_LOGW(TAG,"[UART] E-STOP");
                } break;
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

/* ====== STATS: log 1x/s ====== */
static void task_stats(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    TickType_t period = ticks_from_ms(1000);

    // Para %CPU simples com Idle Hook:
    int64_t last_sample = 0, last_idle = 0;

    for (;;) {
        vTaskDelayUntil(&last, period);

        ESP_LOGI(TAG, "STATS: rpm=%.1f set=%.1f pos=%.1fmm", g_belt.rpm, g_belt.set_rpm, g_belt.pos_mm);

        ESP_LOGI(TAG,
            "ENC: rel=%u fin=%u hard=%u Cmax=%lldus Lmax=%lldus Rmax=%lldus",
            st_enc.releases, st_enc.finishes, st_enc.hard_miss,
            (long long)st_enc.worst_exec_us,
            (long long)st_enc.worst_latency_us,
            (long long)st_enc.worst_response_us);

        ESP_LOGI(TAG,
            "CTRL: rel=%u fin=%u hard=%u Cmax=%lldus Lmax=%lldus Rmax=%lldus",
            st_ctrl.releases, st_ctrl.finishes, st_ctrl.hard_miss,
            (long long)st_ctrl.worst_exec_us,
            (long long)st_ctrl.worst_latency_us,
            (long long)st_ctrl.worst_response_us);

        ESP_LOGI(TAG,
            "SORT: rel=%u fin=%u hard=%u Cmax=%lldus Lmax=%lldus Rmax=%lldus",
            st_sort.releases, st_sort.finishes, st_sort.hard_miss,
            (long long)st_sort.worst_exec_us,
            (long long)st_sort.worst_latency_us,
            (long long)st_sort.worst_response_us);

        ESP_LOGI(TAG,
            "SAFE: rel=%u fin=%u hard=%u Cmax=%lldus Lmax=%lldus Rmax=%lldus",
            st_safe.releases, st_safe.finishes, st_safe.hard_miss,
            (long long)st_safe.worst_exec_us,
            (long long)st_safe.worst_latency_us,
            (long long)st_safe.worst_response_us);

        ESP_LOGI(TAG, 
            "HMI: rel=%u fin=%u soft=%u Cmax=%lldus Lmax=%lldus Rmax=%lldus",
            st_hmi.releases, st_hmi.finishes, st_hmi.soft_miss,
            (long long)st_hmi.worst_exec_us, 
            (long long)st_hmi.worst_latency_us, 
            (long long)st_hmi.worst_response_us);

        // %CPU simples via Idle Hook (se habilitado)
        #if configUSE_IDLE_HOOK
        {
            int64_t now = esp_timer_get_time();
            if (!last_sample) { last_sample = now; last_idle = idle_us_acc; }
            else {
                int64_t dt = now - last_sample;
                int64_t didle = idle_us_acc - last_idle;
                float cpu = 100.0f * (1.0f - (float)didle/(float)dt);
                ESP_LOGI(TAG, "CPU Util (IdleHook): %.1f%% (janela %.0f ms)", cpu, dt/1000.0f);
                last_sample = now; last_idle = idle_us_acc;
            }
        }
        #endif

        // Runtime stats do FreeRTOS (se habilitado no menuconfig)
        #if (configUSE_TRACE_FACILITY==1) && (configGENERATE_RUN_TIME_STATS==1)
            print_runtime_stats();
        #endif
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

    ESP_LOGI(TAG, "Sistema iniciado (polling de touch + UART + métricas RT)");
}
