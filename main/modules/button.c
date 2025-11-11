#include "button.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "BUTTON";

typedef struct {
    gpio_num_t pin;
    int stable_level;      
    int last_level;        
    int64_t last_change_ms; 
    bool debounce_pending;  
    esp_timer_handle_t debounce_timer;  // <- Tambahan
} button_t;

static button_t buttons[MAX_BUTTONS];
static int button_count = 0;
const int debounce_time_ms = 30;

// -----------------------------------------
// Timer callback untuk debounce
// -----------------------------------------
static void debounce_timer_cb(void* arg) {
    int id = (int)arg;
    int current = gpio_get_level(buttons[id].pin);

    if (current == buttons[id].last_level) {
        buttons[id].stable_level = current;
        buttons[id].debounce_pending = false;
    } else {
        // masih berubah, restart timer
        buttons[id].last_level = current;
        esp_timer_start_once(buttons[id].debounce_timer, debounce_time_ms * 1000);
    }
}

// -----------------------------------------
// ISR untuk tombol
// -----------------------------------------
static void IRAM_ATTR button_isr_handler(void* arg) {
    int id = (int)arg;
    if (!buttons[id].debounce_pending) {
        buttons[id].debounce_pending = true;
        esp_timer_start_once(buttons[id].debounce_timer, debounce_time_ms * 1000);
    }
}

// -----------------------------------------
// Inisialisasi tombol
// -----------------------------------------
void button_init(void)
{
    button_count = 0;
    gpio_install_isr_service(0);
    ESP_LOGI(TAG, "Inisialisasi tombol full interrupt-driven");
}

void button_register(gpio_num_t pin)
{
    if (button_count >= MAX_BUTTONS) return;

    // Konfigurasi GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);

    // Pasang ISR
    gpio_isr_handler_add(pin, button_isr_handler, (void*)button_count);

    // Inisialisasi struct
    buttons[button_count].pin = pin;
    buttons[button_count].stable_level = gpio_get_level(pin);
    buttons[button_count].last_level = buttons[button_count].stable_level;
    buttons[button_count].last_change_ms = esp_timer_get_time() / 1000;
    buttons[button_count].debounce_pending = false;

    // Buat timer untuk debounce
    esp_timer_create_args_t timer_args = {
        .callback = &debounce_timer_cb,
        .arg = (void*)button_count,
        .dispatch_method = ESP_TIMER_TASK,  // <- Ganti ESP_TIMER_ISR
        .name = "debounce_timer"
    };
    esp_timer_create(&timer_args, &buttons[button_count].debounce_timer);

    button_count++;
}

// -----------------------------------------
// Fungsi membaca tombol
// -----------------------------------------
int button_count_get(void) {
    return button_count;
}

bool button_get_state(int id)
{
    if (id < 0 || id >= button_count) return -1;
    return buttons[id].stable_level;
}

// -----------------------------------------
// Fungsi toggle, hold, delay tetap sama
// -----------------------------------------
bool button_check_hold_release(int id, int hold_time_ms, int active_time_ms)
{
    if (id < 0 || id >= button_count) return true;

    static int64_t pressStart[MAX_BUTTONS] = {0};
    static bool holdDetected[MAX_BUTTONS] = {false};
    static int64_t activeStart[MAX_BUTTONS] = {0};

    int state = button_get_state(id);
    int64_t now = esp_timer_get_time() / 1000;

    if (state == 0) {
        if (pressStart[id] == 0) pressStart[id] = now;
        if ((now - pressStart[id]) >= hold_time_ms) holdDetected[id] = true;
    } else {
        if (holdDetected[id] && activeStart[id] == 0) activeStart[id] = now;
        pressStart[id] = 0;
        holdDetected[id] = false;
    }

    if (activeStart[id] > 0) {
        if ((now - activeStart[id]) <= active_time_ms)
            return false;
        else
            activeStart[id] = 0;
    }

    return true;
}

bool button_toggle_delay(int id, int active_time_ms)
{
    if (id < 0 || id >= button_count) return true;

    static bool triggered[MAX_BUTTONS] = {false};
    static int64_t activeStart[MAX_BUTTONS] = {0};

    int state = button_get_state(id);
    int64_t now = esp_timer_get_time() / 1000;
    bool output = true;

    if (state == 0) {
        if (!triggered[id]) {
            triggered[id] = true;
            activeStart[id] = now;
            output = false;
        } else if ((now - activeStart[id]) < active_time_ms) {
            output = false;
        } else {
            output = true;
        }
    } else {
        triggered[id] = false;
        activeStart[id] = 0;
        output = true;
    }

    return output;
}

bool button_toggle(int id)
{
    if (id < 0 || id >= button_count) return false;

    static bool state[MAX_BUTTONS] = {false};
    static bool pressedLast[MAX_BUTTONS] = {false};

    int current = button_get_state(id);
    if (current == 0) { 
        if (!pressedLast[id]) {
            state[id] = !state[id];
            pressedLast[id] = true;
        }
    } else { 
        pressedLast[id] = false;
    }

    return state[id];
}
