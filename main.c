#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/util/queue.h"
#include "pico/time.h"

// The pin definitions ₍^. .^₎⟆
#define STEPPER_IN1 2
#define STEPPER_IN2 3
#define STEPPER_IN3 6
#define STEPPER_IN4 13
#define OPTO_FORK 28
#define PIEZO_SENSOR 27
#define LED_PIN 20
#define BUTTON_CALIB 9   // sw_0
#define BUTTON_START 7   // sw_2
#define BUTTON_STOP 12   // rotary switch

// The constants ₍^. .^₎⟆
#define EVENT_OPTO_EDGE 1
#define EVENT_PIEZO_EDGE 2
#define EVENT_BUTTON_PRESS 3
#define PIEZO_DETECTION_TIMEOUT_MS 5000
#define PIEZO_DEBOUNCE_MS 100
#define DISPENSE_INTERVAL_MS 30000
#define MOTOR_STEP_DELAY_MS 2
#define CALIBRATION_TIMEOUT_MS 30000
#define COMPARTMENT_STEPS 512
#define FULL_ROTATION_STEPS 4096
#define IDLE_LED_BLINK_INTERVAL_MS 500

typedef struct {
    uint8_t type;
    uint32_t value;
} event_t;

const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

typedef struct {
    bool calibrated;
    int current_pill_position;
    bool dispensing;
    bool pill_detected;
    bool stop_requested;
} system_state_t;

static queue_t event_queue;
static system_state_t state = {0};
static volatile bool piezo_triggered = false;
static volatile uint32_t last_piezo_time = 0;

void hardware_funk();
void set_motor_step(int step_index);
void single_step(int direction);
void calibrate_motor();
void blink_led(int count, int duration_ms);
bool dispense_pill();
void gpio_callback(uint gpio, uint32_t events);
void stop_motor();
bool wait_for_next_dispense();
void check_events();
void idle_led_blink();

void hardware_funk() {
    stdio_init_all();
    sleep_ms(1000);

    gpio_init(STEPPER_IN1); gpio_set_dir(STEPPER_IN1, GPIO_OUT);
    gpio_init(STEPPER_IN2); gpio_set_dir(STEPPER_IN2, GPIO_OUT);
    gpio_init(STEPPER_IN3); gpio_set_dir(STEPPER_IN3, GPIO_OUT);
    gpio_init(STEPPER_IN4); gpio_set_dir(STEPPER_IN4, GPIO_OUT);
    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(OPTO_FORK); gpio_set_dir(OPTO_FORK, GPIO_IN); gpio_pull_up(OPTO_FORK);
    gpio_init(PIEZO_SENSOR); gpio_set_dir(PIEZO_SENSOR, GPIO_IN); gpio_pull_up(PIEZO_SENSOR);
    gpio_init(BUTTON_CALIB); gpio_set_dir(BUTTON_CALIB, GPIO_IN); gpio_pull_up(BUTTON_CALIB);
    gpio_init(BUTTON_START); gpio_set_dir(BUTTON_START, GPIO_IN); gpio_pull_up(BUTTON_START);
    gpio_init(BUTTON_STOP); gpio_set_dir(BUTTON_STOP, GPIO_IN); gpio_pull_up(BUTTON_STOP);

    gpio_set_irq_enabled_with_callback(OPTO_FORK, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(PIEZO_SENSOR, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON_CALIB, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON_START, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON_STOP, GPIO_IRQ_EDGE_FALL, true);

    queue_init(&event_queue, sizeof(event_t), 10);
    stop_motor();
}

void idle_led_blink() {
    static uint32_t last_blink_time = 0;
    static bool led_state = false;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    if (current_time - last_blink_time >= IDLE_LED_BLINK_INTERVAL_MS) {
        led_state = !led_state;
        gpio_put(LED_PIN, led_state);
        last_blink_time = current_time;
    }
}

void check_events() {
    event_t evt;
    while (queue_try_remove(&event_queue, &evt)) {
        if (evt.type == EVENT_BUTTON_PRESS && evt.value == BUTTON_STOP) {
            printf("Stop button was pressed, stopping the operation\n");
            state.stop_requested = true;
            stop_motor();
        }
    }

    if (!gpio_get(BUTTON_STOP)) {
        state.stop_requested = true;
        stop_motor();
        sleep_ms(100);
    }
}

void set_motor_step(int step_index) {
    step_index = step_index % 8;
    gpio_put(STEPPER_IN1, step_sequence[step_index][0]);
    gpio_put(STEPPER_IN2, step_sequence[step_index][1]);
    gpio_put(STEPPER_IN3, step_sequence[step_index][2]);
    gpio_put(STEPPER_IN4, step_sequence[step_index][3]);
}

void single_step(int direction) {
    static int current_step = 0;

    if (state.stop_requested) return;

    current_step = (current_step + direction + 8) % 8;
    set_motor_step(current_step);
    sleep_ms(MOTOR_STEP_DELAY_MS);

    if (current_step % 4 == 0) check_events();
}

void stop_motor() {
    gpio_put(STEPPER_IN1, 0);
    gpio_put(STEPPER_IN2, 0);
    gpio_put(STEPPER_IN3, 0);
    gpio_put(STEPPER_IN4, 0);
}

void calibrate_motor() {
    printf(" <3 Starting Calibration <3\n");
    state.calibrated = false;
    state.stop_requested = false;

    for (int i = 0; i < FULL_ROTATION_STEPS && !state.stop_requested; i++) {
        single_step(1);
    }

    int steps_to_opening = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while (gpio_get(OPTO_FORK) && steps_to_opening < FULL_ROTATION_STEPS &&
           !state.stop_requested && (to_ms_since_boot(get_absolute_time()) - start_time) < CALIBRATION_TIMEOUT_MS) {
        single_step(1);
        steps_to_opening++;
    }

    if (steps_to_opening >= FULL_ROTATION_STEPS) {
        blink_led(5, 200);
        return;
    }

    int slot_width = 0;
    while (!gpio_get(OPTO_FORK) && slot_width < COMPARTMENT_STEPS && !state.stop_requested) {
        single_step(1);
        slot_width++;
    }

    for (int i = 0; i < slot_width / 2 && !state.stop_requested; i++) {
        single_step(-1);
    }

    stop_motor();
    if (!state.stop_requested) {
        state.calibrated = true;
        state.current_pill_position = 0;
        gpio_put(LED_PIN, 1);
        blink_led(3, 200);
    }
}

bool dispense_pill() {
    if (!state.calibrated) {
        blink_led(5, 100);
        return false;
    }

    printf("Dispensing the pill...\n");
    piezo_triggered = false;
    state.pill_detected = false;

    for (int step = 0; step < COMPARTMENT_STEPS && !state.stop_requested; step++) {
        single_step(1);
        if (piezo_triggered) state.pill_detected = true;
    }

    state.current_pill_position = (state.current_pill_position + 1) % 8;

    if (!state.pill_detected) {
        uint32_t start_time = to_ms_since_boot(get_absolute_time());
        while ((to_ms_since_boot(get_absolute_time()) - start_time) < PIEZO_DETECTION_TIMEOUT_MS) {
            if (piezo_triggered) {
                state.pill_detected = true;
                break;
            }
            sleep_ms(10);
            check_events();
        }
    }

    if (!state.pill_detected) {
        printf("No pill detected! >_< \n");
        blink_led(5, 100);
        return false;
    }

    printf("Pill dispensed! <3 \n");
    return true;
}

bool wait_for_next_dispense() {
    printf("Waiting %d ms until next dispense <3 \n", DISPENSE_INTERVAL_MS);
    uint32_t start_time = to_ms_since_boot(get_absolute_time());

    while ((to_ms_since_boot(get_absolute_time()) - start_time) < DISPENSE_INTERVAL_MS) {
        sleep_ms(100);
        check_events();
        if (state.stop_requested) return false;
    }

    return true;
}

void blink_led(int count, int duration_ms) {
    for (int i = 0; i < count; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(duration_ms);
        gpio_put(LED_PIN, 0);
        sleep_ms(duration_ms);
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    event_t evt;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (gpio == PIEZO_SENSOR && (events & GPIO_IRQ_EDGE_FALL)) {
        if ((now - last_piezo_time) > PIEZO_DEBOUNCE_MS) {
            piezo_triggered = true;
            evt.type = EVENT_PIEZO_EDGE;
            evt.value = 1;
            queue_try_add(&event_queue, &evt);
            last_piezo_time = now;
        }
    } else if ((gpio == BUTTON_CALIB || gpio == BUTTON_START || gpio == BUTTON_STOP) && (events & GPIO_IRQ_EDGE_FALL)) {
        evt.type = EVENT_BUTTON_PRESS;
        evt.value = gpio;
        queue_try_add(&event_queue, &evt);
        if (gpio == BUTTON_STOP) state.stop_requested = true;
    }
}

int main() {
    hardware_funk();
    printf(" <3 Pill Dispenser Ready <3 \n");

    while (true) {
        state.stop_requested = false;
        gpio_put(LED_PIN, 0);

        printf("Waiting for the CALIBRATION button (sw_0)...\n", BUTTON_CALIB);
        while (gpio_get(BUTTON_CALIB)) {
            idle_led_blink();
            sleep_ms(10);
            check_events();
        }

        calibrate_motor();
        if (!state.calibrated) continue;

        // da LED stays ON after calibration
        gpio_put(LED_PIN, 1);
        printf("Position initialised. \n");
        printf("Waiting for START/DISPENSE button (sw_2)...\n", BUTTON_START);

        while (gpio_get(BUTTON_START)) {
            sleep_ms(100);
            check_events();
            if (state.stop_requested) break;
        }

        if (state.stop_requested) continue;

        state.dispensing = true;

        for (int i = 0; i < 7 && !state.stop_requested; i++) {
            printf("Dispensing %d/7...\n", i + 1);
            dispense_pill();
            if (i < 6) wait_for_next_dispense();
        }

        printf("Cycle is complete. <3 Restarting.\n");
    }

    return 0;
}