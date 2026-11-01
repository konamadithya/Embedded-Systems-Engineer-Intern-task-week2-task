/*
 * Project: ESP32 Smart Indoor Environment Monitor - Firmware Simulation
 * Task: Week 2 - Firmware Development and Implementation
 *
 * Host-simulated C firmware demonstrating:
 *   - A 1 ms timer tick interrupt model
 *   - Non-blocking periodic task scheduling
 *   - Simulated sensor acquisition (temperature, humidity, gas resistance)
 *   - PIR motion input simulation
 *   - Threshold and hysteresis alert handling
 *   - UART-style command interface using stdin/stdout
 *   - LED/buzzer output simulation
 *
 * Build (GCC):
 *   gcc -std=c11 -Wall -Wextra -Wpedantic -O2 src/main.c -o firmware_sim
 *
 * Run:
 *   ./firmware_sim
 *
 * Commands:
 *   HELP                 Show commands
 *   STATUS               Print latest readings and output states
 *   MOTION ON|OFF        Simulate PIR motion input
 *   TEMP <value>         Set simulated temperature in deg C
 *   HUM <value>          Set simulated humidity in percent
 *   GAS <value>          Set gas resistance in ohms
 *   QUIT                 Exit simulation
 *
 * Note:
 *   This is a portable host simulation. On a physical ESP32, replace
 *   timer_tick_isr(), uart_poll(), sensor_read_simulated() and
 *   output_update() with ESP-IDF/Arduino peripheral APIs. The ISR must
 *   remain short and should only update a tick/counter or notify a task.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#define SENSOR_PERIOD_MS   5000U
#define DISPLAY_PERIOD_MS  2000U
#define TELEMETRY_PERIOD_MS 30000U
#define ALERT_TEMP_C       30.0f
#define ALERT_HUMIDITY_PCT 70.0f
#define CLEAR_TEMP_C       28.0f
#define CLEAR_HUMIDITY_PCT 65.0f

typedef struct {
    float temperature_c;
    float humidity_pct;
    uint32_t gas_resistance_ohm;
    bool motion;
    bool valid;
} sensor_data_t;

typedef struct {
    bool led_on;
    bool buzzer_on;
    bool alarm_latched;
} output_state_t;

static volatile uint32_t g_system_ms = 0U;
static sensor_data_t g_sensor = {26.0f, 50.0f, 15000U, false, true};
static output_state_t g_output = {false, false, false};
static uint32_t g_last_sensor_ms = 0U;
static uint32_t g_last_display_ms = 0U;
static uint32_t g_last_telemetry_ms = 0U;
static bool g_running = true;

/* Simulated hardware timer interrupt: called once per millisecond.
 * On target hardware this function would be registered as the timer ISR.
 * Keep real interrupt handlers short; do not print or perform network I/O.
 */
static void timer_tick_isr(void)
{
    g_system_ms++;
}

/* Simulated sensor driver. A real implementation would read I2C devices
 * such as SHT31/BME680 and return a validity/error status.
 */
static bool sensor_read_simulated(sensor_data_t *data)
{
    if (data == NULL) {
        return false;
    }
    /* The current values are controlled through UART-style test commands. */
    data->valid = true;
    return true;
}

/* Threshold logic with hysteresis avoids rapid LED/buzzer toggling near
 * a boundary. The alarm latches until all clear thresholds are satisfied.
 */
static void alert_process(const sensor_data_t *data, output_state_t *out)
{
    bool high_condition;

    if ((data == NULL) || (out == NULL) || !data->valid) {
        out->led_on = true;       /* LED indicates a sensor fault */
        out->buzzer_on = false;   /* Avoid repeated audible fault alarms */
        return;
    }

    high_condition = (data->temperature_c >= ALERT_TEMP_C) ||
                     (data->humidity_pct >= ALERT_HUMIDITY_PCT);

    if (high_condition) {
        out->alarm_latched = true;
    } else if ((data->temperature_c <= CLEAR_TEMP_C) &&
               (data->humidity_pct <= CLEAR_HUMIDITY_PCT)) {
        out->alarm_latched = false;
    }

    out->led_on = out->alarm_latched || data->motion;
    out->buzzer_on = out->alarm_latched;
}

/* UART output helpers: stdout represents a serial terminal. */
static void uart_send(const char *message)
{
    if (message != NULL) {
        printf("%s", message);
    }
}

static void print_status(void)
{
    printf("\n--- SYSTEM STATUS ---\n");
    printf("Uptime             : %lu ms\n", (unsigned long)g_system_ms);
    printf("Temperature        : %.1f C\n", g_sensor.temperature_c);
    printf("Humidity           : %.1f %%RH\n", g_sensor.humidity_pct);
    printf("Gas resistance     : %lu ohm (indicative only)\n",
           (unsigned long)g_sensor.gas_resistance_ohm);
    printf("Motion             : %s\n", g_sensor.motion ? "DETECTED" : "CLEAR");
    printf("Sensor status      : %s\n", g_sensor.valid ? "VALID" : "FAULT");
    printf("LED                : %s\n", g_output.led_on ? "ON" : "OFF");
    printf("Buzzer             : %s\n", g_output.buzzer_on ? "ON" : "OFF");
    printf("Alarm latched      : %s\n", g_output.alarm_latched ? "YES" : "NO");
    printf("---------------------\n\n");
}

static void show_help(void)
{
    uart_send("\nCommands:\n"
              "  HELP              Show this help\n"
              "  STATUS            Show latest sensor/output status\n"
              "  MOTION ON|OFF     Simulate PIR input\n"
              "  TEMP <value>      Set temperature in deg C (0..60)\n"
              "  HUM <value>       Set humidity in percent (0..100)\n"
              "  GAS <value>       Set gas resistance in ohms (1..1000000)\n"
              "  QUIT              Exit\n\n");
}

/* Parse user input as if it arrived through a UART receive buffer.
 * Commands are deliberately line-based for easy host testing.
 */
static bool string_equals_ignore_case(const char *a, const char *b)
{
    if ((a == NULL) || (b == NULL)) return false;
    while ((*a != '\0') && (*b != '\0')) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return false;
        a++; b++;
    }
    return (*a == '\0') && (*b == '\0');
}

static void uart_poll(void)
{
    char line[128];
    char command[24] = {0};
    char argument[64] = {0};
    float f_value;
    unsigned long ul_value;
    int fields;

    printf("UART> ");
    if (fgets(line, sizeof(line), stdin) == NULL) {
        g_running = false;
        return;
    }

    fields = sscanf(line, "%23s %63s", command, argument);
    if (fields <= 0) {
        return;
    }

    for (size_t i = 0U; command[i] != '\0'; i++) {
        command[i] = (char)toupper((unsigned char)command[i]);
    }

    if (strcmp(command, "HELP") == 0) {
        show_help();
    } else if (strcmp(command, "STATUS") == 0) {
        print_status();
    } else if (strcmp(command, "QUIT") == 0) {
        g_running = false;
    } else if ((strcmp(command, "MOTION") == 0) && (fields == 2)) {
        if (string_equals_ignore_case(argument, "ON") == 0) {
            g_sensor.motion = true;
            uart_send("PIR input set: motion detected.\n");
        } else if (string_equals_ignore_case(argument, "OFF") == 0) {
            g_sensor.motion = false;
            uart_send("PIR input set: no motion.\n");
        } else {
            uart_send("Invalid MOTION value. Use ON or OFF.\n");
        }
    } else if ((strcmp(command, "TEMP") == 0) && (fields == 2)) {
        char *end = NULL;
        errno = 0;
        f_value = strtof(argument, &end);
        if ((errno == 0) && (end != argument) && (*end == '\0') &&
            (f_value >= 0.0f) && (f_value <= 60.0f)) {
            g_sensor.temperature_c = f_value;
            uart_send("Temperature updated.\n");
        } else {
            uart_send("Invalid temperature. Enter 0..60 C.\n");
        }
    } else if ((strcmp(command, "HUM") == 0) && (fields == 2)) {
        char *end = NULL;
        errno = 0;
        f_value = strtof(argument, &end);
        if ((errno == 0) && (end != argument) && (*end == '\0') &&
            (f_value >= 0.0f) && (f_value <= 100.0f)) {
            g_sensor.humidity_pct = f_value;
            uart_send("Humidity updated.\n");
        } else {
            uart_send("Invalid humidity. Enter 0..100 percent.\n");
        }
    } else if ((strcmp(command, "GAS") == 0) && (fields == 2)) {
        char *end = NULL;
        errno = 0;
        ul_value = strtoul(argument, &end, 10);
        if ((errno == 0) && (end != argument) && (*end == '\0') &&
            (ul_value >= 1UL) && (ul_value <= 1000000UL)) {
            g_sensor.gas_resistance_ohm = (uint32_t)ul_value;
            uart_send("Gas resistance updated.\n");
        } else {
            uart_send("Invalid gas value. Enter 1..1000000 ohm.\n");
        }
    } else {
        uart_send("Unknown command. Type HELP.\n");
    }
}

/* These functions stand in for periodic RTOS tasks. The simulation
 * advances logical time in 1 ms increments between terminal commands.
 */
static void scheduler_run_due_tasks(void)
{
    uint32_t now = g_system_ms;

    if ((uint32_t)(now - g_last_sensor_ms) >= SENSOR_PERIOD_MS) {
        if (!sensor_read_simulated(&g_sensor)) {
            g_sensor.valid = false;
        }
        g_last_sensor_ms = now;
        alert_process(&g_sensor, &g_output);
        printf("[SENSOR] T=%.1f C, RH=%.1f %%RH, GAS=%lu ohm\n",
               g_sensor.temperature_c, g_sensor.humidity_pct,
               (unsigned long)g_sensor.gas_resistance_ohm);
    }

    if ((uint32_t)(now - g_last_display_ms) >= DISPLAY_PERIOD_MS) {
        g_last_display_ms = now;
        printf("[DISPLAY] %.1f C | %.1f %%RH | Motion:%s | LED:%s\n",
               g_sensor.temperature_c, g_sensor.humidity_pct,
               g_sensor.motion ? "YES" : "NO",
               g_output.led_on ? "ON" : "OFF");
    }

    if ((uint32_t)(now - g_last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
        g_last_telemetry_ms = now;
        printf("[TELEMETRY] {\"temperature_c\":%.1f,\"humidity_rh\":%.1f,"
               "\"gas_resistance_ohm\":%lu,\"motion\":%s}\n",
               g_sensor.temperature_c, g_sensor.humidity_pct,
               (unsigned long)g_sensor.gas_resistance_ohm,
               g_sensor.motion ? "true" : "false");
    }
}

/* Host simulation time step. One iteration represents 1 ms of device time.
 * A real MCU receives this tick from a hardware timer peripheral.
 */
static void simulation_advance_ms(uint32_t milliseconds)
{
    for (uint32_t i = 0U; i < milliseconds; i++) {
        timer_tick_isr();
        scheduler_run_due_tasks();
    }
}

int main(void)
{
    uart_send("ESP32 Smart Environment Monitor - Firmware Simulation\n");
    uart_send("Type HELP for available commands.\n");

    show_help();

    while (g_running) {
        /* Simulate a small interval of elapsed time before polling UART. */
        simulation_advance_ms(1000U);
        alert_process(&g_sensor, &g_output);
        uart_poll();
    }

    uart_send("Firmware simulation stopped.\n");
    return EXIT_SUCCESS;
}
