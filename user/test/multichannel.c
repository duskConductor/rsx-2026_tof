// multi_vl53l8cx.c
// Compile e.g.:
//   gcc multi_vl53l8cx.c platform.c vl53l8cx_api.c -lgpiod -o multi_vl53

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>

#include <gpiod.h>
#include <signal.h>

#include "vl53l8cx_api.h"
#include "platform.h"

#define NUM_SENSORS 4
#define STATUS_LED_BCM 9

static volatile int g_stop = 0;


static const int LPN_BCM_PINS[NUM_SENSORS] = {5, 6, 7, 8};
static const uint8_t I2C_ADDRESSES[NUM_SENSORS] = {0x29, 0x2A, 0x2B, 0x2C};
static const char *SENSOR_NAMES[NUM_SENSORS] = {"Zwicky", "Aristotle", "Brahe", "Copernicus"};


static struct gpiod_chip *gpio_chip = NULL;
static struct gpiod_line *lp_lines[NUM_SENSORS] = {0};
static struct gpiod_line *status_led_line = NULL;

typedef struct {
    VL53L8CX_Configuration dev;
    uint8_t i2c_addr7;
    int lp_index;
    const char *name;
} Vl53l8cxSensor;

static Vl53l8cxSensor sensors[NUM_SENSORS];

void sigint_handler (int sig) {
    (void)sig;
    g_stop = 1;
}

static int gpio_init_outputs(void)
{
    gpio_chip = gpiod_chip_open("/dev/gpiochip0");
    if (!gpio_chip) {
        perror("gpiod_chip_open");
        return -1;
    }

    for (int i = 0; i < NUM_SENSORS; ++i) {
        lp_lines[i] = gpiod_chip_get_line(gpio_chip, LPN_BCM_PINS[i]);
        if (!lp_lines[i]) {
            fprintf(stderr, "Failed to get line for BCM %d\n", LPN_BCM_PINS[i]);
            return -1;
        }
        if (gpiod_line_request_output(lp_lines[i], "vl53l8cx_lp", 0) < 0) {
            fprintf(stderr, "Failed to request output for BCM %d\n", LPN_BCM_PINS[i]);
            return -1;
        }
    }

    // NOTE: LED
    status_led_line = gpiod_chip_get_line(gpio_chip, STATUS_LED_BCM);

    if (!status_led_line) {
        fprintf(stderr, "Failed to get line for status LED BCM %d\n", STATUS_LED_BCM);
        return -1;
    }

    if (gpiod_line_request_output(status_led_line, "vl53l8cx_status_led", 0) < 0) {
        fprintf(stderr, "Failed to request output for status LED BCM %d\n", STATUS_LED_BCM);
        return -1;
    }

    return 0;
}

static void gpio_set_lp(int sensor_idx, int value)
{
    if (sensor_idx < 0 || sensor_idx >= NUM_SENSORS) return;
    if (!lp_lines[sensor_idx]) return;

    if (gpiod_line_set_value(lp_lines[sensor_idx], value ? 1 : 0) < 0) {
        fprintf(stderr, "Failed to set LP line %d to %d\n", sensor_idx, value);
    }
}

static void set_status_led(int on)
{
    if (!status_led_line) return;
    gpiod_line_set_value(status_led_line, on ? 1 : 0);
}

static void gpio_cleanup(void)
{
    if (status_led_line){
        gpiod_line_set_value(status_led_line, 0);
        gpiod_line_release(status_led_line);
        status_led_line = NULL;
    }

    for (int i = 0; i < NUM_SENSORS; ++i) {
        if (lp_lines[i]) {
            gpiod_line_release(lp_lines[i]);
            lp_lines[i] = NULL;
        }
    }
    
    if (gpio_chip) {
        gpiod_chip_close(gpio_chip);
        gpio_chip = NULL;
    }
}

static void stop_all_sensors(void){
    for (int i = 0; i < NUM_SENSORS; ++i) {
        vl53l8cx_stop_ranging(&sensors[i].dev);
        gpio_set_lp(i, 0);
    }
    set_status_led(0);
    usleep(20000);
}

static int init_one_sensor(Vl53l8cxSensor *s, uint8_t new_addr7)
{
    int status;

    gpio_set_lp(s->lp_index, 1);
    usleep(20000); 

    status = vl53l8cx_comms_init(&s->dev.platform);
    if (status != 0) {
        printf("[%s] comms_init failed: %d\n", s->name, status);
        return status;
    }

    status = vl53l8cx_init(&s->dev);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] vl53l8cx_init failed: %d\n", s->name, status);
        return status;
    }

    status = vl53l8cx_set_i2c_address(&s->dev, (uint16_t)(new_addr7 << 1));
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] set_i2c_address failed: %d\n", s->name, status);
        return status;
    }


    s->dev.platform.address = (uint16_t)(new_addr7 << 1);
    s->i2c_addr7 = new_addr7;

    status = vl53l8cx_set_resolution(&s->dev, VL53L8CX_RESOLUTION_8X8);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] set_resolution failed: %d\n", s->name, status);
        return status;
    }

    status = vl53l8cx_set_ranging_frequency_hz(&s->dev, 15);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] set_ranging_frequency failed: %d\n", s->name, status);
        return status;
    }

    status = vl53l8cx_start_ranging(&s->dev);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] start_ranging failed: %d\n", s->name, status);
        return status;
    }

    printf("[%s] initialised at 0x%02X\n", s->name, s->i2c_addr7);
    set_status_led(1);
    return VL53L8CX_STATUS_OK;
}

static int setup_all_sensors(void)
{
    int status;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        sensors[i].lp_index = i;
        sensors[i].name = SENSOR_NAMES[i];
    }

    if (gpio_init_outputs() != 0) {
        fprintf(stderr, "GPIO init failed\n");
        return -1;
    }

    for (int i = 0; i < NUM_SENSORS; ++i) {
        gpio_set_lp(i, 0);
    }
    usleep(20000);

    for (int i = 0; i < NUM_SENSORS; ++i) {
        status = init_one_sensor(&sensors[i], I2C_ADDRESSES[i]);
        if (status != VL53L8CX_STATUS_OK) {
            printf("Sensor %d (%s) init failed, status=%d\n",
                   i, sensors[i].name, status);
            return status;
        }
    }

    return VL53L8CX_STATUS_OK;
}

static void poll_all_sensors(void)
{
    VL53L8CX_ResultsData results;
    uint8_t is_ready;
    int status;

    for (int i = 0; i < NUM_SENSORS; ++i) {
        status = vl53l8cx_check_data_ready(&sensors[i].dev, &is_ready);
        if (status == VL53L8CX_STATUS_OK && is_ready) {
            status = vl53l8cx_get_ranging_data(&sensors[i].dev, &results);
            if (status == VL53L8CX_STATUS_OK) {
                printf("[%s,0x%02X]\n",
                        sensors[i].name,
                        sensors[i].i2c_addr7);

                for (int y = 0; y < 8; ++y) {
                    for (int x = 0; x < 8; ++x) {
                        int idx = y * 8 + x;
                        int16_t mm = results.distance_mm[idx];
                        printf("%5d ", mm);
                    }
                    printf("\n");
                }
                printf("\n");
                fflush(stdout);
            } else {
                printf("[%s] get_ranging_data failed: %d\n",
                        sensors[i].name, status);
            }
        }
    }
}

int main(void)
{
    signal(SIGINT, sigint_handler);

    int status = setup_all_sensors();
    if (status != VL53L8CX_STATUS_OK) {
        fprintf(stderr, "Failed to set up VL53L8CX sensors (status %d)\n", status);
        gpio_cleanup();
        return 1;
    }

    while (!g_stop) {
        poll_all_sensors();
        usleep(10000);
    }
    
    stop_all_sensors();
    gpio_cleanup();
    return 0;
}
