// multi_vl53l8cx.c
// Compile e.g.:
//   gcc multi_vl53l8cx.c platform.c vl53l8cx_api.c -lgpiod -o multi_vl53

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>

#include <gpiod.h>

#include "vl53l8cx_api.h"
#include "platform.h"

#define NUM_SENSORS 4

// BCM GPIOs you provided (LP pins for each Pololu board)
static const int LPN_BCM_PINS[NUM_SENSORS] = {5, 6, 7, 8};

// Target 7-bit I2C addresses for each sensor
static const uint8_t I2C_ADDRESSES[NUM_SENSORS] = {0x29, 0x2A, 0x2B, 0x2C};

// Names for logging / CSV header
static const char *SENSOR_NAMES[NUM_SENSORS] = {"Zwicky", "Aristotle", "Brahe", "Copernicus"};

// libgpiod objects
static struct gpiod_chip *gpio_chip = NULL;
static struct gpiod_line *lp_lines[NUM_SENSORS] = {0};

typedef struct {
    VL53L8CX_Configuration dev;  // ST driver configuration struct
    uint8_t i2c_addr7;           // 7-bit I2C address (0x29..0x2C)
    int lp_index;                // index into LPN_BCM_PINS / lp_lines
    const char *name;            // sensor name
} Vl53l8cxSensor;

static Vl53l8cxSensor sensors[NUM_SENSORS];

// ------------ GPIO (LP) helpers using libgpiod ------------

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

static void gpio_cleanup(void)
{
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

// ------------ VL53L8CX per-sensor init / usage ------------

static int init_one_sensor(Vl53l8cxSensor *s, uint8_t new_addr7)
{
    int status;

    // Bring this sensor out of LP reset
    gpio_set_lp(s->lp_index, 1);
    usleep(20000); // ~20 ms boot

    // Initialise comms for this instance (platform.c opens /dev/i2c-1
    // and sets platform.address = 0x52, the default 8-bit address).
    status = vl53l8cx_comms_init(&s->dev.platform);
    if (status != 0) {
        printf("[%s] comms_init failed: %d\n", s->name, status);
        return status;
    }

    // Initialise the sensor: firmware, basic checks, etc.
    status = vl53l8cx_init(&s->dev);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] vl53l8cx_init failed: %d\n", s->name, status);
        return status;
    }

    // Assign a new I2C address (API takes 8-bit address).
    status = vl53l8cx_set_i2c_address(&s->dev, (uint16_t)(new_addr7 << 1));
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] set_i2c_address failed: %d\n", s->name, status);
        return status;
    }

    // Update platform address so platform.c uses the new address.
    s->dev.platform.address = (uint16_t)(new_addr7 << 1);
    s->i2c_addr7 = new_addr7;

    // Example configuration: 8x8 resolution, 15 Hz.
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
    return VL53L8CX_STATUS_OK;
}

static int setup_all_sensors(void)
{
    int status;

    // Attach names and lp_index
    for (int i = 0; i < NUM_SENSORS; ++i) {
        sensors[i].lp_index = i;
        sensors[i].name = SENSOR_NAMES[i];
    }

    if (gpio_init_outputs() != 0) {
        fprintf(stderr, "GPIO init failed\n");
        return -1;
    }

    // Hold all sensors in LP low (reset / I2C disabled)
    for (int i = 0; i < NUM_SENSORS; ++i) {
        gpio_set_lp(i, 0);
    }
    usleep(10000);

    // Bring up sensors one by one, assigning unique addresses
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

    // Simple CSV header using your SENSOR_NAMES
    printf("name,address_hex,distance_mm\n");

    while (1) {
        for (int i = 0; i < NUM_SENSORS; ++i) {
            status = vl53l8cx_check_data_ready(&sensors[i].dev, &is_ready);
            if (status == VL53L8CX_STATUS_OK && is_ready) {
                status = vl53l8cx_get_ranging_data(&sensors[i].dev, &results);
                if (status == VL53L8CX_STATUS_OK) {
                    // Example: just use zone 32 (rough center) for a quick demo.
                    int16_t mm = results.distance_mm[32];
                    printf("%s,0x%02X,%d\n",
                           sensors[i].name,
                           sensors[i].i2c_addr7,
                           mm);
                    fflush(stdout);
                } else {
                    printf("[%s] get_ranging_data failed: %d\n",
                           sensors[i].name, status);
                }
            }
        }
        usleep(10000); // 10 ms
    }
}

// ------------ main ------------

int main(void)
{
    int status = setup_all_sensors();
    if (status != VL53L8CX_STATUS_OK) {
        fprintf(stderr, "Failed to set up VL53L8CX sensors (status %d)\n", status);
        gpio_cleanup();
        return 1;
    }

    poll_all_sensors();  // never returns in this simple example

    gpio_cleanup();
    return 0;
}
