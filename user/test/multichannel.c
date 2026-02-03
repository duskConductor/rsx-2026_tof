#include <stdio.h>
#include <unistd.h>
#include "vl53l8cx_api.h"
#include "platform.h"

#define NUM_SENSORS 4

const uint8_t I2C_ADDRESSES[NUM_SENSORS] = {0x29, 0x2A, 0x2B, 0x2C};
const char *SENSOR_NAMES[NUM_SENSORS] = {"Zwicky", "Aristotle", "Brahe", "Copernicus"};

typedef struct {
    VL53L8CX_Configuration dev;
    uint8_t i2c_addr7;
    int lp_index;   // 0..3, index into LPN_BCM_PINS
} Vl53l8cxSensor;

static Vl53l8cxSensor sensors[NUM_SENSORS];

/* Forward decls for GPIO helpers from above */
int  gpio_init_outputs(void);
void gpio_set_lp(int sensor_idx, int value);
void gpio_cleanup(void);

int init_one_sensor(Vl53l8cxSensor *s, uint8_t new_addr7)
{
    int status;

    /* Bring only this sensor out of reset (LP high). */
    gpio_set_lp(s->lp_index, 1);
    usleep(20000);  // 20 ms boot

    /* Initialise comms; platform.c opens /dev/i2c-1 and sets address = 0x52. */
    status = vl53l8cx_comms_init(&s->dev.platform);
    if (status != 0) {
        printf("comms_init failed: %d\n", status);
        return status;
    }

    status = vl53l8cx_init(&s->dev);
    if (status != VL53L8CX_STATUS_OK) {
        printf("vl53l8cx_init failed: %d\n", status);
        return status;
    }

    /* Change I2C address for this instance. */
    status = vl53l8cx_set_i2c_address(&s->dev, (uint16_t)(new_addr7 << 1));
    if (status != VL53L8CX_STATUS_OK) {
        printf("set_i2c_address failed: %d\n", status);
        return status;
    }

    s->dev.platform.address = (uint16_t)(new_addr7 << 1);
    s->i2c_addr7 = new_addr7;

    status = vl53l8cx_set_resolution(&s->dev, VL53L8CX_RESOLUTION_8X8);
    if (status != VL53L8CX_STATUS_OK) return status;

    status = vl53l8cx_set_ranging_frequency_hz(&s->dev, 15);
    if (status != VL53L8CX_STATUS_OK) return status;

    status = vl53l8cx_start_ranging(&s->dev);
    if (status != VL53L8CX_STATUS_OK) {
        printf("start_ranging failed: %d\n", status);
        return status;
    }

    return VL53L8CX_STATUS_OK;
}

int setup_all_sensors(void)
{
    int status;

    /* Map each sensor to an LP line index. */
    for (int i = 0; i < NUM_SENSORS; ++i) {
        sensors[i].lp_index = i;
    }

    if (gpio_init_outputs() != 0) {
        fprintf(stderr, "GPIO init failed\n");
        return -1;
    }

    /* Hold all in reset. */
    for (int i = 0; i < NUM_SENSORS; ++i) {
        gpio_set_lp(i, 0);
    }
    usleep(10000);

    /* Bring up one at a time, assign addresses from I2C_ADDRESSES. */
    for (int i = 0; i < NUM_SENSORS; ++i) {
        status = init_one_sensor(&sensors[i], I2C_ADDRESSES[i]);
        if (status != VL53L8CX_STATUS_OK) {
            printf("Sensor %d (%s) init failed, status %d\n", i, SENSOR_NAMES[i], status);
            return status;
        }
    }

    return VL53L8CX_STATUS_OK;
}

void poll_all_sensors(void)
{
    VL53L8CX_ResultsData results;
    uint8_t is_ready;
    int status;

    while (1) {
        for (int i = 0; i < NUM_SENSORS; ++i) {
            status = vl53l8cx_check_data_ready(&sensors[i].dev, &is_ready);
            if (status == VL53L8CX_STATUS_OK && is_ready) {
                status = vl53l8cx_get_ranging_data(&sensors[i].dev, &results);
                if (status == VL53L8CX_STATUS_OK) {
                    int16_t mm = results.distance_mm[32]; // pick one zone
                    printf("%s (%#02x): %d mm\n",
                           SENSOR_NAMES[i], sensors[i].i2c_addr7, mm);
                }
            }
        }
        usleep(10000);
    }
}

int main(void)
{
    if (setup_all_sensors() != VL53L8CX_STATUS_OK) {
        fprintf(stderr, "Failed to set up sensors\n");
        gpio_cleanup();
        return 1;
    }

    poll_all_sensors();

    gpio_cleanup();
    return 0;
}