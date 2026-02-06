#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

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

typedef struct {
    double x, y, z;
    double yaw, pitch, roll;
} SensorPose;

static SensorPose poses[NUM_SENSORS] = {
    // Zwicky (Top)
    {0.00, 0.0684, 0.00, 0.0, 0.0, 0.0},
    // Aristotle (Top-Middle)
    {0.05, 0.0456, 0.00, 0.0, 0.0, 0.0},
    // Brahe (Middle-Bottom)
    {0.00, 0.0228, 0.00, 0.0, 0.0, 0.0},
    // Copernicus (Bottom)
    {0.05, 0.00, 0.00, 0.0, 0.0, 0.0}
};

static Vl53l8cxSensor sensors[NUM_SENSORS];
static FILE *csv = NULL;

static double now_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

void sigint_handler (int sig) {
    (void)sig;
    g_stop = 1;
}

static void write_csv_header(void) {
    fprintf(csv, "time,sensor");

    for (int z = 0; z < 64; ++z){
        fprintf(csv, ",zone%d_status,zone%d_x,zone%d_y,zone%d_z", z+1, z+1, z+1, z+1);
    }

    fprintf(csv, "\n");
}

static void write_pointcloud(int sensor_idx, const VL53L8CX_ResultsData *results) {
    const double FOV_DEG = 45.0;
    const double HALF_FOV = FOV_DEG / 2.0;
    const double STEP = FOV_DEG / 8.0;
    const double DEG_TO_RADIANS = M_PI / 180.0;

    double t = now_seconds();
    const Vl53l8cxSensor *s = &sensors[sensor_idx];
    const SensorPose *pose = &poses[sensor_idx];

    fprintf(csv, "%.9f,%s", t, s->name);

    for (int idx = 0; idx < 64; ++idx) {
        int16_t dist_mm = results->distance_mm[idx];
        int status_zone = (dist_mm > 0) ? 1 : 0;

        double xw = 0.0, yw = 0.0, zw = 0.0;

        if (status_zone) {
            int row = idx / 8;
            int col = idx % 8;

            double theta_x = (-HALF_FOV + STEP/2.0) + col * STEP;
            double theta_y = ( HALF_FOV - STEP/2.0) - row * STEP;

            double tx = theta_x * DEG_TO_RADIANS;
            double ty = theta_y * DEG_TO_RADIANS;

            double dist_m = 0.001 * dist_mm;
            double z = dist_m;
            double x = z * tan(tx);
            double y = z * tan(ty);

            // This is just translations
            // I still need to add the rotation
            xw = pose->x + x;
            yw = pose->y + y;
            zw = pose->z + z;
        }

        fprintf(csv, ",%d,%f,%f,%f", status_zone, xw, yw, zw);
    }

    fprintf(csv, "\n");
    fflush(csv);
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

    // gpio_set_lp(s->lp_index, 1);
    // usleep(20000); 
    printf("[%s] About to init at default 0x52 (7-bit 0x29), LP index=%d\n",
       s->name, s->lp_index);

    status = vl53l8cx_comms_init(&s->dev.platform);
    printf("[%s] comms_init returned %d (platform.address=0x%02X)\n", s->name, status, s->dev.platform.address);

    if (status != 0) {
        printf("[%s] comms_init failed: %d\n", s->name, status);
        return status;
    }

    status = vl53l8cx_init(&s->dev);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] vl53l8cx_init failed: %d\n", s->name, status);
        vl53l8cx_comms_close(&s->dev.platform);
        return status;
    }

    printf("[%s] Init OK, changing address to 0x%02X...\n", s->name, new_addr7);
    status = vl53l8cx_set_i2c_address(&s->dev, (uint16_t)(new_addr7 << 1));
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] set_i2c_address failed: %d\n", s->name, status);
        vl53l8cx_comms_close(&s->dev.platform);
        return status;
    }


    s->dev.platform.address = (uint16_t)(new_addr7 << 1);
    s->i2c_addr7 = new_addr7;

    status = vl53l8cx_set_resolution(&s->dev, VL53L8CX_RESOLUTION_8X8);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] set_resolution failed: %d\n", s->name, status);
        vl53l8cx_comms_close(&s->dev.platform);
        return status;
    }

    status = vl53l8cx_set_ranging_frequency_hz(&s->dev, 15);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] set_ranging_frequency failed: %d\n", s->name, status);
        vl53l8cx_comms_close(&s->dev.platform);
        return status;
    }

    status = vl53l8cx_start_ranging(&s->dev);
    if (status != VL53L8CX_STATUS_OK) {
        printf("[%s] start_ranging failed: %d\n", s->name, status);
        vl53l8cx_comms_close(&s->dev.platform);
        return status;
    }

    printf("[%s] initialised at 0x%02X\n", s->name, s->i2c_addr7);
    // set_status_led(1);
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

    // Set all LP low initially
    for (int i = 0; i < NUM_SENSORS; ++i) {
        gpio_set_lp(i, 0);
    }
    usleep(20000);

    for (int i = 0; i < NUM_SENSORS; ++i) {
        // Make sure only THIS senseor is high
        for (int j = 0; j < NUM_SENSORS; ++j) {
            gpio_set_lp(j, (j == i) ? 1 : 0);
        }
        usleep(20000);
        
        status = init_one_sensor(&sensors[i], I2C_ADDRESSES[i]);
        if (status != VL53L8CX_STATUS_OK) {
            printf("Sensor %d (%s) init failed, status=%d\n",
                   i, sensors[i].name, status);
            return status;
        }

        printf("Sensor %d (%s) OK at 0x%02X\n", i, sensors[i].name, sensors[i].i2c_addr7);
    }

    for (int i = 0; i < NUM_SENSORS; ++i) {
        gpio_set_lp(i,1);
    }

    set_status_led(1);

    return VL53L8CX_STATUS_OK;
}

static void poll_all_sensors(void)
{
    VL53L8CX_ResultsData results;
    uint8_t is_ready;
    int status;

    static int frame_count[NUM_SENSORS] = {0};

    for (int i = 0; i < NUM_SENSORS; ++i) {
        status = vl53l8cx_check_data_ready(&sensors[i].dev, &is_ready);
        
        if (status == VL53L8CX_STATUS_OK && is_ready) {
            status = vl53l8cx_get_ranging_data(&sensors[i].dev, &results);
            
            if (status == VL53L8CX_STATUS_OK) {
                frame_count[i]++;

                printf("Frame %d from %s (total %d)\n", frame_count[i], sensors[i].name, frame_count[i]);

                write_pointcloud(i, &results);
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

    csv = fopen("raw_pointcloud.csv", "w");

    if (!csv) {
        perror("fopen csv");
        return 1;
    }
    write_csv_header();

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
    fclose(csv);
    gpio_cleanup();
    return 0;
}
