#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <gpiod.h>
#include <fcntl.h>
#include <signal.h>
#include "vl53l8cx_api.h"

/*
  Don't forget! 
  Edit /boot/config.txt
    dtparam=i2c_arm=on
    dtparam=i2c_arm_baudrate=400000
 */

typedef struct {
    float x_off, y_off, z_off;
    float yaw;
} SensorRig;

#define I2C_BUS_PATH "/dev/i2c-1"
// Temporary Parameter; won't be there for actual program
#define SCAN_DURATION_SECONDS 180
// Where we're saving the data
#define CSV_FILENAME "multichannel_lidar_scan.csv"

#define NUM_SENSORS 4
#define LED_BCM_PIN 9

// Physical Pins & Corresponding BCM Pins
// 29 (Zwicky) -> BCM 5
// 31 (Aristotle) -> BCM 6
// 26 (Brahe) -> BCM 7
// 24 (Copernicus) -> BCM 8

// NOTE: C requires the use of Broadcom pins for gpiod functions
const int LPN_BCM_PINS[] = {5, 6, 7, 8};
// I2C Addresses for the sensors
const uint8_t I2C_ADDRESSES[] = {0x29, 0x2A, 0x2B, 0x2C};
// Sensor names for CSV header
const char *SENSOR_NAMES[] = {"Zwicky", "Aristotle", "Brahe", "Copernicus"};
int sensor_fds[NUM_SENSORS];
extern int i2c_fd;

// Setting up the GPIO Lines
struct gpiod_chip *chip;
struct gpiod_line *lpn_lines[NUM_SENSORS];
struct gpiod_line *led_line;
VL53L8CX_Configuration sensors[NUM_SENSORS];

SensorRig rig[NUM_SENSORS] = {
    {0,68.7,0,0}, // Zwicky - Top
    {0,45.8,0,0}, // Aristotle - Second from top
    {0,22.9,0,0}, // Brahe - Third from top
    {0,0,0,0} // Copernicus - Bottom
};

void calculate_cartesian(int zone, uint32_t dist, float *gx, float *gy, float *gz, int s_idx) {
    if (dist == 0) {
        *gx = *gy = *gz = 0;
        return;
    }

    // NOTE: We're using an 8x8 grid
    // FOV is a 45 degree square
    // Each zone is around 5.6 degrees
    // The center of the grid is 3.5.
    float az = ((zone % 8) - 3.5f) * 5.625f * (M_PI / 180.0f);
    float el = (3.5f - (zone / 8)) * 5.625f * (M_PI / 180.0f);

    // Local coordinates (z is forward)
    float lx = dist * sinf(az);
    float ly = dist * sinf(el);
    float lz = dist * cosf(az) * cosf(el);

    // Rig Tansformation: Yaw rotation + Translation)
    float rad_yaw = rig[s_idx].yaw * (M_PI / 180.0f);
    *gx = rig[s_idx].x_off + (lx * cosf(rad_yaw) + lz * sinf(rad_yaw));
    *gy = rig[s_idx].y_off + ly;
    *gz = rig[s_idx].z_off + (-lx * sinf(rad_yaw) + lz * cosf(rad_yaw));
};

// Cleans up system to ensure nothing assigned out of place
// void cleanup();
// Used when starting the program (cleans up before assigning stuff); used by signal
// void signal_handler(int sig);
// Assigns pins out at start of program
// void setup_gpio();
// Assigns addresses to the sensors (all start with default address)
// void setup_sensors();
// Actually does the 8x8 LiDAR scan
// void run_scan();

// Ensures there's no leftover assignments
void cleanup() {
    printf("\nPerforming cleanup...\n");

    // Shut down all LPn lines and stop range detection
    for (int i = 0; i < NUM_SENSORS; i++){
        if (sensor_fds[i] > 0) {
            vl53l8cx_stop_ranging(&sensors[i]);
        }

        if (lpn_lines[i]){
            gpiod_line_set_value(lpn_lines[i], 0); 
            gpiod_line_release(lpn_lines[i]);
        }
        // Close the I2C file descriptor
        if (sensor_fds[i] >= 0) {
            close(sensor_fds[i]);
            sensor_fds[i] = -1;
        }
    }

    // Shut down LED line
    if (led_line) {
        gpiod_line_set_value(led_line, 0);
        gpiod_line_release(led_line);
    }

    // Shut down chip
    if (chip) {
        gpiod_chip_close(chip);
    }

    printf("Done cleaning up! Good night!!\n");

    exit(0);
}


// Cleans system up before assignments
void signal_handler(int sig) {
    cleanup();
}

// Check to make sure platform.c file exists; It's part of the Linux driver
// Initializes GPIO pins on chip0
void setup_gpio() {
    chip = gpiod_chip_open_by_name("gpiochip0");

    if (!chip) {
        perror("Failed to open gpiochip0");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < NUM_SENSORS; i++){
        lpn_lines[i] = gpiod_chip_get_line(chip, LPN_BCM_PINS[i]);
        gpiod_line_request_output(lpn_lines[i], "lpn", 0);
    }

    led_line = gpiod_chip_get_line(chip, LED_BCM_PIN);

    // Ensure LED is originally off
    gpiod_line_request_output(led_line, "led", 0);
}

// Sets addresses and prepares sensors
void setup_sensors() {
    printf("Starting sensor startup sequence...\n");

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_fds[i] = -1;
        gpiod_line_set_value(lpn_lines[i], 0);
    }

    // 200ms delay
    usleep(200000);

    for (int i = 0; i < NUM_SENSORS; i++) {
        // Bring one sensor online
        gpiod_line_set_value(lpn_lines[i], 1);

        // Boot delay 100ms
        usleep(100000);

        // Open the I2C bus and store the descriptor in our global array
        int fd = open(I2C_BUS_PATH, O_RDWR);
        if (fd < 0) {
            perror("Failed to open I2C bus");
            cleanup();
        }
        sensor_fds[i] = fd;

        // Default address is 0x29 (7-bit)
        sensors[i].platform.address = 0x52; 

        // Initialize all of the sensors (loads firmware)
        uint8_t status = vl53l8cx_init(&sensors[i]);
        if (status != VL53L8CX_STATUS_OK) {
            printf("Sensor %d init failed with status %d\n", i, status);
            cleanup();
        }

        // Change I2C addresses for Aristotle, Brahe, Copernicus
        uint8_t new_address_8bit = I2C_ADDRESSES[i] << 1;
        if (new_address_8bit != 0x52) {
            status = vl53l8cx_set_i2c_address(&sensors[i], new_address_8bit);
            if (status != VL53L8CX_STATUS_OK) {
                printf("Sensor %d address change failed with status %d\n", i, status);
                cleanup();
            }
            // Update the platform struct with the new 8-bit address
            sensors[i].platform.address = new_address_8bit;
        }

        // Set the device to 8x8
        status = vl53l8cx_set_resolution(&sensors[i], VL53L8CX_RESOLUTION_8X8);
        if (status != VL53L8CX_STATUS_OK) {
            printf("Sensor %d resolution set failed with status %d\n", i, status);
            cleanup();
        }

        printf("Configured %s (%d) with address 0x%02X\n", SENSOR_NAMES[i], i, sensors[i].platform.address);
    }
    printf("All sensors configured.\n");
}

// Actually scans
void run_scan(){
    // Toggle on LED; we're starting the scan
    gpiod_line_set_value(led_line, 1); 
    printf("LED ON: Initializing scan\n");

    FILE *csvfile = fopen(CSV_FILENAME, "w");

    if (!csvfile) {
        perror("Failed to open CSV file");
        cleanup();
    }

    fprintf(csvfile, "Timestamp_s,Sensor,Zone,Distance_mm,Signal,Status,X,Y,Z\n");

    time_t start_time = time(NULL);

    while ((time(NULL) - start_time) < SCAN_DURATION_SECONDS){
        long ts = (long)(time(NULL) - start_time);

        // Start ranging on all sensors
        // Turn them on sequentially; ensure they're not all
        // ranging at the same interval; helps prevent interference
        for (int i = 0; i < NUM_SENSORS; i++) {
            vl53l8cx_start_ranging(&sensors[i]);
        }

        // Actually sensing stuff
        for (int i = 0; i < NUM_SENSORS; i++) {
            uint8_t data_ready = 0;
            VL53L8CX_ResultsData results;
            i2c_fd = sensor_fds[i];

            // Polling
            int attempts = 0;
            do {
                vl53l8cx_check_data_ready(&sensors[i], &data_ready);

                if (!data_ready) {
                    usleep(500);
                }

            } while (!data_ready && attempts++ < 100);

            if (data_ready) {
                vl53l8cx_get_ranging_data(&sensors[i], &results);

                for (int j = 0; j < 64; j++) {
                    if (results.nb_target_detected[j] > 0) {
                        float x, y, z;
                        calculate_cartesian(j, results.distance_mm[j], &x, &y, &z, i);
                        fprintf(csvfile, "%ld,%s,%d,%u,%u,%d,%.2f,%.2f,%.2f\n", 
                                ts, SENSOR_NAMES[i], j, results.distance_mm[j], 
                                (uint32_t)results.signal_per_spad[j], 
                                results.target_status[j], x, y, z);
                    }
                }
            }
            // Stop ranging to get a true "single-shot software sync"
            vl53l8cx_stop_ranging(&sensors[i]);
        }
    }

    // Close CSV file, turn off LED, we're stopping the scan
    fclose(csvfile);
    gpiod_line_set_value(led_line, 0); 
    printf("LED OFF: Scan complete.\n");
}

// Main code
int main() {
    signal(SIGINT, signal_handler);
    setup_gpio();
    setup_sensors();
    run_scan();
    cleanup();
    return 0;
}