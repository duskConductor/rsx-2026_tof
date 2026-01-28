#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <gpiod.h>
#include <fcntl.h>
#include <signal.h>
#include "vl53l8cx_api.h"

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

// Setting up the GPIO Lines
struct gpiod_chip *chip;
struct gpiod_line *lpn_lines[NUM_SENSORS];
struct gpiod_line *led_line;
VL53L8CX_Configuration sensors[NUM_SENSORS];

// Used when closing the program
void cleanup();

// Closing out of the program
void cleanup() {
    printf("\nPerforming cleanup...\n");

    // Shut down all LPn lines and stop range detection
    for (int i = 0; i < NUM_SENSORS; i++){
        vl53l8cx_stop_ranging(&sensors[i]);

        if (lpn_lines[i]){
            gpiod_line_set_value(lpn_lines[i], 0); 
            gpiod_line_release(lpn_lines[i]);
        }
        // Close the I2C file descriptor
        if (sensors[i].platform.i2c_bus_fd >= 0) {
            close(sensors[i].platform.i2c_bus_fd);
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

void setup_sensors() {
    printf("Starting sensor startup sequence...\n");

    for (int i = 0; i < NUM_SENSORS; i++) {
        gpiod_line_set_value(lpn_lines[i], 0);
    }

    // 200ms delay
    usleep(200000);

    for (int i = 0; i < NUM_SENSORS; i++) {
        // Bring one sensor online
        gpiod_line_set_value(lpn_lines[i], 1);

        // Boot delay 100ms
        usleep(100000);

        // Default address is 0x29 (7-bit)
        sensors[i].platform.i2c_address = 0x52 >> 1; 
        sensors[i].platform.i2c_bus_fd = open(I2C_BUS_PATH, O_RDWR);

        if (sensors[i].platform.i2c_bus_fd < 0) {
            perror("Failed to open I2C bus");
            cleanup();
        }

        // Initialize all of the sensors (loads firmware)
        uint8_t status = vl53l8cx_init(&sensors[i]);
        if (status != VL53L8CX_STATUS_OK) {
            printf("Sensor %d init failed with status %d\n", i, status);
            cleanup();
        }

        // Change I2C addresses for Aristotle, Brahe, Copernicus
        if (I2C_ADDRESSES[i] != (0x52 >> 1)) {
            // API function requires 8-bit address (e.g. 0x54)
            status = vl53l8cx_set_i2c_address(&sensors[i], I2C_ADDRESSES[i] << 1);

            if (status != VL53L8CX_STATUS_OK) {
                printf("Sensor %d address change failed with status %d\n", i, status);
                cleanup();
            }

            // Update the local device structure with the new 7-bit address
            sensors[i].platform.i2c_address = I2C_ADDRESSES[i];
        }

        // Set the device to 8x8
        status = vl53l8cx_set_resolution(&sensors[i], 1);
        if (status != VL53L8CX_STATUS_OK) {
            printf("Sensor %d resolution set failed with status %d\n", i, status);
            cleanup();
        }

        printf("Configured %s (%d) with address 0x%02X\n", SENSOR_NAMES[i], i, I2C_ADDRESSES[i]);
    }
    printf("All sensors configured.\n");
}

void run_scan(){
    // Toggle on LED; we're starting the scan
    gpiod_line_set_value(led_line, 1); 
    printf("LED ON: Initializing scan\n");

    time_t start_time = time(NULL);
    FILE *csvfile = fopen(CSV_FILENAME, "w");

    if (!csvfile) {
        perror("Failed to open CSV file");
        cleanup();
    }

    fprintf(csvfile, "Timestamp_s,Sensor_Name,Zone,Distance_mm,Signal_Strength,Status\n");

    // fprintf(csvfile, "Timestamp_s");
    // for (int s = 0; s < NUM_SENSORS; s++){
    //     for (int z_idx = 0; z_idx < 64; z_idx++){
    //         // "Timestamp_s, Aristotle_Z1_Dist_mm, Aristotle_Z1_Status"
    //         // fprintf(csvfile, ",%s_Z%d_Dist_mm,%s_Z%d_Status", SENSOR_NAMES[s], z_idx, SENSOR_NAMES[s], z_idx);
            
    //     }
    // }
    // fprintf(csvfile, "\n");

    // Start all sensors in autonomous ranging mode
    for (int i = 0; i < NUM_SENSORS; i++) {
        vl53l8cx_start_ranging(&sensors[i]);
    }

    VL53L8CX_ResultsData results;
    uint8_t data_ready = 0;

    // Currently it's just using the clock to scan for a predetermined time
    while ((time(NULL) - start_time) < SCAN_DURATION_SECONDS) {
        fprintf(csvfile, "%ld", (long)(time(NULL) - start_time));

        // Polling I2C line sequentially
        for (int i = 0; i < NUM_SENSORS; i++) {
            do {
                vl53l8cx_check_data_ready(&sensors[i], &data_ready);
                usleep(100); // Sleep 100 microseconds
            } while (!data_ready);

            vl53l8cx_get_ranging_data(&sensors[i], &results);

            for (int j = 0; j < 64; j++){
                if (results.nb_target_detected[j] > 0) {
                    fprintf(csvfile, ",%d,%d", results.distance_mm[j], results.target_status[j]);
                } else {
                    fprintf(csvfile, ",0,0");
                }
            }

            // Delay after reading one sensor's data (1 millisecond)
            usleep(1000);
        }
        fprintf(csvfile, "\n");
    }

    // Close CSV file, turn off LED, we're stopping the scan
    fclose(csvfile);
    gpiod_line_set_value(led_line, 0); 
    printf("LED OFF: Scan complete.\n");
}

int main() {
    signal(SIGINT, signal_handler);
    setup_gpio();
    setup_sensors();
    run_scan();
    cleanup();
    return 0;
}