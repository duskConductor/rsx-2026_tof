#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#ifndef _NET_IF_H
#error "net/if.h was not included properly
#endif


// Start CAN on Linux: sudo ip link set can0 up type can bitrate 1000000
// Then you can run this code

// This is super simple
// It just turns the motor:
// 0 to 90 to 180 to 90 to 170 to 0
// It writes the angle while turning
// Maybe.
// Maybe also it melts the 80 dollar connectors.
// Maybe not. 
// Help.

// Does exactly what's on the can (pun intended): takes the socket and the number of degrees you want the servo to move and (gasp!) moves the servo
void servo_move(int s, float degrees_to_move);
// Also pretty obvious: takes in the socket and reads the only motor on there (assumes 1 motor right now)
float servo_read(int s);

void servo_move(int s, float degrees_to_move){
    // CAN requires you to send data in a specific frame
    struct can_frame frame;
    int32_t position = (int32_t)(degrees_to_move / 0.01f);

    // TODO: Make the ID a parameter of the function
    frame.can_id = 0x141;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    // The actual command to move comes first
    frame.data[0] = 0xA4;
    // This guy's kind of like a delimiter
    // Like a space between words
    frame.data[1] = 0x00;
    // Setting speed of movement (100 dps)
    frame.data[2] = 0x64;
    frame.data[3] = 0x00;
    // Providing the new position
    frame.data[4] = position & 0xFF;
    frame.data[5] = (position >> 8) & 0xFF;
    frame.data[6] = (position >> 16) & 0xFF;
    frame.data[7] = (position >> 24) & 0xFF;
    
    // Sending the command to the servo!
    write(s, &frame, sizeof(struct can_frame));
}

float servo_read(int s) {
    struct can_frame frame;

    // TODO: Take motor ID as a function parameter
    frame.can_id = 0x141;
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    // First, we need to set the read command (0x92)
    frame.data[0] = 0x92;
    write(s, &frame, sizeof(struct can_frame));

    // This is the good
    // It read and will return the servo's position in actual degrees
    if (read(s, &frame, sizeof(struct can_frame)) > 0 && frame.data[0] == 0x92) {
        int32_t raw = (int32_t)(frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24));
        // Make it degrees
        return raw * 0.01f;
    }

    // This is the bad
    // It tried to read, but either there was nothing to read or it was corrupted or 
    // the frame somehow didn't have the read command (0x92)
    return -1.0f;
}

// Actual main logic
// Everything runs from here; the above two were just functions
int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    // This is just some random sequence to test whether the servo's
    // moving and the absolute encoder is reading the right angles as it does
    float servo_test_sequence[] = {0, 90, 180, 90, 170, 0};
    int num_steps = 6;

    // MAKIN' A SOCKET so that Linux can send and receive CAN
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    for (int i = 0; i < num_steps; i++) {
        float degrees_to_move = servo_test_sequence[i];

        printf("\nSTEP: %d\tDEGREES: %.2f\n", i, degrees_to_move);
        send_position(s, degrees_to_move);

        // Arbitrarily high value used for thresholding
        float current = -999.0f;

        while (fabs(current - degrees_to_move) > 0.5f) {
            current = servo_read(s);
            print("Current Angle: %6.2f deg\n", current);
            fflush(stdout);
            // NOTE: 100ms
            usleep(100000);
        }

        printf("Successfully moved %.1f\n", degrees_to_move);

        // Waits  5 seconds before moving so I can register 
        sleep(5);
    }

    printf("\nFinished moving!!\n");
    close(s);

    return 0;
}