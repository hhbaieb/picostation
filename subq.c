#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "main.pio.h"

#include "utils.h"
#include "values.h"
#include "subq.h"

// Declare global variables
extern volatile uint subq_offset;
extern volatile int current_logical_track;
extern volatile int sector;
extern volatile bool hasData;
extern volatile int num_logical_tracks;
extern bool *is_data_track;
extern int *logical_track_to_sector;

// Declare an array to hold the subq data
uint8_t tracksubq[12];

// Function to print subq data to console
void printf_subq(uint8_t *subqdata) {
    for (int i=0; i<10; i++) {
        printf("%02X ", subqdata[i]);
    }
}

// Function to send subq data to PIO
static inline void send_subq(uint8_t *subqdata) {
    // Initialize the PIO program
    subq_program_init(pio1, SUBQ_SM, subq_offset, SQSO, SQCK);
    // Enable the PIO state machine
    pio_sm_set_enabled(pio1, SUBQ_SM, true);

    // Reverse the bits of the subq data and send it to the PIO state machine
    uint sub1 = (reverseBits(subqdata[0],8) << 24) |
                (reverseBits(subqdata[1],8) << 16) |
                (reverseBits(subqdata[2],8) << 8) |
                (reverseBits(subqdata[3],8));
    uint sub2 = (reverseBits(subqdata[4],8) << 24) |
                (reverseBits(subqdata[5],8) << 16) |
                (reverseBits(subqdata[6],8) << 8) |
                (reverseBits(subqdata[7],8));
    uint sub3 = (reverseBits(subqdata[8],8) << 24) |
                (reverseBits(subqdata[9],8) << 16) |
                (reverseBits(subqdata[10],8) << 8) |
                (reverseBits(subqdata[11],8));
    pio_sm_put_blocking(pio1, SUBQ_SM, reverseBits(sub1,32));
    pio_sm_put_blocking(pio1, SUBQ_SM, reverseBits(sub2,32));
    pio_sm_put_blocking(pio1, SUBQ_SM, reverseBits(sub3,32));
    
    // Send a pulse to the SCOR pin to indicate the end of the subq data
    pio_sm_put_blocking(pio1, SCOR_SM, 1);
}

// Function to start the subq data transmission
void start_subq() {
    // Check if the sector is less than 4500
    if (sector < 4500) {
        // Calculate the subq entry based on the sector and number of logical tracks
        int subq_entry = sector%(3+num_logical_tracks);

        // Generate subq data based on the subq entry
        if (subq_entry == 0) {
            tracksubq[0] = hasData ? 0x61 : 0x21;
            tracksubq[1] = 0x00;
            tracksubq[2] = 0xA0;
            tracksubq[7] = 0x01;
            tracksubq[8] = 0x20;
            tracksubq[9] = 0x00;
        } else if (subq_entry == 1) {
            tracksubq[0] = hasData ? 0x61 : 0x21;
            tracksubq[1] = 0x00;
            tracksubq[2] = 0xA1;
            tracksubq[7] = tobcd(num_logical_tracks);
            tracksubq[8] = 0x00;
            tracksubq[9] = 0x00;
        } else if (subq_entry == 2) {
            int sector_lead_out = logical_track_to_sector[num_logical_tracks+1] - 4500;
            tracksubq[0] = hasData ? 0x61 : 0x21;
            tracksubq[1] = 0x00;
            tracksubq[2] = 0xA2;
            tracksubq[7] = tobcd(sector_lead_out/75/60);
            tracksubq[8] = tobcd((sector_lead_out/75) % 60);
            tracksubq[9] = tobcd(sector_lead_out % 75);
        } else if (subq_entry > 2) {
            int sector_track;
            int logical_track = subq_entry-2;
            if (logical_track == 1) {
                sector_track = 150;
            } else {
                sector_track = logical_track_to_sector[logical_track] - 4500;
            }
            tracksubq[0] = is_data_track[logical_track] ? 0x41 : 0x01;
            tracksubq[1] = 0x00;
            tracksubq[2] = tobcd(logical_track);
            tracksubq[7] = tobcd(sector_track/75/60);
            tracksubq[8] = tobcd((sector_track/75) % 60);
            tracksubq[9] = tobcd(sector_track % 75);
        }

        // Generate subq data for the sector
        tracksubq[3] = tobcd(sector/75/60);
        tracksubq[4] = tobcd((sector/75) % 60);
        tracksubq[5] = tobcd(sector % 75);
        tracksubq[6] = 0x00;
        tracksubq[10] = 0x00;
        tracksubq[11] = 0x00;

        // Send the subq data to the PIO state machine
        send_subq(tracksubq);
    } else {
        // Calculate the logical track based on the sector
        int logical_track = num_logical_tracks+1; // in case seek overshoots past end of disc
        for (int i=0; i<num_logical_tracks+2; i++) { // + 2 for lead in & lead out
            if (logical_track_to_sector[i+1] > sector) {
                logical_track = i;
                break;
            }
        }
        // Calculate the sector within the track
        int sector_track = sector - logical_track_to_sector[logical_track];
        int sector_abs = (sector - 4500);
        int sector_track_after_pause;

        // Calculate the sector within the track after a pause
        if (logical_track == 1) {
            sector_track_after_pause = sector_track - 150;
        } else {
            sector_track_after_pause = sector_track;
        }

        // Update the current logical track
        current_logical_track = logical_track;

        // Generate subq data for the sector
        tracksubq[0] = is_data_track[logical_track] ? 0x41 : 0x01;
        if (logical_track == num_logical_tracks+1) {
            tracksubq[1] = 0xAA;
        } else {
            tracksubq[1] = tobcd(logical_track);
        }
        if (sector_track < 150 && logical_track == 1) { // 2 sec pause track
            tracksubq[2] = 0x00;
            tracksubq[3] = 0x00; // min
            tracksubq[4] = tobcd(1-(sector_track/75)); // sec (count down)
            tracksubq[5] = tobcd(74 - (sector_track % 75)); // frame (count down)
        } else {
            tracksubq[2] = 0x01;
            tracksubq[3] = tobcd(sector_track_after_pause/75/60); // min
            tracksubq[4] = tobcd((sector_track_after_pause/75) % 60); // sec
            tracksubq[5] = tobcd(sector_track_after_pause % 75); // frame
        }
        tracksubq[6] = 0x00;
        tracksubq[7] = tobcd(sector_abs/75/60); // amin
        tracksubq[8] = tobcd((sector_abs/75) % 60); // asec
        tracksubq[9] = tobcd(sector_abs % 75); // aframe
        tracksubq[10] = 0x00;
        tracksubq[11] = ((sector % 2) == 0) ? 0x00 : 0x80;
        if (sector % (50+num_logical_tracks) == 0) {
            printf("l:%-6d\t",sector);
            printf_subq(tracksubq);
            printf("\n");
        }

        // Send the subq data to the PIO state machine
        send_subq(tracksubq);
    }
}