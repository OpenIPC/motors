#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define GPIO_PATH "/sys/class/gpio/gpio"
#define GPIO_EXPORT "/sys/class/gpio/export"
#define GPIO_UNEXPORT "/sys/class/gpio/unexport"

#define NUM_PINS 8

int PAN_PINS[4];
int TILT_PINS[4];

int STEP_SEQUENCE[8][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

void export_gpio(int pin) {
    char path[50];
    FILE *file;

    // Export the GPIO
    file = fopen(GPIO_EXPORT, "w");
    if (file) {
        fprintf(file, "%d", pin);
        fclose(file);
    } else {
        perror("Error exporting GPIO");
    }

    // Set direction to "out"
    snprintf(path, sizeof(path), GPIO_PATH "%d/direction", pin);
    file = fopen(path, "w");
    if (file) {
        fprintf(file, "out");
        fclose(file);
    } else {
        perror("Error setting GPIO direction");
    }
}

void unexport_gpio(int pin) {
    FILE *file = fopen(GPIO_UNEXPORT, "w");
    if (file) {
        fprintf(file, "%d", pin);
        fclose(file);
    } else {
        perror("Error unexporting GPIO");
    }
}

void set_gpio(int pin, int value) {
    char path[50];
    snprintf(path, sizeof(path), GPIO_PATH "%d/value", pin);
    FILE *file = fopen(path, "w");
    if (file) {
        fprintf(file, "%d", value);
        fclose(file);
    }
}

void move_motor(const int pins[], int steps, int delay_ms) {
    int reverse = (steps < 0);
    steps = abs(steps);
    
    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < 8; j++) {
            int index = reverse ? (7 - j) : j;
            for (int k = 0; k < 4; k++) {
                set_gpio(pins[k], STEP_SEQUENCE[index][k]);
            }
            usleep(delay_ms * 1000);
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        fprintf(stderr, "Usage: %s <motor (pan/tilt)> <steps> <delay_ms>\n", argv[0]);
        return 1;
    }

    int steps = atoi(argv[2]);
    int delay_ms = atoi(argv[3]);

    if (steps == 0 || delay_ms <= 0) {
        fprintf(stderr, "Error: Steps cannot be zero, and delay must be positive.\n");
        return 1;
    }

    // Execute fw_printenv to get GPIO pins
    FILE* fp = popen("fw_printenv gpio_motor -n", "r");
    if (!fp) {
        perror("Error executing fw_printenv");
        return 1;
    }

    char gpio_output[100];
    if (!fgets(gpio_output, sizeof(gpio_output), fp)) {
        fprintf(stderr, "Error: Failed to read GPIO pins from fw_printenv.\n");
        pclose(fp);
        return 1;
    }
    pclose(fp);

    // Parse the GPIO pins into PAN_PINS and TILT_PINS
    char* token;
    char* rest = gpio_output; // Use the output directly for tokenization
    int index = 0;

    while ((token = strtok_r(rest, " ", &rest))) {
        int pin = atoi(token);
        if (index < 4) {
            PAN_PINS[index] = pin; // First 4 pins are for pan
        } else {
            TILT_PINS[index - 4] = pin; // Next 4 pins are for tilt
        }
        index++;
    }

    if (index != NUM_PINS) {
        fprintf(stderr, "Error: Expected %d GPIO pins, but got %d.\n", NUM_PINS, index);
        return 1;
    }

    const int *pins = (argv[1][0] == 'p' || argv[1][0] == 'P') ? PAN_PINS :
                      (argv[1][0] == 't' || argv[1][0] == 'T') ? TILT_PINS : NULL;

    if (!pins) {
        fprintf(stderr, "Error: Invalid motor type! Use 'pan' or 'tilt'.\n");
        return 1;
    }

    for (int i = 0; i < 4; i++) {
        export_gpio(pins[i]);
    }

    move_motor(pins, steps, delay_ms);

    for (int i = 0; i < 4; i++) {
        unexport_gpio(pins[i]);
    }

    return 0;
}