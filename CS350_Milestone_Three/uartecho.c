#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Problem:
 *
 * This code is designed to meet the requirements of a project that involves interacting
 * with a micro-controller via UART (Universal Asynchronous Receiver/Transmitter) for
 * controlling an LED based on textual commands received through a serial interface.
 *
 * Solution:
 *
 * The implemented solution demonstrates how to read commands ("ON" and "OFF") from a UART
 * interface and control an LED accordingly. This involves initializing the UART and GPIO
 * (General Purpose Input Output) drivers, configuring UART communication parameters, and
 * continuously reading input from the UART to turn the LED on or off based on the received
 * commands.
 *
 * The operation follows these steps:
 * 1. Initializes the UART and GPIO drivers.
 * 2. Configures a GPIO pin connected to an LED for output.
 * 3. Sets up the UART with specified communication parameters.
 * 4. Enters an infinite loop, reading commands from the UART and controlling the LED based
 *    on the commands received.
 *
 * Key Components:
 * - UART_init(): Initializes the UART driver.
 * - GPIO_init(): Initializes the GPIO driver.
 * - UART_open(): Opens a UART channel with specified parameters.
 * - GPIO_setConfig(): Configures the GPIO pin connected to the LED.
 * - UART_read(): Reads incoming data from the UART.
 * - GPIO_write(): Controls the LED by setting the GPIO pin high or low.
 */

// Constants
const char COMMAND_ON[] = "ON";
const char COMMAND_OFF[] = "OFF";
#define LED_GPIO_CONFIG CONFIG_GPIO_LED_0 // GPIO pin configuration for the LED

// Main Thread
void *mainThread(void *arg0)
{
    char input; // To store the input character
    unsigned char onState = 0; // State for tracking "ON" command
    unsigned char offState = 0; // State for tracking "OFF" command
    UART_Handle uart;
    UART_Params uartParams;

    // Initialize GPIO and UART drivers
    GPIO_init();
    UART_init();

    // Configure the LED pin
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Set up UART parameters and open the UART */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        // Handle UART_open() failure with infinite loop
        while (1);
    }

    // Main loop
       while (1) {
           UART_read(uart, &input, 1); // Read one character from UART

           // Process "ON" command
           if (input == COMMAND_ON[onState]) {
               onState++;
               if (COMMAND_ON[onState] == '\0') { // If entire command received
                   GPIO_write(LED_GPIO_CONFIG, CONFIG_GPIO_LED_ON); // Turn LED on
                   onState = 0; // Reset state variables
                   offState = 0;
               }
           } else {
               onState = 0; // Reset "ON" command state
           }

           // Process "OFF" command
           if (input == COMMAND_OFF[offState]) {
               offState++;
               if (COMMAND_OFF[offState] == '\0') { // If entire command received
                   GPIO_write(LED_GPIO_CONFIG, CONFIG_GPIO_LED_OFF); // Turn LED off
                   onState = 0; // Reset state variables
                   offState = 0;
               }
           } else {
               offState = 0; // Reset "OFF" command state
               }
       }
}
