// Header files
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

//Driver headers
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

// Driver configuration
#include "ti_drivers_config.h"

/* Problem:
 *
 * This project requires developing a smart thermostat system that interfaces with a micro-controller
 * to monitor and control room temperature using a TMP006 temperature sensor via I2C, manage user
 * input via GPIO buttons, and communicate system status through UART to a server. The system must
 * adjust a heating element represented by an LED based on the temperature relative to a user-set
 * threshold and must handle user adjustments to this threshold.
 *
 * Solution:
 *
 * The implemented solution employs the TI CC3220 micro-controller to read temperature data via I2C,
 * update heating control via GPIO, and transmit status updates via UART. This setup enables real-time
 * temperature management and remote monitoring capabilities.
 *
 * The operation follows these steps:
 * 1. Initializes the I2C, GPIO, UART, and Timer drivers to interface with the hardware components.
 * 2. Reads current temperature data from the TMP006 sensor using I2C.
 * 3. Controls an LED (as a heater indicator) based on the temperature readings and user-set threshold.
 * 4. Accepts user input through buttons to adjust the desired temperature threshold.
 * 5. Sends formatted temperature and system status data to a server using UART.
 *
 * Key Components:
 * - I2C_init() and I2C_open(): Initialize and open the I2C channel for communication with the TMP006 sensor.
 * - GPIO_init() and GPIO_setConfig(): Initialize GPIO for button input and LED output control.
 * - UART_init() and UART_open(): Set up UART for sending data to an external server.
 * - Timer_init() and Timer_start(): Configure timers for managing periodic readings and updates.
 * - System functions: Implement additional functionalities such as reading temperatures, adjusting
 *   heating based on temperatures, processing button presses, and formatting/sending data over UART.
 */

// Global Variables
static volatile int setPoint = 25;  // Default set temperature (°C)
static volatile bool heatOn = false;  // Heater state
static volatile unsigned long seconds = 0;  // Elapsed time in seconds since reset

// Timer global handle
Timer_Handle timer0;
volatile unsigned char timerFlag = 0;

// UART global variables
char output[64];  // Output buffer for UART
UART_Handle uart;

// I2C global variables
I2C_Handle i2c;
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[1] = {
    { 0x41, 0x01, "TMP006" }  // Address and register for TMP006
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Function Prototypes
void initUART(void);
void initI2C(void);
void initTimer(void);
void gpioButtonFxn(uint_least8_t index);
int16_t readTemp(void);
void timerCallback(Timer_Handle myHandle, int_fast16_t status);

// Initialize UART
void initUART(void) {
    UART_Params uartParams;
    UART_init();
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        while (1);  // Handle error
    }
}

// Initialize I2C
void initI2C(void) {
    I2C_Params i2cParams;
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        while (1);  // Handle error
    }

    i2cTransaction.slaveAddress = sensors[0].address;
    txBuffer[0] = sensors[0].resultReg;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;
    if (!I2C_transfer(i2c, &i2cTransaction)) {
        while (1);  // Handle error if sensor is not responding
    }
}

// Timer callback function
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    timerFlag = 1;  // Set timer flag on timer period
}

// Initialize Timer
void initTimer(void) {
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 200000;  // Set timer period to 200 ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL || Timer_start(timer0) == Timer_STATUS_ERROR) {
        while (1);  // Handle error
    }
}

// Main application function
void *mainThread(void *arg0) {
    uint32_t timerCount = 0;
    int16_t temperature = 0;
    bool currentHeatOn = false; // Variable to track the current state of the heater for change detection

    // Initialization of peripherals
    initUART();
    initI2C();
    initTimer();
    GPIO_init();

    // Configure LED and buttons
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    // Install Button callbacks
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    // Main loop
    while (1) {
        if (timerFlag) {
            timerFlag = 0;
            timerCount++;

            // Check button status every 200ms
            if (timerCount % 2 == 0) {
                // Button checks are handled in interrupt callbacks
            }

            // Read temperature every 500ms
            if (timerCount % 5 == 0) {
                temperature = readTemp();
            }

            // Update the LED and report to the server every second
            if (timerCount % 5 == 0) {  // 1000ms given a 200ms tick
                // Calculate heating needs and update LED if state changes
                currentHeatOn = heatOn;  // Store current state to check for changes
                if (temperature < setPoint) {
                    heatOn = true;
                } else {
                    heatOn = false;
                }

                // Only update the LED if the heating requirement state has changed
                if (currentHeatOn != heatOn) {
                    GPIO_write(CONFIG_GPIO_LED_0, heatOn ? 1 : 0);  // Update LED based on heating requirement
                }

                // Prepare and send data via UART
                snprintf(output, sizeof(output), "<%02d,%02d,%d,%04lu>\r\n",
                         temperature, setPoint, heatOn ? 1 : 0, seconds++);
                UART_write(uart, output, strlen(output));
            }

            // Reset timer count every 5 seconds to prevent overflow and maintain timing accuracy
            if (timerCount >= 25) {
                timerCount = 0;
            }
        }
    }
    return NULL;
}


/* GPIO Button Interrupt callback */
void gpioButtonFxn(uint_least8_t index) {
    if (index == CONFIG_GPIO_BUTTON_0) {
        setPoint++;
    } else if (index == CONFIG_GPIO_BUTTON_1) {
        setPoint--;
    }
}

/* Read temperature from TMP006 */
int16_t readTemp(void) {
    int16_t temperature = 0;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        temperature = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);
        temperature *= 0.03125;
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    } else {
        snprintf(output, sizeof(output), "Error reading temperature sensor\n\r");
        UART_write(uart, output, strlen(output));
    }
    return temperature;
}
