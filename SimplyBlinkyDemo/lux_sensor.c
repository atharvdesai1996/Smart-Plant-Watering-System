#include <i2c.h>
#include <gpio.h>
#include "stdint.h"
#include "lux_sensor.h"

/* Variable for storing lux value returned from VEML7700 */
float lux;
#define SLAVE_ADDR 0x10

void i2cInit(void)
{
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN6 +
                                               GPIO_PIN7,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

    const eUSCI_I2C_MasterConfig i2cConfig =
    {
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,           // SMCLKs Clock Source
        48000000,                                                              // SMCLK = 48MHz
        EUSCI_B_I2C_SET_DATA_RATE_400KBPS,     // Data transfer rate 400kBPS or 100kBPS
        0,                                                                           // sets threshold for auto stop or UCSTPIFG
        EUSCI_B_I2C_NO_AUTO_STOP                        // sets up the stop condition generation
    };

    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_ADDR);
    I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    I2C_enableModule(EUSCI_B0_BASE);
    I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    I2C_enableInterrupt(EUSCI_B0_BASE, EUSCIB0_IRQn);
}
