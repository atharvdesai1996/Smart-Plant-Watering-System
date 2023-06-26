#include "lux_sensor.h"
#define DATA_SIZE 16

//volatile uint8_t txData[DATA_SIZE];
//volatile uint8_t rxData[DATA_SIZE];

volatile uint8_t txIndex = 0;
volatile uint8_t rxIndex = 0;

// VEML7700 command register addresses
#define COMMAND_REG 0x00  // set to 00c0
//#define CONFIG_REG 0x00
#define DATA_REG 0x04


// Global variable to store the received light level
volatile uint16_t lightLevel = 0;

void I2C_init(void)
{
    // I2C pins P1.6 SDA, P1.7 SCL UCB0SDA/SCL
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN6 +
                                               GPIO_PIN7,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

    const eUSCI_I2C_MasterConfig i2cConfig =
    {
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,                  // SMCLKs Clock Source
        24000000,                                       // SMCLK = 48MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,              // Data transfer rate 100kBPS
        0,                                              // sets threshold for auto stop or UCSTPIFG
        EUSCI_B_I2C_NO_AUTO_STOP                        // sets up the stop condition generation
    };

    I2C_disableModule(EUSCI_B0_BASE);

    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    I2C_setSlaveAddress(EUSCI_B0_BASE, VEML7700_ADDR);
//    I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);


//    I2C_enableInterrupt(EUSCI_B0_BASE, EUSCIB0_IRQn);

    I2C_enableModule(EUSCI_B0_BASE);
    I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 | EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    Interrupt_enableInterrupt(INT_EUSCIB0);
    // Enable and clear the interrupt flag
    Interrupt_enableMaster();
}

void I2C_init_poll(void)
{
    // I2C pins P1.6 SDA, P1.7 SCL UCB0SDA/SCL
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN6 +
                                               GPIO_PIN7,
                                               GPIO_PRIMARY_MODULE_FUNCTION);

    const eUSCI_I2C_MasterConfig i2cConfig =
    {
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,                  // SMCLKs Clock Source
        24000000,                                       // SMCLK = 48MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,              // Data transfer rate 100kBPS
        0,                                              // sets threshold for auto stop or UCSTPIFG
        EUSCI_B_I2C_NO_AUTO_STOP                        // sets up the stop condition generation
    };

    I2C_disableModule(EUSCI_B0_BASE);

    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    I2C_setSlaveAddress(EUSCI_B0_BASE, VEML7700_ADDR);
//    I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);


//    I2C_enableInterrupt(EUSCI_B0_BASE, EUSCIB0_IRQn);

    I2C_enableModule(EUSCI_B0_BASE);

}


// Function to configure the VEML7700 sensor
void configure_sensor(void) {
  // Configure the sensor by writing to the configuration register
  // For example, you can set the integration time, gain, and other parameters here
  // Refer to the VEML7700 datasheet for the configuration options

  // Set the integration time to 800ms (default)
  uint8_t configData[3] = {CONFIG_REG, 0x00, 0xCO};
  I2C_write(VEML7700_ADDR, configData, 2);
}

// Function to write data to the VEML7700 sensor
void I2C_write(uint8_t slaveAddress, uint8_t *data, uint8_t length) {
  // Set the slave address and the mode to transmit
  I2C_setSlaveAddress(EUSCI_B0_BASE, slaveAddress);
  I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  // Start the transmission
  I2C_masterSendMultiByteStart(EUSCI_B0_BASE, *data++);

  // Transmit the data bytes
  while (length-- > 1) {
    I2C_masterSendMultiByteNext(EUSCI_B0_BASE, *data++);
  }

  // Send the stop condition
  I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, *data);
}

// I2C receive interrupt handler
void EUSCIB0_IRQHandler(void) {

    if(I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0))
    {

    }
    else if(I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0))
    {
        // Read the received data
        lightLevel = I2C_slaveGetData(EUSCI_B0_BASE);

        // Clear the receive interrupt flag
        I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    }
    I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 | EUSCI_B_I2C_RECEIVE_INTERRUPT0)
        // Check if the receive interrupt flag is set
}

// Function to read the ambient light level in lux from the VEML7700 sensor
void read_light_level(void) {
  // Trigger a single measurement by writing to the command register
  uint8_t commandData[2] = {COMMAND_REG, 0x01};
  I2C_write(VEML7700_ADDR, commandData, 2);
}

// Main program
void get_veml700_value(void){
    // Initialize the I2C module of VEML7700
     I2C_init();

     // Configure the VEML7700 sensor
     configure_sensor();

     // Enable the I2C receive interrupt
     I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);

     // Enable the global interrupt
     Interrupt_enableInterrupt(INT_EUSCIB0);
     while (1) {
        // Read the ambient light level
        read_light_level();

        // Wait for the receive interrupt to occur
        while (lightLevel == 0);

        // Convert the raw light level to lux (adjust the calculation if necessary)
        float lux = lightLevel * 0.0576;

        // Print the ambient light level (or use it as per your requirement)
        printff(EUSCI_A0_BASE, "Ambient Light Level: %.2f lux\r\n", lux);

        // Reset the lightLevel variable for the next measurement
        lightLevel = 0;

        // Delay for some time before taking the next reading
        // Adjust this delay as per your requirement
        // Delay function should be implemented separately for your specific platform
//        delay(1000);
      }
}




//void sendI2CData()
//{
//    while (I2C_masterIsStopSent(EUSCI_B0_BASE));
//
//    // Load data to be transmitted
//    for (int i = 0; i < DATA_SIZE; i++)
//        txData[i] = i;
//
//    // Set up for a transmit operation
//    txIndex = 0;
//    I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
//    I2C_masterSendStart(EUSCI_B0_BASE);
//
//    // Enable the I2C interrupt
//    I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
//}
//
//
//// Function to write data to the VEML7700 sensor
//void I2C_write(uint8_t reg, uint8_t *data, uint8_t length) {
//      // Set the slave address and the register to write to
//      I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_ADDR);
//      I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
//      I2C_masterSendMultiByteStart(EUSCI_B0_BASE, reg);
//
//      // Send the data bytes
//      uint8_t i;
//      for (i = 0; i < length; i++) {
//        I2C_masterSendMultiByteNext(EUSCI_B0_BASE, data[i]);
//      }
//
//      // Send the stop condition
//      I2C_masterSendMultiByteFinish(EUSCI_B0_BASE);
//}
//
//// Function to read data from the VEML7700 sensor
//void I2C_read(uint8_t reg, uint8_t *data, uint8_t length) {
//  // Set the slave address and the register to read from
//  I2C_setSlaveAddress(EUSCI_B0_BASE, VEML7700_ADDR);
//  I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
//  I2C_masterSendMultiByteStart(EUSCI_B0_BASE, reg);
//
//  // Send the repeated start condition
//  I2C_masterReceiveStart(EUSCI_B0_BASE);
//
//  // Receive the data bytes
//  uint8_t i;
//  for (i = 0; i < length - 1; i++) {
//    data[i] = I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
//  }
//  data[length - 1] = I2C_masterReceiveMultiByteFinish(EUSCI_B0_BASE);
//}
//
//
//void EUSCIB0_IRQHandler()
//{
//    uint32_t status = I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);
//    I2C_clearInterruptFlag(EUSCI_B0_BASE, status);
//
//    if (status & EUSCI_B_I2C_TRANSMIT_INTERRUPT0)
//    {
//        if (txIndex < DATA_SIZE)
//        {
//            // Load data to be transmitted
//            I2C_masterSendSingleByte(EUSCI_B0_BASE, txData[txIndex]);
//            txIndex++;
//        }
//        else
//        {
//            // All data transmitted, send stop condition
//            I2C_masterSendStop(EUSCI_B0_BASE);
//            I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
//        }
//    }
//    else if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
//    {
//        // Receive data
//        rxData[rxIndex] = I2C_masterReceiveSingleByte(EUSCI_B0_BASE);
//        rxIndex++;
//
//        if (rxIndex == DATA_SIZE)
//        {
//            // All data received
//            // Handle the received data as needed
//        }
//    }
//}
//
////int main(void)
////{
////    // Initialize the MSP432 system
////    // ...
////
////    // Initialize the I2C module
////    initI2C();
////
////    // Send I2C data
////    sendI2CData();
////
////    while (1)
////    {
////        // Main program loop
////        // ...
////    }
////}
//
//
//