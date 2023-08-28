#include "lux_sensor_poll.h"
#include "rom_map.h"
#include "string.h"
#include "driverlib.h"
#define DATA_SIZE 16

//volatile uint8_t txData[DATA_SIZE];
//volatile uint8_t rxData[DATA_SIZE];

volatile uint8_t txIndex_p = 0;
volatile uint8_t rxIndex_p = 0;

// VEML7700 command register addresses
#define COMMAND_REG 0x00  // set to 00c0
//#define CONFIG_REG 0x00
#define DATA_REG 0x04
#define moduleInstance EUSCI_B0_BASE


// Global variable to store the received light level
volatile uint16_t lightLevel_p = 0;
volatile uint16_t temp_p = 0;
uint8_t ret_status_p = 0;

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
        3000000,                                       // SMCLK = 48MHz
        EUSCI_B_I2C_SET_DATA_RATE_100KBPS,              // Data transfer rate 100kBPS
        0,                                              // sets threshold for auto stop or UCSTPIFG
        EUSCI_B_I2C_NO_AUTO_STOP                        // sets up the stop condition generation
    };

//    UCB0CTL1 |= UCSWRST;                   // Enable SW reset
//    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC; // I2C Master, synchronous mode
//    UCB0CTL1 = UCSSEL_2 + UCSWRST ;            // Use SMCLK, keep SW reset
//    UCB0BR0 = 10;                             // fSCL = SMCLK/10 = 100kHz
//    UCB0BR1 = 0;
//    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

    I2C_disableModule(EUSCI_B0_BASE);
    I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
    //UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC; // I2C Master, synchronous mode
    //UCB0CTL1 = UCSSEL_2 + UCSWRST ;            // Use SMCLK, keep SW reset
//    UCB0BR0 = 10;                             // fSCL = SMCLK/10 = 100kHz
//    UCB0BR1 = 0;
    //UCB0CTL1 = EUSCI_B_CTLW0_UCSSEL_2 + UCSWRST;              // Use SMCLK, keep SW reset
    //UCB0CTL0 = EUSCI_B_CTLW0_MST | EUSCI_B_CTLW0_MODE_3 | EUSCI_B_CTLW0_SYNC;         // I2C Master, synchronous mode
    EUSCI_B_CMSIS(EUSCI_B0_BASE)->BRW = 12;                             // fSCL = SMCLK/12 = ~100kHz
    I2C_setSlaveAddress(EUSCI_B0_BASE, VEML7700_ADDR);
    I2C_enableModule(EUSCI_B0_BASE);
    I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 | EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_NAK_INTERRUPT);
    I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
}


// Function to configure the VEML7700 sensor
void configure_sensor_poll(void) {
  // Configure the sensor by writing to the configuration register
  // For example, you can set the integration time, gain, and other parameters here
  // Refer to the VEML7700 datasheet for the configuration options

  // Set the integration time to 800ms (default)
  uint8_t configData[3] = {COMMAND_REG, 0xC0, 0x00};
  I2C_write_poll(VEML_ADDR_8BIT_WRITE, configData, 2);

}

uint8_t poll_interruptTX_flag(void)
{
    while(!(EUSCI_B_CMSIS(EUSCI_B0_BASE)->IFG & EUSCI_B_IFG_TXIFG0))
//    {
//        if(EUSCI_B_CMSIS(EUSCI_B0_BASE)->IFG & EUSCI_B_IFG_NACKIFG)
//        {
//            BITBAND_PERI(EUSCI_B_CMSIS(EUSCI_B0_BASE)->CTLW0,EUSCI_B_CTLW0_TXSTP_OFS) = 1;        // Issue stop
//            return 1;
//        }
//    }
    return 0;
}


// Function to write data to the VEML7700 sensor
void I2C_write_poll(uint8_t slaveAddress, uint8_t *data, uint8_t length)
{
    //UCB0I2CSA = VEML7700_ADDR;
    //UCB0IFG &=~ (UCTXIFG + UCNACKIFG);  // Flag Clear
    // The mode to transmit
    MAP_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, *(data++));
    //EUSCI_B_CMSIS(EUSCI_B0_BASE)->CTLW0 |= EUSCI_B_CTLW0_TR + EUSCI_B_CTLW0_TXSTT;
    //Poll for transmit interrupt flag.
    poll_interruptTX_flag();
    //EUSCI_B_CMSIS(EUSCI_B0_BASE)->IFG &= ~UCTXIFG;
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    //Send single byte data 8 bits with Write Bit
    //EUSCI_B_CMSIS(EUSCI_B0_BASE)->TXBUF = slaveAddress;
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    // Start the transmission
    //EUSCI_B_CMSIS(EUSCI_B0_BASE)->TXBUF = *(data++);
    //poll_interruptTX_flag();
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    // Transmit the data bytes
    while (length-- > 1)
    {
        MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, *(data++));
        //EUSCI_B_CMSIS(EUSCI_B0_BASE)->TXBUF = *(data++);
        poll_interruptTX_flag();
    }
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    // Send the stop condition
    BITBAND_PERI(EUSCI_B_CMSIS(EUSCI_B0_BASE)->CTLW0,EUSCI_B_CTLW0_TXSTP_OFS) = 1;
    __delay_cycles(300);
}
void I2C_read_poll(uint8_t slaveAddress, uint8_t *data, uint8_t length)
{
    I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    // The mode to transmit
    //MAP_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    MAP_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    EUSCI_B_CMSIS(moduleInstance)->CTLW0 |= EUSCI_B_CTLW0_TR + EUSCI_B_CTLW0_TXSTT;
    //Poll for transmit interrupt flag.
    poll_interruptTX_flag();
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    //Send single byte data.7
    //EUSCI_B_CMSIS(moduleInstance)->TXBUF = slaveAddress;
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    // Start the transmission
    EUSCI_B_CMSIS(moduleInstance)->TXBUF = *(data++);
    poll_interruptTX_flag();
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    EUSCI_B_CMSIS(moduleInstance)->CTLW0 |= EUSCI_B_CTLW0_TR + EUSCI_B_CTLW0_TXSTT;
    poll_interruptTX_flag();
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    //Send single byte data.7
    //EUSCI_B_CMSIS(moduleInstance)->TXBUF = slaveAddress;
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    //I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
    //while(!I2C_isBusBusy(EUSCI_B0_BASE));
    //__delay_cycles(200);
    // Transmit the data bytes
    while (length-- > 1)
    {
        //while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0));
        while(!(EUSCI_B_CMSIS(EUSCI_B0_BASE)->IFG & EUSCI_B_IFG_RXIFG0))
        lightLevel_p = (EUSCI_B_CMSIS(moduleInstance)->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
        temp_p = lightLevel_p;
        //while(!I2C_isBusBusy(EUSCI_B0_BASE));
        //__delay_cycles(200);
    }
    while(!(EUSCI_B_CMSIS(EUSCI_B0_BASE)->IFG & EUSCI_B_IFG_RXIFG0))
    lightLevel_p = (EUSCI_B_CMSIS(moduleInstance)->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK);
    temp_p |= lightLevel_p << 8;

    //Send stop condition.
    BITBAND_PERI(EUSCI_B_CMSIS(moduleInstance)->CTLW0,EUSCI_B_CTLW0_TXSTP_OFS) = 1;
}


// Function to read the ambient light level in lux from the VEML7700 sensor
void read_light_level_poll(void) {
  // Trigger a single measurement by writing to the command register
  uint8_t commandData[2] = {DATA_REG};
  I2C_read_poll(VEML_ADDR_8BIT_READ, commandData, 2);
}

// Main program
void get_veml700_value_poll(void){
    // Initialize the I2C module of VEML7700
    I2C_init_poll();

     // Configure the VEML7700 sensor
     configure_sensor_poll();

     while(1)
     {
//        // Read the ambient light level
        read_light_level_poll();
//
//        // Wait for the receive interrupt to occur
//        while (temp_p == 0);
//
//        // Convert the raw light level to lux (adjust the calculation if necessary)
//        float lux = temp_p * 0.0576;
//
//        // Print the ambient light level (or use it as per your requirement)
//        printff(EUSCI_A0_BASE, "Ambient Light Level: %.2f lux\r\n", lux);
//
//        // Reset the lightLevel_p variable for the next measurement
//        lightLevel_p = 0;
//        temp_p = 0;
//        // Delay for some time before taking the next reading
//        // Adjust this delay as per your requirement
//        // Delay function should be implemented separately for your specific platform
////        delay(1000);
      }
}
