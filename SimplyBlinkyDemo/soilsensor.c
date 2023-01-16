
/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "driverlib.h"
/* TI includes. */
#include "gpio.h"
#include <stdbool.h>


 volatile uint16_t curADCResult;


const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
};


// Setup the Pin Configurations
void Soil_SetupADC()
{

    curADCResult =0;


   // MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
   // MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();

    // Step 1: Initializing ADC 14
    MAP_ADC14_enableModule();

    // Step 2: Configure number of bits for ADC
   // ADC14 -> CTL1 = ADC14_CTL1_RES_3;

    // Step 3: Configuring GPIOs for Analog In P5.5 / A0
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
            GPIO_PIN5 , GPIO_TERTIARY_MODULE_FUNCTION);

    // Step 4: set ADC clock source
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_32, ADC_DIVIDER_2,  0);

    // Step 5: Configuring ADC Memory
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);

    // Step 6: choose ADC memory , voltage range , channel and differential mode or not
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);

    // Step 7: Setting up the sample timer to automatically step through the sequence convert.
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

    // Step 8: Triggering the start of the sample */
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_ADC14_enableConversion();

    // Step 9: Trigger ADC
    MAP_ADC14_toggleConversionTrigger();

    // Step 10: Enable the interrupts

    MAP_Interrupt_enableInterrupt(INT_ADC14);  // cause step debug fail
    MAP_Interrupt_enableMaster();


   while (1)
   {
       __WFI();
   }

}

void soilsetup_2()
{
    // try https://e2e.ti.com/support/microcontrollers/msp-low-power-microcontrollers-group/msp430/f/msp-low-power-microcontroller-forum/528882/mcu-msp432p401r----code-doesn-t-work-as-expected-when-i-enable-adc_toggle_conversion_trigger
    // try https://cpp.hotexamples.com/examples/-/-/MAP_ADC14_enableConversion/cpp-map_adc14_enableconversion-function-examples.html

    /* Halting WDT  */
        MAP_WDT_A_holdTimer();

        /* Initializing Variables */
        curADCResult = 0;

        /* Initializing ADC (MCLK/1/4) */
        MAP_ADC14_enableModule();
        MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4,
                0);

        /* Configuring GPIOs (5.5 A0) */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
        GPIO_TERTIARY_MODULE_FUNCTION);

        /* Setting DCO to 12MHz */
        CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

        /* Selecting P1.2 and P1.3 in UART mode */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);


        /* Configuring UART Module */
        MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

        /* Enable UART module */
        MAP_UART_enableModule(EUSCI_A0_BASE);

        /* Enabling interrupts */
        MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
        MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
        MAP_Interrupt_enableSleepOnIsrExit();
        MAP_Interrupt_enableMaster();

        /* Configuring ADC Memory */
        MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
        MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A0, false);

        /* Configuring Sample Timer */
        MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

        /* Enabling/Toggling Conversion */
        MAP_ADC14_enableConversion();
        MAP_ADC14_toggleConversionTrigger();

        /* Enabling interrupts */
        MAP_ADC14_enableInterrupt(ADC_INT0);
        MAP_Interrupt_enableInterrupt(INT_ADC14);
        MAP_Interrupt_enableMaster();

        unsigned u = 0;//4321;
        while(1)
        {
            MAP_PCM_gotoLPM0();
        }
}



void soilsetup_3()
{

    /* Halting WDT and disabling master interrupts */
        WDTCTL = WDTPW | WDTHOLD;                 // Stop WDT

        /* Initialize main clock to 3MHz */
        MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_3);
        CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
        CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
        CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );

        /* Selecting P1.0 as output (LED). */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

        /* Selecting P1.2 and P1.3 in UART mode. */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

        /* Configuring UART Module */
        MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

        /* Enable UART module */
        MAP_UART_enableModule(EUSCI_A0_BASE);

        UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
        Interrupt_enableInterrupt(INT_EUSCIA0);
        Interrupt_enableMaster();

        printf("\r\nPrintf support for the launchpad\r\n");

        printf("Decimal(10) :%d\r\n", 10);
        printf("Hex(10)     :%x\r\n", 10);
        printf("float       :%f\r\n", 4.32);

        /* Main while loop */
        while(1)
        {
            MAP_PCM_gotoLPM0();
        }
    }
