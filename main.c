#include "driverlib/driverlib.h"
#include "main.h"
#include "hal_LCD.h"
#include "string.h"
#include "stdio.h"

char ADCState = 0; //busy state of the ADC
int16_t ADCResult = 0; //storage for the ADC conversion result
float T_temp = 0;
float S_temp = 0;
float sensor[4] = {0, 0, 0, 0}; //sensor[0] = T1, sensor[1] = S1, sensor[2] = T2, sensor[3] = S2
char buttonState1 = 0; //current button press state (to allow edge detection)
char buttonState2 = 0;
char string[50];
volatile uint8_t echo;
const char esetup[12] = "\r\nSETTINGS\r\n";
const char explain[362] = "To enable motors in each zone, please type the motor name following the threshold condition.\r\nThe motor names are:\r\nV1 - ventilation motor in zone 1\r\nV2 - ventilation motor in zone 2\r\nI1 - irrigation motor in zone 1\r\nI2 - irrigation motor in zone 2\r\nRange of thresholds for temperature sensor is 20-30C.\r\nRange of threshold for soil moisture sensor is 1-50%\r\n";
const char lsetup[20] = "\r\nEXITING SETTINGS\r\n";
const char c[2] = "Y";
const char nc[2] = "N";
const char V1[3] = "V1";
const char I1[3] = "I1";
const char V2[3] = "V2";
const char I2[3] = "I2";
int i = 0;
char motor[3] = {0,0,0};
char reply[2] = {0,0};
int threshold;
char question1[47] = "Which motor would you like to enable/disable? ";
char question2[54] = "\r\nPlease set the threshold conditions for the motor: ";
char question3[39] = "\r\nAre you done activating the motors? ";
char question4[48] = "\r\nDo you want to enable or disable this motor? ";
char error[18] = "\r\nInvalid input\r\n";
int V1T = 0;
int V2T = 0;
int I1T = 0;
int I2T = 0;
int enV1 = 0;
int enV2 = 0;
int enI1 = 0;
int enI2 = 0;
int day = 200;
int change1 = 0;
int change2 = 0;

void main(void)
{
  Init();
  Settings(); //for motor thresholds
  //enabling motor drivers
  GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
  GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN3);

   while (1){
    //for debouncing LEDs
    if (change1 == 1) {
        change1 = 0;
        if (buttonState1 == 0) {
            buttonState1 = 1;
            buttonState2 = 0;
        } else {
            buttonState1 = 0;
        }
    }

    if (change2 == 1) {
        change2 = 0;
        if (buttonState2 == 0) {
            buttonState2 = 1;
            buttonState1 = 0;
        } else {
            buttonState2 = 0;
        }
    }

       volatile char x = (char) echo;
       if (x == 27) {
          GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN7);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN4);
          GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN2);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN7);
          GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN3);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN6);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);
          Settings();
          x = 0;
       }

       Init_ADC2();
       Convert_ADC();
       int light = (int)ADCResult;

       Init_ADC();

       Store_MUXInputs();

       if(light > day) {
          GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN7);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN4);
          GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN2);

          Temp_Read(sensor[0]);
          if ((int)T_temp > V1T && enV1) {
              GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN7);
              GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN3);
          } else {
              GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN7);
              GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN3);
          }
          Temp_Read(sensor[2]);
          if ((int)T_temp > V2T && enV2) {
              GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN6);
              GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN5);
          } else {
              GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN6);
              GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);
          }
       } else {
           GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN7);
           GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN3);
           GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN6);
           GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);

           Soil_Read(sensor[1]);
           if ((int)S_temp < I1T && enI1) {
               GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN2);
           } else {
               GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN2);
           }

           Soil_Read(sensor[3]);
           if ((int)S_temp < I2T && enI2) {
               GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN7);
               GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN4);
           } else {
               GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN7);
               GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN4);
           }
       }

        //Zone 1
        if (buttonState1 == 1){
                Temp_Read(sensor[0]);
                Soil_Read(sensor[1]);
                sprintf(string, "ZONE 1   T %d   M %d",(int)T_temp, (int)S_temp);
                displayScrollText(string);
        }

        // Zone 2 WHY DOES IT TAKE 2 LOOPS TO JUMP TO ZONE 1
        if (buttonState2 == 1){
            Temp_Read(sensor[2]);
            Soil_Read(sensor[3]);
            sprintf(string, "ZONE 2   T %d   M %d",(int)T_temp, (int)S_temp);
            displayScrollText(string);
        }
}

} //end of main

void Init(void){
    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //initialize interrupts for pushbuttons
    GPIO_enableInterrupt(SW1_PORT, SW1_PIN);
    GPIO_selectInterruptEdge(SW1_PORT, SW1_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);

    GPIO_enableInterrupt(SW2_PORT, SW2_PIN);
    GPIO_selectInterruptEdge(SW2_PORT, SW2_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(SW2_PORT, SW2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    __disable_interrupt();
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        echo = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
    __enable_interrupt();
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN1 , GPIO_PRIMARY_MODULE_FUNCTION);
 //   GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT2, ADC_IN_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_INPUT_A9,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

void Init_ADC2(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);
 //   GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT2, ADC_IN_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_INPUT_A8,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    __disable_interrupt();
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);
    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
    __enable_interrupt();
}

//pushbutton interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt
void P1_ISR(void) {
    __disable_interrupt();

    //Start timer on rising edge, stop on falling edge, print counter value
    change1 = 1;

    GPIO_clearInterrupt(SW1_PORT, SW1_PIN);
    __enable_interrupt();
}


#pragma vector=PORT2_VECTOR
__interrupt
void P2_ISR(void) {
    __disable_interrupt();

    //Start timer on rising edge, stop on falling edge, print counter value
    change2 = 1;

    GPIO_clearInterrupt(SW2_PORT, SW2_PIN);
    __enable_interrupt();
}

void Convert_ADC(){
    ADCState = 1;
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
}

void Settings(void){
    for(i = 0; i<12; i++){
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, esetup[i]);
    }

    Motor();

    for(i = 0; i < 18; i++){
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, lsetup[i]);
    }

}

void Motor(void){
    volatile char x = 0;
    int j = 0;
    int k = 0;
    memset(motor, 0, 3);

    for(i = 0; i < 360; i++){
         EUSCI_A_UART_transmitData(EUSCI_A0_BASE, explain[i]);
    }

    for(i = 0; i < 47; i++){
             EUSCI_A_UART_transmitData(EUSCI_A0_BASE, question1[i]);
    }

    while (strcmp(V1, motor) != 0 && strcmp(I1, motor) != 0 && strcmp(V2, motor) != 0 && strcmp(I2, motor) != 0) {
        x = (char)echo;

        if ((x<91 && x>64) || (x<58 && x>47)) {
            motor[j] = x;
            echo = 0;
            x = 0;
            j++;
        }
        if (j == 2 && strcmp("00", motor) != 0 && strcmp(V1, motor) != 0 && strcmp(I1, motor) != 0 && strcmp(V2, motor) != 0 && strcmp(I2, motor) != 0){
            for(k = 0; k < 17; k++){
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, error[k]);
            }
            Motor();
            return;
        }
    }

    Enable:
       {
           for(i = 0; i < 48; i++) {
               EUSCI_A_UART_transmitData(EUSCI_A0_BASE, question4[i]);
           }

           char EorD = 0;
           j = 0;
           while (j != 1) {
               x = (char)echo;
               if((x<91 && x>64) || (x<58 && x>48)){
                   EorD = x;
                   x = 0;
                   echo = 0;
                   j++;
               }
               if (j == 1 && EorD != 'E' && EorD != 'D') {
                   for(k = 0; k < 17; k++){
                       EUSCI_A_UART_transmitData(EUSCI_A0_BASE, error[k]);
                   }
                   goto Enable;
               }
           }

           if (EorD == 'E') {        //enabling motors
               Threshold(motor);
           } else if (EorD == 'D') { //disabling motors
               if (strcmp(V1, motor) == 0){
                   enV1 = 0;
               }
               if (strcmp(V2, motor) == 0){
                   enV2 = 0;
               }
               if (strcmp(I1, motor) == 0){
                   enI1 = 0;
               }
               if (strcmp(I2, motor) == 0){
                   enI2 = 0;
               }
           }
       }
}

void Threshold(char nmotor[3]){
    for(i = 0; i < 54; i++){
         EUSCI_A_UART_transmitData(EUSCI_A0_BASE, question2[i]);
    }
    int k = 0;
    i = 0;
    char t[2] = {0,0};
    volatile char x = 0;
    while (i < 2) {
        x = (char)echo;
        if((x<91 && x>64) || (x<58 && x>47)){
            t[i] = x;
            x = 0;
            echo = 0;
            i++;
        }
        if (i == 2 && (!(t[1]<58 && t[1]>47) || !(t[0]<58 && t[0]>47))) {
            for(k = 0; k < 17; k++){
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, error[k]);
            }
            Threshold(motor);
            return;
        }
    }

    if (strcmp(V1, motor) == 0){
        enV1 = 1;
        V1T = (t[0]-48)*10 + (t[1]-48);
    }
    if (strcmp(V2, motor) == 0){
        enV2 = 1;
        V2T = (t[0]-48)*10 + (t[1]-48);
    }
    if (strcmp(I1, motor) == 0){
        enI1 = 1;
        I1T = (t[0]-48)*10 + (t[1]-48);
    }
    if (strcmp(I2, motor) == 0){
        enI2 = 1;
        I2T = (t[0]-48)*10 + (t[1]-48);
    }

    Activation();
}

void Activation(void){
    int j = 0;
    int k = 0;
    volatile char x = 0;
    memset(reply, 0, 2);

    for(i = 0; i < 39; i++){
         EUSCI_A_UART_transmitData(EUSCI_A0_BASE, question3[i]);
    }

    while (strcmp(c, reply) != 0 && strcmp(nc, reply) != 0) {
        x = (char)echo;
        if (x<91 && x>64) {
            echo = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
            reply[j] = (char)echo;
            echo = 0;
            x = 0;
            j++;
        }
        if (j == 1 && strcmp("00", reply) != 0 && strcmp(c, reply) != 0 && strcmp(nc, reply) != 0){
            for(k = 0; k < 17; k++){
                EUSCI_A_UART_transmitData(EUSCI_A0_BASE, error[k]);
            }
            Activation();
            return;
        }
    }

    if(strcmp(nc, reply) == 0){
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13);
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10);
        Motor();
    }
}

void Store_MUXInputs(){
    GPIO_setOutputLowOnPin(SELECTOR0_PORT, SELECTOR0_PIN);
    GPIO_setOutputLowOnPin(SELECTOR1_PORT, SELECTOR1_PIN);
    GPIO_setOutputHighOnPin(SELECTOR2_PORT, SELECTOR2_PIN);

    _delay_cycles(300000);

    if ((GPIO_getInputPinValue(SELECTOR0_PORT, SELECTOR0_PIN) == 0) && (GPIO_getInputPinValue(SELECTOR1_PORT, SELECTOR1_PIN) == 0) && (GPIO_getInputPinValue(SELECTOR2_PORT, SELECTOR2_PIN) == 1)){
        if (ADCState == 0){
            Convert_ADC();
        }

        sensor[0] = (float)ADCResult;

    }

    GPIO_setOutputLowOnPin(SELECTOR0_PORT, SELECTOR0_PIN);
    GPIO_setOutputHighOnPin(SELECTOR1_PORT, SELECTOR1_PIN);
    GPIO_setOutputHighOnPin(SELECTOR2_PORT, SELECTOR2_PIN);

    _delay_cycles(300000);

    if ((GPIO_getInputPinValue(SELECTOR0_PORT, SELECTOR0_PIN) == 0) && (GPIO_getInputPinValue(SELECTOR1_PORT, SELECTOR1_PIN) == 1) && (GPIO_getInputPinValue(SELECTOR2_PORT, SELECTOR2_PIN) == 1)){
        if (ADCState == 0){
            Convert_ADC();
        }

        sensor[1] = (float)ADCResult-237;
        if (sensor[1]<0){
            sensor[1]=0;
        }
    }

    GPIO_setOutputHighOnPin(SELECTOR0_PORT, SELECTOR0_PIN);
    GPIO_setOutputHighOnPin(SELECTOR1_PORT, SELECTOR1_PIN);
    GPIO_setOutputHighOnPin(SELECTOR2_PORT, SELECTOR2_PIN);

    _delay_cycles(300000);

    if ((GPIO_getInputPinValue(SELECTOR0_PORT, SELECTOR0_PIN) == 1) && (GPIO_getInputPinValue(SELECTOR1_PORT, SELECTOR1_PIN) == 1) && (GPIO_getInputPinValue(SELECTOR2_PORT, SELECTOR2_PIN) == 1)){
        if (ADCState == 0){
            Convert_ADC();
        }

        sensor[2] = (float)ADCResult;
    }

    GPIO_setOutputHighOnPin(SELECTOR0_PORT, SELECTOR0_PIN);
    GPIO_setOutputLowOnPin(SELECTOR1_PORT, SELECTOR1_PIN);
    GPIO_setOutputHighOnPin(SELECTOR2_PORT, SELECTOR2_PIN);

    _delay_cycles(300000);

    if ((GPIO_getInputPinValue(SELECTOR0_PORT, SELECTOR0_PIN) == 1) && (GPIO_getInputPinValue(SELECTOR1_PORT, SELECTOR1_PIN) == 0) && (GPIO_getInputPinValue(SELECTOR2_PORT, SELECTOR2_PIN) == 1)){
        if (ADCState == 0){
            Convert_ADC();
        }

        sensor[3] = (float)ADCResult;
    }
}

//converts value of temperature into degrees
void Temp_Read(float temp){
    T_temp = temp/1024;
    T_temp = T_temp * 3.3;
    T_temp = T_temp - 0.5;
    T_temp = T_temp*100;
}

//converts value of soil sensor into percentage
void Soil_Read(float temp){
    S_temp = (temp/480)*100;
}

