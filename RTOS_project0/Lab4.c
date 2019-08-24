//*****************************************************************************
//
// Include Files
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "sysctl.h"
#include "gpio.h"
#include "uart.h"
#include "pin_map.h"
#include "adc.h"
#include "pwm.h"

#include "BSP.h"
#include "os.h"




//*****************************************************************************
//
// Define Statements
//
//*****************************************************************************
#define THREADFREQ 1000   // Round Robin Scheduler frequency in Hz
#define GPIO_PA0_U0RX           0x00000001
#define GPIO_PA1_U0TX           0x00000401
#define GPIO_PB0_U1RX           0x00010001
#define GPIO_PB1_U1TX           0x00010401
#define GPIO_PD0_M1PWM0         0x00030005
#define GPIO_PD1_M1PWM1         0x00030405


//*****************************************************************************
//
// Local Function Declarations
//
//*****************************************************************************
void UART0_INIT (void);
void UART0_WRITE(char *data);
char* UART0_READ (void);
void UART1_INIT (void);
void UART1_WRITE(char *data);
char* UART1_READ (void);
void CMD_INTERPRETER(char *data);
void USER_COMM(void);
void ADC0_INIT_SEQ1(void);
void ADC0_INIT_SEQ2(void);
unsigned int ADC0_READ_SEQ1(void);
unsigned int ADC0_READ_SEQ2(void);
void PWM1_INIT(void);
void GPIOA_INIT(void);
void MOTOR_START(char motor, unsigned int speed);
void MOTOR_STOP(char motor);
void MOTOR_DIR(char motor, char direction);
void RIGHT_WALL_ADJUST(void);
void FRONT_WALL_TURN(void);
void Task0(void);
void Task1(void);
void Task2(void);
void Task3(void);
void Task4(void);
void Task5(void);
void Task6(void);




//*****************************************************************************
//
// Global Variable Declarations
//
//*****************************************************************************
bool activeFlag = false;



//*****************************************************************************
//
// Main 'C' Language entry point
//
//*****************************************************************************
int main(void){
		// Initialize Operating System
		OS_Init();
	
		// Initialise UART1 peripheral
		UART1_INIT();

		// Initialize ADC peripherals
		ADC0_INIT_SEQ1();
		ADC0_INIT_SEQ2();
		
		// Initialize PWM0 peripheral
		PWM1_INIT();
	
	  // Initialize GPIOA peripheral
		GPIOA_INIT();
			
		// Add RTOS thread and events
		OS_AddThreads(&USER_COMM, 6, &Task0, 6, &Task1, 6, &Task2, 6, &Task3, 6, &Task4, 6, &Task5, 6, &Task6, 6);	
		OS_AddPeriodicEventThread(&RIGHT_WALL_ADJUST, 50);
		OS_AddPeriodicEventThread(&FRONT_WALL_TURN, 50);
		
		OS_Launch(BSP_Clock_GetFreq()/THREADFREQ); // doesn't return, interrupts enabled in here
		
		return 0;             // this never executes
}



//*****************************************************************************
//
// Local Function Definitions
//
//*****************************************************************************
void UART0_INIT (void) {
	  // Enable UART0 peripheral and GPIO port A.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
	  // Set GPIO B0 and B1 as UART pins.
	  GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	  
	  // Configure the UART for 115,200, 8-N-1 operation.
	  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}

void UART0_WRITE(char data[]) {
		int i = 0;
		
    // Loop while there are more characters to send.
    while((data[i] != '\r') && (data[i] != '\n'))
    {
        // Write the next character to the UART.
        UARTCharPut(UART0_BASE, data[i]);
				i++;
    }
		
		UARTCharPut(UART0_BASE, '\r');
	  UARTCharPut(UART0_BASE, '\n');		 
}
 
char* UART0_READ (void){
	static char data[20] =  "";
	int i = 0;
	
	// Loop while there are more characters to receive. Maximum is 20
	while (i < 20) {
		// Read the next character from the UART
		data[i] =  UARTCharGet(UART0_BASE);
		
		if (data[i] == '\r' || data[i] == '\n') {
			break;
		}
		
		i++;
	}
	data[i++] = '\r';
	data[i] = '\n';
	
	// Clear any remaining characters in the buffer
	while (UARTCharsAvail(UART0_BASE)) {
		UARTCharGetNonBlocking(UART0_BASE);
	}
	
	return data;
}

void UART1_INIT (void) {
	  // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	
	  // Set GPIO B0 and B1 as UART pins.
	  GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	  
	  // Configure the UART for 115,200, 8-N-1 operation.
	  UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}

void UART1_WRITE(char data[]) {
		int i = 0;
		
    // Loop while there are more characters to send.
    while((data[i] != '\r') && (data[i] != '\n'))
    {
        // Write the next character to the UART.
        UARTCharPut(UART1_BASE, data[i]);
				i++;
    }
		
		UARTCharPut(UART1_BASE, '\r');
	  UARTCharPut(UART1_BASE, '\n');		 
}
 
char* UART1_READ (void){
		static char data[20] =  "";
		int i = 0;
		
		// Loop while there are more characters to receive. Maximum is 20
		while (i < 20) {
			// Read the next character from the UART
			data[i] =  UARTCharGet(UART1_BASE);
			
			if (data[i] == '\r' || data[i] == '\n') {
				break;
			}
			
			i++;
		}
		data[i++] = '\r';
		data[i] = '\n';
		
		// Clear any remaining characters in the buffer
		while (UARTCharsAvail(UART1_BASE)) {
			UARTCharGetNonBlocking(UART1_BASE);
		}
		
		return data;
}

void CMD_INTERPRETER(char *data) {
		// Check input string for a valid command
		if ((data[0] == 'G') && (data[1] == 'O')) {
			UART1_WRITE("START\r\n");
			activeFlag = true;
		}
		else if ((data[0] == 'S') && (data[1] == 'T')) {
			UART1_WRITE("STOP\r\n");
			activeFlag = false;
			MOTOR_STOP('r');
			MOTOR_STOP('l');
		}
		else if (data[0] == '\r' && data[1] == '\n') {}
		else {
			UART1_WRITE("Wrong Command\r\n");
		}
}

void USER_COMM(void) {
		char *msg;

		UART1_WRITE("Command: GO / ST\r\n");
	
		while (1) {
			// Read user string
			msg = UART1_READ();
		
			// Check string for valid command
			CMD_INTERPRETER(msg);
		}
}

void ADC0_INIT_SEQ1(void) {
		// Enable ADC0 peripheral and GPIO port E.
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
		// Wait for AC0 to be ready.
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}
			
		// Disable ADC0 sequencer 1 for configuration.
		ADCSequenceDisable(ADC0_BASE, 1);			
			
		// Set GPIO E3 as ADC pin.
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
			
		// Enable sample sequence 1 with an arbitrarily selected priority of 3.
		ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 3);
			
		// Configure steps 0 - 3 on sequence 1. Step 3 is configured as the last conversion.
		ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
														 
		// Enable configured sequence.
		ADCSequenceEnable(ADC0_BASE, 1);
}

void ADC0_INIT_SEQ2(void) {
		// Enable ADC1 peripheral and GPIO port E.
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
		// Wait for ADC1 to be ready.
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}
			
		// Disable ADC1 sequencer 2 for configuration.
		ADCSequenceDisable(ADC0_BASE, 2);	
			
		// Set GPIO E2 as ADC1 pin.
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
		
		// Enable sample sequence 2 with an arbitrarily selected priority of 3.
		ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 3);
		
		// Configure steps 0 - 3 on sequence 2. Step 3 is configured as the last conversion.
		ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
														 
		// Enable configured sequence.
		ADCSequenceEnable(ADC0_BASE, 2);
}

unsigned int ADC0_READ_SEQ1(void) {
		uint32_t pui32ADC0Value, pui32ADC0Values[4];

		// Clear ADC interrrupt flag (good practice).
		ADCIntClear(ADC0_BASE, 1);
	
		// Trigger the ADC conversion.
		ADCProcessorTrigger(ADC0_BASE, 1);

		// Wait for conversion to be completed.
		while(!ADCIntStatus(ADC0_BASE, 1, false)) {}

		// Read ADC Value.
		ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Values);
			
		// Average the 4 step readings for accuracy
		pui32ADC0Value = (pui32ADC0Values[0] + pui32ADC0Values[1] + pui32ADC0Values[2] + pui32ADC0Values[3]) / 4;
		
		// Return read value
		return pui32ADC0Value;
}

unsigned int ADC0_READ_SEQ2(void) {
		uint32_t pui32ADC0Value, pui32ADC0Values[4];

		// Clear ADC interrrupt flag (good practice).
		ADCIntClear(ADC0_BASE, 2);
	
		// Trigger the ADC conversion.
		ADCProcessorTrigger(ADC0_BASE, 2);

		// Wait for conversion to be completed.
		while(!ADCIntStatus(ADC0_BASE, 2, false)) {}

		// Read ADC Value.
		ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0Values);
			
		// Average the 4 step readings for accuracy
		pui32ADC0Value = (pui32ADC0Values[0] + pui32ADC0Values[1] + pui32ADC0Values[2] + pui32ADC0Values[3]) / 4;
		
		// Return read value
		return pui32ADC0Value;
}

void PWM1_INIT(void) {
		// Enable peripherals
		SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	
		// Wait for the peripherals to be ready.
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1)) {}
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}
		
		// Set PWM clock to run at system clock
		SysCtlPWMClockSet(SYSCTL_PWMDIV_1);	
			
		// Set GPIO D0 and D1 as PWM1 bits
		GPIOPinConfigure(GPIO_PD0_M1PWM0);
		GPIOPinConfigure(GPIO_PD1_M1PWM1);
		GPIOPinTypePWM(GPIO_PORTD_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
			
		// Configure the PWM generator 0 for count down mode (left aligned pulses) with immediate updates to the parameters.
		// Generator 0 is tied to PWM pins 0 and 1
		PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
			
		// Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20 microseconds.
		// For a 50 MHz clock, this translates to 1000 clock ticks.
		PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 1000);
	
		// Set the pulse width of outputs 0 and 1 for a 50% duty cycle.
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 500);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 500);

		// Start the timers in generator 0.
		PWMGenEnable(PWM1_BASE, PWM_GEN_0);
			
		// Enable the outputs.
		PWMOutputState(PWM1_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), false); 
}

void GPIOA_INIT(void) {
		// Enable the GPIO port A peripheral
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

		// Wait for the GPIOA module to be ready.
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
			
		// Set pins 2 and 3 as outputs, software controlled
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
			
		// Write a low signal to the output pins
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, !(GPIO_PIN_2 | GPIO_PIN_3)); 
}

void MOTOR_START(char motor, unsigned int speed) {
		// Disable PWM1 generator 0.
		PWMGenDisable(PWM1_BASE, PWM_GEN_0);
	
		// Adjust designated motor speed (0 - 1000).
		switch (motor) {
			case 'r':
				// Right motor uses output 0
				PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, speed);
				PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
				break;
			
			case 'l':
				// Left motor uses output 1
				PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, speed);
				PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
				break;
		}
		
		// Enable PWM1 generator 0.
		PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

void MOTOR_STOP(char motor) {
		// Disable designated output
		switch (motor) {
			case 'r':
				// Right motor uses output 0
				PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
				break;
			
			case 'l':
				// Left motor uses output 1
				PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false);
				break;
		} 
}

void MOTOR_DIR(char motor, char direction) {
		// Adjust designated motor direction
		switch (motor) {
			case 'r':
				// Right motor uses pin 2
				if (direction == 'f')   // forward
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, !GPIO_PIN_2); 
				else if (direction == 'b')  // backward
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
				break;
				
			case 'l':
				// Left motor uses pin 3
				if (direction == 'f')   // forward
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, !GPIO_PIN_3); 
				else if (direction == 'b')   // backward
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
				break;
		}
}

void RIGHT_WALL_ADJUST(void) {
		if (activeFlag) {
			//Set the intial values 
			unsigned int distance;
			const int desiredSideReading = 1500;
			int error, correction;
			static int previousError = 0;
			int pValue = 0, iValue = 0, dValue = 0;
			const float pConstant = 0.25, iConstant = 0.025, dConstant = 0.025;
			const int speed = 1000;
			int speedDecrease;
			
			// Read side sensor
			distance = ADC0_READ_SEQ1();
		
			// Calculate the error value
			error = desiredSideReading-distance;
		
			// Calculate the P,I,D values
			pValue = error * pConstant;
			iValue += error;
			iValue *= iConstant;
			dValue = error - previousError;
		 
		
			// Final error value is calculated using PID equation
			correction = pValue * pConstant + iValue * iConstant + dValue * dConstant;
			
			
			char correction_value[25], error_value[25];	
			sprintf(correction_value, "correction: %d\r\n", correction);
			UART1_WRITE(correction_value);
			sprintf(error_value, "error: %d\r\n", error);
			UART1_WRITE(error_value);
			UART1_WRITE("\r\n");
			
			
			// If the robot is moving towards right wall
			if (correction <= -15 && correction >= -100) {
				speedDecrease = 5.5 * correction;
					
				sprintf(correction_value, "Left wheel speedDecrease: %d\r\n", speedDecrease);
				UART1_WRITE(correction_value);
					
				MOTOR_START('r',speed);
				MOTOR_START('l',speed + speedDecrease);
			}
			// If the robot is moving towards left wall
			else if (correction >= 5 && correction <= 40) {
				speedDecrease = 16.8 * correction;
					
				sprintf(correction_value, "Right wheel speedDecrease: %d\r\n", speedDecrease);
				UART1_WRITE(correction_value);
					
				MOTOR_START('l',speed);
				MOTOR_START('r',speed - speedDecrease);
			}
			// If the robot encounters right turn intersection
			else if (correction > 40) {
				speedDecrease = 9 * correction;
						
				sprintf(correction_value, "Right turn, speedDecrease: %d\r\n", speedDecrease);
				UART1_WRITE(correction_value);
						
				MOTOR_START('l',speed);
				MOTOR_START('r',speed - speedDecrease);
			}
			// If the robot is in center
			else {
				UART1_WRITE("No decrease\r\n");
					
				MOTOR_START('l',speed);
				MOTOR_START('r',speed);
			}
			
			// Update previous cycle error
			previousError = error;
		}
}

void FRONT_WALL_TURN() {
		if (activeFlag) {
			const int speed = 1000;
			unsigned int distanceSide, distanceFront;
		
			// Get distance sensor values
			distanceSide = ADC0_READ_SEQ1();
			distanceFront = ADC0_READ_SEQ2();
		
			// Check for front wall intersection
			if(distanceFront >= 2350 && (distanceSide >= 1100 && distanceSide <= 2700)) {
					// Adjust motor direction and speed
					MOTOR_DIR('l','b');
					MOTOR_START('l',speed);
					MOTOR_START('r',speed);
				
				// Perform u-turn
				while(!(distanceFront <= 1300) || !(distanceSide <= 1800)) {
						distanceSide = ADC0_READ_SEQ1();
						distanceFront = ADC0_READ_SEQ2();
					
						// Small delay to allow for proper sensor reading
						SysCtlDelay(50000);
				}
				MOTOR_DIR('l','f');
			}
		}
}

void Task0(void) {
		while (1) {}
}

void Task1 (void) {
		while (1) {}
}

void Task2 (void) {
		while (1) {}
}

void Task3 (void) {
		while (1) {}
}

void Task4 (void) {
		while (1) {}
}

void Task5 (void) {
		while (1) {}
}

void Task6 (void) {
		while (1) {}
}
