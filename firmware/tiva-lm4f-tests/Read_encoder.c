#include <stdint.h>
#include <stdbool.h>
#include "inc/lm4f230h5qr.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"


int qei0Position, qei1Position, qei0Velocity, qei1Velocity;
int counterJustForDebug;
int timerMs;
long ulLoop;

//void Delay(int timeMs);
void Delay(unsigned long ulSeconds)
{
    //
    // Loop while there are more seconds to wait.
    //
    while(ulSeconds--)
    {
        for(ulLoop = 0; ulLoop < 100000; ulLoop++)
        {
        }

    }
}

//void UART send
void UARTSend(unsigned char* pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    }
}

int main(void)
{
    counterJustForDebug= 0;
    // Set the clocking to run directly from the crystal.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //Enable QEI module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    //Enable UART module
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Set GPIO A0 and A1 as UART pins.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Configure the UART for 115,200, 8-N-1 operation.
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 921600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5);  //Ph A, PortF enabled previously
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);  //Ph B

    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6);  //Ph A, PortD enabled previously
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_7);  //Ph B

//    QEIDisable(QEI1_BASE);
//    QEIDisable(QEI0_BASE);
//    IntEnable(INT_GPIOC);
//    GPIOPinIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5);
//    IntMasterEnable();
    QEIConfigure(QEI1_BASE,(QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 100000);
    QEIConfigure(QEI0_BASE,(QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 100000);
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet()/100);
    QEIVelocityEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);
    QEIEnable(QEI0_BASE);

    Delay(1);
    QEIPositionSet(QEI1_BASE, 0);
    QEIPositionSet(QEI0_BASE, 0);


    while (1) //This is the main loop of the program
    {
       qei1Position = QEIPositionGet(QEI1_BASE);
       qei0Position = QEIPositionGet(QEI0_BASE);
       qei0Velocity = QEIVelocityGet(QEI0_BASE);
       //ROM_SysCtlDelay(ROM_SysCtlClockGet()/3);
       counterJustForDebug++;
       SysCtlDelay(SysCtlClockGet()/30);
    }
}

void GPIOCIntHandler(void)
{
	GPIOPinIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);
	UARTSend((unsigned char*)&qei0Position, 2);
}
// Timer delay, used for "5 clock cycles must elapse between SysCtlPeripheral Enable() "
