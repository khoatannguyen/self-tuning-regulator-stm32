#include "inc/lm4f230h5qr.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include <inc/hw_types.h>

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
double counter;
void SysTickIntHandler(void)
{
	GPIO_PORTF_DATA_R ^= 0x02;
}

int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  //80MHz

	IntMasterEnable();
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);;
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	SysTickPeriodSet(SysCtlClockGet());
	SysTickEnable();
	SysTickIntEnable();
	while(1)
	{
		counter = SysTickPeriodGet();
	}
}
