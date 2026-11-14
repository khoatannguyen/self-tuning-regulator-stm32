# Tiva LM4F tests

`Read_encoder.c` and `Systick.c` are early tests of encoder reading and SysTick timing on a TI Tiva LM4F (Stellaris LaunchPad) board, written before the project moved to the STM32F407.

The self-tuning regulator performed better on the STM32 chip. On the Tiva chip it seemed unable to keep up with the computation, or the code had a mistake, so the Tiva port was not pursued. These files are kept for reference only. The full controller lives in `firmware/` for the STM32.
