# Self-Tuning Regulator for DC Motor Velocity and Position Control (STM32F4)

Paper, thesis and Keil MDK-ARM firmware for **"Design a Self-Tuning Regulator for DC Motor's Velocity and Position Control"** by Khoa Nguyen (Department of Automation and Control Engineering, Ho Chi Minh City University of Technology, 2013).

## Summary

The paper presents the design, implementation, optimisations and experimental results of a Self-Tuning Regulator (STR) that combines Recursive Least Squares (RLS) estimation, with a forgetting factor, and Model Reference Adaptive Control (MRAC). It runs on an ARM Cortex-M4 STM32F4 microcontroller. The estimator identifies the motor's discrete transfer function online: second order for velocity and third order for position. The control law then forces the motor to follow a reference model designed in MATLAB (sample time 5 ms). In experiments on two different DC motors and under load changes, the STR tracks velocity and position with almost no overshoot and no steady-state error, outperforming a PID controller. This shows that adaptive control is practical on a low-cost general-purpose MCU instead of a dedicated DSP kit.

## Hardware (from the code and the paper)

- **MCU:** STM32F407VG (ARM Cortex-M4F, 168 MHz), STM32F4 Discovery-class board
- **Motor:** brushed DC motor driven through an H-bridge
  - PWM: TIM3 CH1 on PC6, 21 kHz
  - Direction: PD14/PD15
- **Encoder:** quadrature incremental encoder
  - Position: TIM5 encoder mode, PA0/PA1
  - Velocity: TIM5 input capture
- **Control loop:** SysTick at 5 ms
- **Debug:** TIM4 timing on PB7. USART3 at 9600 baud streams data to the PC.

## Layout

```
paper/                      Paper (PDF and Word source)
paper/thesis/               Graduation thesis (Vietnamese) and thesis-format paper
firmware/                   Final Keil project sources (main.c, stm32f4xx_it.c, ...)
firmware/MDK-ARM/           Keil uVision 4 project (Project.uvproj)
firmware/variants/          Alternative versions of main.c / stm32f4xx_it.c
  v2-str-pid-sine/          Extended STR/PID switch, sinusoidal set-point, tuned theta(0)
  main_rng-theta-init.c     theta(0) seeded from the hardware RNG
  stm32f4xx_it_position-mode-early.c  Earlier position-mode ISR / PID gains
firmware/tiva-lm4f-tests/   Early TI Stellaris LM4F (Tiva) encoder and SysTick tests
```

[`hardware/`](hardware/README.md) — boards, H-bridge, pin map, motors and rig photos

The STM32 performed better; the Tiva attempt in [`firmware/tiva-lm4f-tests/`](firmware/tiva-lm4f-tests/README.md) seemed unable to compute fast enough or had a bug, so it was not pursued.

## Building in Keil uVision

1. Download the ST **STM32F4xx DSP and Standard Peripherals Library V1.1.0**. It isn't included here.
2. Copy `firmware/` to `Project/STM32F4xx_StdPeriph_Examples/TIM/<name>/` inside the library. The project references `..\..\..\Libraries` and `..\..\..\Utilities`.
3. Open `MDK-ARM/Project.uvproj` in Keil uVision (MDK-ARM 4 or later; uVision 5 will offer to migrate it).
4. Select the `STM324xG_EVAL` target (STM32F407VG), then build with F7 and flash over ST-LINK.
5. In `stm32f4xx_it.c`, select velocity or position mode with `Velocity_Position`, and STR or PID with `Dynamic_STR` / `Controller`.

To use a variant, replace the matching file in `firmware/` with it.

## Author

Khoa Nguyen. ST library files keep their original STMicroelectronics licence headers.

## Acknowledgements

With thanks to Assoc. Prof. Huỳnh Thái Hoàng (HCMUT). See [ACKNOWLEDGEMENTS.md](ACKNOWLEDGEMENTS.md).

## Licence

- **Author firmware** is released under the [MIT License](LICENSE). Copyright (c) 2014 Khoa Tan Nguyen.
- The **paper and thesis** in [`paper/`](paper/) are licensed under [CC BY 4.0](LICENSE-docs).
- **STMicroelectronics** example / library files keep the MCD-ST Liberty SW License Agreement V2, and **Keil** debug scripts keep their original notices. See [NOTICE.md](NOTICE.md).

Repository: https://github.com/khoatannguyen/self-tuning-regulator-stm32
