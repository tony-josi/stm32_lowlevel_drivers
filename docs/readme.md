# Mircrocontrollers and Architechture

- [Mircrocontrollers and Architechture](#mircrocontrollers-and-architechture)
  - [Microcontroller](#microcontroller)
      - [Memory](#memory)
      - [Instruction Set](#instruction-set)
  - [Bit size of Microcontroller](#bit-size-of-microcontroller)
  - [Embedded SRAM](#embedded-sram)
  - [Flash memory overview](#flash-memory-overview)
  - [Clocks](#clocks)
    - [Clock Sources:](#clock-sources)
  - [Before setting any peripherals registers the coresponding peripherals clock should be enabled](#before-setting-any-peripherals-registers-the-coresponding-peripherals-clock-should-be-enabled)
  - [Vector Tables](#vector-tables)
  - [NVIC](#nvic)
  - [Interupts from perpherals](#interupts-from-perpherals)
  - [GPIO](#gpio)
    - [Configuring GPIO Pins](#configuring-gpio-pins)
        - [GPIO_PuPd (Pull-up / Pull-down)](#gpiopupd-pull-up--pull-down)
        - [GPIO_OType (Output Type):](#gpiootype-output-type)
        - [GPIO_Speed](#gpiospeed)
        - [GPIO output level](#gpio-output-level)
        - [GPIO Mode](#gpio-mode)
  - [OSC clock sources for STM32 NUCLEO-G070RB](#osc-clock-sources-for-stm32-nucleo-g070rb)

## Microcontroller

A microcontroller is a small computer on a single metal-oxide-semiconductor integrated circuit chip.

#### Memory
Based on the memory configuration, the microcontroller is further divided into two categories.

* External memory microcontroller − This type of microcontroller is designed in such a way that they do not have a program memory on the chip. Hence, it is named as external memory microcontroller. For example: Intel 8031 microcontroller.

* Embedded memory microcontroller − This type of microcontroller is designed in such a way that the microcontroller has all programs and data memory, counters and timers, interrupts, I/O ports are embedded on the chip. For example: Intel 8051 microcontroller.

#### Instruction Set
Based on the instruction set configuration, the microcontroller is further divided into two categories.

* CISC − CISC stands for complex instruction set computer. It allows the user to insert a single instruction as an alternative to many simple instructions.

* RISC − RISC stands for Reduced Instruction Set Computers. It reduces the operational time by shortening the clock cycle per instruction.


## ARMv6-M instruction set (used in Cortex M0+)

ARMv6-M instruction set comprises:
* All of the 16-bit Thumb instructions from ARMv7-M excluding CBZ, CBNZ and IT.
* The 32-bit Thumb instructions BL, DMB, DSB, ISB, MRS and MSR.

## Thumb Instruction Set
The ARM processor has 2 instruction sets, the traditional ARM set, where the instructions are all 32-bit long, and the more condensed Thumb set, where most common instructions are 16-bit long (and some are 32-bit long). Which instruction set to run can be chosen by the developer, and only one set can be active (i.e. once the processor is switched to Thumb mode, all instructions will be decoded as using the Thumb instead of ARM).

Although they are different instruction sets, they share similar functionality, and can be represented using the same assembly language. For example, the instruction

``` ASM
ADDS  R0, R1, R2
```

can be compiled to ARM (E0910002 / 11100000 10010001 00000000 00000010) or Thumb (1888 / 00011000 10001000). Of course, they perform the same function (add r1 and r2 and store the result to r0), even if they have different encodings. This is the meaning of Thumb instructions are 16 bits long,and have a corresponding 32-bit ARM instruction that has the same effect on processor model.

Every* instruction in Thumb encoding also has a corresponding encoding in ARM, which is meant by the "subset" sentence.

**NOTE:**

The original Thumb-Instruction set only contained 16-bit instructions. Thumb2 introduced mixed 16/32 bit instructions. Thumb(1) was just a compressed version of the ARM-Instruction set. The CPU would enable a "decompressor" on instruction fetching so in the end the CPU still processed ARM-Instructions. For ARM this most probably was a quick but elegant hack to trim down on code size and ICache-Utilization with little changes to the actual cores. Thumb2 added a lot of new features like the mentioned "IT*" Instruction and some 32-Bit instructions.

[source](https://stackoverflow.com/a/10638621/6792356)

## Bit size of Microcontroller

A 32 bit microcontroller can address 32 bits of memory, ie a maximum of `0x FFFF FFFF` (4GB). All peripherals including flash memory, SRAM, GPIO, Advanced High Performance Bus (AHB), A peripheral Bus (APB), etc. are addressed via this memory range.

## Embedded SRAM

STM32G070xx devices feature 32 Kbytes of SRAM (static RAM) with parity check enabled
and 36Kbytes with parity check disabled. STM32G030xx devices have 8Kbytes of SRAM regardless of the parity check configuration.
The SRAM can be accessed by bytes, half-words (16 bits) or full words (32 bits), at maximum system clock frequency without wait state and thus by both CPU and DMA.

## Flash memory overview

The Flash memory is composed of two distinct physical areas:
* The main Flash memory block. It contains the application program and user data if
necessary. (A Main memory block containing 64 pages of 2Kbytes, each page with eight rows of
256bytes for STM32G070) 
* The information block. It is composed of three parts:
    * Option bytes for hardware and memory protection user configuration.
    * System memory which contains the proprietary boot loader code. System memory from which the CPU boots in System memory boot mode. The area is reserved and contains the boot loader used to reprogram the Flash memory through one of the following interfaces: USART1, USART2, I2C1, I2C2, and on STM32G070xx devices also through USART3, SPI1, and SPI2. On the manufacturing line, the devices are programmed and protected against spurious write/erase operations
    * OTP (one-time programmable) area.  1Kbyte (128 double words) OTP (one-time programmable) for user data. The
      OTP data cannot be erased and can be written only once. If only one bit is at 0,
      the entire double word (64 bits) cannot be written anymore, even with the value 0x0000 0000 0000 0000.
      The OTP area cannot be read when RDP level is 1 and boot source is not the
      Main Flash memory area.

The Flash interface implements instruction access and data access based on the AHB
protocol. It implements the prefetch buffer that speeds up CPU code execution. It also implements the logic necessary to carry out the Flash memory operations (Program/Erase)
controlled through the Flash registers.

## Clocks

The device provides the following clock sources producing primary clocks:
* HSI RC - a high-speed fully-integrated RC oscillator producing HSI16 clock (about
16 MHz)
* HSE OSC - a high-speed oscillator with external crystal/ceramic resonator or external
clock source, producing HSE clock (4 to 48LHz)
* LSI RC - a low-speed fully-integrated RC oscillator producing LSI clock (about 32kHz)
* LSE OSC - a low-speed oscillator with external crystal/ceramic resonator or external
clock source, producing LSE clock (accurate 32.768kHz or external clock up to 1MHz)
* I2S_CKIN - pin for direct clock input for I2S1 peripheral

Each oscillator can be switched on or off independently when it is not used, to optimize
power consumption.

### Clock Sources:

* Crystal (External) (HSE)
* RC (Internal) (HSI)
* PLL (Phase Locked Loop) (Internal) - PLL is used to multiply or modify the HSE or HSI clocks to obtain a derived clock value so that it can be used as the source for SYSCLK

## Before setting any peripherals registers the coresponding peripherals clock should be enabled

The clock should be enabled by checking - to which bus the peripheral is connected and use the bus clock enable register to set the clock

[refer](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/learn/lecture/17154402#content)


## Vector Tables

Holds the addresses of various exceptions (System Exceptions and Interupts)

## RCC

RCC (Reset and clock control) is the engine which control clock for all the domain of the MCU. The RCC peripheral is used to control the internal peripherals, as well as the reset signals and clock distribution. The RCC gets several internal (LSI, HSI and CSI) and external (LSE and HSE) clocks. They are used as clock sources for the hardware blocks, either directly or indirectly, via the four PLLs (PLL1, PLL2, PLL3 and PLL4) that allow to achieve high frequencies.



## NVIC

Nested vector interrupt control (NVIC) is a method of prioritizing interrupts, improving the MCU's performance and reducing interrupt latency.

The NVIC:

* facilitates low-latency exception and interrupt handling
* controls power management
* implements System Control Registers.

The NVIC supports up to 240 dynamically reprioritizable interrupts each with up to 256 levels of priority. The NVIC and the processor core interface are closely coupled, which enables low latency interrupt processing and efficient processing of late arriving interrupts. The NVIC maintains knowledge of the stacked (nested) interrupts to enable tail-chaining of interrupts.

You can only fully access the NVIC from privileged mode, but you can pend interrupts in user-mode if you enable the Configuration Control Register (see Configuration Control Register). Any other user-mode access causes a bus fault.

All NVIC registers are accessible using byte, halfword, and word unless otherwise stated.

All NVIC registers and system debug registers are little endian regardless of the endianness state of the processor.

[refer](https://developer.arm.com/docs/ddi0337/e/nested-vectored-interrupt-controller/about-the-nvic)


## Interupts from perpherals

All interupts from perpherals has to reach NVIC for being handled. Some perpherals such as SPI has interupts directly reaching the NVIC others has to be implemented via the **EXTI (External interrupt/event controller).** 

For STM32F4 the external interrupt/event controller consists of up to 23 edge detectors for generating
event/interrupt requests. Each input line can be independently configured to select the type
(interrupt or event) and the corresponding trigger event (rising or falling or both). Each line
can also masked independently. A pending register maintains the status line of the interrupt
requests.

## GPIO

### Configuring GPIO Pins

* GPIO_InitStructure.GPIO_Speed     - GPIO Speed
* GPIO_InitStructure.GPIO_OType     - GPIO Output type
* GPIO_InitStructure.GPIO_PuPd      - GPIO pull-up/pull-down

##### GPIO_PuPd (Pull-up / Pull-down)

In digital circuits, is is important that signal lines are never allowed to "float". That is, they need to always be in a high state or a low state. When floating, the state is undetermined, and causes a few different types of problems.

The way to correct this is to add a resistor from the signal line either to Vcc or Gnd. That way, if the line is not being actively driven high or low, the resistor will cause the potential to drift to a known level.

The ARM (and other microcontrollers) have built-in circuitry to do this. That way, you don't need to add another part to your circuit. If you choose "GPIO_PuPd_UP", for example, it is equivelent to adding a resistor between the signal line and Vcc.

##### GPIO_OType (Output Type):

Push-Pull: This is the output type that most people think of as "standard". When the output goes low, it is actively "pulled" to ground. Conversely, when the output is set to high, it is actively "pushed" toward Vcc. Simplified, it looks like this:

`pushpull`

An Open-Drain output, on the other hand, is only active in one direction. It can pull the pin towards ground, but it cannot drive it high. Imagine the previous image, but without the upper MOSFET. When it is not pulling to ground, the MOSFET is simply non-conductive, which causes the output to float:

`opendrain`

For this type of output, there needs to be a pull-up resistor added to the circuit, which will cause the line to go high when not driven low. You can do this with an external part, or by setting the GPIO_PuPd value to GPIO_PuPd_UP.

The name comes from the fact that the MOSFET's drain isn't internally connected to anything. This type of output is also called "open-collector" when using a BJT instead of a MOSFET.

##### GPIO_Speed

Basically, this controls the slew rate (the rise time and fall time) of the output signal. The faster the slew rate, the more noise is radiated from the circuit. It is good practice to keep the slew rate slow, and only increase it if you have a specific reason.

##### GPIO output level

This is the logical level at which the pin is set after initialization of the GPIO module. 

##### GPIO Mode

Here you can choose if you want the pin to run in a push-pull configuration where the output can be driven to high or low or open-drain where the output can only be driven to logical low and is open otherwise. In the latter case you should add a pull-up resistor so that you can switch between low and high.

[source](https://electronics.stackexchange.com/questions/156930/stm32-understanding-gpio-settings)

## OSC clock sources for STM32 NUCLEO-G070RB
Three clock sources are listed below:
* LSE which is the 32.768 kHz crystal for the STM32 embedded RTC
* MCO which is the 8 MHz clock from the ST-LINK MCU for the STM32 microcontroller
* HSE which is the 8 MHz oscillator for the STM32 microcontroller. This clock is not implemented on the STM32 NUCLEO-G070RB or NUCLEO-G071RB board

