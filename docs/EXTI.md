## Extended interrupt and event controller (EXTI)

The Extended interrupt and event controller (EXTI) manages the CPU and system wakeup through configurable and direct event inputs (lines). It provides wakeup requests to the power control, and generates an interrupt request to the CPU NVIC and events to the CPU event input. For the CPU an additional event generation block (EVG) is needed to generate the CPU event signal.
The EXTI wakeup requests allow the system to be woken up from Stop modes.
The interrupt request and event request generation can also be used in Run modes. The EXTI also includes the EXTI I/O port mux.


### Configuring GPIO Interupts using EXTI lines

[refer](https://www.allaboutcircuits.com/technical-articles/how-to-use-gpio-interrupts/)

### Procedure to turn on an Interupt in STM32G070

* Configure the GPIO Pin Mode as EXTI rising, falling or both (in software structure)
* Configure the EXTICR register to select the port of the pin which should trigger the interupt
* Configure the RTSR1 or FTSR1 register for triggering rising or falling edge in the corresponding EXTI line
* Enable the corresponding EXTI IRQ Number in the NVIC_ISER register (refer. -> [See the ARMv6-M Architecture Reference Manual for more information about the NVIC registers and their addresses, access types, and reset values.])
* Configure the priority of the IRQ using the Interrupt Priority Registers, NVIC_IPR0 - NVIC_IPR7
* Implement the Interupt Service Routines for the respective IRQ with the same name in the weak definiton of the ISR from the startup code in the application code. [refer](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/learn/lecture/14277520#questions)

##### NOTE:

Use SYSCFG interrupt line X status register (SYSCFG_ITLINEX) to collect all pending interrupt sources associated with each interrupt line into a single register. This allows users to check by single read which peripheral requires service in case more than one source is associated to the interrupt line.

All bits in those registers are read only, set by hardware when there is corresponding interrupt request pending and cleared by resetting the interrupt source flags in the peripheral registers.

