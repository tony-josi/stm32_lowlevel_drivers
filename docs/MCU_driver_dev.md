## MCU Driver Development Notes

### Basic Architecture of an Application

* Application Software 
* Device Drivers
    * Device Header Files
        * Contains: 
            * Base addresses of different memories, such as FLASH, SRAM1, SRAM2, etc.
            * Base address of bus domains, such ad AHB, APB
            * Base address of various peripherals in connected to the bus domains
            * Clock management macros
            * IRQ definitions
            * Peripheral definition structures for each peripherals
            * Peripheral register bit def. and masks
* Bare metal MCU