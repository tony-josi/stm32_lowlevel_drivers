
### GPIO

All GPIO pins will be by default open drain (neither high nor low), for practical applications its necessary to pull it down or up (low or high); almost all MCUs have internal pull up or pull down registers that can be configured by setting appropriat CFG registers.

In input mode, for each clock cycle of the AHB bus (to which GPIO is connected) the data from the pin is read to the input register of that pin.

#### GPIO Output modes

* Push - pull -> can drive both high & low
* Open drain -> can drive only low, when high it will float, pull up resistor connected to Vcc is needed to make it high.

#### Example GPIO configuration to drive an LED:

Output mode can be -> Push pull mode
pull up / pull down settings ->  no pull up no pull down

If pull up or pull down is ON the current should flow through that to drive the LED, but the iternal pull up/ down registers have very high resistance, so external low resistance resistors should be used. [refer](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/learn/lecture/14265222#questions)

#### GPIO pin configuration

[src1](https://electronics.stackexchange.com/a/28096/241629)

[src2](https://electronics.stackexchange.com/questions/156930/stm32-understanding-gpio-settings)

Output pins can be driven in three different modes:

* open drain - a transistor connects to low and nothing else
* open drain, with pull-up - a transistor connects to low, and a resistor connects to high
* push-pull - a transistor connects to high, and a transistor connects to low (only one is operated at a time)

Input pins can be a gate input with a:

* pull-up - a resistor connected to high
* pull-down - a resistor connected to low
* pull-up and pull-down - both a resistor connected to high and a resistor connected to low (only   useful in rare cases).

### GPIO port mode register (GPIOx_MODER) (x =A to D, F)

This register controls the mode of each pins of the each GPIO. 

### GPIO Setting particular alternate function (AF) for a pin

AFs can be configured for each pins in a port using:

* GPIO alternate function low register (GPIOx_AFRL) &
* GPIO alternate function high register (GPIOx_AFRH)


### Enabling clock for GPIO ports

I/O port clock enable register (RCC_IOPENR)

#### Example: 
`Bits31:0` MODE\[15:0]\[1:0]: Port x configuration
I/Opiny(y=15to0) 

These bits are written by software to configure the I/O mode.

* 00: Input mode
* 01: General purpose output mode 
* 10: Alternate function mode,  Example: UART, I2C, SPI
* 11: Analog mode (reset state)


## Apart from the register structure what is the difference between GPIOx_ODR and GPIOx_BSRR? Is GPIOx_BSRR an abstraction layer for GPIOx_ODR? I know that a change in GPIOx_BSRR "will" change the GPIOx_ODR but how and what are the diferences?

The BSRR has bitfields that allow you to set and clear bits in a port atomically--without a read-modify-write operation. Instead of reading the ODR value, ORing it with the bits to set, and writing it back, you simply perform a single 32-bit write to the BSRR to set or only the relevant bits.

This often means you don't have to disable interrupts or use other concurrency protections when using the BSRR, and results in smaller and faster code for bit twiddling operations.

[source](https://electronics.stackexchange.com/a/99109/241629)
