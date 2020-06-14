# Peripherals

- [Peripherals](#peripherals)
  - [UART](#uart)
        - [Example:](#example)


## GPIO

All GPIO pins will be by default open drain (neither high nor low), for practical applications its necessary to pull it down or up (low or high); almost all MCUs have internal pull up or pull down registers that can be configured by setting appropriat CFG registers

## UART
The UART function can perform back-to-back transfers. If data is available in time, the transmitter does
not generate an idle line signal, but transmits exactly 1 stop bit followed by the start bit for the next data
frame. An idle line condition only occurs if the transmit data register is empty after the transmit data has
been serially shifted out. The length of a transmit idle line condition is always a multiple of a bit time. The
receiver can handle any length of idle line.

Every data word begins with 1 start bit, which is always a logic zero. Following the start bit, a specified
number of data bits are transmitted least significant bit first; then, if parity is enabled, a parity bit is
generated and transmitted. The end of the data word is marked by 1 stop bit, which is always a logic one.
An idle line consists of successive stop bits, which means that the line is at logic level one while idle.
For example, the ASCII character “A” (represented as 8-bits, hexadecimal 0x41) is always transmitted as
shown below in Example, Note that the data is transmitted lsb first.

##### Example:
| 0 | 10000010 | parity-bit | 1 |

| start | data | parity-bit | stop |

This application note uses the term “bit time” to refer to the time required to transmit or receive one bit of
data. Bit time is determined by baud rate, using the formula:

`Bit Time = 1/Baud Rate`

The receiver detects a data word by sensing the falling edge (high to low transition) of the start bit. Since
the UART function always treats the first falling edge after the initialization service request as a valid start
bit, a receiver must be enabled only when the line is idle. 

## I²C 

(Inter-Integrated Circuit), pronounced I-squared-C, is a synchronous, multi-master, multi-slave, packet switched, single-ended, serial computer bus invented in 1982 by Philips Semiconductor (now NXP Semiconductors). It is widely used for attaching lower-speed peripheral ICs to processors and microcontrollers in short-distance, intra-board communication. Alternatively, I²C is spelled I2C (pronounced I-two-C) or IIC (pronounced I-I-C).

The aforementioned reference design is a bus with a clock (SCL) and data (SDA) lines with 7-bit addressing. The bus has two roles for nodes: master and slave:

* Master node – node that generates the clock and initiates communication with slaves.
* Slave node – node that receives the clock and responds when addressed by the master.

The bus is a multi-master bus, which means that any number of master nodes can be present. Additionally, master and slave roles may be changed between messages (after a STOP is sent).

There may be four potential modes of operation for a given bus device, although most devices only use a single role and its two modes:

* master transmit – master node is sending data to a slave,
* master receive – master node is receiving data from a slave,
* slave transmit – slave node is sending data to the master,
* slave receive – slave node is receiving data from the master.

#### Timing diagram

* Data transfer is initiated with a start condition (S) signaled by SDA being pulled low while SCL stays high.
* SCL is pulled low, and SDA sets the first data bit level while keeping SCL low (during blue bar time).
* The data are sampled (received) when SCL rises for the first bit (B1). For a bit to be valid, SDA must not change between a rising edge of SCL and the subsequent falling edge (the entire green bar time).
* This process repeats, SDA transitioning while SCL is low, and the data being read while SCL is high (B2, ...Bn).
* The final bit is followed by a clock pulse, during which SDA is pulled low in preparation for the stop bit.
* A stop condition (P) is signaled when SCL rises, followed by SDA rising.

## EXTI mux

The EXTI mux allows selecting GPIOs as interrupts and wakeup. The GPIOs are connected via 16 EXTI mux lines to the first 16 EXTI events as configurable event. The selection of GPIO port as EXTI mux output is controlled through the EXTI external interrupt selection register (EXTI_EXTICRx) register.
