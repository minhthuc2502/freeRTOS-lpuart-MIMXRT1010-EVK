# freeRTOS-lpuart-MIMXRT1010-EVK

Project aims to control robotic arm Lynxmotion AL5D by board developement MIMXRT1010-EVK. This board connect to USB Servo Controller by UART.
In this case, i used uart1 for the communication between board and robotic arm. Direct serial connection:
- TX from MIMXRT1010-EVK to RX on the SSC-32U
- RX from MIMXRT1010-EVK to TX on the SSC-32U
- GND on MIMXRT1010-EVK to GND on the SSC-32U
After flashing code on your USB Servo controller, press button SW4 on board MIMXRT1010-EVK to control arm with mode 1 and and press 2 consecutive times to set mode 2.

You can also connect 2 board imxrt1010 with protocol I2C. Board settings :

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
MASTER_BOARD        CONNECTS TO         SLAVE_BOARD
Pin Name     Board Location     Pin Name     Board Location
LPI2C1_SCL   J57-20             LPI2C1_SCL   J57-20
LPI2C1_SDA   J57-18             LPI2C1_SDA   J57-18
GND          J57-14             GND          J57-14
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Press button SW4 on the first board (master) to send commands to second board (slave). Second board will control arm with these commands.

## Requirement

- [MIMXRT1010-EVK](https://www.nxp.com/design/development-boards/i-mx-evaluation-and-development-boards/i-mx-rt1010-evaluation-kit:MIMXRT1010-EVK)
- [Robotic arm Lynxmotion AL5D](http://www.lynxmotion.com/c-130-al5d.aspx)
- Cables

## Software

- [MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE)

## Environment

- Linux version 4.15.0-108-generic
- compiler GCC