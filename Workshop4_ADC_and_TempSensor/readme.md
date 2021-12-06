# Workshop 4 - STM32 ADC and Temperature sensor

Device consists of STM32F407-DISCOVERY and Global Logic Embedded Starter Kit (GLESK) Extension Board.

There are 4 LEDs on a stm32 board, 3 of which are active in this project. 
Blue LED is controlled by the analog input from pin_PA3 connected to a potentiometer. 
Orange LED is controlled by the analog input from stm32 internal temperature sensor.
Green LEd is controlled by the analog input from external temperature sensor, connected to GLESK extension board.

Demo-video on YouTube: -

***

## User manual

<ins>**User control**</ins>

Potentiometer (clockwise) - increase LED_blue brightness.
Potentiometer (anti-clockwise) - descrease LED_blue brightness.

Internal temp. sensor (heat up) - increase LED_orange brightness.
Internal temp. sensor (cool down) - decrease LED_orange brightness.

Ext. temp. sensor (heat up) - increase LED_green brightness.
Ext. temp. sensor (cool down) - decrease LED_green brightness.

***

*Made by Viacheslav Holubnychyi.*
