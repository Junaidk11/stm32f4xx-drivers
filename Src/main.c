/*
 * main.c
 *
 *  Created on: Jun 6, 2020
 *      Author: junaidkhan
 */


#include "gpio_driver.h" 

int main(void){

    return 0; 
}

void EXTI0_IRQHandler(){

    //Handle the Interrupt - call the Driver IRQ handling API here. 
    GPIO_IRQHandling(0); // I/O Pin number 0 deliver's interrupts on the EXTI0 line
}