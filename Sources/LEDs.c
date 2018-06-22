/*
 * LEDs.c
 *
 *  Created on: 22 Jun 2018
 *      Author: 11238639
 */


#include "LEDs.h"
#include "MK70F12.h"

/*! @brief Sets up the LEDs before first use.
 *
 *  @return bool - TRUE if the LEDs were successfully initialized.
 */
bool LEDs_Init(void) {
  // enable the clock for PORTA which is what orange LED uses
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  // set PORTA to use GPIO (General Purpose I/O)
  PORTA_PCR11 |= PORT_PCR_MUX(1);
  PORTA_PCR28  |= PORT_PCR_MUX(1);
  PORTA_PCR29  |= PORT_PCR_MUX(1);
  PORTA_PCR10  |= PORT_PCR_MUX(1);

  // find the PCRS for the other leds

  // turn the LEDs off after initialisation
  LEDs_Off(LED_ORANGE);
  LEDs_Off(LED_BLUE);
  LEDs_Off(LED_GREEN);
  LEDs_Off(LED_YELLOW);


  // setting the orange LED bit in the GPIOA register

  // this is setting the output direction of the GPIOA therefore
  // we must set all the output register bits for the LEDs to 1
  // this will ensure we don't flash the LEDs for a brief time
  GPIOA_PDDR |= LED_ORANGE;
  GPIOA_PDDR |= LED_BLUE;
  GPIOA_PDDR |= LED_GREEN;
  GPIOA_PDDR |= LED_YELLOW;


//  PORTA_PCR11 &= ~PORT_PCR_DSE_MASK;

  return true;

}

/*! @brief Turns an LED on.
 *
 *  @param color The color of the LED to turn on.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_On(const TLED color) {
  // Clear the corresponding GPIOA_PDOR bit for the color param (set to 0)
  GPIOA_PCOR = color;
}

/*! @brief Turns off an LED.
 *
 *  @param color THe color of the LED to turn off.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Off(const TLED color) {
  // Set the corresponding GPIOA_PDOR bit for the color param (set to 1)
  GPIOA_PSOR = color;
}

/*! @brief Toggles an LED.
 *
 *  @param color THe color of the LED to toggle.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Toggle(const TLED color) {
  // Toggle the current state of the corresponding GPIOA_PDOR bit for the color param
  GPIOA_PTOR = color;
}
