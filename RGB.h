/*Implementación para manejar los leds por colores
 * Solo se puede encender o apagar cada color
 *! Asumiendo que ya está el puerto configurado
 * */
#ifndef RGB_H_
#define RGB_H_
#include <stdint.h>

/**TURNING DIFFERENT LED COLOURS ON FUNCTIONS*/

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This initialize all the hardware required for the rgb led.
 	 \param[in] void
 	 \return void
 */

void rgb_init(void);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn on the RED LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void red_on(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn on the BLUE LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void blue_on(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn on the GREEN LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void green_on(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn on the YELLOW LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void yellow_on(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn on the PURPLE LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void purple_on(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn on the WHITE LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void white_on(void);

/**TURNING DIFFERENT LED COLOURS OFF FUNCTIONS*/

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn off the RED LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void red_off(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn off the BLUE LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void blue_off(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn off the GREEN LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void green_off(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn off the YELLOW LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void yellow_off(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn off the PURPLE LED clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void purple_off(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This turn off ALL LEDS from RGB clearing the pin for the corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void rgb_off(void);

/**Sequence Routines*/

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This run the boot routine after a reset and run showing the GREEN LED.
 	 \param[in] DELAY time to waste time in the MCU
 	 \return void
 */
void boot_routine(uint32_t const DELAY);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This run the routine specified by the teacher for SW2 turning on and off the corresponding RGB leds
 	  	  	  for corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void sw2_sequence(void);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This run the routine specified by the teacher for SW3 turning on and off the corresponding RGB leds
 	  	  	  for corresponding GPIO port and pin.
 	 \param[in] void
 	 \return void
 */
void sw3_sequence(void);

#endif /* RGB_H_ */

/***** END OF FILE *****/
