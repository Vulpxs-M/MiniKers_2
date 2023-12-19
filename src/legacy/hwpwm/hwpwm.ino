/*********************************************************************
 This is an example for our Feather Bluefruit modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
 * This sketch use different Hardware PWMs for LED Blue and Red
 * running with different frequency
 * - PWM0 : clock/1  ~ 16Mhz
 * - PWM1 : clock/16  ~ 1Mhz 
 *  
 * While LED RED looks solid, LED BLUE will blink while fading 
 * (due to its lower freq). Furthermore LED RED is inverted 
 * compared to LED BLUE (PWM2) --> They fade in opposite direction. 
 */

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial

int PORT_ledr = 2;
int PORT_ledg = 16;
int PORT_ledb = 21;

/**************************************************************************/
/*!
    @brief  The setup function runs once when reset the board
*/
/**************************************************************************/
void setup()
{  
  // Add LED RED to PWM0
  HwPWM0.addPin( PORT_ledg );

  // Add LED BLUE to PWM1
  HwPWM1.addPin( PORT_ledb );

  // Enable PWM modules with 15-bit resolutions(max) but different clock div
  HwPWM0.begin();
  HwPWM0.setResolution(15);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1); // freq = 16Mhz
  
  HwPWM1.begin();
  HwPWM1.setResolution(15);
  HwPWM1.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_16); // freq = 1Mhz
}

/**************************************************************************/
/*!
    @brief  The loop function runs over and over again forever
*/
/**************************************************************************/
void loop()
{
  const int maxValue = bit(15) - 1;
  
  // fade in from min to max
  for (int fadeValue = 0 ; fadeValue <= maxValue; fadeValue += 1024) 
  {
    // Write same value but inverted for Led Blue
    HwPWM0.writePin(PORT_ledg, fadeValue, false);
    HwPWM1.writePin(PORT_ledb, fadeValue, true);
        
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }

  // fade out from max to min
  for (int fadeValue = maxValue ; fadeValue >= 0; fadeValue -= 1024) 
  {  
    // Write same value but inverted for Led Blue
    HwPWM0.writePin(PORT_ledg, fadeValue, false);
    HwPWM1.writePin(PORT_ledb, fadeValue, true);
    
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
}
