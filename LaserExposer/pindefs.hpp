/*
 * pindefs.hpp
 *
 *  Created on: Mar 15, 2013
 *      Author: walmis
 */

#ifndef PINDEFS_HPP_
#define PINDEFS_HPP_

#include <xpcc/architecture.hpp>


GPIO__INPUT(progPin, 2, 0);

GPIO__OUTPUT(usbConnPin, 1, 30);

GPIO__OUTPUT(ledRed, 0, 25);
GPIO__OUTPUT(ledGreen, 0, 26);

//turn on laser
GPIO__OUTPUT(laserEn, 1, 18);

//laser dac chip select
GPIO__OUTPUT(laserCs, 1, 19);

//mirror motor PLL Lock signal
GPIO__INPUT(mmLock, 1, 29);

//mirror motor enable
GPIO__OUTPUT(_mmEnable, 0, 0);
typedef xpcc::gpio::Invert<_mmEnable> mmEnable;

//photodiode inputs
GPIO__INPUT(photoDiode1, 0, 9);
GPIO__INPUT(photoDiode2, 0, 8);

//stepper motor outputs
GPIO__OUTPUT(stepper1, 2, 5);
GPIO__OUTPUT(stepper2, 2, 3);
GPIO__OUTPUT(stepper3, 2, 2);
GPIO__OUTPUT(stepper4, 2, 4);


#endif /* PINDEFS_HPP_ */
