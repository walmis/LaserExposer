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
GPIO__OUTPUT(laserEn, 0, 18);

//laser dac chip select
#warning "Broken pin Workaround enabled"
GPIO__OUTPUT(laserCs, 0, 1);
GPIO__OUTPUT(laserSck, 0, 10);
GPIO__OUTPUT(laserData, 0, 26);

//GPIO__OUTPUT(laserCs, 1, 19);
//GPIO__OUTPUT(laserSck, 1, 20);
//GPIO__OUTPUT(laserData, 1, 24);

GPIO__IO(sw2, 2, 1);
GPIO__IO(dbgPin, 2, 0);

//mirror motor PLL Lock signal
GPIO__INPUT(mmLock, 1, 29);

//mirror motor enable
GPIO__OUTPUT(_mmEnable, 0, 0);
typedef xpcc::gpio::Invert<_mmEnable> mmEnable;

//photodiode inputs
GPIO__INPUT(photoDiode1, 0, 9);
GPIO__OUTPUT(photoDiode2, 0, 8);

//stepper motor outputs
GPIO__OUTPUT(stepper1A, 2, 5);
GPIO__OUTPUT(stepper1B, 2, 4);
GPIO__OUTPUT(stepper2A, 2, 3);
GPIO__OUTPUT(stepper2B, 2, 2);


typedef xpcc::gpio::Nibble<stepper1A, stepper1B, stepper2A, stepper2B> stepperOutputs;

#endif /* PINDEFS_HPP_ */
