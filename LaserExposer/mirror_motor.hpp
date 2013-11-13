/*
 * mirror_motor.hpp
 *
 *  Created on: Oct 20, 2013
 *      Author: walmis
 */

#ifndef MIRROR_MOTOR_HPP_
#define MIRROR_MOTOR_HPP_

#include <xpcc/architecture.hpp>

using namespace xpcc;
using namespace xpcc::lpc17;

class MirrorMotor : TickerTask {
public:
	MirrorMotor() {
		mmEnable::setOutput(0);
		mmLock::setInput();

		//set mirror clk to MAT0[0]
		Pinsel::setFunc(1, 28, 3);

		lpc17::Timer0::enableTimer(1);

		NVIC_EnableIRQ(TIMER0_IRQn);
	}

	void enable(bool en) {
		mmEnable::set(en);
	}

	void setClk(int frequency) {

		int pr = SystemCoreClock/ 2 / frequency;

		lpc17::Timer0::configureMatch(0, pr,
					(Timer0::MatchFlags)(Timer0::MatchFlags::RESET_ON_MATCH |
					Timer0::MatchFlags::INT_ON_MATCH),
					Timer0::ExtMatchOpt::TIM_EXTMATCH_TOGGLE);

		lpc17::Timer0::resetCounter();
		lpc17::Timer0::enable();

	}

	bool isLocked() {
		return mmLock::read();
	}



};




#endif /* MIRROR_MOTOR_HPP_ */
