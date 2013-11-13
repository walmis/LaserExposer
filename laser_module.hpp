/*
 * laser_module.hpp
 *
 *  Created on: Oct 20, 2013
 *      Author: walmis
 */

#ifndef LASER_MODULE_HPP_
#define LASER_MODULE_HPP_


#include <xpcc/architecture.hpp>
#include <xpcc/driver/dac/mcp4922.hpp>
#include "pindefs.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;

class LaserModule {
public:
	LaserModule() {
		laserEn::setOutput(0);
		laserCs::setOutput(1);

		Pinsel::setFunc(1, 20, 3); //sck0
		Pinsel::setFunc(1, 24, 3); //mosi0


		SpiMaster0::initialize(SpiMaster0::Mode::MODE_0, 10000000);

		dac.initialize();

	}

	inline void enable(bool en) {
		laserEn::set(en);
	}

	void setOutput(int mA) {

		dac.setChannelA( mA* ((680.0f+100.0f)/100.0f)*2, false);

	}


	Mcp4922<SpiMaster0, laserCs, gpio::Unused> dac;

};


#endif /* LASER_MODULE_HPP_ */
