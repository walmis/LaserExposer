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
#include <xpcc/driver/connectivity/spi/software_spi.hpp>
#include "pindefs.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;

typedef SoftwareSpi<laserSck, laserData, gpio::Unused, 500000> SoftSpi;

enum OpCode {
	SET_POWER = 0xC5,
	SET_FOCUS = 0xA5
};

class LaserModule {
public:
	LaserModule() {
		laserEn::setOutput(0);
		laserCs::setOutput(1);

		//Pinsel::setFunc(1, 20, 3); //sck0
		//Pinsel::setFunc(1, 24, 3); //mosi0

		SpiMaster0::initialize(SpiMaster0::Mode::MODE_3, 16000000);
		SoftSpi::initialize();
	}

	inline void disable() {
		laserEn::reset();
	}

	inline void enable(bool en) {
		laserEn::set(en);
	}

	void outputData(uint8_t* buffer, uint16_t length) {
		//start DMA transfer
		if(!SpiMaster0::prepareTransfer(buffer, 0, length)) {
			XPCC_LOG_DEBUG .printf("start failed\n");
		}
	}

	void beginOutput() {
		Pinsel::setFunc<laserEn>(2);

		SpiMaster0::startTransfer();
	}

	void stopOutput() {
		disable();
		Pinsel::setFunc<laserEn>(0);

		SpiMaster0::stopTransfer();
	}

	bool outputComplete() {
		if(SpiMaster0::isFinished()) {
			//switch back to GPIO mode
			Pinsel::setFunc<laserEn>(0);
			return true;
		}
		return false;
	}


	void setFocus(int16_t steps) {
		laserCs::reset();

		SoftSpi::write(SET_FOCUS);

		SoftSpi::write(steps >> 8);
		SoftSpi::write(steps & 0xFF);

		laserCs::set();
	}

	void setOutput(int mA) {
		laserCs::reset();

		SoftSpi::write(SET_POWER);
		SoftSpi::write(mA);

		laserCs::set();

//		delay_us(2);
//
//		laserCs::reset();
//		SoftSpi::write(mA);
//		laserCs::set();
	}


};


#endif /* LASER_MODULE_HPP_ */
