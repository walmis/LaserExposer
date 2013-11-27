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


		SpiMaster0::initialize(SpiMaster0::Mode::MODE_3, 24000000);

		dac.initialize();
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


	void setOutput(int mA) {

		//SpiMaster0::initialize(SpiMaster0::Mode::MODE_0, 10000000);

		//XPCC_LOG_DEBUG .printf("Set output\n");

		if(SpiMaster0::isRunning()) {
			while(!outputComplete());
		}


		dac.setChannelA( mA* ((680.0f+100.0f)/100.0f)*2, false);

	}


	Mcp4922<SpiMaster0, laserCs, gpio::Unused> dac;

};


#endif /* LASER_MODULE_HPP_ */
