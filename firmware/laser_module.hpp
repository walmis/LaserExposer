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
#include <math.h>

using namespace xpcc;
using namespace xpcc::lpc17;

typedef SoftwareSpi<laserSck, laserData, gpio::Unused, 500000> SoftSpi;

class LaserModule {
public:
	LaserModule() {
		laserEn::setOutput(0);
		laserCs::setOutput(1);

		//Pinsel::setFunc(1, 20, 3); //sck0
		//Pinsel::setFunc(1, 24, 3); //mosi0

		SpiMaster0::initialize(SpiMaster0::Mode::MODE_3, 16000000);
		SoftSpi::initialize();

		dac.initialize();
	}

	inline void disable() {
		laserEn::reset();
	}

	inline void enable(bool en) {
		laserEn::set(en);
	}

	void setOutputData(uint8_t* buffer, uint16_t length) {
		LPC_SSP0->DMACR = 3;
		txCfg.channelNum(DMAChannel::Channel_0)
				->dstConn(DMAConnection::SSP0_Tx)
				->srcMemAddr(buffer)
				->transferSize(length>0xFFF ? 0xFFF : length)
				->transferType(DMATransferType::m2p)
				->dmaLLI(0);

		if(length > 0xFFF) {
			lli.reset(&txCfg);
			lli.transferSize(length - 0xFFF);
			lli.srcAddr(buffer + 0xFFF);
			txCfg.dmaLLI(&lli);
		}

		txCfg.setup();
	}

	void beginOutput() {
		Pinsel::setFunc<laserEn>(2);

		txCfg.enable();
	}

	void stopOutput() {
		disable();
		Pinsel::setFunc<laserEn>(0);

		txCfg.disable();
	}
//
//	bool outputComplete() {
//
//		return false;
//	}

	void setOutput(int mA) {
		laserCs::reset();

		const float ma_per_v = (1.0f / 5.6f)*1000;

		uint16_t val = lroundf((mA / ma_per_v) / (2.048f/4096));
		XPCC_LOG_DEBUG .printf("DAC value %d\n", val);
		dac.setChannelA(val);

		laserCs::set();

	}

private:
	Mcp4922<SoftSpi, laserCs, gpio::Unused> dac;

	DMAConfig txCfg;
	DMALLI lli;

};


#endif /* LASER_MODULE_HPP_ */
