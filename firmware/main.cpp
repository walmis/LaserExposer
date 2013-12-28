/*
 * main.cpp
 *
 *  Created on: Feb 28, 2013
 *      Author: walmis
 */
#include <lpc17xx_nvic.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_pinsel.h>

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/debug.hpp>

#include <xpcc/driver/connectivity/usb/USBDevice.hpp>
#include <xpcc/io/terminal.hpp>

#include <xpcc/driver/motor/stepper_motor.hpp>

#include <math.h>
#include <new>

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "LSE v0.9";

#include "pindefs.hpp"
#include "mirror_motor.hpp"
#include "laser_module.hpp"

class UARTDevice : public IODevice {

public:
	UARTDevice(int baud) {

		UART_CFG_Type cfg;
		UART_FIFO_CFG_Type fifo_cfg;

		UART_ConfigStructInit(&cfg);
		cfg.Baud_rate = baud;

		UART_Init(LPC_UART0, &cfg);

		PINSEL_CFG_Type PinCfg;

		PinCfg.Funcnum = 1;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Pinnum = 2;
		PinCfg.Portnum = 0;
		PINSEL_ConfigPin(&PinCfg);
		PinCfg.Pinnum = 3;
		PINSEL_ConfigPin(&PinCfg);

		UART_Init(LPC_UART0, &cfg);

		UART_FIFOConfigStructInit(&fifo_cfg);

		UART_FIFOConfig(LPC_UART0, &fifo_cfg);

		UART_TxCmd(LPC_UART0, ENABLE);
	}

	void
	write(char c) {
		while (!(LPC_UART0->LSR & UART_LSR_THRE)) {
		}

		UART_SendByte(LPC_UART0, c);
	};

	void
	flush(){};

	/// Read a single character
	bool read(char& c) {
		if((LPC_UART0->LSR & 1)) {
			c = (LPC_UART0->RBR & UART_RBR_MASKBIT);
			return true;
		}
		return false;
	}
};

//#define _DEBUG
#define _SER_DEBUG

UARTDevice uart(460800);

USBSerial device(0xffff);
xpcc::IOStream stream(device);

#ifdef _DEBUG
xpcc::log::Logger xpcc::log::info(device);
xpcc::log::Logger xpcc::log::debug(device);
#else
#ifdef _SER_DEBUG
xpcc::log::Logger xpcc::log::info(uart);
xpcc::log::Logger xpcc::log::debug(uart);
#else
xpcc::log::Logger xpcc::log::info(null);
xpcc::log::Logger xpcc::log::debug(null);
#endif
#endif


#define NUM_MSTEPS 48

uint32_t microStepCurve[NUM_MSTEPS];

class Stepper : public StepperMotor<stepperOutputs> {
	typedef StepperMotor<stepperOutputs> Base;
public:


	class ChannelState {
	public:
		ChannelState() {
			pwm_channel = 0;
			state = NUM_MSTEPS;
		}

		void setChannel(uint8_t pwmChannel) {
			pwm_channel = pwmChannel;
		}

		void update() {
			if(state < NUM_MSTEPS) {
				PWM::matchUpdate(pwm_channel, microStepCurve[state],
						PWM::UpdateType::PWM_MATCH_UPDATE_NEXT_RST);
				state++;
			}
		}

		void reset() {
			state = 0;
		}
	protected:
		uint8_t pwm_channel;
		uint8_t state;
	};

	ChannelState pwmChannels[4];

	uint8_t state;

	Stepper() {
		state = 0;

		sw2::setInput();

		PWM::initTimer(1);

		PWM::matchUpdate(0, 1000);
		PWM::configureMatch(0, PWM::MatchFlags::RESET_ON_MATCH | PWM::MatchFlags::INT_ON_MATCH);

		PWM::matchUpdate(3, 0);
		PWM::matchUpdate(4, 0);
		PWM::matchUpdate(5, 0);
		PWM::matchUpdate(6, 0);

		PWM::channelEnable(3);
		PWM::channelEnable(4);
		PWM::channelEnable(5);
		PWM::channelEnable(6);

		//NVIC_SetPriority(PWM1_IRQn, 1);


		pwmChannels[0].setChannel(3);
		pwmChannels[1].setChannel(4);
		pwmChannels[2].setChannel(5);
		pwmChannels[3].setChannel(6);

		Pinsel::setFunc<stepper1A>(1);
		Pinsel::setFunc<stepper1B>(1);
		Pinsel::setFunc<stepper2A>(1);
		Pinsel::setFunc<stepper2B>(1);


		PWM::enable();

		//setMode(DriveMode::HALF_STEP);
		setSpeed(1);
	}

	void handleInterrupt(int irqn) override {
		if(irqn == PWM1_IRQn) {
			//XPCC_LOG_DEBUG .printf("tm\n");
			for(int i = 0; i < 4; i++) {
				pwmChannels[i].update();
			}
			PWM::clearIntPending(PWM::IntType::PWM_INTSTAT_MR0);
		}
	}

	void goHome() {
		move(-10000);
	}

	void setSpeed(uint8_t delay) {
		//XPCC_LOG_DEBUG .printf("speed %d\n", delay);
		int t = (delay * 1000 * 2) / NUM_MSTEPS;

		int pr = SystemCoreClock / (1000000 / t);

		float power = 0.701335f * expf(-0.0238813f * delay);
		if(power < 0.5)
			power = 0.5;

		//XPCC_LOG_DEBUG .printf("%d\n", int(power*100));

		float step = M_PI / (NUM_MSTEPS-1);
		float pos = 0;
		for(int i = 0; i < NUM_MSTEPS; i++) {
			microStepCurve[i] = (uint32_t)(sinf(pos) * pr * power);
			pos += step;
		}

		PWM::matchUpdate(0, pr);

		this->Base::setSpeed(delay);
	}

protected:

	void setOutput(uint8_t bits) override {

		//XPCC_LOG_DEBUG .printf("bits %x\n", bits);

		if(bits == 0) {
			PWM::matchUpdate(3, 0);
			PWM::matchUpdate(4, 0);
			PWM::matchUpdate(5, 0);
			PWM::matchUpdate(6, 0);
		}

		if(!(state & 0b0001) && (bits & 0b0001)) {
			pwmChannels[0].reset();
		}
		if(!(state & 0b0010) && (bits & 0b0010)) {
			pwmChannels[1].reset();
		}
		if(!(state & 0b0100) && (bits & 0b0100)) {
			pwmChannels[2].reset();
		}
		if(!(state & 0b1000) && (bits & 0b1000)) {
			pwmChannels[3].reset();
		}

		state = bits;
	}


	void handleTick() override {

		if(!sw2::read()) {
			if(moveSteps < 0) {
				stop();
				wait();
				resetStepPosition();
			}
		}

		this->Base::handleTick();
	}

};


Stepper stepper;

void boot_jump( uint32_t address ){
   __asm("LDR SP, [R0]\n"
   "LDR PC, [R0, #4]");
}


xpcc::NullIODevice null;

xpcc::log::Logger xpcc::log::error(null);

enum { r0, r1, r2, r3, r12, lr, pc, psr};

extern "C" void HardFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}
extern "C" void UsageFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}
extern "C" void BusFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}

extern "C"
void Hard_Fault_Handler(uint32_t stack[]) {

	//register uint32_t* stack = (uint32_t*)__get_MSP();

	XPCC_LOG_DEBUG .printf("Hard Fault\n");

	XPCC_LOG_DEBUG .printf("r0  = 0x%08x\n", stack[r0]);
	XPCC_LOG_DEBUG .printf("r1  = 0x%08x\n", stack[r1]);
	XPCC_LOG_DEBUG .printf("r2  = 0x%08x\n", stack[r2]);
	XPCC_LOG_DEBUG .printf("r3  = 0x%08x\n", stack[r3]);
	XPCC_LOG_DEBUG .printf("r12 = 0x%08x\n", stack[r12]);
	XPCC_LOG_DEBUG .printf("lr  = 0x%08x\n", stack[lr]);
	XPCC_LOG_DEBUG .printf("pc  = 0x%08x\n", stack[pc]);
	XPCC_LOG_DEBUG .printf("psr = 0x%08x\n", stack[psr]);


	while(1) {
		if(!progPin::read()) {
			for(int i = 0; i < 10000; i++) {}
			NVIC_SystemReset();
		}
	}
}

void sysTick() {

	LPC_WDT->WDFEED = 0xAA;
	LPC_WDT->WDFEED = 0x55;

//	if(progPin::read() == 0) {
//		//NVIC_SystemReset();
//
//		NVIC_DeInit();
//
//		usbConnPin::setOutput(false);
//		delay_ms(100);
//
//		LPC_WDT->WDFEED = 0x56;
//	}

}

MirrorMotor mirrorMotor;
LaserModule laser;


void expandDelay(uint8_t* buffer, uint16_t length, uint16_t value,
		uint16_t &byteindex, uint8_t& bitindex) {

	uint16_t d = value;

	bool laserEn = d & 1;
	d >>= 1;

	//printf("delay %d\n", d);

	if (bitindex) {
		for (int8_t bit = (7 - bitindex); bit >= 0; bit--) {
			if (laserEn)
				buffer[byteindex] |= (1 << bit);
			else
				buffer[byteindex] &= ~(1 << bit);

			bitindex++;
			//printf("stuff bit %d, byte %d\n", bit, byteindex);
			d--;
			if (!d)
				break;
		}

		if (bitindex > 7) {
			byteindex++;
			bitindex = 0;
		}
	}

	int bytes = d / 8;
	int residue = d & 7;
	if(bytes) {
		//XPCC_LOG_DEBUG .printf("ms %d %d\n", byteindex, bytes);
		if(byteindex + bytes > length) {
			XPCC_LOG_DEBUG .printf("overflow\n");
			return;
		}

		memset(buffer+byteindex, laserEn ? 0xFF : 0x00, bytes);
	}
	byteindex += bytes;

	for (int8_t bit = 7; bit >= (8 - residue); bit--) {
		//printf("bit %d, byte %d\n", bit, byteindex);
		if (laserEn)
			buffer[byteindex] |= (1 << bit);
		else
			buffer[byteindex] &= ~(1 << bit);

		bitindex++;
	}

}

class Controller : TickerTask {
public:
	//number of scans per line
	uint16_t numScans;

	volatile uint16_t currentScan;

	bool started = false;
	bool locked = false;

	Timeout<> cfgTimeout;

	uint8_t data[10000];

	volatile uint16_t numItems;

	Controller() {

		CLKPwr::setClkPower(CLKPwr::PType::PCRIT, true);
		CLKPwr::setClkDiv(CLKPwr::ClkType::RIT, CLKPwr::ClkDiv::DIV_1);

		numScans = 0;
		currentScan = 0;

		LPC_RIT->RICTRL |= (1<<0) | (1<<1);

		Timer1::enableTimer(1);

		NVIC_EnableIRQ(TIMER1_IRQn);

	}

	bool isStable() {
		return !cfgTimeout.isActive() && locked && started;
	}

	void handleTick() {
		if(!mirrorMotor.isLocked()) {
			laser.enable(false);
			locked = false;
		}

		if(mirrorMotor.isLocked() && started && !locked) {
			XPCC_LOG_DEBUG .printf("configured\n");
			locked = true;

			cfgTimeout.restart(1000);
			laser.enable(true);

			Timer1::enableTimer(1);
			Timer1::enable();

			startTime = 0;
			scanPeriod = 0;

			GpioInterrupt::enableInterrupt(photoDiode1::Port, photoDiode1::Pin);
		}

		if(cfgTimeout.isExpired() && cfgTimeout.isActive()) {
			XPCC_LOG_DEBUG .printf("configure match\n");
			uint32_t match = ((SystemCoreClock) / 1000000) * (scanPeriod-15);

			Timer1::configureMatch(0,
					match,
					(Timer1::MatchFlags)(Timer1::MatchFlags::RESET_ON_MATCH |
					Timer1::MatchFlags::INT_ON_MATCH));

			cfgTimeout.stop();
		}


		if(!numItems && cfgTimeout.isExpired()) {
			//laser.enable(false);
		}

		if(laser.outputComplete()) {

		}
	}

	void handleInterrupt(int irqn) {
		if(irqn == TIMER1_IRQn) {
			if(locked) {
				laser.stopOutput();
				laser.enable(true);
			}

			Timer1::clearIntPending(Timer1::IntType::TIM_MR0_INT);
		}
	}

	void start(int freq = 500) {

		mirrorMotor.setClk(freq);
		mirrorMotor.enable(true);

		started = true;
		locked = false;

		laser.setOutput(70);
		laser.enable(false);

		GpioInterrupt::enableGlobalInterrupts();

	}

	void scanLine(uint16_t* data, uint16_t len) {

		uint16_t index = 0;
		uint8_t bitIndex = 0;

		for(int i = 0; i < len; i++) {
			expandDelay(this->data, sizeof(this->data), data[i], index, bitIndex);
		}

		if(bitIndex)
			index++;

		XPCC_LOG_DEBUG .printf("num %d\n", index);
		numItems = index;
		currentScan = 0;
	}

	void waitScan() {
		if(numScans == 0) return;
		while(currentScan < numScans);
	}

	uint32_t currentDelay;

	void enableGpioInt() {
		GpioInterrupt::enableInterrupt(photoDiode1::Port, photoDiode1::Pin);
	}

	void disableGpioInt() {
		GpioInterrupt::disableInterrupt(photoDiode1::Port, photoDiode1::Pin);
	}

	void clearGpioInt() {
		GpioInterrupt::checkInterrupt(EINT3_IRQn, photoDiode1::Port,
				photoDiode1::Pin, IntEvent::RISING_EDGE);
	}

	uint32_t startTime;
	uint32_t scanPeriod;

	void endDetectInt() {

		if(!cfgTimeout.isExpired() && locked) {
			if(startTime == 0) {
				startTime = LPC_RIT->RICOUNTER;
			} else {
				uint32_t time = LPC_RIT->RICOUNTER;
				uint32_t diff = time - startTime;
				scanPeriod = diff / (SystemCoreClock/1000000);
				startTime = time;
			}
		} else {
			//prepare DMA transfer
			Timer1::resetCounter();

			if(numScans > 0) {
				if(currentScan >= numScans) {

					while(photoDiode1::read());
					laser.enable(false);

					return;
				}
			}
			currentScan++;

            if(numItems) {
            	laser.outputData(data, numItems);
            }

            while(photoDiode1::read());

            if(numItems) {
            	laser.beginOutput();
            }

            laser.enable(false);
		}
	}

};

Controller controller;


class CmdTerminal : public Terminal {
public:
	CmdTerminal(IODevice& device) : Terminal(device) {
		autoincrement = false;
	};

protected:
	bool autoincrement;

	bool readWord(uint16_t &v) {
		Timeout<> t(10);

		char a = 0;
		char b = 0;

		while(!device.read(a) && !t.isExpired());
		while(!device.read(b) && !t.isExpired());

		if(t.isExpired()) {
			XPCC_LOG_DEBUG .printf("read timeout\n");
			return false;
		}

		v = (b << 8) | a;

		return true;
	}

	void handleCommand(uint8_t nargs, char* argv[]) {
//		for(int i = 0; i < nargs; i++) {
//			XPCC_LOG_DEBUG .printf("%s ", argv[i]);
//		}
//		XPCC_LOG_DEBUG .printf("\n");

		if(cmp(argv[0], "start")) {
			if(nargs == 2) {
				int freq = to_int(argv[1]);
				controller.start(freq);
			} else {
				controller.start();
			}
		}

		if(cmp(argv[0], "mm")) {

			if(cmp(argv[1], "on")) {
				XPCC_LOG_DEBUG .printf("Enable\n");
				mirrorMotor.enable(true);
			}
			else if(cmp(argv[1], "off")) {
				mirrorMotor.enable(false);
			}
			if(cmp(argv[1], "freq")) {
				int speed = to_int(argv[2]);
				XPCC_LOG_DEBUG .printf("Set freq %d\n", speed);
				mirrorMotor.setClk(speed);
			}

		}

		if(cmp(argv[0], "laser")) {
			if(cmp(argv[1], "power")) {
				int power = to_int(argv[2]);
				XPCC_LOG_DEBUG .printf("Set power %d\n", power);
				if(power > 0 && power < 200) {
					laser.setOutput(power);
				}
			} else
			if(cmp(argv[1], "on")) {
				laser.enable(true);
			}else
			if(cmp(argv[1], "off")) {
				laser.enable(false);
			}else
			if(cmp(argv[1], "focus")) {
				int steps = to_int(argv[2]);
				laser.setFocus(steps);
				XPCC_LOG_DEBUG .printf("focus %d\n", steps);
			}
		}
		if(cmp(argv[0], "numscans")) {
			controller.numScans = to_int(argv[1]);
		}
		if(cmp(argv[0], "clear")) {
			laser.enable(false);
			laser.stopOutput();

			controller.scanLine(0, 0);
		}
		if(cmp(argv[0], "linedata")) {

			uint16_t numItems = 0;
			if(!readWord(numItems)) {
				return;
			}

			if(numItems > 100) {
				XPCC_LOG_DEBUG .printf("line cnt:%d\n", numItems);
			}
			//controller.numItems = 0;

			uint16_t items[numItems];

			//controller.numItems = 0;

			int i;
			for(i = 0; i < numItems; i++) {
				if(!readWord(items[i])) {
					return;
				}
			}

			laser.enable(false);
			laser.stopOutput();

			controller.waitScan();
			stepper.wait();

			if(autoincrement) {
				stepper.move(1);
			}

			controller.scanLine(items, numItems);

		} else
		if(cmp(argv[0], "period")) {
			if(controller.isStable()) {
				stream.printf("%d\n", controller.scanPeriod);
			} else {
				stream.printf("%d\n", 0);
			}
		}
		if(cmp(argv[0], "flash")) {
			//cause a WD reset
			LPC_WDT->WDFEED = 0;
			while(1);
		}
		if(cmp(argv[0], "stepper") && cmp(argv[1], "autoinc")) {
			autoincrement = to_int(argv[2]);
		} else
		if(cmp(argv[0], "stepper") && cmp(argv[1], "speed")) {
			int a = to_int(argv[2]);
			stepper.setSpeed(a);
		} else
		if(cmp(argv[0], "stepper") && cmp(argv[1], "move")) {
			int a = to_int(argv[2]);
			stepper.moveTo(a);
		} else
		if(cmp(argv[0], "stepper") && cmp(argv[1], "+")) {
			stepper.move(1);
		} else
		if(cmp(argv[0], "stepper") && cmp(argv[1], "-")) {
			stepper.move(-1);
		} else
		if(cmp(argv[0], "stepper") && cmp(argv[1], "home")) {
			stepper.goHome();
		}
		else
		if(cmp(argv[0], "stepper") && cmp(argv[1], "wait")) {
			stepper.wait();
		}
		else
		if(cmp(argv[0], "stepper") && cmp(argv[1], "isbusy")) {
			stream.printf("%d\n", stepper.isBusy());
		}
	}

};

CmdTerminal cmd(device);
CmdTerminal ucmd(uart);

extern "C"
void EINT3_IRQHandler() {

	if (GpioInterrupt::checkInterrupt(EINT3_IRQn, photoDiode1::Port,
			photoDiode1::Pin, IntEvent::RISING_EDGE)) {
		controller.endDetectInt();

		//clear any interrupts
		GpioInterrupt::checkInterrupt(EINT3_IRQn, photoDiode1::Port,
			photoDiode1::Pin, IntEvent::RISING_EDGE);
	}

}


int main() {
	//debugIrq = true;
	sw1::setOutput(0);

	lpc17::SysTickTimer::enable();
	lpc17::SysTickTimer::attachInterrupt(sysTick);

	xpcc::Random::seed();

	stepper.setIdleTimeout(20);

	NVIC_SetPriority(USB_IRQn, 16);
	NVIC_SetPriority(DMA_IRQn, 15);

	NVIC_SetPriority(EINT3_IRQn, 0);
	NVIC_SetPriority(RIT_IRQn, 0);

	xpcc::PeriodicTimer<> t(500);

	usbConnPin::setOutput(true);
	device.connect();

	ledRed::setOutput(true);

	TickerTask::tasksRun();
}
