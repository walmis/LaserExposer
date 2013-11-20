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

#include <xpcc/driver/motor/linear_metric_stepper.hpp>

#include "mcp415x.hpp"
#include <new>

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "LSE v0.1";

#include "pindefs.hpp"
#include "mirror_motor.hpp"
#include "laser_module.hpp"




class Stepper : public MetricLinearStepper<stepperOutputs> {
	typedef MetricLinearStepper<stepperOutputs> Base;
public:
	Stepper() : MetricLinearStepper<stepperOutputs>(1.0/23.62) {
		sw2::setInput();
		setSpeed(3);
	}

	void goHome() {
		move(-10000);
	}

protected:
	void handleTick() override {
		if(!sw2::read()) {
			if(moveSteps < 0) {
				stop();
				resetStepPosition();
			}
		}

		this->Base::handleTick();
	}

};


Stepper stepper;

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


void boot_jump( uint32_t address ){
   __asm("LDR SP, [R0]\n"
   "LDR PC, [R0, #4]");
}




//xpcc::IOStream stdout(device);

UARTDevice uart(115200);
//xpcc::log::Logger xpcc::log::debug(device);

xpcc::NullIODevice null;

xpcc::log::Logger xpcc::log::error(null);

enum { r0, r1, r2, r3, r12, lr, pc, psr};

extern "C" void HardFault_Handler(void)
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

	if(progPin::read() == 0) {
		//NVIC_SystemReset();

		NVIC_DeInit();

		usbConnPin::setOutput(false);
		delay_ms(100);

		LPC_WDT->WDFEED = 0x56;

		//boot_jump(0);
	}

}

MirrorMotor mirrorMotor;
LaserModule laser;


inline void delay(uint32_t ns) {

	uint32_t t = LPC_RIT->RICOUNTER + (ns-20);
	while(LPC_RIT->RICOUNTER < t);
}

class Controller : TickerTask {
public:

	uint16_t delays[1024];
	volatile uint16_t numDelays;

	Controller() {

		CLKPwr::setClkPower(CLKPwr::PType::PCRIT, true);
		CLKPwr::setClkDiv(CLKPwr::ClkType::RIT, CLKPwr::ClkDiv::DIV_1);

		LPC_RIT->RICTRL |= (1<<0) | (1<<1);

		Timer1::enableTimer(1);

		NVIC_EnableIRQ(TIMER1_IRQn);
	}

	bool started = false;
	bool locked = false;

	Timeout<> cfgTimeout;

	bool isStable() {
		return !cfgTimeout.isActive() && locked && started;
	}

	void handleTick() {
		if(!mirrorMotor.isLocked()) {
			//laser.enable(false);
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
			LPC_RIT->RICOUNTER = 0;
			LPC_RIT->RICOMPVAL = 0xFFFFFFFF;

			GpioInterrupt::enableInterrupt(photoDiode1::Port, photoDiode1::Pin);
		}

		if(cfgTimeout.isExpired() && cfgTimeout.isActive()) {
			XPCC_LOG_DEBUG .printf("configure match\n");
			uint32_t match = ((SystemCoreClock) / 1000000) * (scanPeriod-10);

			Timer1::configureMatch(0,
					match,
					(Timer1::MatchFlags)(Timer1::MatchFlags::RESET_ON_MATCH |
					Timer1::MatchFlags::INT_ON_MATCH));

			cfgTimeout.stop();
		}


		if(!numDelays && cfgTimeout.isExpired()) {
			//laser.enable(false);
		}
	}

	void handleInterrupt(int irqn) {
		if(irqn == TIMER1_IRQn) {
			laser.enable(true);
			//sw2::set(true);


			Timer1::clearIntPending(Timer1::IntType::TIM_MR0_INT);
			//sw2::set(false);
		}
	}

	void start(int freq = 1000) {

		mirrorMotor.setClk(freq);
		mirrorMotor.enable(true);

		started = true;
		locked = false;


		laser.setOutput(70);
		laser.enable(false);

		GpioInterrupt::enableGlobalInterrupts();

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


	void dl_start(uint32_t ticks) {

		LPC_RIT->RICOMPVAL = ticks;
	}

	void dl_wait() {
		while(!(LPC_RIT->RICTRL & 1));
		LPC_RIT->RICTRL |= 1;
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

				//XPCC_LOG_DEBUG .printf("t %d\n", diff);

				startTime = time;

			}

		} else {

//			for(int i  = 0; i < numDelays; i++) {
//				XPCC_LOG_DEBUG .printf("delay %d\n", delays[i]>>1);
//			}

			Timer1::resetCounter();

			while(photoDiode1::read());

			laser.enable(false);

			LPC_RIT->RICOUNTER = 0;
			LPC_RIT->RICOMPVAL = 0xFFFFFFFF;

			LPC_RIT->RICTRL |= 1;

			register uint16_t d;
			register bool enable = 0;



			//while(*delay) {
			for(int i = 0; i < numDelays; i++) {
				d = delays[i];

				enable = d & 1;
				d >>= 1;
				dl_start(d);

				//sw2::set(enable);
				laser.enable(enable);

				dl_wait();


			}
			//sw2::reset();
			laser.enable(false);
		}

		//sw2::set();
		//delay(5000);
//		sw2::reset();

//		currentDelay = 0;
//

//
//		LPC_RIT->RICOUNTER = 0;
//		LPC_RIT->RICOMPVAL = d;
//
//		disableGpioInt();


		//sw2::reset();


	}

};

Controller controller;


//xpcc::USBCDCMSD<TestMSD> device(0xffff, 2010, 0);
//USBMSD<TestMSD> device;
USBSerial device(0xffff);

xpcc::IOStream stream(device);
xpcc::log::Logger xpcc::log::info(device);

//xpcc::log::Logger xpcc::log::debug(uart);
xpcc::log::Logger xpcc::log::debug(device);

class MyTerminal : public Terminal {
public:
	MyTerminal(IODevice& device) : Terminal(device) {};

protected:
	void handleCommand(uint8_t nargs, char* argv[]) {

		if(cmp(argv[0], "laser")) {
			if(cmp(argv[1], "power")) {
				int power = to_int(argv[2]);
				XPCC_LOG_DEBUG .printf("Set power %d\n", power);
				laser.setOutput(power);
			} else
			if(cmp(argv[1], "enable")) {
				laser.enable(true);
			}else
			if(cmp(argv[1], "disable")) {
				laser.enable(false);
			}
		} else

		if(cmp(argv[0], "period")) {
			XPCC_LOG_DEBUG .printf("%d\n", controller.scanPeriod);

		} else

		if(cmp(argv[0], "mm")) {

			if(cmp(argv[1], "en")) {
				XPCC_LOG_DEBUG .printf("Enable\n");
				mirrorMotor.enable(true);
			}
			else if(cmp(argv[1], "dis")) {
				mirrorMotor.enable(false);
			}
			if(cmp(argv[1], "freq")) {
				int speed = to_int(argv[2]);
				XPCC_LOG_DEBUG .printf("Set freq %d\n", speed);
				mirrorMotor.setClk(speed);
			}

		}

		if(cmp(argv[0], "start")) {
			if(nargs == 2) {
				int freq = to_int(argv[1]);
				controller.start(freq);
			} else {
				controller.start();
			}
		}

	}

};

class CmdTerminal : public Terminal {
public:
	CmdTerminal(IODevice& device) : Terminal(device) {};

protected:

	uint16_t readWord() {
		char a;
		char b;

		while(!device.read(a));
		while(!device.read(b));

		return (b << 8) | a;
	}

	void handleCommand(uint8_t nargs, char* argv[]) {
		if(cmp(argv[0], "start")) {
			if(nargs == 2) {
				int freq = to_int(argv[1]);
				controller.start(freq);
			} else {
				controller.start();
			}
		}
		if(cmp(argv[0], "linedata")) {

			laser.enable(false);

			uint16_t numItems = readWord();
			//XPCC_LOG_DEBUG .printf("num items %d\n", numItems);
			controller.numDelays = 0;

			int i;
			for(i = 0; i < numItems; i++) {

				controller.delays[i] = readWord();
				//XPCC_LOG_DEBUG .printf("delay[%d] = %d\n", i, controller.delays[i]>>1);
			}
			controller.delays[i+1] = 0;

			//XPCC_LOG_DEBUG .printf("finished\n");
			controller.numDelays = numItems;
		} else
		if(cmp(argv[0], "period")) {
			IOStream str(device);
			if(controller.isStable()) {
				str.printf("%d\n", controller.scanPeriod);
			} else {
				str.printf("%d\n", 0);
			}
		}
		if(cmp(argv[0], "step")) {
			if(nargs == 2) {
				int a = to_int(argv[1]);
				stepper.move(a);
			} else {
				stepper.step(FORWARD);
			}
		}
		if(cmp(argv[0], "stepper") && cmp(argv[1], "speed")) {
			int a = to_int(argv[2]);
			stepper.setSpeed(a);
		}

		if(cmp(argv[0], "stepper") && cmp(argv[1], "move")) {
			int a = to_int(argv[2]);
			stepper.setPosition(a);
		}
		if(cmp(argv[0], "stepper") && cmp(argv[1], "home")) {
			stepper.goHome();
		}
	}

};

CmdTerminal cmd(device);

MyTerminal terminal(uart);



extern "C"
__attribute__ ((section (".fastcode")))
void EINT3_IRQHandler() {

	if (GpioInterrupt::checkInterrupt(EINT3_IRQn, photoDiode1::Port,
			photoDiode1::Pin, IntEvent::RISING_EDGE)) {
		controller.endDetectInt();

		//clear any interrupts
		GpioInterrupt::checkInterrupt(EINT3_IRQn, photoDiode1::Port,
			photoDiode1::Pin, IntEvent::RISING_EDGE);
	}

}

extern "C"
void TIMER0_IRQHandler() {
	Timer0::clearIntPending(Timer0::IntType::TIM_MR0_INT);
}

//Tsk test;

int main() {
	//debugIrq = true;

//	SpiMaster0::initialize(SpiMaster0::Mode::MODE_0, 25000000);
//	Pinsel::setFunc(1, 20, 3); //SCK
//	Pinsel::setFunc(1, 24, 3); //MOSI

	//LPC_SSP0->CR1 |= 1;


	lpc17::SysTickTimer::enable();
	lpc17::SysTickTimer::attachInterrupt(sysTick);

	xpcc::Random::seed();

	//DMA* inst = DMA::instance();

	NVIC_SetPriority(USB_IRQn, 16);
	NVIC_SetPriority(EINT3_IRQn, 1);
	NVIC_SetPriority(RIT_IRQn, 0);

	xpcc::PeriodicTimer<> t(500);

	usbConnPin::setOutput(true);
	device.connect();
	//msddevice.connect();

	TickerTask::tasksRun();

}
