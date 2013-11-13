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

#include "mcp415x.hpp"
#include <new>

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "LSE v0.1";

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


void boot_jump( uint32_t address ){
   __asm("LDR SP, [R0]\n"
   "LDR PC, [R0, #4]");
}




//xpcc::IOStream stdout(device);

UARTDevice uart(115200);
//xpcc::log::Logger xpcc::log::debug(device);
xpcc::log::Logger xpcc::log::debug(uart);

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


class Controller : TickerTask {
public:

	Controller() {
		end = false;

	}

	bool started = false;
	bool configured = false;

	Timeout<> configurationTimeout;

	void handleTick() {
		if(!mirrorMotor.isLocked()) {
			laser.enable(false);
			configured = false;
		}

		if(mirrorMotor.isLocked() && started && !configured) {
			XPCC_LOG_DEBUG .printf("configured\n");
			configured = true;

			configurationTimeout.restart(1000);
			laser.enable(true);

			Timer0::intOnMatch(0, false);
			GpioInterrupt::enableInterrupt(photoDiode1::Port, photoDiode1::Pin);
		}
	}

	void start() {
		Timer0::intOnMatch(0, false);

		mirrorMotor.setClk(1000);
		mirrorMotor.enable(true);

		started = true;
		configured = false;

		laser.setOutput(70);
		laser.enable(false);

		GpioInterrupt::enableGlobalInterrupts();

	}

	void endDetectInt() {
		//XPCC_LOG_DEBUG .printf("end\n");

		laser.enable(false);

		delay_us(10);
		count = 0;
		Timer0::intOnMatch(0, true);
	}

	void timerInt() {
		//XPCC_LOG_DEBUG .printf("timer\n");
		Timer0::intOnMatch(0, false);

		for(int i = 0; i < 8; i++) {
			laser.enable(true);
			delay_us(1);
			laser.enable(false);
			delay_us(1);
		}

		delay_us(500);
		laser.enable(true);

		for(int i = 0; i < 8; i++) {
			laser.enable(true);
			delay_us(1);
			laser.enable(false);
			delay_us(1);
		}
		laser.enable(true);

//
//		if(count > 10)
//			laser.enable(true);

//		if(count > 11)
//			count = 0;


//		if(count >= 10) {
//			for(int i = 0; i < 8; i++) {
//				laser.enable(true);
//				delay_us(1);
//				laser.enable(false);
//				delay_us(1);
//			}
//
//			for(int i = 0; i < 8; i++) {
//				laser.enable(true);
//				delay_us(10);
//				laser.enable(false);
//				delay_us(10);
//			}
//			count = 0;
//		}

//		Timer0::intOnMatch(0, false);
//		if(end) {
//			end = false;
//
//
//			for(int i = 0; i < 8; i++) {
//				laser.enable(true);
//				delay_us(100);
//				laser.enable(false);
//				delay_us(100);
//			}
//
//			laser.enable(true);
//			GpioInterrupt::enableInterrupt(photoDiode1::Port, photoDiode1::Pin);
////			laser.enable(false);
////			delay_us(500);
////			laser.enable(true);
//		}
	}

	volatile uint8_t count;
	volatile bool end;
};

Controller controller;


//xpcc::USBCDCMSD<TestMSD> device(0xffff, 2010, 0);
//USBMSD<TestMSD> device;
USBSerial device(0xffff);
xpcc::IOStream stream(device);
xpcc::log::Logger xpcc::log::info(device);

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

			controller.start();


		}

	}

};

MyTerminal terminal(uart);


extern "C"
void EINT3_IRQHandler() {

	if (GpioInterrupt::checkInterrupt(EINT3_IRQn, photoDiode1::Port,
			photoDiode1::Pin, IntEvent::RISING_EDGE)) {
		controller.endDetectInt();

	}

}

extern "C"
void TIMER0_IRQHandler() {
	controller.timerInt();
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

	xpcc::PeriodicTimer<> t(500);

	usbConnPin::setOutput(true);
	device.connect();
	//msddevice.connect();

	TickerTask::tasksRun();

}
