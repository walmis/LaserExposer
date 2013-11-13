/*
 * mcp415x.hpp
 *
 *  Created on: Oct 10, 2013
 *      Author: walmis
 */

#ifndef MCP415X_HPP_
#define MCP415X_HPP_

template <typename Spi, typename Cs>
class MCP4151 {
public:
	MCP4151() {
		Cs::set();

		setValue(127);
	}

	MCP4151& operator++() {
		Cs::reset();
		Spi::write(0b00000100);
		Cs::set();

		if(value < 0x100)
			value++;

		return this;
	}

	MCP4151& operator--() {
		Cs::reset();
		Spi::write(0b00001000);
		Cs::set();

		if(value > 0) {
			value--;
		}
		return this;
	}

	void setValue(uint8_t v) {
		Cs::reset();
		Spi::write(0);
		Spi::write(v);
		Cs::set();

		value = v;
	}


private:
	int16_t value;
};


#endif /* MCP415X_HPP_ */
