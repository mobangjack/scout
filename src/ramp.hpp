#pragma once

#include <stdint.h>

class Ramp
{
	public:
	Ramp(uint32_t t);
	double ramp();
	void reset();
	
	protected:
	uint32_t i;
	uint32_t t;
	double r;
};
