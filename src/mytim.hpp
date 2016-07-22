#pragma once

#include <stdint.h>
#include <unistd.h>
//#include <sys/time.h>
#include <time.h>

class Mytim
{
	public:
	Mytim(const uint32_t millis);         
	int64_t remaining();
	bool timeout();
	void reset(const uint32_t millis);
	
	private:
	static timespec timespec_now();
	timespec expiry;
};
