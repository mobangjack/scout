#include "ramp.hpp"

Ramp::Ramp(uint32_t t)
{
	this->i = 0;
	this->t = t;
}

double Ramp::ramp()
{
	if(i>=t)
	{
		return 1;
	}
	else
	{
		r = i/t;
		i++;
		return r;
	}
}

void Ramp::reset()
{
	this->i = 0;
	this->r = 0;
}
