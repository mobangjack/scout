#include "mytim.hpp"

Mytim::Mytim (const uint32_t millis)
  : expiry(timespec_now())
{
  int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);
  if (tv_nsec >= 1e9) {
    int64_t sec_diff = tv_nsec / static_cast<int> (1e9);
    expiry.tv_nsec = tv_nsec - static_cast<int> (1e9 * sec_diff);
    expiry.tv_sec += sec_diff;
  } else {
    expiry.tv_nsec = tv_nsec;
  }
}

int64_t
Mytim::remaining ()
{
  timespec now(timespec_now());
  int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
  millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
  return millis;
}

bool Mytim::timeout()
{
	return remaining < 0;
}

void Mytim::reset(const uint32_t millis)
{
	expiry= timespec_now();
	 int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);
  if (tv_nsec >= 1e9) {
    int64_t sec_diff = tv_nsec / static_cast<int> (1e9);
    expiry.tv_nsec = tv_nsec - static_cast<int> (1e9 * sec_diff);
    expiry.tv_sec += sec_diff;
  } else {
    expiry.tv_nsec = tv_nsec;
  }
}

timespec
Mytim::timespec_now ()
{
	timespec time;
	clock_gettime(CLOCK_MONOTONIC, &time);
	return time;
}

timespec
timespec_from_ms (const uint32_t millis)
{
  timespec time;
  time.tv_sec = millis / 1e3;
  time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
  return time;
}
	
