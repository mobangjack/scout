#pragma once

class Serial
{
	public:
	Serial(const char* port, int baudrate = 115200);
	virtual ~Serial();
	int connect();
	int rx(unsigned char* data, int len);
	int tx(unsigned char* data, int len);
	void release();
	
	protected:
	const char* port;
	int baudrate;
	int fd;
};

